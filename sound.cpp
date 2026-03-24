// ============================================================
// File    : sound.cpp
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : 音声出力の実装。
//           PWMによるSin波トーン生成、SDカードからのWAV再生、
//           旋回角速度(degpersecond)に応じた音程変化、
//           アンプシャットダウン制御（省電力）。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/03/23
// ============================================================
// Handle speaker, amplifier, PWM-audio signals.
#include "settings.h"
#include "sound.h"
#include "gps.h"
#include "mysd.h"
#include "airdata.h"
#include "imu.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"
#include "SdFat.h"

// WAV ファイルの仕様: メタデータなし・8bit unsigned PCM・16kHz モノラル。

extern SdFs SD;
const int sampleRate = 16000;  // サンプルレート [Hz]。タイマー割り込みもこの周波数で動く。

// ============================================================
// ダブルバッファ方式による WAV ストリーミング再生
//
// 考え方:
//   SD カードからの読み込みは遅い（ブロッキング）ため、
//   バッファ 0 / 1 の 2 つを交互に使う。
//   タイマー割り込み（16kHz）が activeBuffer から 1 サンプルずつ出力する間に、
//   メインループが loadBuffer に次のデータを先読みしておく。
//   activeBuffer が空になったら両者を入れ替える（バッファスワップ）。
// ============================================================
const int WAV_HEADER_SIZE = 45;  // WAV ファイルのヘッダサイズ（このバイト数をスキップする）
const int CHUNK_SIZE = 16 * 1024;  // 1 チャンク = 16KB ≒ 1.0秒分のデータ
uint8_t audioBuffer[2][CHUNK_SIZE];  // ダブルバッファ本体
volatile int activeBuffer = 0;       // 現在再生中のバッファ番号（割り込みから参照）
volatile int loadBuffer = 1;         // 次のデータを書き込む先のバッファ番号
volatile uint32_t bufferPos = 0;     // activeBuffer 内の現在再生位置
volatile bool wav_playing = false;   // WAV 再生中フラグ
volatile int wav_override_volume = 0;  // WAV再生時の最低保証ボリューム（0=制限なし）
volatile int wav_playing_priority = 0;  // 現在再生中 WAV の優先度（高い方が優先）
volatile int tone_playing_priority = 0; // 現在再生中トーンの優先度

volatile bool endOfFile = false;            // ファイル末尾に達したフラグ
volatile bool bufferReady[2] = {false, false};  // 各バッファにデータが入っているか
volatile bool bufferSwapRequest = false;    // 割り込み→メインへのバッファスワップ依頼フラグ

// ============================================================
// 優先度付き WAV 再生制御 — ペンディングキュー（最大2件）
//
// 高優先 WAV が割り込んで低優先 WAV を中断した場合、中断された WAV を
// pending_wav[] に保存する。再生終了後に最も高優先の pending を自動再生。
//
// ポインタは文字列リテラルを指すため寿命は無限（コピー不要）。
// filename == nullptr のスロットは「空き」。
// ============================================================
struct PendingWav { const char* filename; int priority; };
static const char* current_wav_filename  = nullptr;  // 現在再生中のファイル名
static PendingWav  pending_wav[2]        = {{nullptr, 0}, {nullptr, 0}};

// pending_wav[] に filename を追加する。
// 空きスロットがあれば追加、両方埋まっている場合は最も低優先なスロットを上書き。
static void push_pending_wav(const char* filename, int priority) {
    // 現在再生中と同じファイルなら追加しない（連続再生不要）
    if (current_wav_filename == filename) return;
    // 同じファイルが既に pending にあれば無視（重複防止）
    for (int i = 0; i < 2; i++) {
        if (pending_wav[i].filename == filename) return;
    }
    // 空きスロットを探す
    for (int i = 0; i < 2; i++) {
        if (pending_wav[i].filename == nullptr) {
            pending_wav[i] = {filename, priority};
            return;
        }
    }
    // 両方埋まっている → 最も低優先なスロットを上書き（または追加を破棄）
    int lowest = (pending_wav[0].priority <= pending_wav[1].priority) ? 0 : 1;
    if (priority > pending_wav[lowest].priority) {
        DEBUGW_P(20260322, "WARN: pending wav overwritten: ");
        DEBUGW_PLN(20260322, pending_wav[lowest].filename);
        pending_wav[lowest] = {filename, priority};
    } else {
        DEBUGW_P(20260322, "WARN: pending wav dropped (queue full): ");
        DEBUGW_PLN(20260322, filename);
    }
}

// pending_wav[] から最も高優先のエントリを取り出してクリアする。なければ nullptr。
static const char* pop_best_pending_wav(int &out_priority) {
    int best = -1;
    for (int i = 0; i < 2; i++) {
        if (pending_wav[i].filename != nullptr) {
            if (best < 0 || pending_wav[i].priority > pending_wav[best].priority) best = i;
        }
    }
    if (best < 0) return nullptr;
    const char* f = pending_wav[best].filename;
    out_priority  = pending_wav[best].priority;
    pending_wav[best] = {nullptr, 0};
    return f;
}


// ============================================================
// トーン専用バッファ（最大 TONE_BUFFER_SIZE 件、Core1 内でのみアクセス）
//
// loop1() が TASK_PLAY_MULTITONE を受け取ると pushToneBuffer() で積み、
// loop_tone() が WAV 再生状況に応じて順番に非ブロッキングで再生する。
// ミューテックス不要（Core1 の loop1 / loop_tone のみがアクセス）。
// ============================================================
#define TONE_BUFFER_SIZE 10
struct ToneEntry { int freq, duration, count, priority, min_volume; bool solo_play; };
static ToneEntry     toneBuffer[TONE_BUFFER_SIZE];
static int           toneBufHead    = 0;
static int           toneBufTail    = 0;

// 現在再生中のトーン状態（loop_tone() が管理する）
static int           tone_remaining = 0;     // 現在エントリの残りカウント
static int           tone_cur_freq  = 0;     // 現在のトーン周波数
static int           tone_cur_dur   = 0;     // 1 トーン / ギャップの長さ [ms]
static bool          tone_in_gap    = false; // true = カウント間の無音ギャップ中
static bool          tone_paused    = false; // true = WAV 割込みで一時停止中
static unsigned long tone_stop_time = 0;     // 現在フェーズの終了時刻 [ms]

// トーンをバッファに追加する。
// 満杯の場合はエラーを Serial と SD に記録してスキップする。
static bool pushToneBuffer(int freq, int dur, int cnt, int pri, int vol, bool solo_play) {
    int next = (toneBufTail + 1) % TONE_BUFFER_SIZE;
    if (next == toneBufHead) {
        DEBUGW_PLN(20260322, "ERR: tone buffer full, tone dropped");
        enqueueTask(createLogSdTask("ERR: tone buffer full, tone dropped"));
        return false;
    }
    toneBuffer[toneBufTail] = {freq, dur, cnt, pri, vol, solo_play};
    toneBufTail = next;
    return true;
}

// File object and size tracking
FsFile audioFile;
uint32_t totalAudioSize = 0;    // WAV ファイルのオーディオデータ総バイト数（ヘッダを除く）
uint32_t audioDataRead = 0;     // これまでに読み込んだバイト数
uint32_t chunksLoaded = 0;      // ロード済みチャンク数（デバッグ用カウンタ）

const unsigned long playInterval = 10000;  // 将来的な連続再生用インターバル（現状未使用）
extern volatile int sound_volume;          // 音量（0〜100）。button.cpp の設定画面から変更。
extern volatile int vario_volume;          // バリオメーター音量（0〜100）。設定画面から変更。
extern volatile bool vario_inhibit;        // true のとき vario を完全無効化する

// ============================================================
// バリオメーター音声用変数
//
// タイマー割り込みが vario_mode=true のとき、WAV/Sinトーンより低優先で
// 鉛直速度に応じたバリオ音（上昇: 断続ビープ / 下降: 連続低音）を出力する。
// update_vario() が Core0 から約100ms ごとに呼ばれ、vspeed に応じてパラメータを更新する。
// ============================================================
volatile bool     vario_mode          = false; // true のとき割り込みがバリオ音を出力
volatile uint32_t vario_phase_inc     = 0;     // バリオ音の周波数（位相増分）
volatile uint32_t vario_on_samples    = 1280;  // ON サンプル数 (80ms × 16kHz = 1280)
volatile uint32_t vario_cycle_samples = 0;     // サイクル長（0=連続, >0=断続）
volatile bool     vario_ascending     = false; // true=上昇中, false=下降中（音量補正用）
volatile bool     sin_playing         = false; // トーン出力中だけ true（割り込みが参照）
static uint32_t   s_vario_phase_acc   = 0;     // バリオ位相アキュムレータ（割り込み専用）
static uint32_t   s_vario_cycle_pos   = 0;     // バリオサイクル位置（割り込み専用）

// ============================================================
// Sin 波トーン生成用変数（位相アキュムレータ方式）
//
// 位相アキュムレータ (phaseAcc) を毎割り込みごとに phaseInc 分進め、
// 上位 8bit を sineTable のインデックスとして使うことで周波数を制御する。
// テーブルに事前計算した sin 値を入れておくことで、
// 割り込みハンドラ内での重い sin() 計算を避けている。
// ============================================================
const int tableSize = 256;     // sin 波テーブルのサンプル数（8bit = 256 点）
uint8_t sineTable[tableSize];  // 事前計算した sin 波の振幅値（0〜255 の unsigned 8bit）

volatile uint32_t phaseAcc = 0;  // 位相アキュムレータ（32bit 固定小数点、上位 8bit が有効）
volatile uint32_t phaseInc = 0;  // 1 割り込みごとに加算する位相増分（周波数に比例）

// wavmode / sinmode: タイマー割り込み内で各モードの出力を有効にするフラグ。
// 両方同時に true になることで WAV + トーン + バリオの加算ミックスが可能。
// solo_play トーン再生中のみ従来通り排他制御する（sin_solo=true のとき WAV を止める）。
bool wavmode = true;   // true のとき割り込みが WAV バッファを再生する
bool sinmode = true;   // true のとき割り込みが Sin 波を出力する
volatile bool sin_solo = false;  // true のとき solo_play トーン再生中（WAV との同時再生を禁止）



// ============================================================
// タイマー割り込みコールバック（16kHz = 62.5μs ごとに呼ばれる）
//
// WAV・Sin トーン・バリオメーター音を加算ミックスして出力する。
// 各ソースのサンプルを中央値(512)基準のオフセットとして mix に加算し、
// 最後に 0〜1023 にハードクリップして PWM に書き込む。
//
// ミックス制御:
//   sin_solo=false (デフォルト): WAV + トーン + バリオ を同時ミックス
//   sin_solo=true  (solo_play トーン): WAV はバッファを進めず待機、トーンのみ出力
// ============================================================
bool __not_in_flash_func(timerCallback)(struct repeating_timer *t) {
  int mix = 0;  // 中央(512)からのオフセット合計

  // 【WAV】sin_solo=true のとき（solo_play トーン再生中）はバッファを進めない
  if (!sin_solo && wavmode && wav_playing && bufferReady[activeBuffer]) {
    // 8bit unsigned PCM (0〜255, 中心128) → オフセット値に変換して加算
    mix += (int)(audioBuffer[activeBuffer][bufferPos] - 128) * 4 * max((int)sound_volume, wav_override_volume) / 100;
    bufferPos++;
    if (bufferPos >= CHUNK_SIZE) {
        bufferPos = 0;
        bufferSwapRequest = true;
        DEBUG_P(20250424,"bufferSwapRequest");
    }
  }

  // 【Sin トーン】アンプ ON 中（sin_playing=true）のときだけ加算
  if (sinmode && sin_playing) {
    phaseAcc += phaseInc;
    mix += (int)(sineTable[(phaseAcc >> 24) % tableSize] - 128) * 4 * max((int)sound_volume, wav_override_volume) / 100;
  }

  // 【バリオ】WAV/トーン再生中も常時ミックス対象
  if (vario_mode) {
    s_vario_cycle_pos++;
    if (vario_cycle_samples == 0 || s_vario_cycle_pos <= vario_on_samples) {
      // ON 区間: sin 波を加算（上昇中は能率補正で音量を絞る）
      s_vario_phase_acc += vario_phase_inc;
      int vario_vol_adj = vario_ascending ? vario_volume * 2 / 5 : vario_volume;
      mix += (int)(sineTable[(s_vario_phase_acc >> 24) % tableSize] - 128) * 4 * vario_vol_adj / 100;
    } else {
      // OFF 区間: 加算なし（サイクルリセットのみ）
      if (s_vario_cycle_pos >= vario_cycle_samples) {
        s_vario_cycle_pos = 0;  // サイクルをリセット（位相は継続してピッチを維持）
      }
    }
  }

  // ハードクリップして PWM に出力
  int out = 512 + mix;
  if (out < 0)    out = 0;
  if (out > 1023) out = 1023;
  pwm_set_gpio_level(PIN_PWMTONE, out);
  return true;
}


// PAM8302 アンプの電源を制御する（省電力対策）。
// 再生中だけ HIGH にし、それ以外では LOW にしてアンプをシャットダウンする。
// PIN_AMP_SD は SD（シャットダウン）ピンに相当。
void setAmplifierState(bool enable) {
    digitalWrite(PIN_AMP_SD, enable ? HIGH : LOW);
    DEBUG_P(20250424,"Amplifier ");
    DEBUG_P(20250424,enable ? "enabled" : "disabled");
}

// loadBuffer（待機側バッファ）に次のチャンクを SD から読み込む。
// 戻り値: 読み込み成功なら true、スキップまたは失敗なら false。
//
// 処理内容:
//   1. loadBuffer がまだ使用中（bufferReady==true）なら何もしない
//   2. ファイルから最大 CHUNK_SIZE バイト読む
//   3. チャンクが途中で終わった場合（ファイル末尾）、残りを 128（無音）でパディング
//      ※ 8bit unsigned PCM の無音値は 128（0 は最大負圧でノイズになる）
//   4. bufferReady[loadBuffer] = true にして割り込みが使える状態にする
bool __not_in_flash_func(loadNextChunk)() {
    if (audioFile && !bufferReady[loadBuffer] && audioDataRead < totalAudioSize) {
        uint32_t remaining = totalAudioSize - audioDataRead;
        uint32_t toRead = (remaining < CHUNK_SIZE) ? remaining : CHUNK_SIZE;

        uint32_t bytesRead = audioFile.read(audioBuffer[loadBuffer], toRead);

        if (bytesRead > 0) {
            audioDataRead += bytesRead;
            chunksLoaded++;
            if (bytesRead < CHUNK_SIZE) {
                // 末尾の 0x00 パディングを 128（無音）で上書きする（memset より先に行う）
                // 8-bit unsigned PCM では 0x00 = 最大負圧（≠無音）のため、そのまま再生するとノイズになる
                // 末尾から遡って 0x00 が続く間だけ 128 に置換する
                int i = (int)bytesRead - 1;
                while (i >= 0 && audioBuffer[loadBuffer][i] == 0x00) {
                    audioBuffer[loadBuffer][i] = 128;
                    i--;
                }
            

                // ファイル末尾：残りバッファを 128（無音）で埋める
                // 0 で埋めると直流成分でポップノイズが出るため 128 を使う
                memset(audioBuffer[loadBuffer] + bytesRead, 128, CHUNK_SIZE - bytesRead);

                endOfFile = true;
                DEBUG_PLN(20250424,"endOfFile");
            }

            bufferReady[loadBuffer] = true;
            return true;
        } else {
            endOfFile = true;
            return false;
        }
    }
    return false;
}

extern bool sdError;

// WAV ファイルの再生を開始する。
// priority: 優先度（高い値が高優先。現在再生中の方が高優先なら再生せずにリターン）。
//
// 処理フロー:
//   1. 優先度チェック：現在の再生が高優先なら中止
//   2. 再生状態を全リセット（バッファ・位置・フラグ）
//   3. SD カードから WAV ファイルを開き、ヘッダ（WAV_HEADER_SIZE バイト）をスキップ
//   4. 最初のチャンクを loadBuffer に読み込む
//   5. バッファをスワップして activeBuffer に昇格し、アンプを ON にして再生開始
void startPlayWav(const char* filename, int priority, int min_volume) {
    // 優先度チェック①：再生中 WAV の方が高優先なら新リクエストを pending に保存
    if (wav_playing && wav_playing_priority > priority) {
        push_pending_wav(filename, priority);
        DEBUGW_P(20250503,"WAV pending (lower priority than playing WAV): ");
        DEBUGW_PLN(20250427,filename);
        return;
    }

    // 優先度チェック②（solo_play トーンのみ）: 同等以上の優先度なら WAV を pending に回す
    // 通常トーン（sin_solo=false）は WAV とミックス再生するため pending に回さない
    bool tone_in_progress = sin_playing || tone_in_gap || tone_paused;
    if (tone_in_progress && sin_solo && tone_playing_priority >= priority) {
        push_pending_wav(filename, priority);
        DEBUGW_P(20250503,"WAV pending (lower priority than solo_play tone): ");
        DEBUGW_PLN(20250503,filename);
        return;
    }

    // solo_play トーンが進行中で WAV の優先度が高い → トーンを一時停止して WAV を開始
    // 通常トーン（sin_solo=false）はそのままミックス再生するため停止しない
    if (tone_in_progress && sin_solo) {
        sin_playing = false;
        tone_paused = true;
        pwm_set_gpio_level(PIN_PWMTONE, 512);
        if (!vario_mode) setAmplifierState(false);
        DEBUGW_P(20250503,"Solo tone paused by higher-priority WAV: ");
        DEBUGW_PLN(20250503,filename);
    }

    // 現在再生中の低優先 WAV を pending に退避してから上書き
    if (wav_playing && current_wav_filename != nullptr) {
        push_pending_wav(current_wav_filename, wav_playing_priority);
        DEBUGW_P(20250503,"Interrupted wav saved to pending.:");
        DEBUGW_PLN(20250503,current_wav_filename);
    }
    DEBUG_P(20250503,"wav start:");
    DEBUG_PLN(20250503,priority);

    // WAV モードに切り替え。
    // solo_play トーン中のみ sinmode を落とす（通常トーンはミックスのため継続）
    wavmode = true;
    if (sin_solo) sinmode = false;
    wav_playing = false;
    delay(50);  // 割り込みが現在のバッファ出力を終えるのを待つ

    // 再生状態を全リセット
    endOfFile = false;
    audioDataRead = 0;
    chunksLoaded = 0;
    bufferPos = 0;
    bufferReady[0] = false;
    bufferReady[1] = false;
    activeBuffer = 0;      // 最初は buffer0 をアクティブに（後でスワップする）
    loadBuffer = 1;        // 最初のデータは buffer1 に読み込む
    bufferSwapRequest = false;

    // 前のファイルが開いていれば閉じる
    if (audioFile) {
        audioFile.close();
    }

    // WAV ファイルを開く
    audioFile = SD.open(filename, FILE_READ);
    if (!audioFile) {
        sdError = true;
        DEBUG_P(20250424,"Error opening file: ");
        DEBUG_PLN(20250424,filename);
        enqueueTask(createLogSdTask(filename));  // SD にエラーログを保存
        enqueueTask(createPlayMultiToneTask(500, 200, 2)); // エラー音を鳴らす
        return;
    }

    // ファイルサイズからヘッダを引いたバイト数が実際の音声データ量
    uint32_t fileSize = audioFile.size();
    totalAudioSize = fileSize - WAV_HEADER_SIZE;

    DEBUG_P(20250424,"File: ");
    DEBUG_P(20250424,filename);
    DEBUG_P(20250424,", Size: ");
    DEBUG_P(20250424,fileSize);
    DEBUG_P(20250424," bytes, Audio data: ");
    DEBUG_P(20250424,totalAudioSize);
    DEBUG_P(20250424," bytes (");
    DEBUG_P(20250424,(float)totalAudioSize / sampleRate);
    DEBUG_PLN(20250424," seconds)");

    // WAV ヘッダ（45バイト）をスキップして音声データの先頭へ移動
    audioFile.seek(WAV_HEADER_SIZE);

    // 最初のチャンクを loadBuffer に読み込む
    loadNextChunk();  // → buffer1 にデータが入り bufferReady[1]=true になる

    // データが用意できたらバッファをスワップして再生開始
    if (bufferReady[loadBuffer]) {
        // buffer1（データ済み）をアクティブにする
        int temp = activeBuffer;
        activeBuffer = loadBuffer;
        loadBuffer = temp;

        bufferPos = 0;
        wav_playing = true;
        current_wav_filename = filename;   // 現在再生中のファイル名を記録
        wav_override_volume = min_volume;  // 最低保証ボリューム（0=制限なし）をセット
        if(sound_volume == 0 && min_volume == 0){ return; }  // volume=0 かつ override なし ならアンプ ON しない（ポップノイズ防止）
        setAmplifierState(true);   // アンプを ON にする
        wav_playing_priority = priority;
        DEBUG_P(20250424,"Playback started");
    } else {
        DEBUG_P(20250424,"Failed to load initial audio data");
    }
}

// 再生を停止し、アンプをシャットダウンする。
// バリオメーターが使用中の場合はアンプを切らない（バリオ音がすぐ再開するため）。
void stopPlayback() {
    wav_playing = false;
    wav_override_volume = 0;  // 最低保証ボリュームをリセット
    delay(5);  // 割り込みが確実に止まるまで待つ
    pwm_set_gpio_level(PIN_PWMTONE, 512); // 10bit 中点 (=無音) に戻す。これを忘れるとポップノイズが出る。
    delay(5);  // 信号が安定してからアンプを切る
    if (!vario_mode) {
        setAmplifierState(false);  // バリオ使用中はアンプ維持
    }
    DEBUG_P(20250424,"Playback stopped");
}




// Core1 のメインループから毎ループ呼ばれる WAV 再生管理関数。
//
// 役割1: バッファスワップ処理
//   タイマー割り込みが bufferSwapRequest を立てたら、
//   次のバッファが準備済みかチェックしてスワップを実行する。
//   ファイル末尾でバッファが尽きたら stopPlayback() を呼ぶ。
//   バッファが間に合わなかった場合は「バッファアンダーラン」警告を出す。
//
// 役割2: 先読みロード
//   再生中に loadBuffer が空ならば loadNextChunk() を呼んで次のデータを先読みする。
//   これにより、再生と読み込みが並行して進む。
void __not_in_flash_func(loop_sound)(){
    unsigned long currentTime = millis();

    // バッファスワップ依頼を処理（割り込みの外側で実行するのでメモリ競合が起きない）
    if (bufferSwapRequest && wav_playing) {
        bufferSwapRequest = false;
        bufferReady[activeBuffer] = false;  // 使い終わったバッファを空き状態にする

        if (bufferReady[loadBuffer]) {
            // 次のバッファが準備済み → スワップして継続再生
            int temp = activeBuffer;
            activeBuffer = loadBuffer;
            loadBuffer = temp;
            DEBUG_P(20240424,"Swapped to buffer ");
            DEBUG_P(20240424,activeBuffer);
            DEBUG_P(20240424,", chunks loaded so far: ");
            DEBUG_PLN(20240424,chunksLoaded);
        } else if (endOfFile) {
            // ファイル末尾でバッファも尽きた → 再生終了
            DEBUG_PLN(20240424,"End of file reached");
            current_wav_filename = nullptr;
            stopPlayback();
            // pending WAV があれば最高優先のものを再生する
            // ただし toneBuffer にトーンが残っている場合は loop_tone() に委ねる。
            // （WAV終了直後に pending を即起動すると、ガードで待機中のトーンと同時再生になるため）
            if (toneBufHead == toneBufTail) {
                int prio = 0;
                const char* pf = pop_best_pending_wav(prio);
                if (pf != nullptr) {
                    DEBUGW_P(20250503,"Replaying pending wav:");
                    DEBUGW_PLN(20250503, pf);
                    startPlayWav(pf, prio);
                }
            }
        } else {
            // バッファアンダーラン: SD 読み込みが再生に追いつかなかった（警告のみ）
            DEBUGW_PLN(20250508,"WARNING: Buffer underrun detected!");
        }
    }

    // 再生中に loadBuffer が空ければ次のチャンクを先読みする
    if (wav_playing && !bufferReady[loadBuffer] && !endOfFile) {
        if (loadNextChunk()) {
            DEBUG_P(20240424,"Loaded chunk #");
            DEBUG_P(20240424,chunksLoaded);
            DEBUG_P(20240424," into buffer ");
            DEBUG_PLN(20240424,loadBuffer);
        }
    }

    // デバッグ用: 1 秒ごとに再生進捗（%）を出力
    static unsigned long lastStatusTime = 0;
    if (wav_playing && (currentTime - lastStatusTime >= 1000)) {
        lastStatusTime = currentTime;
        float percentComplete = (float)audioDataRead / totalAudioSize * 100.0;
        DEBUG_P(20250424,"Playing: ");
        DEBUG_P(20250424,percentComplete);
        DEBUG_P(20250424,"% complete (");
        DEBUG_P(20250424,chunksLoaded);
        DEBUG_PLN(20250424," chunks loaded)");
    }
}


// 最後に音が鳴った時のtrue track、つまり "track" 音声再生用のために、前回の GPS 真方位を記録しておく変数。
float last_tone_tt = 0;  // 前回の update_tone() 呼び出し時の GPS 真方位 [deg]

// ============================================================
// 音符周波数テーブル（C3 ～ C8）
// update_tone() が生成した周波数を nearest_note_frequency() で
// 最も近い音符にスナップするために使用する。
// ============================================================
float note_frequencies[] = {
    130.81,  // C3 (Do)
    146.83,  // D3 (Re)
    164.81,  // E3 (Mi)
    174.61,  // F3 (Fa)
    196.00,  // G3 (So)
    220.00,  // A3 (La)
    246.94,  // B3 (Si)
    261.63,  // C4 (Do)
    293.66,  // D4 (Re)
    329.63,  // E4 (Mi)
    349.23,  // F4 (Fa)
    392.00,  // G4 (So)
    440.00,  // A4 (La)
    493.88,  // B4 (Si)
    523.25,  // C5 (Do)
    587.33,  // D5 (Re)
    659.25,  // E5 (Mi)
    698.46,  // F5 (Fa)
    783.99,  // G5 (So)
    880.00,  // A5 (La)
    987.77,  // B5 (Si)
    1046.50, // C6 (Do)
    1174.66, // D6 (Re)
    1318.51, // E6 (Mi)
    1396.91, // F6 (Fa)
    1567.98, // G6 (So)
    1760.00, // A6 (La)
    1975.53, // B6 (Si)
    2093.00, // C7 (Do)
    2349.32, // D7 (Re)
    2637.02, // E7 (Mi)
    2793.83, // F7 (Fa)
    3135.96, // G7 (So)
    3520.00, // A7 (La)
    3951.07, // B7 (Si)
    4186.01  // C8 (Do)
};

// 入力周波数に最も近い音符周波数を二分探索で返す。
// 「旋回音」として連続的な周波数を出すのではなく、
// 音楽的な音程にスナップさせることで聞き取りやすくする。
float nearest_note_frequency(float input_freq) {
    int left = 0;
    int right = sizeof(note_frequencies) / sizeof(note_frequencies[0]) - 1;

    // Handle cases outside the known note frequencies
    if (input_freq <= note_frequencies[left]) {
        return note_frequencies[left];
    }
    if (input_freq >= note_frequencies[right]) {
        return note_frequencies[right];
    }

    // Binary search for the closest frequency
    while (left <= right) {
        int mid = (left + right) / 2;

        if (note_frequencies[mid] == input_freq) {
            return note_frequencies[mid];
        } else if (note_frequencies[mid] < input_freq) {
            left = mid + 1;
        } else {
            right = mid - 1;
        }
    }

    // After binary search, 'left' will be the smallest number greater than 'input_freq'
    // and 'right' will be the largest number smaller than 'input_freq'.
    // Find the nearest one.
    if (fabs(input_freq - note_frequencies[left]) < fabs(input_freq - note_frequencies[right])) {
        return note_frequencies[left];
    } else {
        return note_frequencies[right];
    }
}

// Core1 の setup で 1 度だけ呼ばれる音声システムの初期化。
//
// 処理内容:
//   1. アンプ制御ピン (PIN_AMP_SD) を出力に設定し、シャットダウン状態で開始
//   2. PWM を音声出力用に設定する
//      - clkdiv=2: 125MHz / 2 / 1024 ≒ 61kHz の PWM 周波数（可聴域の十分上）
//      - wrap=1023: 10bit 分解能（0〜1023）中心=512
//   3. 16kHz のリピートタイマーを設定して timerCallback を登録する
//      - add_repeating_timer_us(-62, ...) ≒ 62.5μs ごとに割り込み
//      - 負値は「前回の起動から計測」を意味し、ジッターを減らす
//   4. Sin 波テーブルを事前計算する
//      - 値範囲: 128 ± (127 * SIN_VOLUME)
//      - SIN_VOLUME で最大振幅を調整（1.0 で最大、クリッピング防止のため通常は小さめ）
// CORE1
void setup_sound(){
  // アンプ制御ピンを初期化（まずシャットダウン状態にする）
  pinMode(PIN_AMP_SD, OUTPUT);
  setAmplifierState(false);

  // PWM ピンを音声出力用に初期化
  pinMode(PIN_PWMTONE, OUTPUT);
  uint slice_num = pwm_gpio_to_slice_num(PIN_PWMTONE);
  pwm_config config = pwm_get_default_config();

  pwm_config_set_clkdiv(&config, 2);   // クロック分周: 125MHz / 2 / 1024 ≒ 61kHz の PWM 周波数
  pwm_config_set_wrap(&config, 1023);  // 10bit 分解能（0〜1023 のデューティ比）

  pwm_init(slice_num, &config, true);
  gpio_set_function(PIN_PWMTONE, GPIO_FUNC_PWM);

  // 初期値を 10bit 中点（512）に設定して無音状態にする
  pwm_set_gpio_level(PIN_PWMTONE, 512);

  // 16kHz の繰り返しタイマーを登録する（-62.5μs → 1/16000 秒ごとに呼ばれる）
  static struct repeating_timer timer;
  add_repeating_timer_us(-1000000 / sampleRate, timerCallback, NULL, &timer);

  // Sin 波テーブルを事前計算して格納する
  // sineTable[i] = 128 + 127 * SIN_VOLUME * sin(2π * i / 256)
  for (int i = 0; i < tableSize; i++) {
      sineTable[i] = 128 + 127 * SIN_VOLUME * sin(2 * M_PI * i / tableSize);
  }
}

unsigned long trackwarning_until;  // コース警告音を鳴らし続ける期限 [ms]

// Core0 のメインループから約 1 秒ごとに呼ばれる音声トーン更新関数。
//
// 処理ロジック（2 段階）:
//   【警告1: コース逸脱警告】
//     前回呼び出し時の GPS 真方位 (last_tone_tt) と現在の真方位の差 (angle_diff) が
//     15° を超えたとき → コース逸脱と判断して警告音 + "track.wav" を再生する。
//     速度が 2.0m/s 以上のときのみ発動（静止中の GPS ブレを無視するため）。
//
//   【警告2: 旋回角速度トーン】
//     上記に当てはまらない場合で、degpersecond が 2.0 以上かつ速度 2.0m/s 以上のとき、
//     旋回の強さに応じた音程のトーンを鳴らす。
//     周波数の計算: freq = |degpersecond| * 600 - 680
//       - 2 deg/s → 520Hz,  5 deg/s → 2320Hz,  上限は 3136Hz（G7, スピーカー限界）
//     速度や旋回が小さいときは短いトーン（80ms）、大きいときは長めのトーン（160ms）。
// CORE0
unsigned long last_update_tone = 0;
void update_tone(float degpersecond){
    // 前回の呼び出しから 900ms 以上経過していなければスキップ（過剰な再生防止）
    if(millis() - last_update_tone < 900){
        DEBUG_PLN(20250508,"update_tone: rate limited, skip");
        return;
    }
    last_update_tone = millis();

  // 前回の方位との差を計算し、-180〜+180 の範囲に収める
  int relativedif = get_gps_truetrack()-last_tone_tt;
  if(relativedif > 180)
    relativedif -= 360;
  if(relativedif < -180)
    relativedif += 360;
  int angle_diff = abs(relativedif);

  //【警告1】方位変化が 15° 超 → コース逸脱警告
  if(angle_diff > 15){
    last_tone_tt = get_gps_truetrack();
    if(get_gps_mps() > 2.0){
        trackwarning_until = millis()+8000;
        // 変化量に応じて警告音の回数を変える（30°超なら 5 回、15°超なら 3 回）
        enqueueTask(createPlayMultiToneTask(3136,120,angle_diff>30?5:3)); // G7（スピーカー上限付近）
        if(good_sd()){
            enqueueTask(createPlayWavTask("wav/track.wav")); // 音声警告ファイルも再生
        }
    }
  }
  //【警告2】旋回角速度トーン: 2.0 deg/s 以上 かつ 2.0 m/s 以上のとき発動
  // angle_diff > 15 のときは警告1 が優先されるためここには来ない
  // バリオ音（700〜1300Hz）と混同しないよう、固定高音 2093Hz（C7）のピピ2回に統一
  else if(abs(degpersecond) > 2.0 && get_gps_mps() > 2.0){
    enqueueTask(createPlayMultiToneTask(2093, 80, 2));
  }
}


// Sin 波トーンの周波数を設定する（位相アキュムレータ方式）。
// 位相増分 (phaseInc) を周波数から計算し、割り込みが自動的にその周波数で Sin 波を出力する。
//
// 計算式: phaseInc = (frequency * tableSize * 2^24) / sampleRate
//   - 32bit の phaseAcc を毎割り込み phaseInc 分進める
//   - 上位 8bit (= phaseAcc >> 24) が 0〜255 のテーブルインデックスになる
//   - → 1 秒間に frequency 回テーブルをループすることで周波数が決まる
void playNextNote(int frequency) {
    phaseAcc = 0;  // 位相をリセット（音が途切れないよう最初は 0 から）
    
    phaseInc = (frequency * tableSize * (1ULL << 24)) / 16150; // 1ULL で 64bit 演算してオーバーフロー防止
    //16150 は sampleRate より少し高めにして、実際の周波数がやや高く出るように補正（耳で聞いた感じの補正、スピーカーの特性を考慮）
    DEBUG_P(20250412,"FRQ: ");
    DEBUG_PLN(20250412,frequency);
}


// トーン再生要求をバッファに積む。実際の再生は loop_tone() が担当する。
// WAV 優先制御・タイミング管理は loop_tone() / startPlayWav() が行う。
void playTone(int freq, int duration, int counter, int priority, int min_volume, bool solo_play) {
    pushToneBuffer(freq, duration, counter, priority, min_volume, solo_play);
}


// toneBuffer から次のエントリを取り出してトーン再生を開始する。
// loop_tone() から呼ばれる（WAV 未再生時のみ）。
static void startNextToneEntry() {
    ToneEntry& e    = toneBuffer[toneBufHead];
    toneBufHead     = (toneBufHead + 1) % TONE_BUFFER_SIZE;

    tone_remaining        = e.count;
    tone_cur_freq         = e.freq;
    tone_cur_dur          = e.duration;
    tone_playing_priority = e.priority;
    wav_override_volume   = e.min_volume;
    tone_in_gap           = false;
    sin_solo              = e.solo_play;  // solo_play フラグを ISR 参照変数に反映

    sinmode     = true;
    sin_playing = true;
    setAmplifierState(true);
    playNextNote(tone_cur_freq);
    tone_stop_time = millis() + tone_cur_dur;
}

// ============================================================
// Core1 のメインループから毎ループ呼ばれるトーン再生管理関数。
//
// toneBuffer に積まれたトーンを非ブロッキングで順番に再生する。
// delay() を使わず millis() でタイミングを管理する。
//
// WAV との優先制御:
//   WAV 再生中 (wav_playing=true): 出力を停止し、WAV 終了まで待機
//   WAV 終了後: 一時停止していたトーンを再開（tone_paused=true の場合）
//   WAV 開始時の優先度制御は startPlayWav() が行う
//     - WAV priority > tone priority: WAV 割込み → トーン一時停止・WAV 後に再開
//     - WAV priority ≤ tone priority: WAV を pending に退避 → トーン完了後に再生
// ============================================================
void loop_tone() {
    // solo_play トーン再生中のみ WAV を待機させる
    // （sin_solo=false の通常トーンは WAV とミックス再生するためここでは止めない）
    if (wav_playing && sin_solo) {
        if (sin_playing) {
            sin_playing = false;
            tone_paused = true;
            pwm_set_gpio_level(PIN_PWMTONE, 512);  // 無音（中点）
            if (!vario_mode) setAmplifierState(false);
        }
        return;
    }

    // WAV 割込みからの再開（solo_play トーンが WAV に中断された後の復帰）
    if (tone_paused) {
        tone_paused = false;
        sinmode     = true;
        sin_playing = true;
        setAmplifierState(true);
        playNextNote(tone_cur_freq);
        tone_stop_time = millis() + tone_cur_dur;
        return;
    }

    // ギャップ中: 時間が経過したら次のトーンを開始
    if (tone_in_gap) {
        if (millis() >= tone_stop_time) {
            tone_in_gap = false;
            sin_playing = true;
            setAmplifierState(true);
            playNextNote(tone_cur_freq);
            tone_stop_time = millis() + tone_cur_dur;
        }
        return;
    }

    // トーン再生中: 時間が経過したら終了処理
    if (sin_playing) {
        if (millis() >= tone_stop_time) {
            sin_playing = false;
            pwm_set_gpio_level(PIN_PWMTONE, 512);  // 無音（中点）
            // WAV 再生中はアンプを切らない（WAV がアンプを使用中）
            if (!vario_mode && !wav_playing) setAmplifierState(false);

            tone_remaining--;
            if (tone_remaining > 0) {
                // カウント残り → ギャップ（無音）を挟んで次のトーンへ
                tone_in_gap    = true;
                tone_stop_time = millis() + tone_cur_dur;
            } else {
                // このエントリ完了 → クリーンアップ
                sinmode               = false;
                sin_solo              = false;  // solo_play フラグをリセット
                tone_playing_priority = 0;
                wav_override_volume   = 0;
                if (toneBufHead != toneBufTail) {
                    startNextToneEntry();  // 次のエントリを開始
                } else if (!wav_playing) {
                    // 全トーン完了かつ WAV 未再生 → pending WAV があれば再生
                    int prio = 0;
                    const char* pf = pop_best_pending_wav(prio);
                    if (pf != nullptr) startPlayWav(pf, prio);
                }
            }
        }
        return;
    }

    // 未再生: バッファに積まれていれば開始
    if (toneBufHead != toneBufTail) {
        // 高優先度WAV再生中はトーン開始を待機（fixed.wavなどが埋もれないよう）
        if (wav_playing && wav_playing_priority > toneBuffer[toneBufHead].priority) {
            // 待機 — 次ループで再チェック
        } else {
            startNextToneEntry();
        }
    }
}


// ============================================================
// バリオメーター音声更新関数（Core0 のメインループから毎ループ呼ぶ）
//
// 内部で 100ms レート制限しているので毎ループ呼んでよい。
// get_airdata_vspeed() で 1Hz 更新される鉛直速度を読み、
// デッドバンド閾値を超えたときにバリオ音パラメータを更新する。
//   KF融合中（BNO085+MS5611）: ±0.3 m/s
//   MS5611単独            : ±0.6 m/s
//
// 上昇（vspeed > dead_band）: 断続ビープ
//   ピッチ = 700 + vspeed*300 Hz（1.0→1000Hz, 3.0→1600Hz, 上限 2200Hz）
//   ON 時間 120ms 固定 / OFF 時間 = 700/vspeed - 120 ms（最小 50ms）
//
// 下降（vspeed < -dead_band）: 連続低音
//   ピッチ = 480 - |vspeed|*30 Hz（2.0→420Hz, 下限 220Hz）
//
// アンプ管理:
//   バリオ開始エッジで amp を ON（WAV/Sinトーン未使用時）。
//   バリオ停止エッジで amp を OFF（WAV/Sinトーン未使用時）。
//   WAV/Sinトーンが使用中の場合は amp を操作しない（相手任せ）。
// CORE0
// ============================================================
void update_vario() {
    if (vario_inhibit) return;  // inhibit 中はバリオ音を出さない
    static unsigned long last_call = 0;
    if (millis() - last_call < 100) return;
    last_call = millis();

    // BNO085 が使えるときは Kalman 融合済みの上昇率を使う（応答が速く正確）。
    // BNO085 非接続時は MS5611 単独の上昇率にフォールバックする。
    float vspeed = get_imu_ok() ? get_imu_vspeed() : get_airdata_vspeed();
    // デッドバンド閾値: Kalman 融合中（BNO085+MS5611 両方接続）は精度が高いため狭くする。
    float dead_band = (get_imu_ok() && get_airdata_ok()) ? 0.3f : 0.6f;
    bool should_vario = (vspeed > dead_band || vspeed < -dead_band) && (vario_volume > 0);

    static bool prev_vario = false;

    if (should_vario) {
        // パラメータを vspeed に応じて毎回更新（10Hz 解像度で音が変わる）
        if (vspeed > dead_band) {
            // 上昇: 断続ビープ
            float v = vspeed < 2.0f ? vspeed : 2.0f;  // 2.0 m/s で振り切り
            int freq = (int)(700 + v * 300);        // 820〜2200 Hz（2.0 m/s → 1300Hz）
            vario_phase_inc = (uint32_t)((freq * (long)tableSize * (1ULL << 24)) / 16000);
            int off_ms = (int)(700.0f / v) - 120;
            if (off_ms < 50) off_ms = 50;
            vario_on_samples     = 120 * 16;         // 120ms × 16kHz = 1920 サンプル
            vario_cycle_samples  = (uint32_t)((120 + off_ms) * 16);
            vario_ascending      = true;             // 上昇フラグ ON（音量補正用）
        } else {
            // 下降: 連続低音
            float v = (-vspeed) < 2.0f ? (-vspeed) : 2.0f;  // 2.0 m/s で振り切り
            int freq = (int)(480 - v * 30);
            if (freq < 300) freq = 300;              // 下限 300Hz
            vario_phase_inc     = (uint32_t)((freq * (long)tableSize * (1ULL << 24)) / 16000);
            vario_cycle_samples = 0;                 // 0 = 連続出力
            vario_ascending     = false;             // 下降フラグ（音量補正なし）
        }

        if (!prev_vario) {
            // バリオ開始エッジ: 位相/サイクルをリセットしてアンプ ON
            s_vario_phase_acc = 0;
            s_vario_cycle_pos = 0;
            vario_mode = true;
            if (!wav_playing && !sin_playing) {
                setAmplifierState(true);
            }
        } else {
            vario_mode = true;  // パラメータ更新後に確実に true を維持
        }
    } else {
        if (prev_vario) {
            // バリオ停止エッジ: アンプ OFF（WAV/Sinトーン未使用時のみ）
            vario_mode = false;
            if (!wav_playing && !sin_playing) {
                pwm_set_gpio_level(PIN_PWMTONE, 512);
                delay(3);
                setAmplifierState(false);
            }
        }
    }

    prev_vario = should_vario;
}