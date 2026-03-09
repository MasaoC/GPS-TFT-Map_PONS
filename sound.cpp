// ============================================================
// File    : sound.cpp
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : 音声出力の実装。
//           PWMによるSin波トーン生成、SDカードからのWAV再生、
//           旋回角速度(degpersecond)に応じた音程変化、
//           アンプシャットダウン制御（省電力）。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/02/26
// ============================================================
// Handle speaker, amplifier, PWM-audio signals.
#include "settings.h"
#include "sound.h"
#include "gps.h"
#include "mysd.h"
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
volatile int wav_playing_priority = 0;  // 現在再生中 WAV の優先度（高い方が優先）
volatile int tone_playing_priority = 0; // 現在再生中トーンの優先度

volatile bool endOfFile = false;            // ファイル末尾に達したフラグ
volatile bool bufferReady[2] = {false, false};  // 各バッファにデータが入っているか
volatile bool bufferSwapRequest = false;    // 割り込み→メインへのバッファスワップ依頼フラグ


// File object and size tracking
FsFile audioFile;
uint32_t totalAudioSize = 0;    // WAV ファイルのオーディオデータ総バイト数（ヘッダを除く）
uint32_t audioDataRead = 0;     // これまでに読み込んだバイト数
uint32_t chunksLoaded = 0;      // ロード済みチャンク数（デバッグ用カウンタ）

const unsigned long playInterval = 10000;  // 将来的な連続再生用インターバル（現状未使用）
extern volatile int sound_volume;          // 音量（0〜100）。button.cpp の設定画面から変更。

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

// wavmode / sinmode: タイマー割り込み内でどちらのモードで出力するかを切り替えるフラグ。
// 同時に true にはしない（WAV 再生中はトーンをオフ、トーン再生中は WAV をオフ）。
bool wavmode = true;   // true のとき割り込みが WAV バッファを再生する
bool sinmode = true;   // true のとき割り込みが Sin 波を出力する



// ============================================================
// タイマー割り込みコールバック（16kHz = 62.5μs ごとに呼ばれる）
//
// wavmode が true のとき:
//   activeBuffer の bufferPos 位置から 1 サンプルを読み出し、
//   PWM デューティ比として出力する。
//   bufferPos が CHUNK_SIZE に達したら bufferSwapRequest を立て、
//   メインループにバッファスワップを依頼する。
//
// sinmode が true のとき:
//   位相アキュムレータ (phaseAcc) を phaseInc 分進め、
//   sineTable[(phaseAcc >> 24)] の値を PWM 出力する。
//   → 上位 8bit がテーブルインデックス = 周波数を決める。
//
// ※ 両モードを同時に true にすると両方が PWM に書き込むため、
//    呼び出し元で必ずどちらか一方だけを有効にすること。
// ============================================================
bool timerCallback(struct repeating_timer *t) {
  if(wavmode){
    if (!wav_playing || !bufferReady[activeBuffer]) {
        return true;  // バッファが準備できていなければスキップ
    }
    // activeBuffer の現在位置から 1 サンプルを PWM に出力（音量スケール適用）
    pwm_set_gpio_level(PIN_PWMTONE, audioBuffer[activeBuffer][bufferPos]*sound_volume/100.0);
    bufferPos++;

    // バッファ末尾まで再生したら、メインループへスワップを依頼
    if (bufferPos >= CHUNK_SIZE) {
        bufferPos = 0;          // 位置をリセット（スワップ後すぐに使えるように）
        bufferSwapRequest = true;  // メインループで処理するフラグを立てる
        DEBUG_P(20250424,"bufferSwapRequest");
    }
  }
  if(sinmode){
    // 位相アキュムレータを進め、上位 8bit でテーブルを引いて Sin 波を出力
    phaseAcc += phaseInc;
    pwm_set_gpio_level(PIN_PWMTONE, sineTable[(phaseAcc >> 24) % tableSize]*sound_volume/100.0);
  }
  //keep running
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
bool loadNextChunk() {
    if (audioFile && !bufferReady[loadBuffer] && audioDataRead < totalAudioSize) {
        uint32_t remaining = totalAudioSize - audioDataRead;
        uint32_t toRead = (remaining < CHUNK_SIZE) ? remaining : CHUNK_SIZE;

        uint32_t bytesRead = audioFile.read(audioBuffer[loadBuffer], toRead);

        if (bytesRead > 0) {
            audioDataRead += bytesRead;
            chunksLoaded++;
            if (bytesRead < CHUNK_SIZE) {
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
void startPlayWav(const char* filename, int priority) {
    // 優先度チェック：再生中ファイルの優先度の方が高ければキャンセル
    if(wav_playing && wav_playing_priority > priority){
        DEBUGW_P(20250503,"Start play wav canceled due to priority.:");
        DEBUGW_PLN(20250427,filename);
        return;
    }
    DEBUG_P(20250503,"wav start:");
    DEBUG_PLN(20250503,priority);

    // WAV モードに切り替えてから、割り込みが止まるまで少し待つ
    wavmode = true;
    sinmode = false;
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
        if(sound_volume == 0){ return; }  // volume=0 ならアンプ ON しない（ポップノイズ防止）
        setAmplifierState(true);   // アンプを ON にする
        wav_playing_priority = priority;
        DEBUG_P(20250424,"Playback started");
    } else {
        DEBUG_P(20250424,"Failed to load initial audio data");
    }
}

// 再生を停止し、アンプをシャットダウンする。
void stopPlayback() {
    wav_playing = false;
    delay(5);  // 割り込みが確実に止まるまで待つ
    pwm_set_gpio_level(PIN_PWMTONE, 128);  // 無音レベルに戻す（プツッ防止）
    delay(5);  // 信号が 128 に安定してからアンプを切る
    setAmplifierState(false);
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
void loop_sound(){
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
            stopPlayback();
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
//      - clkdiv=8: 125MHz / 8 / 256 ≒ 61kHz の PWM 周波数（可聴域の十分上）
//      - wrap=255: 8bit 分解能（0〜255）
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

  pwm_config_set_clkdiv(&config, 8);   // クロック分周: 音質に影響するため 32 以下推奨
  pwm_config_set_wrap(&config, 255);   // 8bit 分解能（0〜255 のデューティ比）

  pwm_init(slice_num, &config, true);
  gpio_set_function(PIN_PWMTONE, GPIO_FUNC_PWM);

  // 初期値を中点（128）に設定して無音状態にする
  pwm_set_gpio_level(PIN_PWMTONE, 128);

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
        DEBUG_PLN(20250508,"errrr");
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
  else if(abs(degpersecond) > 2.0 && get_gps_mps() > 2.0){
    int freq = abs(degpersecond)*600-680; // 旋回強度を周波数にマッピング（最低 520Hz 程度）
    if(freq > 3136){
      freq = 3136; // スピーカー性能の上限でクリップ
    }
    // 旋回が穏やかまたは速度が遅い場合はトーンを短くする（誤報感を減らす）
    int newduration = (abs(degpersecond)<3 || get_gps_mps() < 4.0)?80:160;
    enqueueTask(createPlayMultiToneTask(nearest_note_frequency(freq), newduration,1));
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


// 指定した周波数・時間のトーンを 1 回だけ鳴らす低レベル関数。
// アンプを ON にして duration ms 待ってから OFF にする。
void myTone(int freq,int duration){
  digitalWrite(PIN_AMP_SD,HIGH);  // アンプ ON
  playNextNote(freq);              // 周波数を設定（割り込みが出力を開始）
  delay(duration);                 // 指定時間鳴らし続ける
  digitalWrite(PIN_AMP_SD,LOW);   // アンプ OFF
}


// 指定した周波数・時間・回数・優先度で Sin 波トーンを再生する。
// WAV 再生中（かつ優先度が同等以上）の場合はキャンセルする。
//
// 注意: このループ中は WAV 再生を一時停止させている（wavmode=false）。
// ただし WAV 再生を「ここで待ってはいけない」。
// Core1 が WAV 再生を続けるためには loop1() が動き続ける必要があり、
// このブロックが長すぎると WAV が止まる可能性がある。
void playTone(int freq, int duration, int counter,int priority){
    // 優先度チェック: WAV の方が同等以上に優先なら再生しない
    if(wav_playing && wav_playing_priority >= priority){
        DEBUG_P(20250503,"failed playTone due to Wav file playing.");
        DEBUG_PLN(20250503,priority);
        // ここで WAV 終了を待ってはいけない（loop1 が止まるため）
        return;
    }
    tone_playing_priority = priority;

    // WAV モードを無効にして Sin 波モードで再生する
    wavmode = false;
    wav_playing = false;
    endOfFile = true;

  sinmode = true;
  myTone(freq, duration);   // 1 回目
  if(counter <= 1){
    return;
  }
  // 2 回目以降: duration ms 待ってから繰り返す
  for(int i = counter; i > 1; i--){
    delay(duration);
    myTone(freq, duration);
  }
}