// ============================================================
// File    : imulog.cpp
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : 生 IMU / GNSS 速度データの高レート バイナリロガー（実装）。
//           形式・目的・レコード定義は imulog.h のコメントを参照。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/08/17
// ============================================================

#include "settings.h"
#include "imulog.h"
#include "mysd.h"
#include "SdFat.h"

extern SdFs SD;

// ============================================================
// 二重バッファ
// ============================================================
// Core0 が active 側に詰め、満杯になったら pending を立てて反対側へ切り替える。
// Core1 は TASK_FLUSH_IMULOG 経由で pending 側を SD へ書き出す。
//
// 排他について:
//   バッファの受け渡しは「Core0 が imulog_take_pending() で busy を立てる
//   → タスクを enqueue → Core1 が dequeue して書き出す」という一方向の流れ。
//   enqueueTask()/dequeueTask() が taskQueueMutex を取るため、
//   そこがメモリバリアとして機能し、Core1 は busy=true 以降の状態を必ず観測できる。
//   （プロジェクト内の replay リングバッファと同じ考え方）
static uint8_t  imulog_buf[2][IMULOG_BUF_BYTES];
static volatile uint16_t imulog_fill[2]    = {0, 0};   // 各バッファの格納レコード数
static volatile bool     imulog_pending[2] = {false, false};  // 満杯・引き渡し待ち
static volatile bool     imulog_busy[2]    = {false, false};  // Core1 が書き出し中
static volatile uint8_t  imulog_active     = 0;        // Core0 が詰めている側

static volatile bool     imulog_enabled = IMULOG_DEFAULT_ENABLED;  // 設定による有効/無効
static volatile bool     imulog_paused  = false;  // リプレイ中などの一時停止
static uint16_t          imulog_seq     = 0;

static volatile uint32_t imulog_dropped = 0;
static volatile uint32_t imulog_written = 0;

// ---- SD 側の状態（Core1 のみが触る）----
static FsFile imuFileStatic;
static char   imuOpenedFilename[24] = "";
static int    imuFlushCount = 0;


void imulog_set_enabled(bool on) { imulog_enabled = on; }
bool imulog_get_enabled()        { return imulog_enabled; }

// リプレイ中の一時停止。Core0 から呼ぶ。
// 停止に入る瞬間、書き出し待ちのバッファは捨てる。
// これをしないと、リプレイ解除後に take_pending() が古いレコードを拾い、
// 「再生した飛行の日付」ではなく当日のファイルへ過去の時刻のデータが混ざってしまう。
void imulog_set_paused(bool p) {
    if (p && !imulog_paused) {
        for (int i = 0; i < 2; i++) {
            if (!imulog_busy[i]) {   // Core1 が書き出し中のものは触らない
                imulog_fill[i]    = 0;
                imulog_pending[i] = false;
            }
        }
    }
    imulog_paused = p;
}

uint32_t imulog_get_dropped() { return imulog_dropped; }
uint32_t imulog_get_written() { return imulog_written; }


// ============================================================
// imulog_push(): レコードを 1 件積む（Core0 専用）
// ============================================================
// 呼び出し側で有効判定やバッファ残量の確認は不要。
// 詰められない場合は捨てて imulog_dropped を増やすだけで、決してブロックしない。
void imulog_push(uint8_t id, uint32_t s_us, uint8_t acc,
                 float v0, float v1, float v2, float v3) {
    if (!imulog_enabled || imulog_paused) return;

    // 連番は「捨てた場合も消費する」こと。
    // そうしないとファイル上の seq が詰まってしまい、取りこぼしが痕跡を残さない。
    // 連番が飛んでいる = そこで records が捨てられた、と PC 側で判定できるようにする。
    uint16_t myseq = imulog_seq++;

    uint8_t a = imulog_active;

    // ---- 現在のバッファが満杯なら反対側へ切り替える ----
    if (imulog_fill[a] >= IMULOG_RECS_PER_BUF) {
        uint8_t other = (uint8_t)(1 - a);
        // 反対側がまだ書き出し待ち／書き出し中なら空きが無い。
        // SD が詰まっている状況なので、ここで待つと Core0 が止まる。捨てる。
        if (imulog_pending[other] || imulog_busy[other]) {
            imulog_dropped++;
            return;
        }
        imulog_pending[a] = true;   // Core0 の imulog_take_pending() が拾う
        imulog_fill[other] = 0;
        imulog_active = other;
        a = other;
    }

    // ---- レコード書き込み ----
    ImuLogRec rec;
    rec.t_us = time_us_32();
    rec.s_us = s_us;
    rec.id   = id;
    rec.acc  = acc;
    rec.seq  = myseq;
    rec.v[0] = v0;
    rec.v[1] = v1;
    rec.v[2] = v2;
    rec.v[3] = v3;

    memcpy(&imulog_buf[a][imulog_fill[a] * sizeof(ImuLogRec)], &rec, sizeof(ImuLogRec));
    imulog_fill[a]++;
}


// ============================================================
// imulog_take_pending(): 書き出し待ちバッファを Core1 へ引き渡す（Core0 専用）
// ============================================================
// 戻り値が 0/1 なら、その索引で必ず TASK_FLUSH_IMULOG を enqueue すること。
// -1 を返した場合は待ちが無い（何もしない）。
int imulog_take_pending() {
    for (int i = 0; i < 2; i++) {
        if (imulog_pending[i] && !imulog_busy[i]) {
            imulog_busy[i]    = true;   // Core1 へ引き渡し中
            imulog_pending[i] = false;
            return i;
        }
    }
    return -1;
}


// ============================================================
// imulog_release_pending(): 引き渡しに失敗したバッファを戻す（Core0 専用）
// ============================================================
// enqueueTask() はキュー満杯時にタスクを捨てるため、
// imulog_take_pending() で busy にしたバッファが Core1 へ届かないことがある。
// その場合ここで pending へ戻し、次のループで再度引き渡しを試みる。
// これをしないと busy のまま誰も解放せず、2 枚とも失った時点で記録が恒久停止する。
void imulog_release_pending(int bufidx) {
    if (bufidx < 0 || bufidx > 1) return;
    imulog_pending[bufidx] = true;
    imulog_busy[bufidx]    = false;
}


// ============================================================
// imulog_write_buffer(): バッファを SD へ書き出す（Core1 専用）
// ============================================================
// save_imu_replaydata() と同じ方針: ファイルは開きっぱなしにし、
// ファイル名が変わったときだけ開き直す。sync() は数バッファに 1 回。
void imulog_write_buffer(int bufidx, const char* filename,
                         int year, int month, int day, int hour, int minute, int second) {
    if (bufidx < 0 || bufidx > 1) return;

    uint16_t n = imulog_fill[bufidx];

    if (good_sd() && n > 0) {
        // ファイル名が変わった場合（日付変更等）は sync してから閉じて開き直す
        if (imuFileStatic.isOpen() && strcmp(imuOpenedFilename, filename) != 0) {
            imuFileStatic.sync();
            imuFileStatic.close();
            imuOpenedFilename[0] = '\0';
            imuFlushCount = 0;
        }

        if (!imuFileStatic.isOpen()) {
            if (!SD.exists("imuraw")) {
                SD.mkdir("imuraw");
            }
            if (imuFileStatic.open(filename, O_RDWR | O_CREAT | O_APPEND)) {
                strncpy(imuOpenedFilename, filename, sizeof(imuOpenedFilename) - 1);
                imuOpenedFilename[sizeof(imuOpenedFilename) - 1] = '\0';
                imuFileStatic.timestamp(T_CREATE | T_ACCESS | T_WRITE,
                                        year, month, day, hour, minute, second);
            }
        }

        if (imuFileStatic.isOpen()) {
            size_t bytes = (size_t)n * sizeof(ImuLogRec);
            if (imuFileStatic.write(imulog_buf[bufidx], bytes) == bytes) {
                imulog_written += n;
            }
            // 4 バッファごと（約 2 秒）に sync。CSV の 2 秒フラッシュ方針に合わせる。
            if (++imuFlushCount >= 4) {
                imuFileStatic.timestamp(T_WRITE, year, month, day, hour, minute, second);
                imuFileStatic.sync();
                imuFlushCount = 0;
            }
        }
    }

    // SD が使えない場合もバッファは必ず解放する（解放しないと Core0 が詰まる）
    imulog_fill[bufidx] = 0;
    imulog_busy[bufidx] = false;
}
