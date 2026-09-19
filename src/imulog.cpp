// ============================================================
// File    : imulog.cpp
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : 生 IMU / GNSS 速度データの高レート バイナリロガー（実装）。
//           形式・目的・レコード定義は imulog.h のコメントを参照。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/09/17
// ============================================================

#include "../settings.h"
#include "imulog.h"
#include "../mysd.h"
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

static const bool        imulog_enabled = IMULOG_DEFAULT_ENABLED;  // ビルド時固定（実行時の切替手段は無い）
static volatile bool     imulog_paused  = false;  // リプレイ中などの一時停止
static uint16_t          imulog_seq     = 0;

static volatile uint32_t imulog_dropped = 0;
static volatile uint32_t imulog_written = 0;

// ---- SD 側の状態（Core1 のみが触る）----
static FsFile imuFileStatic;
static char   imuOpenedFilename[24] = "";
static int    imuFlushCount = 0;

// ---- 起動識別（Core1 のみが触る）----
// boot_id は起動ごとにランダムに振り直すだけの RAM 変数。フラッシュには保存しない。
// 必要なのは「隣り合う起動を区別できること」だけで、通算起動回数には意味が無い。
// 24bit に丸めるのは、レコードの float v[0] に誤差なく載せるため
// （float32 は 2^24 までの整数を正確に表す）。
static uint32_t imuBootId    = 0;
static uint16_t imuOpenCount = 0;   // この起動で何番目に開いたファイルか
static char     imuNofixName[24] = "";   // 解決済みの nofixNNN.bin（起動ごとに 1 回決める）



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
// nofixNNN.bin の名前解決（Core1 専用）
// ============================================================
// "nofix012.bin" のような名前から 12 を取り出す。合わなければ 0。
// 旧形式の "nofix.bin"（数字なし）も 0 を返すので、最大値の計算に影響しない。
// tolower を使わず自前で比較しているのは、SdFat が返す名前が短縮名（大文字）の
// 場合があるため。strcasecmp は newlib の <strings.h> 側にあり、
// Arduino 環境で確実に引けるとは限らないので依存しない。
static bool imulog_ieq(const char* a, const char* b) {
    for (; *a && *b; a++, b++) {
        char ca = (*a >= 'A' && *a <= 'Z') ? (char)(*a + 32) : *a;
        char cb = (*b >= 'A' && *b <= 'Z') ? (char)(*b + 32) : *b;
        if (ca != cb) return false;
    }
    return *a == '\0' && *b == '\0';
}

static int imulog_nofix_index(const char* nm) {
    static const char kPrefix[] = "nofix";
    for (int i = 0; i < 5; i++) {
        char c = (nm[i] >= 'A' && nm[i] <= 'Z') ? (char)(nm[i] + 32) : nm[i];
        if (c != kPrefix[i]) return 0;
    }
    const char* p = nm + 5;
    int n = 0, digits = 0;
    while (*p >= '0' && *p <= '9' && digits < 4) { n = n * 10 + (*p - '0'); p++; digits++; }
    if (digits == 0 || digits > 3) return 0;
    if (!imulog_ieq(p, ".bin")) return 0;
    return n;
}

// 既存の最大番号 +1 を採る。ディレクトリ走査は **1 回だけ**。
// SD.exists() を番号ごとに呼ぶ実装にしてはいけない。あれは毎回ディレクトリを
// 先頭から読み直すので、ファイルが増えるほど Core1 が長く止まり、
// その間に Core0 側のバッファが溢れて「取りこぼし」を自分で作り込む。
static const char* imulog_resolve_nofix() {
    if (imuNofixName[0]) return imuNofixName;   // この起動では解決済み

    // imuraw/ がまだ無い場合は dir.open() が失敗して maxn=0 のまま抜ける。
    // その状況では既存ファイルも無いので nofix001.bin で正しい
    // （ディレクトリ作成は呼び出し側の open 直前でやる）。
    int maxn = 0;
    FsFile dir;
    if (dir.open(IMULOG_DIR, O_RDONLY)) {
        dir.rewind();
        FsFile f;
        char nm[32];
        while (f.openNext(&dir, O_RDONLY)) {
            if (!f.isDir() && f.getName(nm, sizeof(nm))) {
                int n = imulog_nofix_index(nm);
                if (n > maxn) maxn = n;
            }
            f.close();
        }
        dir.close();
    }
    if (maxn >= 999) maxn = 998;   // 4 桁になって名前が化けるのを防ぐ（上書きはしない）

    snprintf(imuNofixName, sizeof(imuNofixName), IMULOG_DIR "/nofix%03d.bin", maxn + 1);
    return imuNofixName;
}


// ============================================================
// BOOT レコードの書き出し（Core1 専用）
// ============================================================
// ファイルを開くたびに先頭へ 1 件置く。同じ起動で nofix → 日付と切り替わったら
// 両方に同じ boot_id が入り、PC 側で 2 ファイルを突き合わせられる。
static void imulog_write_boot_marker() {
    if (imuBootId == 0) {
        // 起動ごとに 1 回。RP2350 ではハードウェア TRNG を引く
        // （ROSC 経路の hard_assert はこのボードではリンクされない）。
        // 0 は「未初期化」と区別したいので避けるが、**無限ループにはしない**。
        // 乱数源が壊れて 0 を返し続けた場合に Core1 が止まると、
        // SD と音声ごと巻き添えになる。数回で諦めて時刻で埋める。
        for (int i = 0; i < 4 && imuBootId == 0; i++) {
            imuBootId = rp2040.hwrand32() & 0xFFFFFF;
        }
        if (imuBootId == 0) {
            imuBootId = (time_us_32() & 0xFFFFFF) | 1;   // 最後の砦。0 だけは避ける
        }
    }

    ImuLogRec b;
    memset(&b, 0, sizeof(b));
    b.t_us = time_us_32();
    b.s_us = (uint32_t)BUILDDATE;    // float では表せないので uint32 の枠に入れる
    b.id   = IMULOG_ID_BOOT;
    b.acc  = IMULOG_ACC_NONE;
    b.seq  = IMULOG_SEQ_NONE;        // 連番に参加しない
    b.v[0] = (float)imuBootId;
    b.v[1] = (float)imuOpenCount;
    // imulog_written には数えない（written + dropped = 消費した seq を保つため）
    imuFileStatic.write(&b, sizeof(b));
    imuOpenCount++;
}


// ============================================================
// imulog_write_buffer(): バッファを SD へ書き出す（Core1 専用）
// ============================================================
// save_imu_replaydata() と同じ方針: ファイルは開きっぱなしにし、
// ファイル名が変わったときだけ開き直す。sync() は数バッファに 1 回。
void imulog_write_buffer(int bufidx, const char* filename,
                         int year, int month, int day, int hour, int minute, int second) {
    ASSERT_SD_CORE1("imulog_write_buffer");
    if (bufidx < 0 || bufidx > 1) return;

    uint16_t n = imulog_fill[bufidx];

    if (good_sd() && n > 0) {
        // Core0 からの合言葉を、この起動ぶんの nofixNNN.bin へ差し替える。
        // 比較にも open にも同じ文字列を使うので、以降の処理は素通りでよい。
        // 解決は起動ごとに 1 回だけなので、ここはほぼ strcmp 1 回で抜ける。
        if (strcmp(filename, IMULOG_NOFIX_REQUEST) == 0) {
            filename = imulog_resolve_nofix();
        }

        // ファイル名が変わった場合（測位できた・日付変更等）は sync してから閉じて開き直す
        if (imuFileStatic.isOpen() && strcmp(imuOpenedFilename, filename) != 0) {
            imuFileStatic.sync();
            imuFileStatic.close();
            imuOpenedFilename[0] = '\0';
            imuFlushCount = 0;
        }

        if (!imuFileStatic.isOpen()) {
            // ★ SD.exists() をこのブロックの外へ出さないこと。
            //   外へ出すと毎バッファ（約 2.2 回/秒）ディレクトリ検索が走る。
            //   ここなら開き直すときだけで済む。
            if (!SD.exists(IMULOG_DIR)) {
                SD.mkdir(IMULOG_DIR);
            }
            if (imuFileStatic.open(filename, O_RDWR | O_CREAT | O_APPEND)) {
                strncpy(imuOpenedFilename, filename, sizeof(imuOpenedFilename) - 1);
                imuOpenedFilename[sizeof(imuOpenedFilename) - 1] = '\0';
                imuFileStatic.timestamp(T_CREATE | T_ACCESS | T_WRITE,
                                        year, month, day, hour, minute, second);
                // 開いた直後に BOOT レコード。データより前に置く必要があるのでここ。
                imulog_write_boot_marker();
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
