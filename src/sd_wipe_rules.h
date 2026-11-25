// ============================================================
// File    : src/sd_wipe_rules.h
// Project : PONS v7.1 / v7.2
// Role    : Re:PONS から SD を整理するとき（ログをゴミ箱へ移す・戻す・ゴミ箱を空にする。mysd.cpp の serial_wipe_* /
//           serial_trash_*）の「どのファイルを扱ってよいか」・ゴミ箱の名前の決まり・本体の画面の番号・パスワードの照合・
//           一覧の指紋・1 件ずつ移す／消す手順。SdFat に依存しない（tools/sdwipe_test で PC 上でそのまま試す）。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/25
// ============================================================
//
// ★ 消すのは「本体が自分で書いた名前」のファイルだけ。名前の形が少しでも違えば消さない。
//   data/ には人が過去大会のデータを置く（settings.h の REPLAY_FLIGHT_DIR。例 2026-07-26_0804_fused.csv）。
//   フォルダごと消したり「.csv なら全部」で消したりすると、それも消える。
//   設定（settings.txt）・地図（mapdata.csv）・目的地・パイロンの上書き・GNSS の DBD・BT1 の推定・ジャイロのゼロ点・
//   過放電の解除ファイルは、ルートにあって名前の形が合わないので消えない。
//   本体が書く名前を変えたら（mysd.cpp の saveCSV / saveRxCSV、.ino の imu_fname、imulog.cpp の nofix）ここも直すこと。
//   直さないと、新しい名前のログが「残すもの」に回って消えなくなる（消しすぎる方向には壊れない）。
//
// ★ ゴミ箱（2026-11-25〜）: 「消す」はゴミ箱へ移すだけ（名前の付け替え。空き容量は増えない）。
//   trash/<回>/<元のパス>（例 trash/003_20261125-1430/data/2026-11-25_0804.csv）。<回> は 3 桁の通し番号と、Re:PONS が
//   渡す PC の日時（本体は測位の前に時刻を知らないことがある）。本当に消すのは「ゴミ箱を空にする」だけ。
//   戻すとき、同じ名前が既にあれば（log.txt や、その起動の CSV は移したあとすぐ作り直される）上書きせず、拡張子の前に
//   "_r<回>" を付けて戻す（log_r003.txt）。この形も本体が書いた名前として扱う（もう一度ゴミ箱へ移せる）。

#ifndef SD_WIPE_RULES_H
#define SD_WIPE_RULES_H

#include <stdint.h>
#include <stddef.h>

// 消すものの種類（PONS:WIPE_SCAN / PONS:WIPE の <mask> のビット。Re:PONS の sd_wipe.js の WIPE_CATS と同じ）
#define SDWIPE_CAT_FLIGHT    0x01   // 自機の飛行 CSV: data/YYYY-MM-DD_HHMM.csv（0.960 まではルートに書いていたので、ルートの同じ形も）
#define SDWIPE_CAT_RECEIVED  0x02   // 受信ログ: received/YYYY-MM-DD_HHMM.csv
#define SDWIPE_CAT_IMURAW    0x04   // 生 IMU ログ: imuraw/YYYYMMDD.bin・nofixNNN.bin・nofix.bin（古い形）
#define SDWIPE_CAT_LOGTXT    0x08   // 本体の記録: ルートの log.txt
#define SDWIPE_CAT_ALL       0x0F

// 見るフォルダ（mysd.cpp が名前に直す。ここでは名前を持たない＝settings.h と二重に持たない）。この順に走査する
enum {
    SDWIPE_DIR_ROOT = 0,   // ルート（log.txt と、古いファームの飛行 CSV）
    SDWIPE_DIR_FLIGHT,     // REPLAY_FLIGHT_DIR（data）
    SDWIPE_DIR_RECEIVED,   // REPLAY_RECEIVED_DIR（received）
    SDWIPE_DIR_IMURAW,     // IMULOG_DIR（imuraw）
    SDWIPE_DIR_COUNT
};

#define SDWIPE_PASSWORD_MAX  32   // パスワードの最大の長さ（バイト）。PONS:WIPE の行は 127 バイトまで
#define SDWIPE_MAX_WRONG     3    // 続けてこの回数違えたら、本体を再起動するまで受けない
#define SDWIPE_TRASH_DIR     "trash"   // ゴミ箱（SD のルート）
#define SDWIPE_BATCH_MAX     999       // ゴミ箱の回の通し番号の上限（越えたら移さない。空にすれば 1 から）
#define SDWIPE_BATCH_NAME_MAX 23       // 回のフォルダの名前の長さの上限（"003_20261125-1430" は 17）
#define SDWIPE_CODE_TTL_MS   300000UL  // 本体の画面に出す番号が使える長さ（5 分）。1 回頼むと当たり外れにかかわらず使い切る

// ---- 名前の形（大文字・小文字は区別しない。SdFat が 8.3 の名前を大文字で返すことがあるため）----
static inline bool sdwipe_digits(const char* p, int n) {
    for (int i = 0; i < n; i++) if (p[i] < '0' || p[i] > '9') return false;
    return true;
}
static inline bool sdwipe_ieq(const char* a, const char* b) {
    for (; *a && *b; a++, b++) {
        char ca = (*a >= 'A' && *a <= 'Z') ? (char)(*a + 32) : *a;
        char cb = (*b >= 'A' && *b <= 'Z') ? (char)(*b + 32) : *b;
        if (ca != cb) return false;
    }
    return *a == '\0' && *b == '\0';
}
static inline size_t sdwipe_len(const char* s) { size_t n = 0; while (s[n]) n++; return n; }

// "YYYY-MM-DD_HHMM.csv"（saveCSV / saveRxCSV の名前）
static inline bool sdwipe_is_flight_csv_name(const char* n) {
    return sdwipe_len(n) == 19 && sdwipe_digits(n, 4) && n[4] == '-' && sdwipe_digits(n + 5, 2) && n[7] == '-' &&
           sdwipe_digits(n + 8, 2) && n[10] == '_' && sdwipe_digits(n + 11, 4) && sdwipe_ieq(n + 15, ".csv");
}
// "YYYYMMDD.bin"・"nofixNNN.bin"・"nofix.bin"（.ino の imu_fname と imulog.cpp の imulog_resolve_nofix）
static inline bool sdwipe_is_imuraw_name(const char* n) {
    const size_t len = sdwipe_len(n);
    if (len == 12 && sdwipe_digits(n, 8) && sdwipe_ieq(n + 8, ".bin")) return true;
    if (sdwipe_ieq(n, "nofix.bin")) return true;
    if (len == 12 && sdwipe_digits(n + 5, 3) && sdwipe_ieq(n + 8, ".bin")) {
        static const char kNofix[] = "nofix";
        for (int i = 0; i < 5; i++) {
            char c = (n[i] >= 'A' && n[i] <= 'Z') ? (char)(n[i] + 32) : n[i];
            if (c != kNofix[i]) return false;
        }
        return true;
    }
    return false;
}

// 拡張子の直前が "_r" ＋ 3 桁（ゴミ箱から戻したときに付けた印）なら、その 5 字の始まりの位置。無ければ -1
static inline int sdwipe_restore_suffix_pos(const char* n) {
    const int len = (int)sdwipe_len(n);
    int dot = -1;
    for (int i = len - 1; i >= 0; i--) if (n[i] == '.') { dot = i; break; }
    if (dot < 5) return -1;
    const int p = dot - 5;
    if (n[p] != '_' || (n[p + 1] != 'r' && n[p + 1] != 'R') || !sdwipe_digits(n + p + 2, 3)) return -1;
    return p;
}

// ゴミ箱から戻すときの別名: 拡張子の前に "_r<回 3 桁>" を入れる（"log.txt" → "log_r003.txt"）。
// もう印が付いている名前・拡張子の無い名前・入りきらないときは false（二重に付けない＝戻すのをやめる）
static inline bool sdwipe_restore_name(const char* name, int batch_no, char* out, size_t outsz) {
    if (!name || !out || batch_no < 0 || batch_no > SDWIPE_BATCH_MAX) return false;
    if (sdwipe_restore_suffix_pos(name) >= 0) return false;
    const int len = (int)sdwipe_len(name);
    int dot = -1;
    for (int i = len - 1; i >= 0; i--) if (name[i] == '.') { dot = i; break; }
    if (dot <= 0) return false;
    if ((size_t)len + 5 + 1 > outsz) return false;
    int o = 0;
    for (int i = 0; i < dot; i++) out[o++] = name[i];
    out[o++] = '_'; out[o++] = 'r';
    out[o++] = (char)('0' + batch_no / 100); out[o++] = (char)('0' + (batch_no / 10) % 10); out[o++] = (char)('0' + batch_no % 10);
    for (int i = dot; i < len; i++) out[o++] = name[i];
    out[o] = '\0';
    return true;
}

// フォルダ dir（SDWIPE_DIR_*）の中のファイル name が、どの種類のログか。扱ってよくなければ 0。
// Mac が SD に作る "._<名前>"（AppleDouble）は、元の名前が扱うものなら一緒に扱う（元が無くなると中身の無いごみになる）。
// ゴミ箱から戻した "_r<回>" の付いた名前も、印を除いた名前で見る。
static inline uint8_t sdwipe_category(int dir, const char* name) {
    if (!name) return 0;
    if (name[0] == '.' && name[1] == '_') name += 2;
    char base[64];
    const int sp = sdwipe_restore_suffix_pos(name);
    if (sp >= 0) {
        const size_t len = sdwipe_len(name);
        if (len - 5 + 1 > sizeof(base)) return 0;
        size_t o = 0;
        for (size_t i = 0; i < len; i++) if ((int)i < sp || (int)i >= sp + 5) base[o++] = name[i];
        base[o] = '\0';
        name = base;
    }
    switch (dir) {
    case SDWIPE_DIR_ROOT:
        if (sdwipe_ieq(name, "log.txt")) return SDWIPE_CAT_LOGTXT;
        if (sdwipe_is_flight_csv_name(name)) return SDWIPE_CAT_FLIGHT;
        return 0;
    case SDWIPE_DIR_FLIGHT:
        return sdwipe_is_flight_csv_name(name) ? SDWIPE_CAT_FLIGHT : 0;
    case SDWIPE_DIR_RECEIVED:
        return sdwipe_is_flight_csv_name(name) ? SDWIPE_CAT_RECEIVED : 0;
    case SDWIPE_DIR_IMURAW:
        return sdwipe_is_imuraw_name(name) ? SDWIPE_CAT_IMURAW : 0;
    default:
        return 0;
    }
}

// パスワードの照合。本体に設定が無い（空）ときは何を送られても false（消去を受けない）。
// 長さの違いも含めて最後まで比べる（どこで違ったかを時間で漏らさない。USB の先は人なので念のため程度）。
static inline bool sdwipe_password_match(const char* expected, const char* given) {
    if (!expected || !given || expected[0] == '\0') return false;
    const size_t le = sdwipe_len(expected), lg = sdwipe_len(given);
    if (le > SDWIPE_PASSWORD_MAX || lg > SDWIPE_PASSWORD_MAX) return false;
    uint8_t diff = (uint8_t)(le != lg);
    for (size_t i = 0; i < SDWIPE_PASSWORD_MAX; i++) {
        const uint8_t a = (i < le) ? (uint8_t)expected[i] : 0;
        const uint8_t b = (i < lg) ? (uint8_t)given[i] : 0;
        diff |= (uint8_t)(a ^ b);
    }
    return diff == 0;
}

// ---- ゴミ箱の回のフォルダの名前（"003_20261125-1430" または "003"）----
// 先頭の 3 桁の通し番号。形が違えば -1（人が trash/ に置いたフォルダなど。番号の数え上げに入れない）
static inline int sdwipe_batch_number(const char* d) {
    if (!d || !sdwipe_digits(d, 3) || (d[3] != '\0' && d[3] != '_')) return -1;
    return (d[0] - '0') * 100 + (d[1] - '0') * 10 + (d[2] - '0');
}
// Re:PONS が渡す PC の日時 "YYYYMMDD-HHMM"（月 1〜12・日 1〜31・時 0〜23・分 0〜59）
static inline bool sdwipe_stamp_ok(const char* s) {
    if (!s || sdwipe_len(s) != 13 || !sdwipe_digits(s, 8) || s[8] != '-' || !sdwipe_digits(s + 9, 4)) return false;
    const int mo = (s[4] - '0') * 10 + (s[5] - '0'), dd = (s[6] - '0') * 10 + (s[7] - '0');
    const int hh = (s[9] - '0') * 10 + (s[10] - '0'), mi = (s[11] - '0') * 10 + (s[12] - '0');
    return mo >= 1 && mo <= 12 && dd >= 1 && dd <= 31 && hh <= 23 && mi <= 59;
}
// 回のフォルダの名前を作る。stamp が使えなければ番号だけ
static inline bool sdwipe_batch_name(int no, const char* stamp, char* out, size_t outsz) {
    if (no < 1 || no > SDWIPE_BATCH_MAX || !out || outsz < 4) return false;
    out[0] = (char)('0' + no / 100); out[1] = (char)('0' + (no / 10) % 10); out[2] = (char)('0' + no % 10);
    size_t o = 3;
    if (sdwipe_stamp_ok(stamp)) {
        if (outsz < 3 + 1 + 13 + 1) return false;
        out[o++] = '_';
        for (int i = 0; i < 13; i++) out[o++] = stamp[i];
    }
    out[o] = '\0';
    return true;
}
// PONS:TRASH_RESTORE で受けた回の名前が、ゴミ箱の 1 段目のフォルダの名前として使えるか（"/"・".."・空白などを弾く）。
// 本体が作る形（sdwipe_batch_number が読める）に限る
static inline bool sdwipe_batch_arg_ok(const char* s) {
    if (!s || sdwipe_batch_number(s) < 0) return false;
    const size_t len = sdwipe_len(s);
    if (len > SDWIPE_BATCH_NAME_MAX) return false;
    for (size_t i = 0; i < len; i++) {
        const char c = s[i];
        if (!((c >= '0' && c <= '9') || c == '_' || c == '-')) return false;
    }
    return true;
}
// 本体の画面の番号（4 桁ちょうど）
static inline bool sdwipe_parse_code(const char* s, uint16_t* out) {
    if (!s || !sdwipe_digits(s, 4) || s[4] != '\0') return false;
    if (out) *out = (uint16_t)((s[0] - '0') * 1000 + (s[1] - '0') * 100 + (s[2] - '0') * 10 + (s[3] - '0'));
    return true;
}

// 一覧の指紋（FNV-1a 32bit）。消すファイルのパスを走査の順に足す。大きさは入れない
// （PC 接続の間も log.txt は伸び続けるので、大きさを入れると確かめた直後でも食い違う。見るのは「同じファイルの組か」）。
#define SDWIPE_FP_INIT 2166136261u
static inline uint32_t sdwipe_fp_add(uint32_t h, const char* path) {
    for (const char* p = path; *p; p++) { h ^= (uint8_t)*p; h *= 16777619u; }
    h ^= (uint8_t)'\n';
    h *= 16777619u;
    return h;
}

// ---- 消す手順 ----
#define SDWIPE_BATCH     16   // 1 周で名前を控えて消す数
#define SDWIPE_PATH_MAX  80   // 控えるパスの長さ（消すものは "imuraw/._nofix001.bin" ほどで、これより十分短い）

struct SdWipeResult {
    uint32_t done;      // 消した数
    uint32_t failed;    // 消せなかった数（読み出し専用の印など）
    uint64_t freed;     // 消したファイルの大きさの合計 [B]
    bool     stopped;   // 止められた（PONS:CANCEL・切断・ボタン・USB を抜いた）
};

// 確かめた count 件を消す。
//   walk(fn)    : 消すファイルを走査の順に fn(path, size) へ渡す。fn が false を返したらそこで終える。止められたら false を返す
//   remove(path): 消せたら true
//   stop()      : やめるなら true（1 件ごとに見る）
//   progress(済んだ数, count): 1 件ごと
// ★ 走査しながらは消さない: SDWIPE_BATCH 件ずつ名前を控え、走査を終えて（フォルダを閉じて）から消す。開いているフォルダの中身を
//   消しながら読み進める形にしない。消せなかったファイルは次の周でも先頭に出てくる（それより前は消えている）ので、その数だけ飛ばす。
// ★ count 件より多くは消さない（確かめた数を超えて見つかっても、そこで終える）。1 周で必ず 1 件以上を「消した／消せなかった」に
//   数えるので、周の数は count 以下で終わる。
template <typename Walk, typename Remove, typename Stop, typename Progress>
static inline SdWipeResult sdwipe_delete_batched(uint32_t count, Walk walk, Remove remove, Stop stop, Progress progress) {
    static char     batch[SDWIPE_BATCH][SDWIPE_PATH_MAX];
    static uint32_t bsize[SDWIPE_BATCH];
    SdWipeResult r = {0, 0, 0, false};
    for (uint32_t pass = 0; pass <= count && r.done + r.failed < count; pass++) {
        int n = 0;
        uint32_t skip = r.failed;
        const uint32_t room = count - r.done - r.failed;
        const bool walked = walk([&](const char* path, uint32_t size) -> bool {
            if (skip > 0) { skip--; return true; }
            // ★ 入りきらないパスは切り詰めない（切り詰めた名前が別のファイルを指しうる）。空にして「消せなかった」に数える
            size_t len = 0;
            while (path[len] && len < SDWIPE_PATH_MAX) len++;
            if (len < SDWIPE_PATH_MAX) { for (size_t i = 0; i <= len; i++) batch[n][i] = path[i]; }
            else                       batch[n][0] = '\0';
            bsize[n] = size;
            n++;
            return n < SDWIPE_BATCH && (uint32_t)n < room;
        });
        if (!walked) { r.stopped = true; break; }
        if (n == 0) break;
        for (int i = 0; i < n; i++) {
            if (stop()) { r.stopped = true; return r; }
            if (batch[i][0] != '\0' && remove(batch[i])) { r.done++; r.freed += bsize[i]; }
            else                                         r.failed++;
            progress(r.done + r.failed, count);
        }
    }
    return r;
}

#endif
