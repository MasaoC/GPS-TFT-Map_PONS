// ============================================================
// File    : tools/sdwipe_test/main.cpp
// Project : PONS v7.1 / v7.2
// Role    : src/sd_wipe_rules.h（Re:PONS から SD のログを消す PONS:WIPE）のホスト単体テスト。
//           1) 消す名前の形: 本体が書く名前だけが当たり、設定・地図・過去大会のデータ（..._fused.csv）は当たらない
//           2) 本体が実際に作る名前（mysd.cpp / .ino / imulog.cpp と同じ書式）が当たる
//           3) パスワードの照合（空の設定は常に断る・長さ違い・32 バイトまで・, を含む）
//           4) 一覧の指紋（順番と名前で変わる）
//           5) 16 件ずつ控えて消す手順: 全部消える / 消せないファイルを飛ばして残りを消す / 確かめた数より多く消さない / 止めると止まる
//           6) ゴミ箱: 戻したときの "_r<回>" の名前・その名前も扱う・回のフォルダの名前・PC の日時・TRASH_RESTORE の引数・画面の番号
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/11/25
// ============================================================
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <set>
#include "src/sd_wipe_rules.h"

static int g_fail = 0, g_pass = 0;
#define CHECK(cond, msg, ...) do { \
    if (cond) g_pass++; \
    else { g_fail++; printf("  [FAIL] line %d: " msg "\n", __LINE__, ##__VA_ARGS__); } \
} while (0)

// ---- 1) 名前の形 ----
static void test_names() {
    printf("[1] 消す名前の形\n");
    struct Case { int dir; const char* name; uint8_t want; };
    const Case cases[] = {
        // data/: 本体の飛行 CSV だけ
        { SDWIPE_DIR_FLIGHT, "2026-11-25_0804.csv",         SDWIPE_CAT_FLIGHT },
        { SDWIPE_DIR_FLIGHT, "2026-11-25_0804.CSV",         SDWIPE_CAT_FLIGHT },   // 8.3 の名前が大文字で返る
        { SDWIPE_DIR_FLIGHT, "._2026-11-25_0804.csv",       SDWIPE_CAT_FLIGHT },   // Mac の AppleDouble
        { SDWIPE_DIR_FLIGHT, "2026-07-26_0804_fused.csv",   0 },                   // 過去大会のデータ（人が置く）
        { SDWIPE_DIR_FLIGHT, "._2026-07-26_0804_fused.csv", 0 },
        { SDWIPE_DIR_FLIGHT, "2026-11-25_804.csv",          0 },
        { SDWIPE_DIR_FLIGHT, "2026-11-25_08041.csv",        0 },
        { SDWIPE_DIR_FLIGHT, "x2026-11-25_0804.csv",        0 },
        { SDWIPE_DIR_FLIGHT, "2026-11-25_0804.csv.bak",     0 },
        { SDWIPE_DIR_FLIGHT, "2026-11-25_0804.txt",         0 },
        { SDWIPE_DIR_FLIGHT, "2026_11_25_0804.csv",         0 },
        { SDWIPE_DIR_FLIGHT, "2026-11-25 0804.csv",         0 },
        { SDWIPE_DIR_FLIGHT, "2026-1a-25_0804.csv",         0 },
        { SDWIPE_DIR_FLIGHT, "README.txt",                  0 },
        { SDWIPE_DIR_FLIGHT, "20261113.bin",                0 },                   // 生 IMU の形でも data/ では消さない
        { SDWIPE_DIR_FLIGHT, "log.txt",                     0 },
        { SDWIPE_DIR_FLIGHT, "",                            0 },
        { SDWIPE_DIR_FLIGHT, "._",                          0 },
        // received/
        { SDWIPE_DIR_RECEIVED, "2026-11-25_0804.csv",       SDWIPE_CAT_RECEIVED },
        { SDWIPE_DIR_RECEIVED, "._2026-11-25_0804.csv",     SDWIPE_CAT_RECEIVED },
        { SDWIPE_DIR_RECEIVED, "boat_2026-11-25.csv",       0 },
        // imuraw/
        { SDWIPE_DIR_IMURAW, "20261113.bin",                SDWIPE_CAT_IMURAW },
        { SDWIPE_DIR_IMURAW, "20261113.BIN",                SDWIPE_CAT_IMURAW },
        { SDWIPE_DIR_IMURAW, "nofix001.bin",                SDWIPE_CAT_IMURAW },
        { SDWIPE_DIR_IMURAW, "NOFIX999.BIN",                SDWIPE_CAT_IMURAW },
        { SDWIPE_DIR_IMURAW, "nofix.bin",                   SDWIPE_CAT_IMURAW },   // 古い形
        { SDWIPE_DIR_IMURAW, "._20261113.bin",              SDWIPE_CAT_IMURAW },
        { SDWIPE_DIR_IMURAW, "nofix1.bin",                  0 },
        { SDWIPE_DIR_IMURAW, "nofix0001.bin",               0 },
        { SDWIPE_DIR_IMURAW, "nofixabc.bin",                0 },
        { SDWIPE_DIR_IMURAW, "nofiz001.bin",                0 },
        { SDWIPE_DIR_IMURAW, "2026111.bin",                 0 },
        { SDWIPE_DIR_IMURAW, "20261113.bin.part",           0 },
        { SDWIPE_DIR_IMURAW, "20261113.csv",                0 },
        { SDWIPE_DIR_IMURAW, "2026-11-25_0804.csv",         0 },
        { SDWIPE_DIR_IMURAW, "decoded.csv",                 0 },
        // ルート: log.txt と、0.960 までルートに書いていた飛行 CSV だけ
        { SDWIPE_DIR_ROOT, "log.txt",                       SDWIPE_CAT_LOGTXT },
        { SDWIPE_DIR_ROOT, "LOG.TXT",                       SDWIPE_CAT_LOGTXT },
        { SDWIPE_DIR_ROOT, "._log.txt",                     SDWIPE_CAT_LOGTXT },
        { SDWIPE_DIR_ROOT, "2025-07-26_1539.csv",           SDWIPE_CAT_FLIGHT },
        { SDWIPE_DIR_ROOT, "log1.txt",                      0 },
        { SDWIPE_DIR_ROOT, "log.txt.bak",                   0 },
        { SDWIPE_DIR_ROOT, "settings.txt",                  0 },
        { SDWIPE_DIR_ROOT, "settings.tmp",                  0 },
        { SDWIPE_DIR_ROOT, "mapdata.csv",                   0 },
        { SDWIPE_DIR_ROOT, "destinations.csv",              0 },
        { SDWIPE_DIR_ROOT, "override_pilon_coordinate.csv", 0 },
        { SDWIPE_DIR_ROOT, "override_pilon_coordinate.tmp", 0 },
        { SDWIPE_DIR_ROOT, "gnss_dbd.bin",                  0 },
        { SDWIPE_DIR_ROOT, "gnss_dbd.tmp",                  0 },
        { SDWIPE_DIR_ROOT, "rtc_backup.txt",                0 },
        { SDWIPE_DIR_ROOT, "gyro_bias.txt",                 0 },
        { SDWIPE_DIR_ROOT, "batt_unlock.txt",               0 },
        { SDWIPE_DIR_ROOT, "20261113.bin",                  0 },
        { SDWIPE_DIR_ROOT, "2026-07-26_0804_fused.csv",     0 },
        // 知らないフォルダの番号
        { SDWIPE_DIR_COUNT, "2026-11-25_0804.csv",          0 },
        { -1,               "log.txt",                      0 },
    };
    for (const Case& c : cases) {
        const uint8_t got = sdwipe_category(c.dir, c.name);
        CHECK(got == c.want, "dir=%d '%s' -> %u（期待 %u）", c.dir, c.name, got, c.want);
    }
    CHECK(sdwipe_category(SDWIPE_DIR_FLIGHT, nullptr) == 0, "nullptr は 0");
}

// ---- 2) 本体が実際に作る名前（書式は mysd.cpp の saveCSV / saveRxCSV、.ino の imu_fname、imulog.cpp の nofix と同じ）----
static void test_firmware_names() {
    printf("[2] 本体が作る名前がすべて当たる\n");
    char nm[32];
    int miss = 0;
    for (int y = 2024; y <= 2099; y += 25)
        for (int mo = 1; mo <= 12; mo++)
            for (int d = 1; d <= 31; d += 10)
                for (int h = 0; h < 24; h += 7)
                    for (int mi = 0; mi < 60; mi += 13) {
                        snprintf(nm, sizeof(nm), "%04d-%02d-%02d_%02d%02d.csv", y, mo, d, h, mi);
                        if (sdwipe_category(SDWIPE_DIR_FLIGHT, nm) != SDWIPE_CAT_FLIGHT) miss++;
                        if (sdwipe_category(SDWIPE_DIR_RECEIVED, nm) != SDWIPE_CAT_RECEIVED) miss++;
                        snprintf(nm, sizeof(nm), "%04d%02d%02d.bin", y, mo, d);
                        if (sdwipe_category(SDWIPE_DIR_IMURAW, nm) != SDWIPE_CAT_IMURAW) miss++;
                    }
    for (int n = 1; n <= 999; n++) {
        snprintf(nm, sizeof(nm), "nofix%03d.bin", n);
        if (sdwipe_category(SDWIPE_DIR_IMURAW, nm) != SDWIPE_CAT_IMURAW) miss++;
    }
    CHECK(miss == 0, "本体が作る名前のうち %d 個が当たらない", miss);
}

// ---- 3) パスワード ----
static void test_password() {
    printf("[3] パスワードの照合\n");
    CHECK(sdwipe_password_match("pons-2027", "pons-2027"), "同じなら通る");
    CHECK(!sdwipe_password_match("pons-2027", "pons-2026"), "1 文字違い");
    CHECK(!sdwipe_password_match("pons-2027", "pons-202"), "短い");
    CHECK(!sdwipe_password_match("pons-2027", "pons-20277"), "長い");
    CHECK(!sdwipe_password_match("pons-2027", "PONS-2027"), "大文字・小文字は区別する");
    CHECK(!sdwipe_password_match("pons-2027", ""), "空を送られた");
    CHECK(!sdwipe_password_match("", ""), "本体の設定が空なら、空を送られても断る");
    CHECK(!sdwipe_password_match("", "anything"), "本体の設定が空なら断る");
    CHECK(!sdwipe_password_match(nullptr, "x"), "nullptr");
    CHECK(!sdwipe_password_match("x", nullptr), "nullptr");
    CHECK(sdwipe_password_match("a,b,c", "a,b,c"), ", を含んでもよい（行の最後に置く）");
    const std::string p32(SDWIPE_PASSWORD_MAX, 'k'), p33(SDWIPE_PASSWORD_MAX + 1, 'k');
    CHECK(sdwipe_password_match(p32.c_str(), p32.c_str()), "32 バイトは通る");
    CHECK(!sdwipe_password_match(p33.c_str(), p33.c_str()), "33 バイトは断る");
    CHECK(!sdwipe_password_match(p32.c_str(), p33.c_str()), "33 バイトを送られた");
}

// ---- 4) 指紋 ----
static uint32_t fp_of(const std::vector<std::string>& v) {
    uint32_t h = SDWIPE_FP_INIT;
    for (const auto& s : v) h = sdwipe_fp_add(h, s.c_str());
    return h;
}
static void test_fingerprint() {
    printf("[4] 一覧の指紋\n");
    const std::vector<std::string> a = { "log.txt", "data/2026-11-25_0804.csv", "imuraw/20261125.bin" };
    CHECK(fp_of(a) == fp_of(a), "同じなら同じ");
    CHECK(fp_of(a) != fp_of({ "data/2026-11-25_0804.csv", "log.txt", "imuraw/20261125.bin" }), "順番が違えば違う");
    CHECK(fp_of(a) != fp_of({ "log.txt", "data/2026-11-25_0805.csv", "imuraw/20261125.bin" }), "1 つの名前が違えば違う");
    CHECK(fp_of(a) != fp_of({ "log.txt", "data/2026-11-25_0804.csv" }), "1 つ少なければ違う");
    CHECK(fp_of({ "ab", "c" }) != fp_of({ "a", "bc" }), "区切りをまたいでも違う");
    CHECK(fp_of({}) == SDWIPE_FP_INIT, "空は初期値");
}

// ---- 5) 消す手順 ----
struct FakeFile { std::string path; uint32_t size; bool locked; bool present; bool target = true; };   // target: 消すものの形（走査で渡す）
struct FakeSd {
    std::vector<FakeFile> files;   // 走査の順
    int walks = 0, removes = 0;
    int stop_after_removes = -1;   // この数を消したら stop() が true
    int stop_in_walk = -1;         // この回目の走査の途中で止められる
    std::set<std::string> removed_paths;
};

static SdWipeResult run_delete(FakeSd& sd, uint32_t count) {
    return sdwipe_delete_batched(count,
        [&](auto fn) -> bool {
            sd.walks++;
            if (sd.walks > 10000) return false;   // 無限ループの見張り
            for (auto& f : sd.files) {
                if (!f.present || !f.target) continue;
                if (sd.stop_in_walk == sd.walks) return false;
                if (!fn(f.path.c_str(), f.size)) return true;
            }
            return true;
        },
        [&](const char* path) -> bool {
            for (auto& f : sd.files) {
                if (f.present && f.path == path) {
                    if (f.locked) return false;
                    f.present = false;
                    sd.removes++;
                    sd.removed_paths.insert(path);
                    return true;
                }
            }
            return false;
        },
        [&]() -> bool { return sd.stop_after_removes >= 0 && sd.removes >= sd.stop_after_removes; },
        [&](uint32_t, uint32_t) {});
}

static FakeSd make_sd(int n, std::set<int> locked = {}) {
    FakeSd sd;
    for (int i = 0; i < n; i++) {
        char p[40];
        snprintf(p, sizeof(p), "data/2026-11-%02d_%04d.csv", 1 + i % 28, i);
        sd.files.push_back({ p, (uint32_t)(1000 + i), locked.count(i) > 0, true });
    }
    return sd;
}

static void test_delete() {
    printf("[5] 16 件ずつ控えて消す手順\n");
    {   // 全部消える（16 の倍数・半端・1 件・0 件）
        for (int n : { 0, 1, 15, 16, 17, 40, 333 }) {
            FakeSd sd = make_sd(n);
            const SdWipeResult r = run_delete(sd, (uint32_t)n);
            uint64_t want_bytes = 0;
            for (int i = 0; i < n; i++) want_bytes += 1000 + i;
            CHECK(r.done == (uint32_t)n && r.failed == 0 && !r.stopped, "n=%d: done=%u failed=%u stopped=%d", n, r.done, r.failed, r.stopped);
            CHECK(r.freed == want_bytes, "n=%d: freed=%llu（期待 %llu）", n, (unsigned long long)r.freed, (unsigned long long)want_bytes);
            int left = 0;
            for (auto& f : sd.files) left += f.present;
            CHECK(left == 0, "n=%d: %d 件残った", n, left);
        }
    }
    {   // 消せないファイルを飛ばして残りを消す（先頭・途中・16 件目の境・最後）
        const std::set<int> locked = { 0, 5, 15, 16, 31, 39 };
        FakeSd sd = make_sd(40, locked);
        const SdWipeResult r = run_delete(sd, 40);
        CHECK(r.done == 34 && r.failed == 6 && !r.stopped, "done=%u failed=%u stopped=%d（期待 34/6）", r.done, r.failed, r.stopped);
        bool ok = true;
        for (size_t i = 0; i < sd.files.size(); i++) ok &= (sd.files[i].present == (locked.count((int)i) > 0));
        CHECK(ok, "消せないファイルだけが残る");
        CHECK(sd.walks <= 40, "走査は %d 回（件数を超えない）", sd.walks);
    }
    {   // 全部消せない: 止まらずに回り続けない
        std::set<int> all;
        for (int i = 0; i < 20; i++) all.insert(i);
        FakeSd sd = make_sd(20, all);
        const SdWipeResult r = run_delete(sd, 20);
        CHECK(r.done == 0 && r.failed == 20 && !r.stopped, "done=%u failed=%u", r.done, r.failed);
        CHECK(sd.walks <= 21, "走査は %d 回", sd.walks);
    }
    {   // 確かめた数より多くは消さない（走査のほうが多く見つけても）
        FakeSd sd = make_sd(40);
        const SdWipeResult r = run_delete(sd, 20);
        CHECK(r.done == 20 && sd.removes == 20, "done=%u removes=%d（期待 20）", r.done, sd.removes);
    }
    {   // 確かめた数より少なかった（あいだに減った）: あるだけ消して終わる
        FakeSd sd = make_sd(10);
        const SdWipeResult r = run_delete(sd, 30);
        CHECK(r.done == 10 && r.failed == 0, "done=%u failed=%u", r.done, r.failed);
    }
    {   // 消している途中で止める（ボタン・切断・CANCEL）
        FakeSd sd = make_sd(40);
        sd.stop_after_removes = 21;
        const SdWipeResult r = run_delete(sd, 40);
        CHECK(r.stopped && r.done == 21 && sd.removes == 21, "stopped=%d done=%u removes=%d（期待 21 で止まる）", r.stopped, r.done, sd.removes);
    }
    {   // 走査の途中で止められた
        FakeSd sd = make_sd(40);
        sd.stop_in_walk = 2;
        const SdWipeResult r = run_delete(sd, 40);
        CHECK(r.stopped && r.done == 16, "stopped=%d done=%u（期待 1 周目の 16 件で止まる）", r.stopped, r.done);
    }
    {   // 入りきらない長いパスは切り詰めずに「消せなかった」に数える（切り詰めた名前と同じ名前の別のファイルを消さない）。
        // 消すものの形は短いので本体では起きない（下で確かめる）が、手順の側でも守る
        FakeSd sd;
        const std::string longp = "data/" + std::string(100, 'x') + ".csv";
        const std::string cut = longp.substr(0, SDWIPE_PATH_MAX - 1);
        sd.files.push_back({ longp, 1, false, true });
        sd.files.push_back({ cut, 1, false, true, false });   // 消すものではない（走査で渡さない）が、SD.remove の名前としては在る
        sd.files.push_back({ "data/2026-11-25_0804.csv", 1, false, true });
        const SdWipeResult r = run_delete(sd, 2);
        CHECK(r.failed == 1 && r.done == 1, "長いパスは消せなかったに数え、もう 1 つは消す（done=%u failed=%u）", r.done, r.failed);
        CHECK(sd.removed_paths.count(cut) == 0, "切り詰めた名前の別のファイルは消さない");
        CHECK(sd.files[0].present, "長いパスのファイルは残る");
    }
    {   // 長いパスが先頭にあっても、そのあとの消すファイルへ進む（消せなかったの数だけ飛ばす）
        FakeSd sd;
        sd.files.push_back({ "data/" + std::string(100, 'y') + ".csv", 1, false, true });
        for (int i = 0; i < 20; i++) {
            char p[40];
            snprintf(p, sizeof(p), "data/2026-11-01_%04d.csv", i);
            sd.files.push_back({ p, 1, false, true });
        }
        const SdWipeResult r = run_delete(sd, 21);
        CHECK(r.done == 20 && r.failed == 1, "done=%u failed=%u（期待 20/1）", r.done, r.failed);
    }
    {
        // 消すものの形のパスで一番長いもの（"received/._2026-11-25_0804.csv"）が SDWIPE_PATH_MAX に収まる
        const char* longest = "received/._2026-11-25_0804.csv";
        CHECK(strlen(longest) < SDWIPE_PATH_MAX, "最長のパス %zu 字 < %d", strlen(longest), SDWIPE_PATH_MAX);
        CHECK(sdwipe_category(SDWIPE_DIR_RECEIVED, "._2026-11-25_0804.csv") == SDWIPE_CAT_RECEIVED, "その名前は消すもの");
    }
}

// ---- 6) ゴミ箱と番号 ----
static void test_trash() {
    printf("[6] ゴミ箱（戻したときの名前・回のフォルダ・日時・番号）\n");
    char out[64];
    struct RN { const char* in; int no; const char* want; };   // want = nullptr なら作らない
    const RN rn[] = {
        { "log.txt",                  3,   "log_r003.txt" },
        { "2026-11-25_0804.csv",      12,  "2026-11-25_0804_r012.csv" },
        { "20261125.bin",             999, "20261125_r999.bin" },
        { "nofix007.bin",             1,   "nofix007_r001.bin" },
        { "nofix.bin",                45,  "nofix_r045.bin" },
        { "._2026-11-25_0804.csv",    3,   "._2026-11-25_0804_r003.csv" },
        { "log_r003.txt",             4,   nullptr },   // 二重に付けない
        { "LOG_R003.TXT",             4,   nullptr },
        { "noext",                    3,   nullptr },
        { ".hidden",                  3,   nullptr },
        { "log.txt",                  1000, nullptr },
        { "log.txt",                  -1,  nullptr },
    };
    for (const RN& c : rn) {
        const bool ok = sdwipe_restore_name(c.in, c.no, out, sizeof(out));
        if (c.want) CHECK(ok && strcmp(out, c.want) == 0, "restore_name('%s', %d) -> '%s'（期待 '%s'）", c.in, c.no, ok ? out : "(false)", c.want);
        else        CHECK(!ok, "restore_name('%s', %d) は作らない", c.in, c.no);
    }
    CHECK(!sdwipe_restore_name("2026-11-25_0804.csv", 3, out, 24), "入りきらない（24 < 25）");
    CHECK(sdwipe_restore_name("2026-11-25_0804.csv", 3, out, 25), "ちょうど入る（24 字 + 終わり）");

    // 戻した名前も同じ種類として扱う（もう一度ゴミ箱へ移せる）
    struct C { int dir; const char* name; uint8_t want; };
    const C cs[] = {
        { SDWIPE_DIR_FLIGHT,   "2026-11-25_0804_r003.csv",       SDWIPE_CAT_FLIGHT },
        { SDWIPE_DIR_FLIGHT,   "._2026-11-25_0804_r003.csv",     SDWIPE_CAT_FLIGHT },
        { SDWIPE_DIR_RECEIVED, "2026-11-25_0804_R003.CSV",       SDWIPE_CAT_RECEIVED },
        { SDWIPE_DIR_IMURAW,   "20261125_r003.bin",              SDWIPE_CAT_IMURAW },
        { SDWIPE_DIR_IMURAW,   "nofix007_r012.bin",              SDWIPE_CAT_IMURAW },
        { SDWIPE_DIR_IMURAW,   "nofix_r001.bin",                 SDWIPE_CAT_IMURAW },
        { SDWIPE_DIR_ROOT,     "log_r003.txt",                   SDWIPE_CAT_LOGTXT },
        { SDWIPE_DIR_ROOT,     "._log_r003.txt",                 SDWIPE_CAT_LOGTXT },
        { SDWIPE_DIR_FLIGHT,   "2026-11-25_0804_r03.csv",        0 },
        { SDWIPE_DIR_FLIGHT,   "2026-11-25_0804_r0003.csv",      0 },
        { SDWIPE_DIR_FLIGHT,   "2026-11-25_0804_x003.csv",       0 },
        { SDWIPE_DIR_FLIGHT,   "2026-11-25_0804_r003_r004.csv",  0 },
        { SDWIPE_DIR_FLIGHT,   "2026-07-26_0804_fused_r003.csv", 0 },
        { SDWIPE_DIR_FLIGHT,   "_r003.csv",                      0 },
        { SDWIPE_DIR_ROOT,     "settings_r003.txt",              0 },
        { SDWIPE_DIR_ROOT,     "log_r003",                       0 },
    };
    for (const C& c : cs) {
        const uint8_t got = sdwipe_category(c.dir, c.name);
        CHECK(got == c.want, "dir=%d '%s' -> %u（期待 %u）", c.dir, c.name, got, c.want);
    }
    // 本体が作る名前に印を付けても同じ種類のまま
    {
        const struct { int dir; const char* n; } fw[] = {
            { SDWIPE_DIR_FLIGHT, "2026-11-25_0804.csv" }, { SDWIPE_DIR_RECEIVED, "2026-11-25_0804.csv" },
            { SDWIPE_DIR_IMURAW, "20261125.bin" }, { SDWIPE_DIR_IMURAW, "nofix123.bin" }, { SDWIPE_DIR_IMURAW, "nofix.bin" },
            { SDWIPE_DIR_ROOT, "log.txt" }, { SDWIPE_DIR_ROOT, "2025-07-26_1539.csv" },
        };
        int miss = 0;
        for (const auto& f : fw)
            for (int no = 1; no <= SDWIPE_BATCH_MAX; no += 37)
                if (!sdwipe_restore_name(f.n, no, out, sizeof(out)) || sdwipe_category(f.dir, out) != sdwipe_category(f.dir, f.n)) miss++;
        CHECK(miss == 0, "印を付けた名前のうち %d 個が元と違う種類", miss);
    }

    // 回の通し番号
    CHECK(sdwipe_batch_number("003_20261125-1430") == 3, "003_… は 3");
    CHECK(sdwipe_batch_number("003") == 3, "003 は 3");
    CHECK(sdwipe_batch_number("999") == 999, "999");
    CHECK(sdwipe_batch_number("03") == -1, "2 桁は数えない");
    CHECK(sdwipe_batch_number("0034") == -1, "4 桁は数えない");
    CHECK(sdwipe_batch_number("003x") == -1, "003x は数えない");
    CHECK(sdwipe_batch_number("old") == -1, "人が置いたフォルダは数えない");
    CHECK(sdwipe_batch_number(nullptr) == -1, "nullptr");

    // PC の日時
    CHECK(sdwipe_stamp_ok("20261125-1430"), "ふつうの日時");
    CHECK(sdwipe_stamp_ok("20270101-0000") && sdwipe_stamp_ok("20261231-2359"), "端");
    CHECK(!sdwipe_stamp_ok("20261325-1430") && !sdwipe_stamp_ok("20261100-1430") && !sdwipe_stamp_ok("20261132-1430"), "月日の範囲");
    CHECK(!sdwipe_stamp_ok("20261125-2400") && !sdwipe_stamp_ok("20261125-1460"), "時分の範囲");
    CHECK(!sdwipe_stamp_ok("-") && !sdwipe_stamp_ok("") && !sdwipe_stamp_ok("20261125_1430") && !sdwipe_stamp_ok("20261125-143"), "形の違い");

    // 回のフォルダの名前
    CHECK(sdwipe_batch_name(3, "20261125-1430", out, sizeof(out)) && strcmp(out, "003_20261125-1430") == 0, "003_20261125-1430: '%s'", out);
    CHECK(sdwipe_batch_name(12, "-", out, sizeof(out)) && strcmp(out, "012") == 0, "日時が無ければ番号だけ: '%s'", out);
    CHECK(sdwipe_batch_name(999, "bad", out, sizeof(out)) && strcmp(out, "999") == 0, "日時の形が違えば番号だけ");
    CHECK(!sdwipe_batch_name(0, "-", out, sizeof(out)) && !sdwipe_batch_name(1000, "-", out, sizeof(out)), "番号の範囲");
    CHECK(!sdwipe_batch_name(3, "20261125-1430", out, 17), "入りきらない");
    CHECK(strlen("003_20261125-1430") <= SDWIPE_BATCH_NAME_MAX, "回の名前は上限に入る");
    CHECK(sdwipe_batch_name(3, "20261125-1430", out, sizeof(out)) && sdwipe_batch_number(out) == 3 && sdwipe_batch_arg_ok(out),
          "作った名前は番号に読めて TRASH_RESTORE に渡せる");

    // TRASH_RESTORE の引数
    CHECK(sdwipe_batch_arg_ok("003_20261125-1430") && sdwipe_batch_arg_ok("003"), "本体が作る名前は通す");
    const char* bad[] = { "", "..", "../003", "003/../x", "003/data", "003\\x", "003 ", "003_abc", "abc", "03",
                          "003_20261125-1430-1430-14301", "003,x" };
    for (const char* b : bad) CHECK(!sdwipe_batch_arg_ok(b), "TRASH_RESTORE '%s' は断る", b);
    CHECK(!sdwipe_batch_arg_ok(nullptr), "nullptr");

    // 画面の番号
    uint16_t code = 0;
    CHECK(sdwipe_parse_code("0000", &code) && code == 0, "0000");
    CHECK(sdwipe_parse_code("4821", &code) && code == 4821, "4821");
    CHECK(sdwipe_parse_code("9999", &code) && code == 9999, "9999");
    CHECK(!sdwipe_parse_code("123", &code) && !sdwipe_parse_code("12345", &code) && !sdwipe_parse_code("12a4", &code) &&
          !sdwipe_parse_code("", &code) && !sdwipe_parse_code(" 123", &code) && !sdwipe_parse_code(nullptr, &code), "形の違いは断る");
}

int main() {
    test_names();
    test_firmware_names();
    test_password();
    test_fingerprint();
    test_delete();
    test_trash();
    printf("\n%d 項目合格 / %d 項目不合格\n", g_pass, g_fail);
    printf("結果: %s\n", g_fail == 0 ? "PASS — SD のログの消去 全テスト合格" : "FAIL");
    return g_fail == 0 ? 0 : 1;
}
