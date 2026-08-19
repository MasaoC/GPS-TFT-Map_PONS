// ============================================================
// File    : imulog.h
// Project : PONS v6 (Pilot Oriented Navigation System for HPA)
// Role    : 生 IMU / GNSS 速度データの高レート バイナリロガー。
//           姿勢 ESKF（GNSS 速度援用の姿勢推定）を PC 上でオフライン開発するための
//           入力データを SD に記録する。機上では推定を行わない。
//
// なぜ生データが要るか:
//   BNO085 内蔵フュージョンは加速度計が測る「比力」を鉛直とみなすため、
//   定常旋回中はロールを過小評価し、加減速中はピッチがずれる。
//   これを直すには GNSS 速度から対地加速度を与えて真の重力方向を求める必要がある。
//   その ESKF は実飛行データに対して PC 上で開発・チューニングしてから移植する。
//
//   ※ ESKF には SH2_ACCELEROMETER（重力込みの生比力）を使う。
//     SH2_LINEAR_ACCELERATION は「BNO085 自身の（旋回中に誤る）姿勢推定」を使って
//     重力を除去した値なので、まさに信用できない情報が混入しており使えない。
//
// ---- バイナリ形式（PC 側デコーダはこの定義に従うこと）----
// ファイルは ImuLogRec（28 バイト固定長・リトルエンディアン）の並びのみ。
// ヘッダーやフッターは無い。途中で電源断しても最後の完全レコードまでは読める。
//
//   offset size type      field
//   0      4    uint32    t_us   ホスト時刻 time_us_32() [µs]（全種別共通の時間軸）
//   4      4    uint32    s_us   センサー時刻 sv.timestamp [µs]（GNSS では iTOW [ms]）
//   8      1    uint8     id     レコード種別（IMULOG_ID_*）
//   9      1    uint8     acc    accuracy / status（BNO085 の 0-3、無い場合 0xFF）
//   10     2    uint16    seq    連番（全種別通し）。欠落検出用
//   12     16   float[4]  v      種別ごとの意味（下表）
//
// ※ t_us は 32bit で約 71.6 分でラップする。PC 側でアンラップすること。
//   またログは O_APPEND なので、同じ日に複数回起動すると 1 ファイルに複数セッションが
//   連結され、境界で t_us が 0 付近へ戻る。セッションごとに分けてから扱うこと。
//
// ※【重要】s_us（sv.timestamp）は時刻として使えない。
//   sh2 が「ホスト時刻 - 遅延」を計算する際にアンダーフローしており、
//   実測では値が 2^32 近傍（符号付きなら小さな負値）に張り付いていた。
//   時刻が要る場合は t_us から復元すること。BNO085 はレポートを一定周期で
//   スケジュールするので、区間内で等間隔に割り付ければ正確な時刻が得られる
//   （tools/imulog/decode_imulog.py の reconstruct_time() がこれを行う）。
// ※ seq は「捨てたレコードでも消費する」。したがって seq が飛んでいる箇所は
//   SD 書き出しが間に合わずに取りこぼした地点を意味する（imulog_get_dropped() と対応）。
//
//   id                     v[0]   v[1]   v[2]   v[3]   s_us
//   IMULOG_ID_GYRO   0x01  gx     gy     gz     -      センサー時刻   [rad/s]
//   IMULOG_ID_ACCEL  0x02  ax     ay     az     -      センサー時刻   [m/s²] 重力込み
//   IMULOG_ID_MAG    0x03  mx     my     mz     -      センサー時刻   [µT]
//   IMULOG_ID_GAMERV 0x04  qw     qx     qy     qz     センサー時刻   BNO085 の推定（比較用）
//   IMULOG_ID_RV     0x05  qw     qx     qy     qz     センサー時刻   地磁気補正（比較用）
//   IMULOG_ID_LINACC 0x06  lax    lay    laz    -      センサー時刻   既存バリオ KF 用（比較用）
//   IMULOG_ID_GNSSVEL 0x10 velN   velE   velD   sAcc   iTOW [ms]     [m/s] NED、sAcc も m/s
//   IMULOG_ID_BARO   0x20  press  alt    temp   -      0             [hPa][m][℃]
//
// ---- コア分担 ----
//   imulog_push()          : Core0 から呼ぶ。リングではなく二重バッファに詰めるだけ。
//   imulog_take_pending()  : Core0 から呼ぶ。書き出し待ちバッファがあれば索引を返す。
//   imulog_write_buffer()  : Core1 がタスク経由で呼ぶ。実際の SD 書き込み。
//
// Author  : MasaoC (@masao_mobile)
// Updated : 2026/08/17
// ============================================================

#ifndef IMULOG_H
#define IMULOG_H

#include <Arduino.h>

// ---- レコード種別 ----
#define IMULOG_ID_GYRO     0x01
#define IMULOG_ID_ACCEL    0x02
#define IMULOG_ID_MAG      0x03
#define IMULOG_ID_GAMERV   0x04
#define IMULOG_ID_RV       0x05
#define IMULOG_ID_LINACC   0x06
#define IMULOG_ID_GNSSVEL  0x10
#define IMULOG_ID_BARO     0x20

#define IMULOG_ACC_NONE    0xFF   // accuracy 情報を持たないレコード

// 1 レコード 28 バイト。146 レコード = 4088 バイトを 1 バッファとする
// （SD の 4KB 境界に収まる最大数。約 0.45 秒分）。
#define IMULOG_RECS_PER_BUF 146
#define IMULOG_BUF_BYTES    (IMULOG_RECS_PER_BUF * 28)

typedef struct __attribute__((packed)) {
    uint32_t t_us;
    uint32_t s_us;
    uint8_t  id;
    uint8_t  acc;
    uint16_t seq;
    float    v[4];
} ImuLogRec;

// ---- 有効/無効（SD 設定で切り替え）----
void imulog_set_enabled(bool on);
bool imulog_get_enabled();

// リプレイ中の一時停止（Core0 から呼ぶ）。
// 停止に入る瞬間に書き出し待ちバッファを捨てるため、
// リプレイ解除後に過去のレコードが当日のファイルへ混ざることはない。
void imulog_set_paused(bool paused);

// ---- Core0: レコードを 1 件積む ----
// 無効時・バッファ枯渇時は捨ててカウントするだけ（呼び出し側で判定不要）。
void imulog_push(uint8_t id, uint32_t s_us, uint8_t acc,
                 float v0, float v1 = 0.0f, float v2 = 0.0f, float v3 = 0.0f);

// ---- Core0: 書き出し待ちバッファの索引を取り出す ----
// 戻り値 0/1 が書き出し待ちの索引、-1 なら待ちなし。
// 取り出した索引は「Core1 へ引き渡し済み」として扱われるので、
// 呼び出し側は必ず TASK_FLUSH_IMULOG を enqueue すること。
int imulog_take_pending();

// ---- Core0: 引き渡しに失敗したバッファを戻す ----
// imulog_take_pending() の後に TASK_FLUSH_IMULOG の enqueue が失敗したら必ず呼ぶこと。
// 呼ばないとそのバッファは busy のまま誰にも解放されず、
// 2 枚とも失った時点で記録が恒久停止する（再起動するまで復帰しない）。
void imulog_release_pending(int bufidx);

// ---- Core1: バッファを SD へ書き出す（タスクから呼ばれる）----
void imulog_write_buffer(int bufidx, const char* filename,
                         int year, int month, int day, int hour, int minute, int second);

// ---- 診断 ----
uint32_t imulog_get_dropped();     // バッファ枯渇で捨てたレコード数（累計）
uint32_t imulog_get_written();     // SD へ書いたレコード数（累計）

#endif // IMULOG_H
