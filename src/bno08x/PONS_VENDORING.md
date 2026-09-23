# Adafruit_BNO08x をリポジトリに取り込んだ理由と、ローカル改変の一覧

## 取り込み元

| | |
|---|---|
| ライブラリ | **Adafruit BNO08x v1.2.7**（Arduino ライブラリマネージャ版） |
| 同梱 | CEVA/Hillcrest の SH-2 ドライバ（`sh2*.c/h` `shtp.c/h`）。Adafruit がそのまま同梱している |
| 取り込み日 | 2026-09-22（PONS 0.972） |
| 置き場所 | `src/bno08x/`（Arduino は `src/` 以下を再帰的にコンパイルする） |

`Adafruit_BusIO` は**取り込んでいない**。一般的なライブラリで、ここで問題を踏んだことが 1 度も無いため。
インストール済みのものがそのまま使われる。

## なぜ取り込んだか

このライブラリの内部に対して、**ソース側で直さないと解決しない問題を複数踏んでいる**。
ライブラリフォルダを直接編集しても git に残らず、他の機体・他の PC で再現できない。
3 台運用で「この 1 台だけ挙動が違う」は最悪の壊れ方なので、リポジトリに入れて固定した。

**`#include <Adafruit_BNO08x.h>` と書いてはいけない。**山括弧だと Arduino が
インストール済みの方を拾い、ここの改変が効かないまま静かにビルドが通る。
`imu.cpp` のように `#include "src/bno08x/Adafruit_BNO08x.h"` と相対で書くこと。
**ビルドが正しいかは `<build-path>/sketch/src/bno08x/*.o` があることで確かめられる**
（インストール版が使われていると `<build-path>/libraries/Adafruit_BNO08x/` 側に出る）。

## ローカル改変の一覧

**改変したら必ずここに追記すること。**上流へ戻す/更新するときの唯一の手掛かりになる。

### (1) `Adafruit_BNO08x.cpp` — `spihal_wait_for_int()` の待ちの粒度

`delay(1)` 刻みをやめ、最初の 3ms はスピン、その先は 200µs 刻みにした。総予算 500ms は据え置き。

元の実装は 1ms 刻みで待つ。`spihal_read()` が 1 パケットにつき **2 回**、
`spihal_write()` でも 1 回通るので、**145 パケット/秒では最大 290ms/秒 Core0 が止まる**。
I2C 経路（`i2chal_read`）は INT を待たず `delay` も無いため、この負荷は
**SPI へ移ってから初めて現れた**（0.944 の I2C 構成では起きていなかった。
レート・ポーリング周期・クロックはすべて同一で、違いはバスだけ）。

実害は次のとおり。

- GRV が要求 15Hz に対し **11Hz** まで痩せる。GRV はバリオの**重力方向の基準**そのもの
  （`VARIO_USE_RAW_ACCEL=1` の経路で生比力を地球座標へ回すのに使う）。
  遅れると重力の引き算が斜めになり、傾けたときに**偽の鉛直加速度**が入って V/S が暴れる
- Core0 が遅くなるので、同じく Core0 が回している **MS5611 のレートも道連れ**で落ちる（44→36Hz）

転送そのものは 1MHz × 約 30 バイト = 0.25ms で、待ちが 8 倍重い。
**SPI クロックを上げても効かない。速度ではなく待ち方の問題。**

### (2) `Adafruit_BNO08x.cpp` — `spihal_read()` の計測カウンタ

`pons_pkt_max` / `pons_pkt_toobig` / `pons_hdr_change` を追加した。
60 秒ログの `spi pktmax=… toobig=… hdrchg=…` に出る。**消さないこと。**

- `pktmax` … 観測した最大パケット長。**384 → 768 に広げた根拠がこれ**（実測 276）
- `toobig` … `packet_size > len` で**読み出さずに戻った**回数。**1 でも出たら異常**
- `hdrchg` … ヘッダ読みと本体読みで長さが食い違った回数。SPI は ① ヘッダ 4 バイト
  ② INT 待ち ③ 本体、と別トランザクションなので、その隙に中身が変わりうる

### (3) `sh2_hal.h` — 受信バッファを 384 → 768 へ

`SH2_HAL_MAX_TRANSFER_IN` と `SH2_HAL_MAX_PAYLOAD_IN`。

**上流の既定 384 では余裕が 39% しか無い**（実測 pktmax=276）。
Core0 が長く止まるとレポートが溜まってパケットが大きくなる。
そして超えたとき、両 HAL とも

```c
if (packet_size > len) return 0;   // ★ パケットを排出しない
```

で戻るため、**パケットがデバイス側に残り、次も同じ判定になる**。
排出する実装に直すのが本筋だが、SPI は 1 トランザクションで読み切る必要があり
（部分読みしても次の CS アサートで先頭に戻る）、捨てるための一時バッファが
結局同じ大きさ要る。**広げるほうが素直**と判断した。
RAM は shtp.c の `inPayload[]` と `inTransfer[]` が各 +384 バイト増えるだけ。

### (4) `Adafruit_BNO08x.cpp` / `.h` — `_init()` の失敗箇所を外へ出す

上流の `_init()` は `sh2_open()` でも `sh2_getProdIds()` でも `false` を返すだけで、
どちらで落ちたのか分からない。実機の `[IMU] BNO085 recovery FAILED` から
原因を絞れなかったので、グローバル 2 つに記録するようにした。

```c
uint8_t pons_init_fail_step;    // 0=成功 1=spi/i2c begin 2=sh2_open 3=sh2_getProdIds
int32_t pons_init_fail_status;  // SH2 のステータスコード（sh2_err.h）
```

`imu.cpp` が復旧失敗ログに載せる。**これが次の (5) の発見に直結した。**

### (5) 改変なし — `sh2_close()` を呼ぶのは PONS 側（重要）

**上流のライブラリは、2 回目以降の `begin_SPI()` / `begin_I2C()` が必ず失敗する。**

`shtp.c` の SHTP インスタンスプールは

```c
#define MAX_INSTANCES (1)
static shtp_t instances[MAX_INSTANCES];
static shtp_t *getInstance(void) {
    for (int n = 0; n < MAX_INSTANCES; n++)
        if (instances[n].pHal == 0) return &instances[n];   // 空きの判定は pHal==0
    return 0;
}
```

で、`pHal` を 0 に戻すのは `shtp_close()`（＝`sh2_close()`）だけ。ところが
`Adafruit_BNO08x::_init()` は `sh2_open()` を呼ぶだけで close しない。
したがって 2 回目は `getInstance()` が 0 → `shtp_open()` が 0 →
`sh2_open()` が `SH2_ERR` → `_init()` が false。

**実機 2026-09-23 で `[IMU] BNO085 recovery FAILED` が毎回出ていたのはこれ。**
リセット後に INT は返っていた（＝チップは生きていた）のに、再初期化だけが
構造的に失敗していた。**つまり通信途絶からの復旧は一度も成功したことがなかった。**

ライブラリ側は直していない。**`imu.cpp` の `imu_bus_begin()` が再オープンの前に
`sh2_close()` を呼ぶ**ことで回避している。上流を更新してもこの回避は残すこと。
`spihal_close()` / `i2chal_close()` は実質何もしないので、相手が無応答でも安全。

## 手を付けていない既知の問題

**`packet_size > len` でパケットを排出しない**（上流のまま。上記 (3) 参照）。
768 に広げて当たりにくくしたが、当たったときに抜けられない構造は残っている。
`toobig` が 1 でも出たら、そこを直す必要がある。


**`sh2.c` の全オペレーションが `timeout_us` を宣言していない**（= 0 = 無限待ち）。
`sh2_getFrs` / `sh2_setFrs` / `sh2_getMetadata` などが応答しないと戻ってこない。
現状は呼ぶ側で回避している（メタデータ読み出しは `#ifndef RELEASE` に隔離、
FRS は使わない）。取り込んだので直せる状態にはあるが、**必要になるまでは触らない**。

## 上流を更新するときの手順

1. 新しい版の `src/*` を `src/bno08x/` へ上書き
2. 上の「ローカル改変の一覧」を 1 件ずつ当て直す
3. `<build-path>/sketch/src/bno08x/*.o` が出ていることを確認
4. 実機で `rate GRV=15.0 LACC=15.0 RV=5.0` が出ることを確認（改変 (1) が効いている証拠）
5. 通信途絶からの復旧が効いているかを確認する（上の (5) の回避が効いている証拠）。
   `imu_update()` で一時的に `bno085_ok = false` にすれば復旧経路に入るので、
   `[IMU] BNO085 recovery OK` が出ることを見る。**ここが
   `FAILED ... init_step=2` なら `sh2_close()` の呼び出しが外れている。**

## ライセンス

- `Adafruit_BNO08x.{h,cpp}` … **BSD 3-Clause**（`license.txt`。Copyright (c) 2019 Bryan Siepert for Adafruit Industries）
- `sh2*.{c,h}` `shtp.{c,h}` … **Apache License 2.0**（Hillcrest Laboratories, Inc.。`NOTICE.txt` と各ファイル冒頭）

どちらも原文を同梱してある。**帰属表示を消さないこと。**
