# CLAUDE.md

PONS v7 — 人力飛行機パイロット向けの GNSS 航法装置ファームウェア。
RP2350B(80QFN) 自作基板 + 2.8" TFT、Arduino（arduino-pico 4.5.1）。

**コメントとドキュメントは日本語で書く。**
ここには「ソースを読んでも分からないこと」だけを置く。仕様の説明は
[README.md](README.md)（開発者向け）と Google Docs の説明書（利用者向け）にある。

---

## ビルド

Arduino IDE で `GPS_TFT_map.ino` を開く。ボードは **Generic RP2350** / Flash **16MB (no FS)**。
必要ライブラリは `TFT_eSPI` / `SdFat` / `Adafruit_BusIO`。
**`Adafruit_BNO08x` はインストール不要** — [src/bno08x/](src/bno08x/) に取り込んである。
`#include <Adafruit_BNO08x.h>` と山括弧で書くとインストール済みの方を拾い、
ローカル改変が効かないまま静かに通る。理由と改変点は
[src/bno08x/PONS_VENDORING.md](src/bno08x/PONS_VENDORING.md)。

コンパイル確認だけなら CLI が使える（IDE 同梱の arduino-cli。動作確認済み）:

```sh
"/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli" \
  compile -b rp2040:rp2040:generic_rp2350:flash=16777216_0 --build-path /tmp/ponsbuild .
```

- **TFT_eSPI は User_Setup を書き換えないと映らない。** サンプルは
  [TFT_eSPI/CopySetupFile_TFT_eSPI.h](TFT_eSPI/CopySetupFile_TFT_eSPI.h)。
  パネル種別（ST7789 / ILI9341）の選択もここで、`settings.h` 側には分岐が無い。
- `IMU_BUS_SPI` を触ったら **SPI / I2C 両方でビルドを通す**。`#else` 側は普段
  コンパイルされないので壊れても気づけない。
- `VECTORMAP_HIRES` は先に `tools/vectormap/build_vectormap.py --variant hires` で
  データ生成が要る（未生成なら `#error` で止まる）。

### リリース前チェック（すべて [settings.h](settings.h) 冒頭）

- `RELEASE` — 有効に。無効だと `setup1()` が `while(!Serial)` で待ち、実機が起動しない。
- `RELEASE_GNSS` — GNSS シミュレーション（`DEBUG_GNSS_SIM_*`）が全部コメントアウト済みか。
- `BUILDDATE` / `BUILDVERSION` — 更新する。README のバージョン記載も合わせる。
- どちらかが抜けていると `#warning NOT RELEASE!` が出る。これが唯一の保険。

### git の扱い（**指示があるまで触らない**）

- **push は絶対にしない。** 指示されたときだけ。
- **コミットも指示されるまでしない。** 修正が終わっても作業ツリーに置いたままにする。
  複数の修正をまとめて 1 回でコミットする運用のため、勝手に区切らない。
- コミットメッセージは `0.966 何をしたか` の形式（先頭にバージョン）。
  **バージョンは毎回 `settings.h` の `BUILDVERSION` を読んでから書く**
  （記憶で書いて 1 つ古い番号を付ける事故が実際に 2 回起きた）。

## テスト

機上コードの自動テストは無い。**姿勢 ESKF だけ PC 上で検証できる。
`attitude.cpp` を触ったら必ず走らせること。**

```sh
cd tools/attitude_test && make   # 協調旋回 / 磁気偏角の符号 / BNO085 断線時
```

**`attitude.cpp` と `tools/imulog/eskf.py` は同じ数式の二重実装。
片方を直したらもう片方も必ず合わせる。**

---

## 破ると静かに壊れる約束

- **Core0 から SD を直接触らない。** SD と音声は Core1 専用で、[mysd.h](mysd.h) の
  タスクキューだけが両者を繋ぐ。追加は「`TaskType` → `create*Task()` → `loop1()` の
  switch」の 3 点セット。
- **表示の供給元は 3 系統ある。** 実センサー / リプレイ（`getReplayMode()`）/
  無線ミラー（`link_mirror_active()`）。表示項目を足すときは 3 つとも考える。
  **リプレイ中・ミラー中は SD への記録を止める**分岐が `loop()` の各所にある
  （再生した日付のファイルへ現在値を書いて実飛行ログを汚さないため）。
- **警報を描画関数の中に書かない。** その画面を出しているときしか鳴らなくなる。
  発報は `loop()` の最上位へ。ナビの判断（Auto10km の折返し・コース警報・旋回角速度）は
  `nav_alarm_tick()` にまとまっている。**ここに描画を混ぜないこと。**
- **設定値は settings.h に一箇所だけ。** 例えばバリオのデッドバンドは音（sound.cpp）と
  VSI のグレー線（display_tft.cpp）が同じ定数を見ている。片方だけ直すと
  「線は出ているのに鳴らない」になる。
- **PONS Link は下り一方向で、上りの通信路そのものを持たない。**
  大会レギュレーション上の要件なので、この非対称性を壊さない。
- **`.ino` の初期化順のコメントは消さない。** `core1_separate_stack` /
  `mutex_init` の位置 / `setup1()` 内の `Serial.println("")` は、
  どれも消すと動かなくなる理由が書いてある。
  **`link_setup()` は起動画面より前、`link_preflight_restart()` は `setup()` の
  末尾**、という並びも同じ種類の約束。前者は起動画面に E220 の結果を出すため、
  後者は送信前チェックの監視窓が `millis()` 基準だから
  （起動画面のアニメーション約 8 秒を挟むと窓が先に過ぎ、雑音を 1 つも取れずに
  NOT MEASURED になる）。読み上げ `link_announce_mode()` も末尾。
  起動音より先に喋らせないため。
- **`src/` からルート側のヘッダは `"../mysd.h"` と 1 段戻る**（スケッチ直下は
  include パスに入っていない）。
- **BNO085 を再初期化する前に `sh2_close()` を呼ぶ。** 取り込んだライブラリの
  `_init()` は `sh2_open()` を呼ぶだけで close せず、`shtp.c` のインスタンスプールは
  `MAX_INSTANCES=1` で空き判定が `pHal==0`。**そのため 2 回目以降の `begin_SPI()` は
  構造的に必ず失敗する。** 症状は「リセット後に INT は返るのに `recovery FAILED`」で、
  **通信途絶からの復旧が一度も成功しない**（2026-09-23 に実機で判明）。
  回避は `imu_bus_begin()` の冒頭。`sh2_opened` のフラグごと消さないこと。
  詳細は [src/bno08x/PONS_VENDORING.md](src/bno08x/PONS_VENDORING.md) (5)。
  2026-09-23 に実機で検証済み（`[IMU] BNO085 recovery OK (INT wait 12ms)`）。
- **BNO085 の CS(GPIO41) は、NRST を打つ前に HIGH で駆動しておくこと。**
  基板に H_CSN のプルアップが無いため、駆動しないとリセット〜ブートの 400ms の間
  CS が浮き、**BNO085 が起動しない**（v7 実機の初回通電で実際に踏んだ）。
  実体は `imu_bus_select_protocol()` の中。リセット手順から呼ばれるので消さないこと。
- **E220 はモード切替の「あと」も AUX の立ち上がりを待つ。**
  `set_mode()` が切替前だけ待って後を待たないと、遷移中のセルフチェック
  （データシート 5.4 / 図 36。その間 AUX は Low）に書き込みがぶつかり、
  **黙って捨てられる。エラーも返らない。** 症状は「状態レジスタがどの
  コマンド形式でも無応答」で、**書式の問題に見えるが届いていないだけ**。
  v7 立ち上げで最も長く嵌まった。同じ理由で **起床直後に最初の問い合わせを
  叩かないこと**（プリフライトは 1 回目を意図的に後ろへずらしてある）。
  詳細は [docs/pons_link.md](docs/pons_link.md) §6。
- **mode 3 へ潜るときは `enter_config_mode()` を必ず通す。**
  `set_mode` + `set_baud` を自分で並べると、最後の `delay(E220_CFG_SETTLE_MS)` を
  落とす。AUX が High に戻ってもモジュールはまだコマンドを受け付けず、
  `set_baud()` の `end()/begin()` も線を一度落とすので、**最初の 1 バイトが化ける**。
  症状は `FF FF FF`（エラー応答）が返って設定の読み返しが失敗し、
  画面が「Module NO RESPONSE」になること。`e220_reconfigure()` だけこれが抜けていて、
  **設定画面で CH を変えた瞬間に無応答**になった（2026-09-23）。
  起動時だけ通っていたのは `e220_setup()` の `delay(120)` が代わりをしていたため。
- **E220 の状態レジスタ（環境ノイズ 0xA3 等）は Strict Mode でしか読めない。**
  `0x09` bit6 が切替フラグで**不揮発**。`ensure_strict_mode()` が毎起動で読み、
  無効なときだけ書く（3 台の個体差を焼くだけで吸収するため）。
  ver.1 互換の `C0 C1 C2 C3 00 02` は**使ってはいけない** — 実機で返事が無く、
  代わりに**電波が出る**。
- **BNO085 の読み出しを飢えさせない。** Core0 が数十 ms 止まると BNO085 の
  レポートが溜まり、**そのあと読んだジャイロが化ける**（gx=+v, gy=-v, gz=-v と
  3 軸が 1 つの値の符号違いのコピーになる。加速度計は同時刻に何も感じていない）。
  BNO085 内部の融合がそれを積分するので GRV と LACC も同時に飛び、**バリオの
  V/S が暴れる**。2026-09-23 に実機で確定: `loop()` で 500ms ごとに 60ms 止める
  だけで 14.8 件/分、同じ 60ms の間 `imu_update()` を回すとゼロ。
  **長いブロッキング処理の中では `imu_service_if_due()` を呼ぶこと**
  （地図描画・E220 の AUX 待ち・ボーレート切替・起動画面の各待ちに入れてある）。
  60 秒ログの `gapmax=` が実測値。ここが数十 ms に伸びたら疑う。
  **さらに悪いことに、1 秒空くと `imu_update()` が「通信途絶」と誤判定して
  生きている BNO085 をリセットしに行き、復旧に失敗してそのまま死ぬ。**
  2026-09-23 に実機で発生（起動画面の 3.5 秒ホールドが原因。それまでは
  `link_setup()` が起動画面の後ろにあり、中の `imu_service_if_due()` が
  偶然その穴を塞いでいた）。`imu_update()` の冒頭に
  **「呼び出し間隔が空いていたらその回の判定を見送る」ガード**を入れてあるが、
  ガードに頼らず待ちループ側でも呼ぶこと。
- 方位はすべて真方位。磁方位は廃止済み。
- デバッグ出力 `DEBUG_P(date, txt)` の**第 1 引数はその print を書いた日付**。
  `BUILDDATE` から一定以上古いものは出なくなる（`RELEASE` 時は全部消える）。
- 各ファイル冒頭の `File / Project / Role / Author / Updated` ブロックは新規ファイルにも付ける。

---

## どこに何があるか

各ファイルの役割は冒頭のヘッダーコメントにある。ここは索引だけ。

| | |
|---|---|
| [settings.h](settings.h) | 全設定・定数・ピン定義・デバッグマクロ。まずここ |
| [GPS_TFT_map.ino](GPS_TFT_map.ino) | Core0/Core1 の setup/loop、ボタン、**警報の発報** |
| [display_tft.cpp](display_tft.cpp) | TFT 描画**だけ**。判断や発報は置かない |
| [attitude.cpp](attitude.cpp) | 姿勢 ESKF。風推定と自動ロールトリムもここ |
| [imu.cpp](imu.cpp) | BNO085 と、MS5611 融合のバリオ用 Kalman filter |
| [link.cpp](link.cpp) / [e220.cpp](e220.cpp) | 無線。link=テレメトリの意味 / e220=UART の叩き方だけ |
| [lora_link/link_proto.h](lora_link/link_proto.h) | PONS Link の通信プロトコル本体（`LinkTelem` 構造体・CRC・無線方式に依存しない設計） |
| [src/](src/) | 仕様が固まって普段いじらないもの（button / sound / vectormap / imulog / flashdata） |
| [src/bno08x/](src/bno08x/) | 取り込んだ BNO085 ライブラリ。**改変したら PONS_VENDORING.md に追記** |

- ナビの考え方: [docs/pons_navigation.md](docs/pons_navigation.md) — 音の鳴り方: [docs/pons_sound.md](docs/pons_sound.md)
- 無線の設計・電波法: [docs/pons_link.md](docs/pons_link.md) — 実機立ち上げ: [docs/pons_link_bringup.md](docs/pons_link_bringup.md)
- SD カードの雛形: [sd/](sd/) — レイアウトとログの列の意味は README の「SD カードの構成」
- PC 側ツールの一覧は README の「ツール」節
- git 管理外の生成物: `production/` と `src/flashdata/vectormap_data_hires.cpp`
- ライセンスは 4 本立て。コードは MIT、`vectormap_data.cpp` は ODbL 1.0（OpenStreetMap 由来）、
  `src/bno08x/` は BSD 3-Clause（Adafruit）と Apache 2.0（Hillcrest の SH-2）。
  地図データとライブラリに手を入れるときは帰属表示を残す。
