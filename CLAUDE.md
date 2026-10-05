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

**Flash Size の 16MB は 0.982 から必須。** 音声を FLASH に焼いて約 1.9MB 増えたので、
**2MB 指定ではリンクが通らない**（`region FLASH overflowed by 1700076 bytes`。
領域は Flash Size − 8KB なので 2MB では 2,088,960B しかなく、所要は約 3.79MB）。
入るのは 4MB 以上。0.981 までは 1.7MB だったので 2MB でも通っていた。
**IDE のボード設定はスケッチに保存されない**ので、別の PC や IDE の入れ直し後に必ず踏む。

コンパイル確認だけなら CLI が使える（IDE 同梱の arduino-cli。動作確認済み）:

```sh
"/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli" \
  compile -b rp2040:rp2040:generic_rp2350:flash=16777216_0 --build-path /tmp/ponsbuild .
```

- **TFT_eSPI は User_Setup を書き換えないと映らない。** サンプルは
  [TFT_eSPI/CopySetupFile_TFT_eSPI.h](TFT_eSPI/CopySetupFile_TFT_eSPI.h)。
  パネル種別（ST7789 / ILI9341）の選択もここで、`settings.h` 側には分岐が無い。
- **BNO085 は i2c1 固定。** 0.976 で SPI 対応を削除した（触るとジャイロが化ける
  問題が SPI でしか起きなかったため）。戻す必要が出たら 0.975 の
  `imu.cpp` / `settings.h` を見ること。`IMU_BUS_SPI` はもう無い。
- `VECTORMAP_HIRES` は先に `tools/vectormap/build_vectormap.py --variant hires` で
  データ生成が要る（未生成なら `#error` で止まる）。
- **音声は本体 FLASH に焼いてある。** 原本は [wav/](wav/)（SD ではない。0.982 で移した）。
  差し替えたら `python3 tools/gen_wav_flash.py` で
  [src/flashdata/wav_data.cpp](src/flashdata/wav_data.cpp) を作り直すこと。
  **忘れると古い音のまま黙って焼ける。** 生成物は git に入れてあるので、
  音を触らないなら実行は不要。

### リリース前チェック（すべて [settings.h](settings.h) 冒頭）

- `RELEASE` — 有効に。無効だと `setup1()` が `while(!Serial)` で待ち、実機が起動しない。
- `RELEASE_GNSS` — GNSS シミュレーション（`DEBUG_GNSS_SIM_*`）が全部コメントアウト済みか。
- `BUILDDATE` / `BUILDVERSION` — 更新する。README のバージョン記載も合わせる。
- どちらかが抜けていると `#warning NOT RELEASE!` が出る。これが唯一の保険。
- `python3 tools/gen_wav_flash.py --check` — `wav/` と焼き込み済みの音声が
  CRC32 で一致するか。音声を差し替えて生成を忘れていると非 0 で落ちる。

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
  switch」の 3 点セット。**音声は SD を触らなくなった（0.982）が Core1 専用のまま**
  （バッファとタイマー割り込みの状態を動かすため）。見張りは `ASSERT_CORE1`、
  SD 用の `ASSERT_SD_CORE1` とは別。流用するとログに「SD access from Core0」と
  嘘が出て読む人を SD の方へ誤誘導する。
- **音の出る条件に `good_sd()` を付けない。** 音声は本体 FLASH（`WAV_BLOB`）にあり、
  SD の有無と無関係に鳴る。0.982 まで 7 箇所に「SD が無ければビープで代替」の
  分岐があり、**SD が壊れた機体では警報がぜんぶビープになって種類が区別できなかった**。
  [src/sound.cpp](src/sound.cpp) は SdFat を include しない。戻さないこと。
  ただし [display_tft.cpp](display_tft.cpp) の起動時 500Hz×10 は別物で、
  **SD 故障を音で知らせる唯一の手段**。起動音と重なって鳴る。消さないこと。
- **音声を差し替えたら `tools/gen_wav_flash.py` を回す。** 原本は [wav/](wav/) だが、
  実機が読むのは [src/flashdata/wav_data.cpp](src/flashdata/wav_data.cpp) に焼いた方。
  回さないと**古い音のまま黙って焼ける**。リリース前は `--check`（CRC32 照合）。
  逆に、ソースに無い音声名を書いたときは生成が**ビルド前に止める**
  （実機では `ERR wav not in flash` が出るまで気づけないため）。
- **音声の同一性は `WAV_ENTRIES[]` の添字で見る。名前のポインタで比べない。**
  同じ `"wav/track.wav"` でも別の .cpp に書かれたリテラルは別アドレスになり得る。
  0.981 まで `strcmp` とポインタ比較が混在していて、**いつ静かに外れてもおかしくない**
  状態だった（同一ファイル内にしか出て来ないのでたまたま成立していた）。
- **状態を読み上げる音声には排他グループ（`WAV_EXCL_*`）を付ける。** pending は
  「割り込まれた音声の救済」だが、**いまの状態を述べる読み上げには救済が害になる**
  （古い方があとから流れて**もう事実でないことを喋る**）。無線モードは設定画面で
  OFF→SENDER→RECEIVER の回転式なので **RECEIVER へ行くには SENDER を必ず通り**、
  0.984 まで通過した `sender_mode.wav` が pending から復帰して、**受信モードにした
  直後の機体が「送信モード」と読み上げていた**。モードの読み上げは飛行前の操作ミスに
  気づく手段なので、これは目的の反転。一覧と仕組みは
  [docs/pons_sound.md](docs/pons_sound.md) §3「排他グループ」。
- **pending キューは `min_volume` も運ぶ。** 落とすと `battery_low.wav`
  （優先度 1・最低音量 60）が pending から復帰したときに 60 を失い、
  音量設定 0 の機体で**電池切れの警告が完全に無音になる**
  （優先度 1 は最低なので、他の WAV とぶつかれば必ず pending を通る）。
  0.982 で代替ビープを消したので、この経路が唯一の警告になった。
- **表示の供給元は 3 系統ある。** 実センサー / リプレイ（`getReplayMode()`）/
  無線ミラー（`link_mirror_active()`）。表示項目を足すときは 3 つとも考える。
  **リプレイ中・ミラー中は SD への記録を止める**分岐が `loop()` の各所にある
  （再生した日付のファイルへ現在値を書いて実飛行ログを汚さないため）。
- **警報を描画関数の中に書かない。** その画面を出しているときしか鳴らなくなる。
  発報は `loop()` の最上位へ。ナビの判断（Auto10km の折返し・コース警報・旋回角速度）は
  `nav_alarm_tick()` にまとまっている。**ここに描画を混ぜないこと。**
- **設定値は settings.h に一箇所だけ。** 例えばバリオのデッドバンドは音（sound.cpp）と
  VSI のグレー線（display_tft.cpp）が同じ定数を見ている。片方だけ直すと
  「線は出ているのに鳴らない」になる。**「どちらの定数を選ぶか」も
  `vario_deadband_mps()`（imu.cpp）に集約した** — 0.983 まで定数は共有しつつ
  判定式だけが別々（`get_imu_alive()` と `get_imu_ok()`）で、いつ食い違っても
  おかしくなかった。V/S の供給元そのものも `vario_vspeed_mps()` / `vario_source_ok()`。
- **無線のミラー中に自機の状態を混ぜない。** 受信モードは**送信機の画面を再現する**
  のが主旨。警告や品質表示だけが自機のまま残りやすく、0.983 の実機で 10 箇所以上
  見つかった（機体は健全なのにボートが「ESKF CALIBRATION REQUIRED」を出す、
  ボートの波の上下動でバリオが鳴る、など）。判断は
  **「その表示は、いま画面に出している数字の話か」**。一覧と各項目の出どころは
  [docs/pons_link.md](docs/pons_link.md) §5「ミラー中に『自機の状態』を混ぜない」。
- **リプレイ中とデモ中は無線を「OFF 扱い」にする。判定は `link_rf_suppressed()` /
  `link_live_ui_ok()` の 2 つだけで、条件を呼び出し側に書き写さない。**
  画面の数字が電波と無関係なので、無線の状態を混ぜると**そのとき送信機／受信機
  だったかのような画面**になる（0.984 まで受信枠・`NO SIGNAL` ポップアップ・
  艇アイコン・S/R 電池が出ていた）。ミラー中の原則を裏返した同じ間違いで、
  基準も同じ**「その表示は、いま画面に出している数字の話か」**。
  **`link_mode_setting` は書き換えないこと** — SD に保存される設定なので、
  デモのために OFF を代入すると**翌日そのまま無線が黙って飛ぶ**。
  デモが送信を止める理由は特に分かりにくい: デモは `stored_*` を上書きしないので
  **フライト CSV には残らない**のに、`link_push_telemetry()` は `get_gnss_lat()` など
  **accessor 経由**なので**仮想機体はそのまま電波に乗る**（`fixflags` だけ実 GNSS 由来
  なので、屋外では**ボートに「本物の飛行」として届く**）。
  一覧は [docs/pons_link.md](docs/pons_link.md) §5「リプレイ中・デモ中は無線を『OFF 扱い』にする」。
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
- **GPIO42 / GPIO44 は I2C バスそのもの。`INPUT_PULLUP` で停めること。**
  R46/R47 の 0Ω で H_SCL/H_SDA に連結されており、`GPIO35`/`GPIO34` と
  ショートしている。RP2350 のパッドを入力・プル無しで放置すると Low 側へ
  張り付くことがあり、外部 4.7kΩ と分圧して約 2.1V（VIH 2.31V 未満）＝
  **Low に見えてバスを殺す**。実機 2026-09-27 に
  `stage=2(no-SHTP) SDA=0 SCL=0` で 5 回中 4 回起動失敗した。
  実体は `imu_bus_park_unused_pins()`。**出力にはしないこと。**
- **GPIO41 を imu.cpp から駆動しないこと。あれは E220 の M0/M1 専用。**
  `R53` を外してあるので **BNO085 の H_CSN とは繋がっていない**（H_CSN は
  基板上で未接続。I2C では don't care）。0.975 まで「H_CSN を浮かせない」という
  誤った理由で HIGH 駆動しており、**H_CSN に届かないまま E220 を mode 3 へ
  叩き落としていた**（IMU 復旧からも呼ばれるので飛行中に無線が黙る）。
- **I2C が固まったら NRST だけでは戻らない。** NRST は BNO085 を初期化するが
  **RP2350 の I2C ブロックは初期化されない**。転送が途中で切れると RP2350 が
  SCL を、スレーブが SDA を握ったまま止まり、**3 回のリトライが全部同じ理由で
  失敗する**。`imu_i2c_bus_recover()`（ペリフェラルを外して SCL を 9 回叩き
  STOP を作る）をリトライの **NRST より先に**通すこと。
- **E220 はモード切替の「あと」も AUX の立ち上がりを待つ。**
  `set_mode()` が切替前だけ待って後を待たないと、遷移中のセルフチェック
  （データシート 5.4 / 図 36。その間 AUX は Low）に書き込みがぶつかり、
  **黙って捨てられる。エラーも返らない。** 症状は「状態レジスタがどの
  コマンド形式でも無応答」で、**書式の問題に見えるが届いていないだけ**。
  v7 立ち上げで最も長く嵌まった。同じ理由で **起床直後に最初の問い合わせを
  叩かないこと**（プリフライトは 1 回目を意図的に後ろへずらしてある）。
  詳細は [docs/pons_link.md](docs/pons_link.md) §6。
- **AUX は「いま High か」を 1 回見るだけでは足りない。立ち下がりを見るか、
  遷移全体を覆う時間を待つこと。** mode 3 → mode 0 の切替直後はモジュールが
  セルフチェックを**始める前**で、AUX は mode 3 のアイドル High のまま。そこを
  サンプルして「起床完了」と判定すると、セルフチェック中に書いて**黙って捨てられる**。
  `e220_send()` は AUX が High なので成功を返し、**`Sent` は増えるのに電波が出ない**。
  0.976 の `e220_wake_ready()` がこれを踏んだ（実機 2026-09-27: 受信機が 300 秒で
  2 発のみ・`bad=0`・RSSI −34dBm）。実体は `s_wakeSawLow` と `E220_WAKE_SETTLE_MS`。
- **送信したら AUX の立ち下がりで「本当に送ったか」を確かめる。**
  `link_tx_tick()` の `LTX_DRAINING` が AUX の Low を待ってから mode 3 へ落とす。
  落ちないまま `LINK_TX_TXSTART_MS` 経ったら**モジュールに捨てられた**ので
  `noRf` を数える（60 秒ログ `LINK TX: ... noRf=`）。
  **`Sent` は「UART へ書けた回数」でしかない。** 電波が出た証明にはならない。
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
- **生ログ（`src/imulog.h`）の レコード ID を再利用しない。** 退避済みの過去ログは
  id だけでレコードを解釈するので、止めた ID に別の意味を割り当てると
  **古いログが静かに誤デコードされる**（値はもっともらしく出る）。
  0.984 で止めた `IMULOG_ID_MAG 0x03` は欠番としてコメントを残してある。
  `tools/imulog/decode_imulog.py` の `KINDS` も同じ理由で定義を残す。
- **融合出力（GRV / RV / LACC / 動的校正）に触る前に `imu_caps()` を見る。**
  [imu_sensor.h](imu_sensor.h) の能力ビット。SCH16T に差し替えると全部落ちるので、
  「有って当然」と書いた箇所は**0 を本物として表示する**（姿勢比較の行なら
  0 は「水平で北向き」というもっともらしい値になる）。
  画面の分岐で `return` してはいけない箇所がある点にも注意
  （IMU/ESKF 2/3 ページは下にメニューと `pushSprite` があり、早期 return すると
  ページが真っ白になって操作もできなくなる）。
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
| [imu_sensor.h](imu_sensor.h) | IMU チップと推定の境界。**能力ビット `imu_caps()` と feed の口**。融合出力（GRV/RV/LACC/校正）に触る前にここを見る |
| [link.cpp](link.cpp) / [e220.cpp](e220.cpp) | 無線。link=テレメトリの意味 / e220=UART の叩き方だけ |
| [lora_link/link_proto.h](lora_link/link_proto.h) | PONS Link の通信プロトコル本体（`LinkTelem` 構造体・CRC・無線方式に依存しない設計） |
| [src/](src/) | 仕様が固まって普段いじらないもの（button / sound / vectormap / imulog / flashdata） |
| [wav/](wav/) | 音声の原本。**FLASH へ焼く元**で、SD には入れない |
| [src/flashdata/wav_data.cpp](src/flashdata/wav_data.cpp) | 焼き込んだ音声の実体。`tools/gen_wav_flash.py` の生成物。手で編集しない |
| [src/bno08x/](src/bno08x/) | 取り込んだ BNO085 ライブラリ。**改変したら PONS_VENDORING.md に追記** |

- ナビの考え方: [docs/pons_navigation.md](docs/pons_navigation.md) — 音の鳴り方: [docs/pons_sound.md](docs/pons_sound.md)
- 無線の設計・電波法: [docs/pons_link.md](docs/pons_link.md) — 実機立ち上げ: [docs/pons_link_bringup.md](docs/pons_link_bringup.md)
- BNO085 を I2C へ切り替えるとき: [docs/imu_i2c_bringup.md](docs/imu_i2c_bringup.md)
  （0.944 との挙動差・ライブラリ改変の影響・JP1 と R46/R47 の手順）
- SD カードの雛形: [sd/](sd/) — レイアウトとログの列の意味は README の「SD カードの構成」
- PC 側ツールの一覧は README の「ツール」節
- git 管理外の生成物: `production/` と `src/flashdata/vectormap_data_hires.cpp`
- ライセンスは 4 本立て。コードは MIT、`vectormap_data.cpp` は ODbL 1.0（OpenStreetMap 由来）、
  `src/bno08x/` は BSD 3-Clause（Adafruit）と Apache 2.0（Hillcrest の SH-2）。
  地図データとライブラリに手を入れるときは帰属表示を残す。
