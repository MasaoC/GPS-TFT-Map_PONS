# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

PONS v7 (Pilot Oriented Navigation System for HPA) — 人力飛行機パイロット向けの GNSS 航法装置ファームウェア。
RP2350B(80QFN) 自作基板 + 2.8" TFT。Arduino（arduino-pico）で書く。
コメント・ドキュメントは日本語で書く（既存のスタイルに合わせる）。

## ビルドと書き込み

Arduino IDE でスケッチ `GPS_TFT_map.ino` を開いてビルドする。CLI ビルドの設定はリポジトリに入っていない。

- コア: arduino-pico（Earle Philhower）。IntelliSense 設定は [.vscode/c_cpp_properties.json](.vscode/c_cpp_properties.json)（4.5.1 / rp2350 / variant `rpipico2` を前提に書いてある）。
- 必要ライブラリ: `TFT_eSPI` / `SdFat` / `Adafruit_BNO08x`。
- **TFT_eSPI は User_Setup を書き換えないと映らない。** サンプルは [TFT_eSPI/CopySetupFile_TFT_eSPI.h](TFT_eSPI/CopySetupFile_TFT_eSPI.h)。パネル種別（ST7789 / ILI9341）の選択もここで、`settings.h` 側には分岐が無い。
- FLASH サイズ設定は地図データと連動する（下記 `VECTORMAP_HIRES`）。既定（軽量地図・約1.3MB）は 2MB 設定でも通る。

### リリース前に確認する定義（すべて [settings.h](settings.h) 冒頭）

- `RELEASE` — 有効にする。無効だと `setup1()` が `while(!Serial)` で USB 接続を待ち、実機が起動しない。
- `RELEASE_GPS` — GPS シミュレーション（`DEBUG_GPS_SIM_*`）が全部コメントアウトされていること。
- `BUILDDATE` / `BUILDVERSION` — 更新する。コミットメッセージも `0.962 何をしたか` の形式（先頭にバージョン）。
- `RELEASE` か `RELEASE_GPS` が抜けているとコンパイル時に `#warning NOT RELEASE!` が出る。これが唯一の保険。

## テスト

機上コードの自動テストは無い。**姿勢 ESKF だけ PC 上で検証できる。`attitude.cpp` を触ったら必ず走らせること。**

```sh
cd tools/attitude_test && make        # 3 本まとめてビルド＆実行
./test        # 協調旋回でバンクを復元できるか
./decl_test   # 磁気偏角の符号
./dead_test   # BNO085 断線時に安全側へ倒れるか
```

`attitude.cpp` と `tools/imulog/eskf.py` は**同じ数式・同じマウント補正の二重実装**。片方を直したらもう片方も必ず合わせる。

その他の PC 側ツール（詳細は [README.md](README.md) の「ツール」節、各サブフォルダの README）:
`tools/vectormap/build_vectormap.py`（内蔵地図生成）、`tools/imulog/`（生 IMU 解析）、`tools/createmovie_csv2mp4.py`、`tools/logcsv2kml4earth.py`、`tools/kml_to_mapcsv.py`。

## アーキテクチャ

### デュアルコアとタスクキュー

Core0 = 描画・GPS・センサー・ボタン・警報、Core1 = SD と音声。両者は [mysd.h](mysd.h) のタスクキュー（`Task` / `TASK_*` / `enqueueTask()` / `dequeueTask()`、`taskQueueMutex` 保護、深さ 40）だけで繋がる。
**Core0 から SD を直接触らないこと。** 新しい SD 操作を足すときは `TaskType` に追加し、`create*Task()` を作り、[GPS_TFT_map.ino](GPS_TFT_map.ino) の `loop1()` の switch に分岐を足す、の 3 点セット。

壊れやすい初期化順序が `.ino` にある（コメントで理由が書いてあるので消さない）:
- `bool core1_separate_stack = true;` — TFT_eSPI の `drawWideLine` がスタックを食うため必須。
- `setup1()` 冒頭の `mutex_init(&taskQueueMutex)` — Core0 の `setup()` と並行に走るので、何よりも先。
- `setup1()` 内の `Serial.println("")` — これが無いと Core1 の SD 初期化が失敗する（原因未解明のワークアラウンド）。
- 音声 ISR は `setup1()` で登録しても **Core0 で動く**（default alarm pool の仕様）。

### 表示データの供給元は 3 通りある

表示側は「実センサー」「SD リプレイ」「無線受信（ミラー）」を意識しないで済むよう、アクセサ関数で切り替える設計になっている。

| 供給元 | 判定 | 値の取り方 |
|---|---|---|
| 実センサー | 通常 | `get_gps_*()` / `attitude_get_*()` |
| リプレイ | `getReplayMode()` | `replay_get_*()` / `get_replay_attitude()` など（[gps.h](gps.h)） |
| 無線受信 | `link_mirror_active()` | [link.h](link.h) の供給関数（**意図的に replay と同じ形**にしてある） |

新しい表示項目を足すときはこの 3 系統すべてを考える。また、**リプレイ中・ミラー中は SD への記録を止める**分岐が `loop()` の各所にある（再生した日付のファイルへ現在値を書き込んで実飛行ログを汚さないため）。

### 画面

`screen_mode`（`MODE_MAP` / `MODE_SETTING` / `MODE_IMUDETAIL` / `MODE_WIRELESS` ほか、[settings.h](settings.h)）で分岐。描画は [display_tft.cpp](display_tft.cpp) がスプライトへ描いてから `pushSprite` する（`backscreen` 240×240 / `header_footer` / `vsi_sprite`）。
地図の背景は **FLASH 内蔵のベクタ地図**（[src/vectormap.cpp](src/vectormap.cpp) + `src/flashdata/vectormap_data.cpp`）。SD の BMP タイル方式は 0.93 で廃止済み。
`VECTORMAP_HIRES` を有効にすると hires 版に切り替わるが、先に `tools/vectormap/build_vectormap.py --variant hires` で生成が必要（未生成なら `#error` で止まる）＋ボード設定を 16MB(no FS) にする。生成物は git 管理外。

### モジュール分担

| ファイル | 役割 |
|---|---|
| [GPS_TFT_map.ino](GPS_TFT_map.ino) | エントリポイント。Core0/Core1 の setup/loop、ボタンコールバック、旋回角速度・コース警告 |
| [settings.h](settings.h) | 全設定・定数・ピン定義・デバッグマクロの司令塔 |
| [navdata.h](navdata.h)/.cpp | 座標変換（Mercator）・距離・真方位、目的地とコースモード |
| [display_tft.h](display_tft.h)/.cpp | TFT 描画全般 |
| [gps.h](gps.h)/.cpp | UBX NAV-PVT 解析、衛星情報、リプレイ再生 |
| [mysd.h](mysd.h)/.cpp | SD 操作、タスクキュー、設定保存、CSV ログ |
| [imu.h](imu.h)/.cpp | BNO085 と、MS5611 融合のバリオ用 Kalman filter |
| [attitude.h](attitude.h)/.cpp | 姿勢 ESKF（GNSS 速度援用）。風推定・自動ロールトリムもここ |
| [airdata.h](airdata.h)/.cpp | MS5611（非ブロッキング。i2c0 = GPIO32/33） |
| [link.h](link.h)/.cpp | PONS Link のテレメトリ層（意味を持つ） |
| [e220.h](e220.h)/.cpp | E220-900T22S(JP) の下位ドライバ（UART の叩き方だけ。意味は知らない） |
| [lora_link/link_proto.h](lora_link/link_proto.h) | 無線ペイロード定義（機上と PC 側で共有） |
| [src/](src/) | 仕様が固まって普段いじらないもの（button / sound / vectormap / imulog / flashdata）。Arduino IDE のタブに出ないだけでコンパイルはされる |

**`src/` からルート側のヘッダを見るときは `"../mysd.h"` のように 1 段戻る**（スケッチ直下は include パスに入っていない）。

### PONS Link（v7〜）

同一ファームウェアを 3 台に焼き、SD の設定で TX（機体）/ RX（ボート・プラットフォーム）を切り替える。
**下り一方向のみで、上りの通信路そのものが存在しない**（大会レギュレーション上の要件）。この非対称性を壊す変更をしないこと。
警報は受信側が受け取った値から**自分で計算し直して**鳴らす（機体と同じタイミングで同じ音が出る）。
プロトコルと電波法の扱いは [docs/pons_link.md](docs/pons_link.md)、実機立ち上げ手順は [docs/pons_link_bringup.md](docs/pons_link_bringup.md)。

### SD カードのレイアウト

ルート直下は**設定ファイルだけ**（`settings.txt` / `mapdata.csv` / `destinations.csv` / `override_pilon_coordinate.csv` / `logo.bmp` / `wav/`）。
`data/` = 自機の飛行 CSV、`received/` = 受信ログ、`imu_replaydata/` = ESKF 結果（5Hz）、`imuraw/` = 生 IMU バイナリ（28B 固定長）、`euler/` = 0.93 以前の姿勢ログ（読み込みのみ）、`/log.txt` = イベントログ。
リプレイ一覧は `data/` と `received/` しか見ない（除外リストを不要にするための構造）。列の意味は README の「ログの記録項目」にある。
リポジトリの [sd/](sd/) がカードの中身の雛形。

## この repo 特有の注意

- **設定値は settings.h に一箇所だけ置く。** 例えばバリオのデッドバンドは音（sound.cpp）と VSI のグレー線（display_tft.cpp）が同じ `VARIO_DEADBAND_*` を見ている。片方だけ直すと「線は出ているのに鳴らない」になる。同種の注意コメントが各所にあるので従う。
- デバッグ出力は `DEBUG_P(date, txt)` 系マクロ。第 1 引数は「その print を書いた日付」で、`BUILDDATE - PRINTREVERSEDATE_NUM` より新しいものだけが出る（古いデバッグ出力が自然に消える仕組み）。`RELEASE` 時は全部消える。
- 各ファイル冒頭に `File / Project / Role / Author / Updated` のヘッダーコメントブロックがある。新規ファイルも同じ形式にする。
- 方位はすべて真方位。磁方位は廃止済み。
- 削除済みの機能（Quectel GPS、SD の BMP タイル地図、v6 の輝度制御）は「なぜ消したか」がコメントに残っている。復活させたい場合は git 履歴を参照。
- `production/`（KiCad の発注提出物）と `src/flashdata/vectormap_data_hires.cpp` は生成物なので git 管理外。
- ライセンスは 2 本立て: コードは MIT、`vectormap_data.cpp` は ODbL 1.0（OpenStreetMap 由来）。地図データに手を入れるときは帰属表示を残す。
