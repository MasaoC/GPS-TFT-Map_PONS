# PONS v7.1 全機能細分化リスト ＆ コード監査完了サマリー

- **監査日**: 2026-11-10（検証・修正対応完了: 2026-11-10 / コミット `dfbab6b`）
- **対象ブランチ / バージョン**: `PONSv7.1` (`BUILDVERSION "1.04"`, `BUILDDATE 20261107`)
- **監査対象**:
  - コアファームウェア全 28 ファイル（約 33,405 行）
  - PC / Web ツール・生成スクリプト・単体テスト全 35 ファイル（約 14,904 行）
- **監査・対応ステータス**:
  - 監査で指摘された全 72 件を 1 件ずつ机上検証。
  - **実バグ（計 58 件）**: ファームウェアおよびツール群にてすべて修正対応を完了（コミット `dfbab6b` にて反映、PONSv7 へも移植完了）。修正済み項目はすべて解決したため削除。
  - **誤検出・仕様確認（計 14 件）**: コード精査の結果、意図通りの設計・安全策であることを確認。将来の監査での再誤検出を防止するため、該当コード各所に最小限の解説コメントを追記完了。
  - **残存バグ件数: 0 件**。BUG-03 / 24・37・46 は 2026-11-11 に実機で直っているのを確かめた。
    残りの実機確認は BUG-16（[pons_todo.md](pons_todo.md) §4.1）と無線（BUG-59 など、同 §4.5）。
  - `BUG-59`(1)（ブロッキング版 `e220_wake()` の起床判定が弱い）は 2026-11-10 には実害が無いので保留したが、
    2026-11-11 に直した: `e220_wake_ready()` と同じ判定で待つ（[pons_link.md](pons_link.md) §6.1）。実機では未確認。

---

## 目次

1. [第1部：プロジェクト全体の機能別細分化リスト（全9大カテゴリ・42機能）](#第1部プロジェクト全体の機能別細分化リスト全9大カテゴリ42機能)
2. [付録：検証・修正対応サマリーおよび PONSv7 への移植状況](#付録検証修正対応サマリーおよび-ponsv7-への移植状況)

---

## 第1部：プロジェクト全体の機能別細分化リスト（全9大カテゴリ・42機能）

### カテゴリ1：システム基盤・デュアルコア制御・死活監視・入力管理
- **1-1. グローバル設定・ハードウェア定義・コンパイル検査**: `settings.h`
- **1-2. 起動死活監視・XOSC待機・HardFault捕捉 (`bootwatch`)**: `bootwatch.h`, `bootwatch.cpp`
- **1-3. デュアルコア初期化 (`setup`/`setup1`)・メインループ (`loop`/`loop1`)**: `GPS_TFT_map.ino` (L300–735, L2009–2354)
- **1-4. 画面モード遷移・設定放置タイムアウト・移動開始自動退場**: `GPS_TFT_map.ino` (L1757–1850, L2356–2705)
- **1-5. ボタンデバウンス（短押し/長押し/ダブルクリック）・設定メニュー定義**: `src/button.h`, `src/button.cpp`
- **1-6. WebSerial コマンドディスパッチ（`PONS:CONNECT`/`LIST`/`GET`/`PING`/`DBD`）**: `GPS_TFT_map.ino` (L1852–2002)

### カテゴリ2：GNSS測位・UBX制御・時刻管理・DBDアシスト
- **2-1. UBXプローブ・モジュール自動初期化・設定再送Watchdog**: `gnss.h`, `gnss.cpp` (L480–895, L1843–1873)
- **2-2. UBXフレームパーサ（`NAV-PVT`/`DOP`/`SAT`/`ORB`/`MON-RF`/`MGA-ACK`）**: `gnss.cpp` (L900–1430)
- **2-3. デモ飛行シミュレーション・リプレイ行適用・最高速(`MAX G/S`)集計**: `gnss.cpp` (L1435–1825, L1942–2292)
- **2-4. GNSS航法データベース (`MGA-DBD`) SD退避・復元・TTFF試験**: `gnss_dbd.h`, `gnss_dbd.cpp`
- **2-5. RTCバックアップ電池 (`BT1`) 残量推定・SD永続化**: `rtc_backup.h`, `rtc_backup.cpp`

### カテゴリ3：航法計算・目的地管理・コース逸脱警報・ベクタ地図
- **3-1. 球面三角法（Haversine距離・真方位・Mercator投影）**: `navdata.h`, `navdata.cpp` (L153–329)
- **3-2. 目的地リスト管理・パイロン座標オーバーライド・Auto10km折返し制御**: `navdata.cpp` (L34–150, L600–693)
- **3-3. 航跡リングバッファ (`LatLonManager`)・旋回角速度・コース逸脱警報**: `navdata.cpp` (L260–302), `GPS_TFT_map.ino` (L2967–3181)
- **3-4. 内蔵OSMベクタ地図エンジン（LOD選択・タイルカリング・リング/折れ線描画）**: `src/vectormap.h`, `src/vectormap.cpp`

### カテゴリ4：IMUドライバ(SCH16T)・姿勢推定ESKF・推定TAS・気圧バリオKF
- **4-1. SCH16T SPIドライバ（48bitフレーム・CRC8検査・自動復旧・250Hzポーリング）**: `imu_sensor.h`, `imu_sch16t.cpp`
- **4-2. 12状態誤差状態カルマンフィルタ (`attitude` ESKF) 伝播・GNSS速度観測更新・重力観測**: `attitude.h`, `attitude.cpp` (L1–1200)
- **4-3. 機体取付ゼロ点較正 (`APPLY`)・自動ロールトリム・平均ピッチ (`Pavg`)・風推定**: `attitude.cpp` (L1201–1516)
- **4-4. ピッチ角連動の推定対気速度 (`TAS`)・ヘッダ大枠切替**: `tas.h`, `tas.cpp`
- **4-5. MS5611 気圧センサ非ブロッキング計測・トリム平均・自動復旧**: `airdata.h`, `airdata.cpp`
- **4-6. 3状態鉛直バリオカルマンフィルタ（加速度＋気圧高度＋GNSS高度/鉛直速度融合）**: `imu.h`, `imu.cpp`

### カテゴリ5：TFT画面描画・マルチページUI・環境光調光・Readiness・電源管理
- **5-1. メイン地図画面レイヤ合成（ベクタ地図・ポリゴン・航跡・誘導線・風矢印・姿勢オーバーレイ）**: `display_tft.h`, `display_tft.cpp` (L1–3200)
- **5-2. ヘッダ・フッタ・VSIバー・各種ポップアップバナー描画**: `display_tft.cpp` (L3201–4450)
- **5-3. 設定メニュー画面・SD詳細・リプレイ選択・地図一覧画面**: `display_tft.cpp` (L4451–5271)
- **5-4. 専門詳細画面（`IMU/ESKF`・`TAS案内`・`VARIO`・`GNSS`・`WIRELESS`・`CHSCAN`・`PCCONNECT`）**: `display_pages.cpp`
- **5-5. VEML7700 環境光センサ自動調光・バッテリーセーバー点滅・アイドル減光**: `backlight.h`, `backlight.cpp`
- **5-6. 発進前準備完了判定 (`Readiness`)・READY/NOT READY音声通知**: `readiness.h`, `readiness.cpp`
- **5-7. 電源監視・充電IC故障検知・過放電EEPROMラッチ・電源切り忘れ警告**: `power.h`, `power.cpp`

### カテゴリ6：SDカード管理・Core1タスクキュー・リプレイ・生IMUロガー
- **6-1. Core0→Core1 タスクキュー基盤・クリティカルセクション制御**: `mysd.h`, `mysd.cpp` (L110–315)
- **6-2. SDIO/SPIハイブリッド初期化・ホットプラグ自動復旧・設定/CSV/ログ書込**: `mysd.cpp` (L316–1540, L2360–2700)
- **6-3. フライトCSVリプレイ再生エンジン（SPSCリングバッファ・カラム自動判別）**: `mysd.cpp` (L1541–2195)
- **6-4. SDブラウザ・WebSerialファイル一覧/ストリーミング転送**: `mysd.cpp` (L2196–2359, L2701–3036)
- **6-5. 生IMU/GNSS高レートバイナリロガー (`imulog` 二重バッファ)**: `src/imulog.h`, `src/imulog.cpp`

### カテゴリ7：サウンド・音声再生・バリオ音制御
- **7-1. フラッシュ/SDハイブリッドWAV再生エンジン・排他/優先度割り込み・ハードウェアTimer ISR**: `src/sound.h`, `src/sound.cpp` (L1–920)
- **7-2. マルチトーン再生・旋回音 (`update_tone`)・バリオ音 (`update_vario`)・アンプ制御**: `src/sound.cpp` (L921–1293)

### カテゴリ8：LoRa無線リンク (E220-900T22S)・Wire2プロトコル・ミラーリング
- **8-1. E220-900T22S ドライバ（モード制御・AUX監視・レジスタ設定・RSSI取得）**: `e220.h`, `e220.cpp`
- **8-2. PONS Link プロトコル定義・Wire2 20バイト回転スロット符号化/復号**: `lora_link/link_proto.h`, `lora_link/link_wire2.h`
- **8-3. 非ブロッキング送信状態機械・送信前キャリアチェック・Chスキャン・受信ミラーリング**: `link.h`, `link.cpp`

### カテゴリ9：PC・Web解析ツール・アセット生成・単体テスト群
- **9-1. WebSerial対応フライトビューア (`PONS Flight Viewer & Player`)**: `tools/flight_viewer/index.html`
- **9-2. OSMベクタ地図生成ツール**: `tools/vectormap/build_vectormap.py`
- **9-3. フラッシュ内蔵データ生成・画像/音声変換スクリプト群**: `tools/gen_wav_flash.py`, `tools/gen_guide_flash.py`, `tools/gen_logo_flash.py`, `tools/gen_link_icons.py`, `tools/clean_wav8.py`, `tools/png2bmp.py`
- **9-4. 動画生成・フライト前処理・KML/KMZ変換スクリプト群**: `tools/createmovie_csv2mp4.py`, `tools/flight_preprocess.py`, `tools/kml_to_mapcsv.py`, `tools/logcsv2kml4earth.py`, `tools/logcsv2tourkmz.py`
- **9-5. オフライン生IMUログ解析・Python版ESKF検証ツール群**: `tools/imulog/decode_imulog.py`, `tools/imulog/eskf.py`, `tools/imulog/merge_flight.py`, `tools/imulog/plot_response.py`, `tools/imulog/road_geometry.py`, `tools/imulog/test_eskf.py`, `tools/imulog/yaw_kml.py`
- **9-6. フライトダイナミクス解析スクリプト群 (`flightanalysis`)**: `tools/flightanalysis/*.py`
- **9-7. ホスト単体テスト (`attitude_test`)・SCH16T実機検証スケッチ (`sch16t_test`)**: `tools/attitude_test/*`, `tools/sch16t_test/sch16t_test.ino`

---

## 付録：検証・修正対応サマリーおよび PONSv7 への移植状況

### 1. 検証および修正対応サマリー（2026-11-10 実施）
- 全 72 件の指摘に対し、ソースコードの精査およびアルゴリズムの整合性確認を実施。
- 本当であった 58 件について、実機の挙動と安全性を最優先に修正を適用（コミット `dfbab6b`）。
- 主な修正領域：
  - **無線ミラーリング**: 未 Fix 時の航跡・Auto10km 誤遷移防止、機体速度による航跡間隔制御、日界処理。
  - **バリオ・音響**: MS5611 途絶時のフェールセーフ消音、警報 WAV とバリオアンプの排他制御、リプレイ時の V/S 整合。
  - **姿勢 ESKF / SCH16T**: サンプル間隔 `dt` の実測値追従、SPI 48bit フレームの加速度先行供給、50ms 途絶検知と Core0 ブロック解消。
  - **画面描画**: GS/TT 赤線の断線判定集約、詳細画面のフォント指定固定、平均ピッチ丸め精度の向上。
  - **データ・SD**: リプレイリングバッファの排他保護、地図 CSV パースの堅牢化、ログバッファ長拡張。
  - **PC/Webツール**: Flight Viewer の WebSerial 安定化、OSM ベクタ地図生成ツールのタグ漏れ修正、解析スクリプトの気圧高度換算対応。

### 2. PONSv7（0.999v7）への移植状況
- ハードウェア（基板）に依存しない修正項目は、旧バージョン `PONSv7` へもすべて手作業で移植・反映を完了。
- ファームウェアは `arduino-cli` によるコンパイルを通過確認済み。
- 姿勢推定系は `tools/attitude_test` の 3 本のテストスケッチ（`main.cpp`, `dead_test.cpp`, `yaw_test.cpp`）および `test_eskf.py` がすべて PASS することを確認済み。
- **v7 へ持ち込まなかった項目（ハードウェア・仕様差異による除外）**:
  - `BUG-19`: v7（BNO085）は FIFO でまとめて届くため 64 サンプル平均を維持。
  - `BUG-15, 16, 17, 18`: SCH16T ドライバ関連（v7 は BNO085 搭載）。
  - `BUG-46`: 「SCH16T 必須」は v7.1 特有の要件。
  - `BUG-39`: v7 には設定画面の過放電表示行が存在しない。
  - `BUG-44`: v7 には照度センサー（VEML7700）が未搭載。
  - `BUG-47`: v7 には充電残量推定のログ機能が未搭載。
  - `BUG-58`: 過放電警報の停止ロジック（v7 に未実装）。
