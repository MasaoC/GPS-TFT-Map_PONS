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
    残りの実機確認は BUG-16（[pons_todo.md](pons_todo.md) §1）と BUG-59（同 §2 の無線）。
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

---

## 2026-11-13 IMU / ESKF 周り監査（追加リスト）

IMU（SCH16T ドライバ・バリオ KF）・ESKF（`attitude.cpp`）・呼び出し元（起動／設定ロード／表示／UI）・Python 側オフラインツールを含めて総点検し、仕様・既知 TODO との混同がないかを再確認した上で抽出した結果。

### 1. バグ候補（再確認済み / `BUG-59` は上記無線項目で使用済みのため `BUG-60` から採番）

- **`BUG-60` [高] SCH16T IRQ モードで `IMU_ACC_MAX_AGE_US` 鮮度ゲートが形骸化し、時刻基準もずれている**（`imu_sch16t.cpp`, `imu.cpp`）
  - `drain_ring()` は `take_sample(..., r.t_us)` を呼び、`take_sample()` は `imu_feed_accel(a)` を呼ぶが、`imu_feed_accel()` はサンプル時刻 `r.t_us` ではなく `time_us_32()` を `_acc_last_us` に記録している。
  - さらに IRQ モード時の `imu_drv_poll()` は `s_taken == s_taken_seen` の間 `IMU_DRV_WAIT` を返し、`imu_update()` は `st == IMU_DRV_WAIT` で即 `return` するため、後続の `(time_us_32() - _acc_last_us) > IMU_ACC_MAX_AGE_US`（250 ms 超タイムアウト）に到達しない。1 件でもサンプルが取れた場合は直前に `_acc_last_us = time_us_32()` が更新されるためやはり発火せず、`IMU_DRV_DEAD`（1000 ms）まで `_acc_stale_skips` も `_acc_vsum` / `kf_last_predict_us` のリセットも動かない。

- **`BUG-61` [高] 起動直後〜SD リカバリ中に Core0（IMU/ESKF 更新）と Core1（SD 設定ロード）が並行して設定・状態を上書き競合する**（`GPS_TFT_map.ino`, `mysd.cpp`, `attitude.cpp`）
  - Core0 は `setup()` で `imu_setup(); attitude_setup();`（`imu_sch16t` の 250 Hz IRQ 開始）を実行後、`sd_setup_complete` 待ち（最大 5000 ms）や `gnss_setup()` 内で `imu_service_if_due()`（`attitude_on_gyro()` 等）を回している。
  - 同時に Core1 は `setup1()` → `setup_sd()` → `loadSettings()`（起動時 SD 失敗時は通常ループ中の `try_sd_recovery()` → `setup_sd(false)` → `loadSettings()`）を実行し、`level_roll_off_` / `level_pitch_off_` の非アトミックな read-modify-write（`attitude_get_level_offset` / `attitude_set_level_offset`）、`attitude_finish_settings_load()` による `roll_trim_deg_`・`trim_run_count_`・`trim_run_sum_` の初期化、各種モデル係数や有効フラグの更新を排他制御無しで行う。

- **`BUG-62` [高] IMU 長時間途絶明け（`dt_meas > ESKF_IMU_TIMEOUT_US`）で ESKF の姿勢・速度・共分散がリセット／拡大されない**（`attitude.cpp`, `imu_sch16t.cpp`）
  - `imu_fresh()` は `(time_us_32() - last_gyro_us_) < ESKF_IMU_TIMEOUT_US`（0.5 s）で判定しており、`attitude_on_gyro()` が `dt_meas` 判定より前に `last_gyro_us_ = t_us` を更新するため、`try_recovery()` 成功後の最初の 1 サンプルで即座に `imu_fresh() == true`（`attitude_ready() == true`）に戻る。
  - 一方、`dt_meas > (float)ESKF_IMU_TIMEOUT_US * 1e-6f` の分岐では `pitch_avg_reset(); return;` のみを行っており、途絶前の `q_`・締まったままの `P_`・`v_`・`bt_since_us_`・`trim_roll_sum_` が残存する。途絶中に 60° 未満しか姿勢が変わっていなければ `ESKF_BIGTILT` も発火せず（60° 以上でもホールド 2 秒待ち）、誤った状態と狭い共分散のまま `attitude_on_gnss_velocity()` が再開される。

- **`BUG-63` [中] GNSS 断中に ESKF 積分速度 `sp <= WIND_MIN_SPEED_MPS` へ落ちると保持中の推定風を破棄してしまう**（`attitude.cpp`）
  - `attitude_on_gyro()` の `straight_flight` 判定は `gnss_vel_fresh()` を条件に含めているが、推定風のクリア条件は `} else if (!rpy_enabled_ || !wind_enabled_ || sp <= WIND_MIN_SPEED_MPS) {` で `gnss_vel_fresh()` を確認していない。
  - 飛行中の GNSS 途絶中に加速度バイアス等のドリフトで ESKF 速度 `sp` が `WIND_MIN_SPEED_MPS`（2.0 m/s）以下へ落ちると `wind_fill_s_` / `wind_e_` / `wind_n_` が 0 クリアされ、「GNSS 断中は直前の風を保持してオレンジ `×` で表示する」設計と矛盾する（逆に GNSS 未受信の地上静止中に `v_` がドリフトして `> 2 m/s` になるとクリアされない）。

- **`BUG-64` [中] `yaw_suspect_` 待機中の低速・静止状態（`sp < ESKF_YAW_INIT_MIN_SPEED_MPS`）で GNSS 速度更新と `last_gnss_vel_us_` 更新が停止する**（`attitude.cpp`）
  - `relevel_big_tilt()` 等で `yaw_suspect_ = true` になった後、`attitude_on_gnss_velocity()` は `sp < ESKF_YAW_INIT_MIN_SPEED_MPS`（3.0 m/s）の間 `yaw_check_arm_ = 0; return;` で早期リターンする。
  - ヨー整合性の検証だけを保留すべき低速・静止時にも、ESKF の速度観測更新および関数末尾の `last_gnss_vel_us_ = time_us_32(); gnss_vel_seen_ = true;` が実行されないため、`v_` のドリフトが補正されず、`gnss_vel_fresh()` 切れで画面にオレンジ `×` が残り続ける。

- **`BUG-65` [中] `imu_update()` の `IMU_DRV_DEAD` フォールバックが `kf_predict_baro_only(dt)` ではなく `kf_predict(0.0f, dt)` を呼んでいる**（`imu.cpp`）
  - `IMU_DRV_DEAD` 分岐が `kf_predict(0.0f, dt);` を呼んでいるため、学習済み加速度 Z バイアス `kf_x[2]` が残っている状態では毎周期 `-kf_x[2]` の仮想加速度が入り、`kf_q_bias` と `F[1][2] = -dt` によって気圧更新が `kf_x[2]` を引きずり続ける。
  - `KF_ACC_GUARD` 分岐および Python 側 `tools/imulog/vario.py` はいずれも `kf_predict_baro_only(dt)` でバイアスを凍結・分離している。`IMU_DRV_DEAD`（1000 ms）後は `imu_present() == false` となり `get_imu_vspeed_raw()` も `kf_x[1]` を直接返すため、機体側バリオ出力へそのまま影響する。

- **`BUG-66` [低〜中] `mysd.cpp` の KF 設定読込（`setKfQVel` / `setKfQBias` / `setKfR`）が `atof()` を直接使用している**（`mysd.cpp`, `attitude.cpp`）
  - `mysd.cpp` 自身の注意事項（「`atoi()` / `atof()` を直接使ってはいけない」）に反し、`float v = atof(value); if (v > 0.0f)` でそのまま `imu_set_kf_params(...)` へ渡している。`"inf"` は `v > 0.0f` を通過して `imu_kalman_baro_update()` を `inf`/`NaN` で破綻させうるほか、不正文字列や極端値に対して `read_setting_float()` 相当のクランプ・`log_sdf("ERR ...")` 記録が無い。関連して `attitude_set_airspeed_*`（`attitude.cpp`）も `> 0.0f` のみで `isfinite()` 防御が無い。

### 2. ブラッシュアップ候補（改善・整理余地）

- **固定サンプルゲインの `dt` スケーリング化**（`attitude.cpp`, `imu.cpp`, `settings.h`）
  - `ESKF_RELEVEL_GAIN`（`0.004f`）や `ESKF_UP_LP_GAIN`（`0.005f`）は 250 Hz 固定サンプル前提の定数ゲインであり、`tools/imulog/eskf.py` / `vario.py` の時間スケーリング式（`min(1, dt/1)`・`dt/(0.8+dt)`）と揃えればサンプル落ち・レート変更への耐性が上がる。
- **`level_update()` 入口の静止判定 `amag` における `ba_` 減算の統一**（`attitude.cpp`）
  - `relevel_big_tilt()` や `level_update()` 内の残差計算では `ba_` を差し引く一方、`level_update()` 入口の `amag` 判定では生の `accel_last_` を使っており、`ba_` 学習値との間にズレが生じる余地がある。
- **`tools/attitude_test/dead_test.cpp` 等の回帰テスト拡充**（`tools/attitude_test/dead_test.cpp`）
  - 現行 `dead_test.cpp` の復帰確認は同一水平姿勢・静止入力のみで `attitude_ready()` を見る構成のため、途絶中の姿勢変化・復帰直後の GNSS 再開・`yaw_suspect_` 低速待機ケースをテスト追加すると再発防止になる。
- **姿勢精度表示 `Y95%%%.0f` のフォーマット誤記修正**（`display_tft.cpp`）
  - `backscreen.printf("Y95%%%.0f\xC2\xB0", acc95);` が `"Y95%15°"` のように `%` を文字として出力している（同ファイルのログ出力は `"yaw acc +-%.0f deg(95%%)"`）。`Y95:%.0f°` または `Y95±%.0f°` への整理候補。
- **`imu_sch16t.cpp` のコメント不整合・未使用計測・ホットパス最適化余地**（`imu_sch16t.cpp`, `attitude.cpp`）
  - `s_crc_err` / `s_ce_err` / `s_sat_err` / `s_stat_bad` について「シリアルと SD ログへ出している」とコメントされているが未出力（テレメトリ出力自体は `docs/imu_sch16t_plan.md` §2.7 / `docs/pons_todo.md` §5 で保留中）。また 64 フレームごとに算出している `s_temp_c` の参照先が無い。
  - 性能面の余地として、`crc8_frame()` の逐次ビットループ（250 Hz × 6 フレーム）は LUT 化可能、また `attitude_on_gnss_velocity()` の 12x12 密行列積も疎性・対称性で削減余地あり（現状不具合ではない）。

### 3. 誤検出・仕様確認済み（再確認で対象外とした項目）

以下の項目はコード確認時に一見疑わしく見えたが、設計書・コメント・テスト・プロトコル仕様を再確認した結果、意図的仕様または既存管理済み TODO であると判断しリストから除外した。

- `KF_GNSS_VEL_ENABLED == 0` とし、`imu_kalman_gnss_update()`（高度更新）のみ有効としている点（`settings.h`, `imu.cpp`）: 2026-11-12 の意図的仕様変更（`docs/pons_todo.md` §6, `docs/pons_todo_notes.md` §4.1）。
- `kf_predict()` で `0.5*a*dt^2` 項を省略し `kf_q_vel` を固定約 30 ms 周期前提としている点（`imu.cpp`）: コメント明記の意図的チューニング。
- `learn_gyro_bias` の判定基準差（FW はサンプル数 `1250`、`eskf.py` は秒数 `5.0`）および `level_update()` の `bg_` 上限緩和: `eskf.py`・`docs/imu_sch16t_plan.md` §6.4 記載済みの意図的仕様。
- SCH16T SPI プロトコル（out-of-frame ダミーフレーム、`EOI` ロック、`CTRL_USER_IF` RMW、`LPF2` 150 Hz ルール）（`imu_sch16t.cpp`）: データシート準拠で実装済みを確認（`docs/imu_sch16t_plan.md` §5, `docs/imu_sch16t_bringup.md` §3）。
- SCH16T の CRC / STAT / SAT / セルフテスト結果の 60 秒ログ・UI 出力未実装: `docs/imu_sch16t_plan.md` §2.7 および `docs/pons_todo.md` §5・§6 で管理済みの既知 TODO。
- `relevel_big_tilt()` 後の `|d| <= 45°` で `v_` を上書きしない仕様、`airspeed_from_pitch()` の経路角非補正、`update_static()` の非減算 `g` 使用、`warn_muted_off_mount()` のバリオ非ゲート、`calib_date_stale` の `readiness` 非参入: 設計通りの意図的仕様を確認。
- `decode_imulog.py` の `WRAP` ラグ計算および `imu.cpp` の `up_lp_` 確立前 `_acc_vsum += 0` フォールバック: `yaw_test` 等で正当性を確認済み。
- `src/imulog.cpp` の Core0/Core1 二重バッファ制御変数（`imulog_fill`, `imulog_pending`, `imulog_busy`, `imulog_active`）: `volatile` 指定および `enqueueTask()`/`dequeueTask()` の `taskQueueMutex` メモリバリアで整合が保たれていることを確認。

### 4. 確かめた結果と対応（2026-11-13。v7.1 だけ・未コミット・実機未確認）

| ID | 判定 | 対応 |
|---|---|---|
| BUG-60 | 本当。ただし害は小さい。割り込みで読んでいると、サンプルが止まっても `IMU_DRV_WAIT` で返り、鮮度の安全網に来ない。起きるのは「0.15〜0.5 秒止まって戻った最初の predict が、空白をまたいだ dt を、戻ってから届いた数サンプルの平均で積分する」こと | WAIT の道でも同じ安全網（`acc_stale_gate()`）を通す。60 秒ログの `stale=` は止まった区間ごとに 1 回数える（v7.1 ではいままで一度も数えていなかった）。「時刻基準のずれ」（`_acc_last_us` が読んだ時刻ではなく受け取った時刻）は**わざとそのまま**: 描画で受け取りが遅れても、リングに溜まったサンプルは空白ではない。読んだ時刻で見ると、150ms を超える遅れで正しいサンプルを捨ててしまう |
| BUG-61 | 排他が無いのは本当。実害は無いと判断して直さない。起動時の待ち（最大 5 秒）の間、Core0 は ESKF を回すだけ。Core1 が書く値（オフセット・トリム・KF の係数）はどれも 32bit 1 個ずつの書き込み。Core0 がそれらを書き換えるのは APPLY（画面の操作）とロールトリム（60 秒の直進の窓）だけで、待ちの間には起きない。走行中の SD 復旧で設定を読むのは、起動時に一度も読めなかったときの 1 回だけ。起こりうる最悪は「ロールのオフセットだけ新しい値で 1 サンプル計算する」 | なし |
| BUG-62 | 本当 | 大きな傾きの置き直しと同じ扱いにした（ユーザーの判断）: 傾きと方位の不確かさを広げ、方位は保って 3 m/s 以上で航跡と比べる（45° を超えていたら入れ直す）。速度は次の測位で入れ直す。姿勢そのものは動かさない。`imu_gap_recover()`・eskf.py・`dead_test` の 4)。ログに `ESKF IMU GAP` |
| BUG-63 | 本当 | 遅いかどうかは `gnss_vel_fresh()` のときだけ見る（断中は風を保持） |
| BUG-64 | 本当 | 止まっているとき（GNSS 1 m/s 未満）だけ、方位の確かめは待ったまま速度の観測を入れる（ユーザーの判断。止まっていれば方位のずれは速度に効かない）。1〜3 m/s は今までどおり入れない。attitude.cpp・eskf.py・`dead_test` の 5) |
| BUG-65 | 本当 | `kf_predict_baro_only()` にして、入るときに相関を 0 にする。11/13 のエレベーターの気圧で比べると、今までの式ではバイアスが ±0.1 m/s² 振れた（IMU が戻ったときに V/S が流れる）。止まっているときの揺れは 0.049 → 0.013 m/s、遅れは 2.5 → 2.8 秒 |
| BUG-66 | 本当 | KF の 3 つを `read_setting_float()` に通す（範囲 `kf_q_vel` 1e-5〜100・`kf_q_bias` 1e-8〜1・`kf_R` 0.001〜10000）。`imu_set_kf_params()` と `attitude_set_airspeed_*()` も inf を弾く。範囲外のログは `%g` に変えた（0.0005 が 0.00 に見えていた） |

ブラッシュアップ候補:

- 固定ゲインの dt 化: 見送り。機上は割り込みで 4ms ごとに読み、届いたサンプルを 1 つずつ全部通すので、1 サンプルあたりのゲインで時定数が決まる（`ESKF_RELEVEL_GAIN` はもともと `1/IMU_RATE_GYRO_HZ` で作ってある）。eskf.py が dt で書いているのは、生ログが 62.5Hz だから
- `amag` で `ba_` を引くか: 見送り。0.5 m/s² の粗い門で、`ba_` は実測でそれよりずっと小さい。eskf.py も生の値で同じ（変えるなら両方）
- `dead_test` の拡充: 4)（途絶中の 90° 旋回）と 5)（途絶明けの静止）を足した。どちらも直す前のコードでは落ちる
- `Y95%%%.0f`: ユーザーの判断で今のまま（「Y95%15°」で慣れている）
- imu_sch16t.cpp のコメント: 健全性カウンタは「シリアルと SD ログへ出している」と書いてあったが、どこにも出していなかったので直した。`s_temp_c` は前から「書くだけ」と書いてある。CRC の表引き・12x12 の行列の削減は見送り（今は困っていない）

確かめたこと: tools/attitude_test の 3 本が通る。test_eskf.py が通る。11/11・11/12 の生ログを eskf.py に流すと、姿勢は直す前と同じ（どちらの日も途絶は 0 回）。v7.1 のビルドが通る。実機では未確認。
v7 には同じ穴のあった BUG-63・65・66 を入れた（1.00v7 `040bd73`。v7 の attitude_test が通り、ビルドも通る）。BUG-60 は v7 に無い
（BNO085 はポーリングなので安全網は働いている）。BUG-62・64 は v7 に入れていない `ESKF_BIGTILT_*` の上に作ったので v7.1 だけ。

---

## 2026-11-13 GNSS 周り監査（追加リスト）

### バグ（要修正）

- **BUG-67（優先度: 中）: リプレイモード終了直後に `gnss_ubx_watchdog()` が誤発火し GNSS を不必要に再初期化する**
  - **現象**: `replaymode_gnssoff` を伴うリプレイを終了して通常運用へ復帰した直後、GNSS モジュールが正常受信中であっても再設定が走り、一時的な測位断および DBD 状態リセットが発生する。
  - **原因**: `gnss_loop()` は `replaymode_gnssoff == true` の間 `gnss_ubx_watchdog()` をスキップしつつ `GNSS_SERIAL` のバイトをパースせず読み捨てる（`gnss_connection = true`, `gnss_last_byte_ms = millis()` は更新）。一方 `set_replaymode(false)` は `ubx_date_valid`/`ubx_time_valid`/`ubx_pvt_valid` 等をクリアするものの `last_nmea_time[]`（`get_gnss_nmea_time(0)`）をリセットしない。そのため解除直後に `ubx_age > GNSS_UBX_LOST_RECONFIG_MS`（10 秒超）かつ `bytes_alive == true`（`GNSS_CONN_TIMEOUT_MS` 未満）が即座に成立し、`gnss_setup_impl(true)`（`9600` bps 再開 → `gnss_dbd_on_setup()` で `s_ti`/`s_ti_tries` 初期化 → `38400` bps 再開・全設定再送）が誤発火する。
  - **箇所**: `GPS_TFT_map/gnss.cpp` (`set_replaymode` L1918-1957, `gnss_ubx_watchdog` L1980-2005, `gnss_loop` L2034, L2074-2081, `gnss_setup_impl` L1204, L1227, L1280), `GPS_TFT_map/settings.h` (L94, L98)
  - **推奨修正**: `set_replaymode(false)` で `replaymode_gnssoff` を解除する際、`last_nmea_time[0]` または `gnss_ubx_watchdog()` の猶予タイマー（`last_gnss_setup_time` 等）を更新し、解除後に UBX 受信が立ち上がるまでの猶予を確保する。

- **BUG-68（優先度: 高）: LoRa 時刻アシストの送信側状態判定および受信側テレメトリ受理が GNSS 切断/鮮度切れを検査していない**
  - **現象**: 送信側で GNSS が切断または更新停止（`LINK_ST_GNSS_LOST`）しても、LoRa テレメトリ上の時刻が不良とマークされない場合があり、さらに受信側はその古い時刻を取り込んで通常の時刻アシスト要求（`LINK_BC_REQ`）も抑止してしまう。
  - **原因**:
    1. `lassist_time_status_bits()` は `own_utc_ms_now()` と異なり `get_gnss_connection()` や PVT 経過時間（`age > 3000`）を確認せず、`get_gnss_date()`/`get_gnss_time()` の `isValid()` と凍結した `own_tacc_ms()` のみで `LINK_ST_TIME_BAD` / `LINK_ST_TIME_COARSE` / `0` を返す。
    2. 送信側 `link.cpp` は `LINK_ST_GNSS_LOST` を立てつつ `get_gnss_date()`/`get_gnss_time()` をそのまま詰める。
    3. 受信側 `lassist_on_telem()` は `LINK_ST_TIME_WANTED` のみ参照し `LINK_ST_GNSS_LOST` を検査せず、`link_w2_utc_ms()` が `LINK_W2_TIME_NONE` 以外を返すと `s_srcHeardMs = s_acT0` を更新（`req_due()` で `LINK_BC_REQ` 送信を抑止）した上で `consider_time()` へ渡してしまう。
  - **箇所**: `GPS_TFT_map/link_assist.cpp` (`own_utc_ms_now` L158-167, `req_due` L325-333, `lassist_on_telem` L485-501, `lassist_time_status_bits` L505-513), `GPS_TFT_map/link.cpp` (L1666-1670, L1777, L1784)
  - **推奨修正**: `lassist_time_status_bits()` に `get_gnss_connection()` および PVT 鮮度判定（あるいは `own_utc_ms_now()` 相当の検査）を入れて無効時は `LINK_ST_TIME_BAD` を返し、`lassist_on_telem()` でも `LINK_ST_GNSS_LOST` 時は時刻受理および `s_srcHeardMs` 更新を行わないようにする。

- **BUG-69（優先度: 中）: `draw_nomapdata()` が LoRa ミラーモードを考慮せず警告表示が不整合になる**
  - **現象**: LoRa ミラー表示中（`link_mirror_active()`）にローカル GNSS が未接続だと、ミラー側の未測位・弱受信警告が出ない（逆にローカル GNSS が接続中で未測位・0 衛星だと、ミラー側の測位状態と食い違った警告条件になり得る）。
  - **原因**: `gnss_speed_track_invalid()`、`eskf_display_gnss_stale()`、`gnss_disconnect_warning_shown()` は `link_mirror_active()` と `LINK_ST_GNSS_LOST` を考慮している一方、`draw_nomapdata()` は `!get_gnss_connection()`, `!get_gnss_fix()`, `get_gnss_numsat() == 0` のローカル GNSS 状態のみを見て早期リターンおよび表示判定を行っている。
  - **箇所**: `GPS_TFT_map/display_tft.cpp` (`draw_nomapdata` L4522-4527; 対照: L660-670, L679-685, L3156-3159)
  - **推奨修正**: `draw_nomapdata()` でも `link_mirror_active()` 時はミラー先の GNSS 状態（接続・測位・衛星数/`LINK_ST_GNSS_LOST`）を参照する共通ヘルパー経由で判定する。

- **BUG-70（優先度: 中）: `ubxWaitAck()` が `ACK-ACK`/`ACK-NAK` ペイロードの `(clsID, msgID)` を検証せず、連続コマンド送信時に ACK 取り違えが起き得る**
  - **現象**: `gnss_enable_mon_rf(1)` 等で複数 UBX 設定パケットを連続送信した際、遅延・ズレた前のコマンドの `UBX-ACK-ACK` を後続の `ubxWaitAck()` が誤って成功扱いする可能性がある。
  - **原因**: `ubxWaitAck()` は `case 6` でペイロードを無条件に読み飛ばしており、`ACK-ACK`（`0x05 0x01`）/ `ACK-NAK`（`0x05 0x00`）ペイロードに含まれる対象クラス/ID を照合しない。`gnss_enable_mon_rf(1)` は `0x06 0x8A` と `0x06 0x01` を連続送信した直後に `ubxWaitAck(300)` を 2 回呼び、さらに直後の step ⑩ `CFG-NAVSPG-ACKAIDING`（`0x06 0x8A`）で `ubxWaitAck(600)` を呼ぶため、前の ACK/NAK の遅延や欠落で応答対応がズレ得る。
  - **箇所**: `GPS_TFT_map/gnss.cpp` (`gnss_enable_mon_rf` L732-745, `ubxWaitAck` L1139-1164, L1410-1427)
  - **推奨修正**: `ubxWaitAck(timeout_ms, expected_cls, expected_id)` のように期待する `(clsID, msgID)` を引数化し、ペイロード 2 バイト目までと照合する。

- **BUG-71（優先度: 中）: DBD の `COLD`/`COLD,TIME` コマンド投入直後に旧時刻フラグが残り、`s_ep_tk_ms` と `TB_KNOWN` が即座に誤確定する**
  - **現象**: 未測位（`!fix`）かつ旧時刻が有効（`gnss_time_known()`）な状態で `COLD` / `COLD,TIME` コマンドを実行すると、GNSS リセット後の真の時刻獲得（`NAV-PVT`）を待たずにエピソードの時刻獲得時刻 `s_ep_tk_ms` と `TB_KNOWN` がほぼ 0 ms で確定してしまう。
  - **原因**: `gnss_dbd_command()` は `!fix` かつ `gnss_time_known()` のとき `COLD`/`COLD,TIME` の実行を許可するが、`ubx_date_valid`/`ubx_time_valid` 等をクリアしない。直後の `episode_tick()` で `!fix` により `s_ep_nofix = true` が即成立し、同じ周回または次周回で `s_ep_tk_ms == 0 && gnss_time_known()` が真となって、リセット前の古い時刻を元に `s_ep_tk_ms` を記録し `time_block()` も `TB_KNOWN` を返す。
  - **箇所**: `GPS_TFT_map/gnss_dbd.cpp` (`time_block` L540-546, `episode_tick` L830-848, `gnss_dbd_command` L1187-1205)
  - **推奨修正**: `COLD` / `COLD,TIME`（およびリセット系テスト開始時）に `ubx_date_valid`/`ubx_time_valid`（または `gnss_time_known()` 用の世代/無効化）を確実にリセットし、リセット後の新規 `NAV-PVT` 受信まで時刻既知とみなさないようにする。

- **BUG-72（優先度: 高）: LoRa Wire2 のパケット途絶復旧時に `RHAVE_DATE` クリアと `hour_known` 維持が不整合となり、UTC 日付跨ぎで `link_w2_next_day()` がスキップされ得る**
  - **現象**: `LINK_W2_STALE_FRAMES` 超過〜30 分未満の通信途絶が UTC 23 時台から 00 時台を跨いで復旧すると、`hour` は `23 -> 0` に繰り上がるのに日付が前日のまま進まない状態になり得る。
  - **原因**: `link_wire2.h` のギャップ処理は `gap_ms > LINK_W2_STALE_FRAMES * per_ms` で `st->slot_have = 0` として `RHAVE_DATE` を消去する一方、`st->hour_known` は `gap_ms >= 30 分` まで保持（`st->prev_tod` も維持）する。そのため途絶明けの `tod` 折り返し判定（`tod + 18000 < st->prev_tod`）で `st->hour == 23` の繰り上がりが起きた際、`(st->slot_have & RHAVE_DATE)` が `0` のため `link_w2_next_day(t)` が呼ばれない。
  - **箇所**: `GPS_TFT_map/lora_link/link_wire2.h` (`link_w2_decode` L279-286, L321-326)
  - **推奨修正**: 日付値自体は有効か、あるいは途絶復旧時の `hour` 繰り上がりで保持中の日付を進められるよう `link_w2_next_day(t)` の条件を見直し（または `slot_have` クリア時に時刻/日付の整合再同期ルールを明確化）、`prev_tod` の無効化条件も `hour_known` と揃える。

- **BUG-73（優先度: 中）: `NAV-SAT` 受信処理の呼び出し前にフレーム長 (`used_len >= 8`) が検証されていない**
  - **現象**: 8 バイト未満の短い・破損した `UBX-NAV-SAT` フレームを受信した場合でも、`removeStaleSatellites()` が無条件で実行され、`ubx_buf[5]` がログ出力・NMEA 履歴バッファへそのまま参照される。
  - **原因**: `handle_navsat()` 冒頭には `if (len < 8) return;` のガードがあるが、呼び出し元の `gnss.cpp` 側では長さ確認前に `removeStaleSatellites()` を呼び、戻り後にも長さ未確認のまま `ubx_buf[5]` を `Serial.println` および `snprintf(..., "UBX-NAV-SAT svs=%d", ubx_buf[5])` に渡している。
  - **箇所**: `GPS_TFT_map/gnss.cpp` (`handle_navsat` L382-384, 呼び出し箇所 L837-850)
  - **推奨修正**: 呼び出し元で `used_len >= 8` を検証してから `removeStaleSatellites()`・`handle_navsat()`・ログ生成を行う（または `handle_navsat()` 内で長さと `numSvs` 整合を確認した上で stale 削除・通知を完結させる）。

---

### ブラッシュアップ項目（品質・保守性向上）

1. **ミリ秒以下（`nano`）の扱いとビットシフト記述の改善** (`GPS_TFT_map/gnss.cpp:267-268`)
   - `UBX-NAV-PVT` の `nano` が負（整数秒境界手前での丸め）のとき `ubx_cs` を無条件に `0` へクランプしており、時刻精度向上や丸め処理の余地がある。また `p[19] << 24` は `uint32_t` キャストを明示する方が C++ 規格上安全。
2. **マップフッター時刻表示の鮮度/切断考慮** (`GPS_TFT_map/display_tft.cpp:3930-3950`, `GPS_TFT_map/gnss.cpp:1513-1516, 1891`)
   - `draw_map_footer()` は `get_maxgs()` と並べて `get_gnss_time()` を無条件表示しており、`get_gnss_connection()` 断や時刻停止時も最後の時刻で固まったままになる（`update_maxgs()` も受信/ミラー経由の `ubx_hour`/`ubx_min` を参照）。状態に応じた外挿や非表示・警告色の検討余地がある。
3. **リプレイ機能まわりの状態初期化の整理** (`GPS_TFT_map/gnss.cpp:1690, 1944-1956`)
   - `first_time_logged_replay` が関数内 `static bool` のためリプレイ 2 回目以降にリセットされず、また `set_replaymode(false)` が `stored_numsats`・位置・高度・`ubx_hacc_mm`/`vacc_mm`/`sacc_mmps`・`gsa_*` をクリアしないため、通常復帰後しばらくリプレイ値が画面等に残存し得る。
4. **`gnss_dbd_after_setup()` の呼び出し条件と `s_gid_polls` 上限** (`GPS_TFT_map/gnss.cpp:1464-1482`, `GPS_TFT_map/gnss_dbd.cpp:988-996`)
   - `gnss_dbd_after_setup()` が `if (send_config)` 内にしかなく設定スキップ時に呼ばれない点、および GNSS ID ポーリングが `s_gid_polls >= 4` で打ち切りになり再試行手段がない点は堅牢化できる。
5. **12 衛星超過 PRN の表現と充電ポップアップのミラー整合** (`GPS_TFT_map/gnss.cpp:131, 492`, `GPS_TFT_map/display_pages.cpp:987`, `GPS_TFT_map/display_tft.cpp:3277`)
   - `GSA_MAX_PRN = 12`（表示側も最大 12 PRN）はマルチ GNSS 環境（M10 等で 12 機超を使用時）に制限となる。また `draw_charge_popup()` は `!get_gnss_connection()` のみで早期リターンしており、`gnss_disconnect_warning_shown()` 等と比べ `link_mirror_active()` の考慮が抜けている。

---

### 誤検出・仕様として除外した主な確認項目

以下の項目はコード・設計書・プロトコル仕様を再確認した結果、意図的仕様または既存管理済み TODO であると判断しリストから除外した。

- **DBD 起動時復元・PC 時刻投入・LoRa 時刻アシスト拡張・コールドスタート時 ANA-OFF の未実装/調整項目**: `docs/pons_todo.md` §2 および `docs/pons_ttff_assist_plan.md` で設計・管理済みの既知作業項目。
- **照度センサ異常と GNSS 切断が同時発生した際の無音・暗所誤判定等の扱い**: `docs/pons_todo.md` §3 で管理済みの既知 TODO。
- **UBX ハードコードコマンドのチェックサム整合性（`PRT`, `PVT`, `SAT2`, `SAT20`, `DOP`, `NAV5`, `ANA_ON`, `ANA_OFF`, `AOPS`, `RATE2HZ`）**: Fletcher-8 を再計算しすべて一致（チェックサム誤記なし）を確認済み。
- **`gnss_ubx_watchdog()` のバックオフ制御（`30000UL << ...`, `s_reconfig_streak` 上限 3）および `link_assist` の粗密精度閾値（`LINK_ASSIST_TACC_MAX_MS`, `LINK_ASSIST_TACC_FINE_MS`）**: 再初期化の連続発火防止と時刻品質切り分けのための意図的仕様。
- **`gnss_loop()` 内 `replaymode_gnssoff` 時のシリアル読み捨て処理**: リプレイ中に実モジュールの切断誤検知を防ぎつつ UART バッファ溢れを回避する意図的仕様（※終了時の挙動不備は `BUG-67` として分離）。
- **`rtc_backup.cpp` の BT1 電池寿命推定および `readiness.cpp` の `RDY_GNSS` プリフライト判定**: 設計通りの意図的仕様を確認。
- **シリアルコマンド経由の強制 `COLD`/`COLD,TIME` 投入（`handle_serial_command`, `gnss_dbd_command()`）**: TTFF 検証用の明示的デバッグ機能（※リセット直後の計測状態不備のみ `BUG-71` として分離）。
