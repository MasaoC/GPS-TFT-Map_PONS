# PONS v7.1 バグリスト ＆ 機能細分化リスト

- **最終監査日**: 2026-11-14（カテゴリ1〜9 全量監査＋誤検出・仕様セルフチェック実施）
- **対象ブランチ / バージョン**: `PONSv7.1` (`BUILDVERSION "1.06"`, `BUILDDATE 20261113`)
- **残存バグ件数**: **3 件**（`BUG-01`〜`127` まで、直したものは一覧から消した。`BUG-128`〜`130` が未修正。`BUG-74`〜`127` は実機未確認で、[pons_todo.md](pons_todo.md) §2 で管理）
- **整理方針**:
  - 直したものは一覧から消す（経緯はコードのコメントと git の履歴）。
  - **直さないと決めたもの**（バグ・ブラッシュアップとも）は、その場所のコードに
    `意図して直さない（YYYY-MM-DD の監査）: 理由` の 1 行コメントを残して一覧から消す。
    **監査のときはこのコメントを読み、同じものを挙げ直さないこと**（`grep -rn "意図して直さない"` で一覧になる）。
  - 誤検出も同じく、必要ならコードにコメントを残して消す。

---

## 目次

1. [未修正バグ一覧](#未修正バグ一覧)
2. [ブラッシュアップ・堅牢化候補](#ブラッシュアップ堅牢化候補)
3. [第1部：プロジェクト全体の機能別細分化リスト（全9大カテゴリ・42機能）](#第1部プロジェクト全体の機能別細分化リスト全9大カテゴリ42機能)

---

## 未修正バグ一覧

| ID | 重要度 | カテゴリ | 場所 (`ファイル:行`) | 現象・原因・影響 | 修正方針 |
| :--- | :---: | :--- | :--- | :--- | :--- |
| **BUG-128** | **中** | 9-1 (`flight_viewer`) | `tools/flight_viewer/index.html:1588-1597` | **`estimateSideslipBeta()` のヨー角 360° アンラップ計算で、累積角 `prevY` が `540°` を超えると JavaScript の負数剰余 (`% 360`) により 1 ステップで `-360°` のジャンプが発生する**: `let diff = (yVal - prevY + 540) % 360 - 180; prevY += diff;` で、`prevY` はアンラップ後の累積角である一方、`yVal` (`records[i].yaw`) は `[0, 360)` のラップ角である。時計回り（右旋回）で北 (`360° -> 0°`) を 2 回跨いで `prevY > yVal + 540` になると `yVal - prevY + 540 < 0` となり、JavaScript の `%` 演算子は負の剰余を返すため `diff` が本来より `-360°` 小さくなって `unwrappedYaw[i]` に `-360°` のステップ（2Hz ログで `psiDot ≒ -720 deg/s`）が生じ、推定横滑り角 `Est.Beta` が `±15.0°`（クリップ上限）へ張り付く。 | `let diff = (((yVal - prevY) % 360) + 540) % 360 - 180;` のように正の剰余へ正規化してから `[-180, 180)` に収める。 |
| **BUG-129** | **中** | 9-1 (`flight_viewer`) | `tools/flight_viewer/index.html:2177-2193` | **`drawVectorTriangle()`（風の三角形 HUD）で風速 `wspd > 0` のとき、対地移動ベクトル (`TT` 漆黒太線) が GNSS 実測の `(TrueTrack, gs)` ではなく `TAS + Wind` の合成先端 `(ttTargetX, ttTargetY)` を描画してしまう**: L2190–2191 で `d.tt` と `d.gs` から本来の対地ベクトル終点 `(ttDirectX, ttDirectY)` を計算していながら、L2192 で `(d.wspd !== null && d.wspd > 0) ? ttTargetX : ttDirectX` としている。① 実フライト CSV では風 (`wspd`, `wdir`) は直進中のみ更新される 120 秒 LPF 値、`TAS` は 10 秒平均ピッチ由来（または旧ログ/サンプルでは `gs - cos(yaw - wdir)*wspd` のスカラー近似）、さらに横滑り角 `estBeta` が加わるため、`TAS + Wind` (`ttTargetX, ttTargetY`) は瞬時の GNSS 対地速度ベクトル `(ttDirectX, ttDirectY)` と一致しない（初期画面の `loadSampleFlight()` でも HUD 数値は `Yaw: 328° / TT: 328°` なのに黒太矢印 `TT` は `316°` 方向へ 12° ずれて描画される）。② さらに `d.yaw === null` や `tas === null`（ヨー未確定時や姿勢なし受信ログ）や `d.wdir === null` でも `d.wspd > 0` だけで `ttTargetX, ttTargetY` が選ばれ、`TT` 矢印が消えるか風ベクトルと重なって描画される。 | 対地移動ベクトル (`TT`) の終点は常に `(ttDirectX, ttDirectY)` を描画する（また風ベクトル `WIND` も `tas !== null && tas > 0 && d.yaw !== null` のときのみ `tasX, tasY` から伸ばす）。 |
| **BUG-130** | **低** | 9-4 (KML/KMZ変換) | `tools/logcsv2kml4earth.py:36, 59`<br>`tools/logcsv2tourkmz.py:20, 62` | **`logcsv2kml4earth.py` と `logcsv2tourkmz.py` が旧 CSV 列名 `Altitude` をハードコードしており、v0.924 以降の PONS CSV で `KeyError: 'Altitude'` によりクラッシュする**: v0.924 以降の飛行 CSV（`mysd.cpp:2558`）では GNSS 高度列が `GNSS_Altitude`（およびバリオ高度 `KF_Altitude`）に改名されており、`createmovie_csv2mp4.py` 等は両対応化されているが、この 2 スクリプトは `row["Altitude"]` / `dropna(subset=[..., "Altitude", ...])` のまま残っている。 | `'KF_Altitude'` または `'GNSS_Altitude'` があればそれを使い、無ければ旧 `'Altitude'` を使うよう列名を自動判別する。 |

---

## ブラッシュアップ・堅牢化候補

1. **[Sound / Volume 0 Amplifier Control] 音量 0（ミュート）かつ最低音量保証なしのトーン再生時に PAM8302 アンプ (`PIN_AMP_SD`) が毎回 ON/OFF される**
   - **場所**: `src/sound.cpp` (L587–L593, L1080, L1121–L1125, L1149, L1160)
   - **内容**: WAV 再生側（`startPlayWav()` L587–593 および `amp_guard()` の `wav_audible` 判定 L1121）は、`sound_volume == 0 && min_volume == 0`（無音）のときにポップノイズ防止と省電力のためアンプ `PIN_AMP_SD` を ON にしない配慮が入っている。一方、トーン再生側（`startNextToneEntry()` L1080、`loop_tone()` の復帰 L1149 / L1160、`amp_guard()` の `sin_playing` 判定 L1122）は `sound_volume == 0 && tone_override_volume == 0`（ISR 出力が完全無音 `31` 固定）のときでも無条件に `setAmplifierState(true)` を呼んでアンプを ON にしているため、音量 0 設定中もボタン操作や旋回トーンのたびにアンプが ON/OFF される。
   - **改善案**: `const bool tone_audible = sin_playing && (sound_volume > 0 || tone_override_volume > 0);` とし、`tone_audible` のときのみアンプを ON にする。

2. **[VectorMap Builder / Polygon Ring Limit] `build_vectormap.py` の面レコード生成 (`IS_AREA`) でリング数上限 (`MAX_RINGS_PER_REC = 64`) と外周リング退化が検査されていない**
   - **場所**: `tools/vectormap/build_vectormap.py` (L713–L728, L759)
   - **内容**: 折れ線レコード側は `len(batch) >= MAX_RINGS_PER_REC` (`64`、ファーム側 `src/vectormap.cpp` の `VM_MAX_RINGS = 64` と同一) で分割しているが、面レコード側 (`IS_AREA(cls)`) は総点数 `sum(len(r) for r in rings) > max_pts` のみ検査して `len(rings) > MAX_RINGS_PER_REC` を検査していない。現在の琵琶湖・日本地図データでは 1 ポリゴン内の島・穴が 63 個を超える箇所は無いため実害は出ていないが、将来細かい湖沼や島嶼を含む面ポリゴンで `65 <= len(rings) <= 255` になると警告なしでバイナリに出力され、ファームウェア側 `vm_draw_record()` (`src/vectormap.cpp:176`: `if (nrings == 0 || nrings > VM_MAX_RINGS) return next;`) でポリゴン全体が無言でスキップされる（`256` 個以上では `struct.pack` が例外を出す）。また外周リング `poly.exterior` が量子化で 2 点以下に潰れて内周穴だけ残った場合も弾かれない。
   - **改善案**: 外周リングが 3 点未満ならスキップし、内周リングを含めて `len(rings) > MAX_RINGS_PER_REC` の場合は面積の大きい内周穴上位 `MAX_RINGS_PER_REC - 1` 個に絞る（または `DROPPED["area"]` に計上する）。

3. **[Flight Analysis & Movie Tools / Zero Pressure Guard] `plot_pitch_altitude.py` と `createmovie_csv2mp4.py` の気圧高度計算に気圧欠損行 (`pressure <= 300 hPa`) の除外ガードがない**
   - **場所**: `tools/flightanalysis/plot_pitch_altitude.py` (L100–L102), `tools/createmovie_csv2mp4.py` (L230–L232)
   - **内容**: `plot_phugoid.py` (L189)・`plot_phugoid_windows.py` (L155)・`plot_vertical_raw.py` (L119) では MS5611 未計測行（`pressure == 0`）を `_pm = d['pressure'].to_numpy() > 300.0` で除外して補間しているが、`plot_pitch_altitude.py` と `createmovie_csv2mp4.py` は `pressure` 列をそのまま `(pres / p_ref) ** (1.0 / 5.255)` に入れているため、CSV 内に 1 行でも `pressure == 0` があると気圧高度が `+44330 m` に跳ねて前後のローパス合成（`FUSE_TAU = 30s`）とドリフト補正が崩れる。
   - **改善案**: `plot_phugoid.py` と同様に `pressure > 300.0` の有効行のみで補間してから気圧高度を算出する。

---

## 第1部：プロジェクト全体の機能別細分化リスト（全9大カテゴリ・42機能）

### カテゴリ1：システム基盤・デュアルコア制御・死活監視・入力管理
- **1-1. グローバル設定・ハードウェア定義・コンパイル検査**: `settings.h`
- **1-2. 起動死活監視・XOSC待機・HardFault捕捉 (`bootwatch`)**: `bootwatch.h`, `bootwatch.cpp`
- **1-3. デュアルコア初期化 (`setup`/`setup1`)・メインループ (`loop`/`loop1`)**: `GPS_TFT_map.ino` (L272–449, L468–745, L2191–2500)
- **1-4. 画面モード遷移・設定放置タイムアウト・移動開始自動退場**: `GPS_TFT_map.ino` (L1545–1954, L2506–3083)
- **1-5. ボタンデバウンス（短押し/長押し/ダブルクリック）・設定メニュー定義**: `src/button.h`, `src/button.cpp`
- **1-6. WebSerial コマンドディスパッチ（`PONS:CONNECT`/`LIST`/`GET`/`PING`/`DBD`）**: `GPS_TFT_map.ino` (L1956–2184)

### カテゴリ2：GNSS測位・UBX制御・時刻管理・DBDアシスト
- **2-1. UBXプローブ・モジュール自動初期化・設定再送Watchdog**: `gnss.h`, `gnss.cpp` (L756–978, L1371–1767, L2261–2329, L2685–2699)
- **2-2. UBXフレームパーサ（`NAV-PVT`/`DOP`/`SAT`/`ORB`/`MON-RF`/`MGA-ACK`）**: `gnss.cpp` (L236–754, L997–1185, L1232–1360, L2461–2509, L2747–2761)
- **2-3. デモ飛行シミュレーション・リプレイ行適用・最高速(`MAX G/S`)集計**: `gnss.cpp` (L1770–2255, L2331–2459, L2519–2741)（関連: `mysd.cpp`, `display_tft.cpp`, `display_pages.cpp`, `power.cpp`, `GPS_TFT_map.ino`）
- **2-4. GNSS航法データベース (`MGA-DBD`) SD退避・復元・TTFF試験**: `gnss_dbd.h`, `gnss_dbd.cpp` (全 1304 行)（関連: `mysd.cpp`, `link_assist.cpp`）
- **2-5. RTCバックアップ電池 (`BT1`) 残量推定・SD永続化**: `rtc_backup.h`, `rtc_backup.cpp` (全 236 行)（関連: `mysd.cpp`, `power.cpp`）

### カテゴリ3：航法計算・目的地管理・コース逸脱警報・ベクタ地図
- **3-1. 球面三角法（Haversine距離・真方位・Mercator投影）**: `navdata.h`, `navdata.cpp` (L153–258, L304–329)
- **3-2. 目的地リスト管理・パイロン座標オーバーライド・Auto10km折返し制御**: `navdata.cpp` (L34–150, L600–693), `GPS_TFT_map.ino` (L3209–3301)
- **3-3. 航跡リングバッファ (`LatLonManager`)・旋回角速度・コース逸脱警報**: `navdata.cpp` (L260–302), `GPS_TFT_map.ino` (L3160–3448)
- **3-4. 内蔵OSMベクタ地図エンジン（LOD選択・タイルカリング・リング/折れ線描画）**: `src/vectormap.h`, `src/vectormap.cpp` (全 297 行), `src/flashdata/vectormap_data.cpp`, `src/flashdata/vectormap_data_hires.cpp`

### カテゴリ4：IMUドライバ(SCH16T)・姿勢推定ESKF・推定TAS・気圧バリオKF
- **4-1. SCH16T SPIドライバ（48bitフレーム・CRC8検査・自動復旧・250Hzポーリング）**: `imu_sensor.h`, `imu_sch16t.cpp` (全 853 行)
- **4-2. 12状態誤差状態カルマンフィルタ (`attitude` ESKF) 伝播・GNSS速度観測更新・重力観測**: `attitude.h`, `attitude.cpp` (L1–1168)
- **4-3. 機体取付ゼロ点較正 (`APPLY`)・自動ロールトリム・平均ピッチ (`Pavg`)・風推定**: `attitude.cpp` (L1170–1779)
- **4-4. ピッチ角連動の推定対気速度 (`TAS`)・ヘッダ大枠切替**: `tas.h`, `tas.cpp` (全 149 行)
- **4-5. MS5611 気圧センサ非ブロッキング計測・トリム平均・自動復旧**: `airdata.h`, `airdata.cpp` (全 788 行)
- **4-6. 3状態鉛直バリオカルマンフィルタ（加速度＋気圧高度＋GNSS高度/鉛直速度融合）**: `imu.h`, `imu.cpp` (全 874 行)

### カテゴリ5：TFT画面描画・マルチページUI・環境光調光・Readiness・電源管理
- **5-1. メイン地図画面レイヤ合成（ベクタ地図・ポリゴン・航跡・誘導線・風矢印・姿勢オーバーレイ）**: `display_tft.h`, `display_tft.cpp` (L1–2119, L4292–4586)
- **5-2. ヘッダ・フッタ・VSIバー・各種ポップアップバナー描画**: `display_tft.cpp` (L2844–4290)
- **5-3. 設定メニュー画面・SD詳細・リプレイ選択・地図一覧・過放電警告・PC接続画面**: `display_tft.cpp` (L2120–2843, L4592–5478)
- **5-4. 専門詳細画面（`IMU/TAS案内`・`VARIO`・`GNSS`・`CHSCAN`）**: `display_pages.cpp` (全 1256 行)
- **5-5. VEML7700 環境光センサ自動調光・バッテリーセーバー点滅・アイドル減光**: `backlight.h`, `backlight.cpp` (全 637 行)
- **5-6. 発進前準備完了判定 (`Readiness`)・READY/NOT READY音声通知**: `readiness.h`, `readiness.cpp` (全 178 行)
- **5-7. 電源監視・充電IC故障検知・過放電EEPROMラッチ・充電残量推定**: `power.h`, `power.cpp` (全 933 行)

### カテゴリ6：SDカード管理・Core1タスクキュー・リプレイ・生IMUロガー
- **6-1. Core0→Core1 タスクキュー基盤・クリティカルセクション制御**: `mysd.h`, `mysd.cpp` (L128–220)
- **6-2. SDIO/SPIハイブリッド初期化・ホットプラグ自動復旧・設定/CSV/ログ書込**: `mysd.cpp` (L221–1508, L2375–2959)
- **6-3. フライトCSVリプレイ再生エンジン（SPSCリングバッファ・カラム自動判別）**: `mysd.cpp` (L1509–2271)
- **6-4. SDブラウザ・WebSerialファイル一覧/ストリーミング転送**: `mysd.cpp` (L2272–2370, L2960–3174)
- **6-5. 生IMU/GNSS高レートバイナリロガー (`imulog` 二重バッファ)**: `src/imulog.h`, `src/imulog.cpp` (全 347 行)

### カテゴリ7：サウンド・音声再生・バリオ音制御
- **7-1. フラッシュ/SDハイブリッドWAV再生エンジン・排他/優先度割り込み・ハードウェアTimer ISR**: `src/sound.h`, `src/sound.cpp` (L1–993)
- **7-2. マルチトーン再生・旋回音 (`update_tone`)・バリオ音 (`update_vario`)・アンプ制御**: `src/sound.cpp` (L995–1338)

### カテゴリ8：LoRa無線リンク (E220-900T22S)・Wire2プロトコル・ミラーリング
- **8-1. E220-900T22S ドライバ（モード制御・AUX監視・レジスタ設定・RSSI取得）**: `e220.h`, `e220.cpp` (全 956 行)
- **8-2. PONS Link プロトコル定義・Wire2 20バイト回転スロット符号化/復号・ミニビーコン**: `lora_link/link_proto.h` (全 430 行), `lora_link/link_wire2.h` (全 486 行), `lora_link/link_beacon.h` (全 137 行)
- **8-3. 非ブロッキング送信状態機械・送信前キャリアチェック・Chスキャン・受信ミラーリング・起動時アシスト窓**: `link.h`, `link.cpp` (全 2059 行), `link_assist.h`, `link_assist.cpp` (全 527 行)

### カテゴリ9：PC・Web解析ツール・アセット生成・単体テスト群
- **9-1. WebSerial対応フライトビューア (`PONS Flight Viewer & Player`)**: `tools/flight_viewer/index.html` (全 3972 行)
- **9-2. OSMベクタ地図生成ツール**: `tools/vectormap/build_vectormap.py` (全 1143 行)
- **9-3. フラッシュ内蔵データ生成・画像/音声変換スクリプト群**: `tools/gen_wav_flash.py` (全 436 行), `tools/gen_guide_flash.py` (全 229 行), `tools/gen_logo_flash.py` (全 178 行), `tools/gen_link_icons.py` (全 168 行), `tools/clean_wav8.py` (全 189 行), `tools/png2bmp.py` (全 98 行)
- **9-4. 動画生成・フライト前処理・KML/KMZ変換スクリプト群**: `tools/createmovie_csv2mp4.py` (全 922 行), `tools/flight_preprocess.py` (全 883 行), `tools/kml_to_mapcsv.py` (全 53 行), `tools/logcsv2kml4earth.py` (全 78 行), `tools/logcsv2tourkmz.py` (全 94 行)
- **9-5. オフライン生IMUログ解析・Python版ESKF/バリオKF検証ツール群**: `tools/imulog/decode_imulog.py` (全 402 行), `tools/imulog/eskf.py` (全 772 行), `tools/imulog/merge_flight.py` (全 207 行), `tools/imulog/plot_response.py` (全 329 行), `tools/imulog/road_geometry.py` (全 149 行), `tools/imulog/test_eskf.py` (全 215 行), `tools/imulog/vario.py` (全 329 行), `tools/imulog/yaw_kml.py` (全 101 行)
- **9-6. フライトダイナミクス解析スクリプト群 (`flightanalysis`)**: `tools/flightanalysis/plot_instability.py` (全 209 行), `tools/flightanalysis/plot_lateral_modes.py` (全 724 行), `tools/flightanalysis/plot_phugoid.py` (全 536 行), `tools/flightanalysis/plot_phugoid_windows.py` (全 429 行), `tools/flightanalysis/plot_pitch_altitude.py` (全 248 行), `tools/flightanalysis/plot_spectrogram.py` (全 181 行), `tools/flightanalysis/plot_spectrum.py` (全 170 行), `tools/flightanalysis/plot_vertical_accel.py` (全 677 行), `tools/flightanalysis/plot_vertical_raw.py` (全 280 行)
- **9-7. ホスト単体テスト (`attitude_test`)・SCH16T実機検証スケッチ (`sch16t_test`)**: `tools/attitude_test/*` (`main.cpp` 113 行, `yaw_test.cpp` 515 行, `dead_test.cpp` 144 行, `Arduino.h` 15 行, `testclock.cpp` 2 行, `settings.h` 38 行, `Makefile` 20 行, `README.md` 80 行), `tools/sch16t_test/sch16t_test.ino` (全 1495 行)
