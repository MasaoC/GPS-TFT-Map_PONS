# PONS v7.1 全機能細分化リスト ＆ コード監査バグリスト

- **監査日**: 2026-11-10
- **対象ブランチ / バージョン**: `PONSv7.1` (`BUILDVERSION "1.04"`, `BUILDDATE 20261107`)
- **監査対象**:
  - コアファームウェア全 28 ファイル（約 33,405 行）
  - PC / Web ツール・生成スクリプト・単体テスト全 35 ファイル（約 14,904 行）

---

## 第0部：検証結果（2026-11-10。全 72 件を検証済み）

1 件ずつコードを読んで確かめた結果。「誤検出」は直していない（理由を書いた）。実機では未確認。

**PONSv7 への移植（2026-11-10）**: 基板に関係しない直しは v7（0.999v7）へ手で移植した（コンパイル・attitude_test・test_eskf.py は通過）。
**持ち込まなかったもの**: BUG-19（BNO085 はまとめて届くので v7 は 64 サンプルの平均のまま）、BUG-15・16・17・18（SCH16T のドライバ）、
BUG-46（「SCH16T を必ずつなぐ」は v7.1 の決めごと）、BUG-39（v7 には設定画面の過放電の行が無い）、BUG-44（v7 には照度センサーが無い）、
BUG-47（v7 には充電の残量推定のログが無い）、BUG-58 の過放電の警報の停止（v7 に無い）。BUG-23 の「途絶明けに平均ピッチの窓を空ける」は
v7 の作りに合わせて書き直した。flight viewer は v7 も v7.1 のものを使うので移植は要らない。
ファームは arduino-cli でコンパイル、`attitude.cpp` は `tools/attitude_test` の 3 本が PASS（出力は変更前と同じ）。

| ID | 判定 | 結果 |
|---|---|---|
| BUG-06 | ほぼ誤検出 | `update_moving()` は `get_gnss_connection()` も見ており、GNSS が生きていれば fix なしの NAV-PVT が 0.5 秒で上書きする。黙った GNSS のときだけ残るので、日時と同じく位置・速度もリプレイを抜けたら落とすようにした（gnss.cpp `set_replaymode`） |
| BUG-07 | 本当 | ミラー中は電波のフレームだけを刻みにし、送信機が fix していないフレームでは位置を進めない（(0,0) で Auto10km が復路へ切り替わり SD に保存していた）。ボートの NAV-PVT で刻みを立てない。航跡の間隔は `get_gnss_mps()`。`try_enque_savecsv()` でもミラー中は航跡を足さない |
| BUG-08 | 本当 | 入れ直しが済んだ・要らないと決まったら `ST_IDLE` へ。入れ直し待ち（`ST_WAIT`）でも試験コマンドは新しい試験として受ける |
| BUG-19 | 本当（影響は過大） | 回転は「永久に 88% 失う」のではなく続く 0.25 秒へ薄まる。ユーザー確認のうえ dt を実際のサンプル間隔に（eskf.py と同じ。0 以下・0.5 秒超は伝播しない） |
| BUG-20 | 本当（影響は過大） | 待機中の維持で数秒で戻るので「永久に飛ぶ」は誤り。ユーザー確認のうえ、走行中の保険の初期化は sAcc ≤ `ESKF_SACC_MAX` かつ 3m/s 以上のときだけ |
| BUG-23 | 本当 | `attitude_pitch_avg_valid()` / `attitude_get_tas()` / `attitude_get_wind()` が `attitude_ready()` を見る（無線・CSV の TAS・画面が一斉に従う）。途絶明けは平均ピッチの窓を空ける |
| BUG-28 | 本当 | `vario_vspeed_mps()` が `vario_source_ok()` を見る（MS5611 が途絶えたら音も 0）。無線は `vario_source_ok()` のときだけ高度・V/S・気圧を載せる |
| BUG-29 | 本当 | `vario_*` 3 関数にリプレイの分岐（CSV の KF_Vspeed だけを見る）。無線の V/S は `vario_vspeed_mps()`（機体の画面と同じ値） |
| BUG-31 | 誤検出 | 毎フレーム `draw_compass()` などが `AA_FONT_SMALL` を読み直してから `draw_gs_track()` に入る。`draw_notice_2lines()` だけ約束どおり地図用フォントで抜けるようにした（見た目は変わらない） |
| BUG-38 | 本当（診断画面なので実害は小） | ページ 0 のコース座標 6 行（`MAPLIST_COURSE_ROWS`）を数える |
| BUG-40 | 本当 | 見出しの字形が前の画面の残したフォントで決まっていた。`header_font_detail()` で `AA_FONT_SMALL` を明示（ふだん見えていた形）。TAS 曲線の画面は抜けるときに `unloadFont()` しない |
| BUG-46 | 本当（別の穴も） | `get_imu_ok()` は途絶 1 秒で false になるので、黙った IMU も「載っていない」扱いで READY と言っていた。ユーザー確認で「v7.1 は SCH16T を必ずつなぐ」→ 起動時に無いのも黙ったのも RDY_IMU・設定画面は赤の × |
| BUG-53 | 本当（窓は 1µs 程度） | `init_replay()` の巻き戻しと `replay_pop()` を `taskQueueMutex` で守る |
| BUG-56 | 誤検出 | `imulog_push()` は `imulog_paused` を見ている。止めた瞬間に書き出し待ちも捨てる |
| BUG-57 | (1) 本当 / (2) 誤検出 | (1) 3ms 待った後にもう一度確かめてからアンプを切る＋Core1 の安全弁（鳴らしている WAV・トーンがあるのにアンプが切れていたら戻す）。(2) `vario_inhibit` は settings.txt の読み込みでしか変わらない |
| BUG-62 | 本当 | 列名 `tas_mps` を探す |
| BUG-63 | (2) 本当 / (1) 意図どおり / (3) ほぼ起きない | ダイアログを閉じたら本体は 60 秒で地図へ戻る作りは意図どおり（ユーザー確認）。開き直したら一覧を取り直す。ヘッダーボタンの表示を動作（一覧を開く）に合わせた。転送中の CANCEL_OK / ERROR も拾う |
| BUG-65 | 本当 | pbf から取れるクラス（`PBF_TAGS`）だけを返す。空のクラスが走査キャッシュに焼かれていた |
| BUG-67 | 誤検出（クラッシュしない） | Pillow 12.3 でも `Image.FLIP_TOP_BOTTOM` はある（削除は撤回された）。使っていない `import requests` を外し、`clean_wav8.py` の一時ファイルを `tempfile` に |
| BUG-68 | 一部本当 | 2026-07-26 専用の台本で、その CSV（v0.92）の列名は `Altitude`。両方の列名・小数秒・起動直後（10 秒未満）の `N:SD INIT` に対応。(4) 風向は自分の出力ファイル内で「吹いてくる向き」に一貫しており誤検出 |
| BUG-71 | 本当 | ユーザー確認のうえ 3 本とも `pressure` から気圧高度を出す（GNSS 高度を「気圧高度」と呼んでいた）。`np.trapz` は NumPy 2.0 で削除ではなく非推奨だが、`trapezoid` と両対応にした |
| **Medium-High** | | |
| BUG-15 | 本当（どれも 1 サンプル＝4ms ぶん） | ドライバが同じサンプルの加速度を先に渡す（`imu_sch16t.cpp`）。3 点（1 サンプル前の加速度・最初のサンプルの漏れ・復旧直後の古い加速度）が同時に直る。順番は `imu_sensor.h` の約束に書いた |
| BUG-26 | (1) 本当 / (2) 一部 | (1) ADC の 0（＝変換していない）と 0xFFFFFF は I2C の失敗として数え、300〜1100hPa を外れる気圧は基準高度にも値にも使わない（起動して最初の 1 回で踏むと基準高度が NaN で固まっていた）。(2) 窓の途中の瞬時値は VARIO DETAIL の表示用で意図どおり（KF へは窓のトリム平均だけ）。無効な瞬時値だけ入れないようにした |
| BUG-59 | (1) 潜在・保留 / (2) 誤検出 | (1) ブロッキング版 `e220_wake()` は起床の判定が弱いが、直後に書き込む経路が無い（電波は `e220_wake_ready()` を通る）。ユーザー確認で保留し、`e220_wake()` と docs/pons_link.md §6.1 に注意を書いた。(2) モードの往復は無害で実機で確かめた手順。受信のゴミは同期探しで捨てられ、mode 0 で受信バッファを捨てるのは §6.8 の禁止事項 |
| **Medium** | | |
| BUG-02 | 誤検出 | `get_gnss_altitude()` にデモの分岐は無く、デモ中も本物の GNSS 高度（引用された `10 + 10*sin` の式は無い） |
| BUG-03 | 本当 | IMU/ESKF 画面を出るとき（Exit・動き出して地図へ）は案内の音声のグループを必ず止める（最後の 1 文「…APPLY してください」が地図の上で鳴り続けた） |
| BUG-04 | 本当（小） | PC 接続のまま CONNECT を受けても戻り先を上書きしない。前の画面の後始末は、抜けると元の画面へ戻る作りなので要らない |
| BUG-09 | 一部本当 | 打ち切りでも送りかけのフレームは書き切る（次のコマンドが捨てられた）。COLD でキューが満杯でも "not restored" を残す。ST_WAIT / ST_TEST_LOAD を打ち切らないのは意図どおり |
| BUG-10 | 誤検出 | `GNSS_DBD_LOAD_WAIT_MS` は 20000ms で、固有 ID のポールの再送（約 7・12・17 秒）は窓に入る |
| BUG-12 | 本当 | `init_destinations()` は内蔵の番号（0〜3）を選んでいたら残す（飛行中に SD を掴み直すとパイロットの選んだ目的地が PLATHOME に変わった） |
| BUG-14 | 本当 | コース逸脱の積算は fix も見る（`get_gnss_fix()` は供給元ごとに答える）。GNSS 断線時は NAV-PVT が来ないので呼ばれない（リストのその点は誤り） |
| BUG-16 | 本当 | ユーザー確認のうえ、CRC の合うフレームが 50ms 来なければ居ないと見切る（`SCH16T_READY_NOFRAME_MS`）。飛行中に黙った SCH16T で Core0 が 1 分ごとに約 1 秒止まっていた |
| BUG-17 | (1)(2) 本当 / (3) 誤検出 | `set_dry()` と初期化の確認の読み出しで CRC を確かめる。(3) の「present は変わらない」という約束は今の imu_sensor.h に無い（BUG-46 で readiness 側を直した） |
| BUG-21 | 本当（数値の差は小） | C++ の `+Q` を重力観測より前へ（eskf.py と同じ教科書の順）。ヨーレートの回転は機上に合わせて eskf.py を伝播前の姿勢に。eskf.py の test も PASS |
| BUG-24 | 本当 | APPLY で平均ピッチの窓をオフセットの変わった分だけずらす（捨てずに、その場で新しい基準に）。風の LPF は地上で捨てられるので触らない |
| BUG-32 | 一部本当 | 供給元が替わったらヨーのヒステリシスを捨てる。`held` は 2 つの関数で別々（共有ではない）。分岐の順の指摘は、デモ中はミラーしないので誤検出 |
| BUG-33 | 誤検出 | デモが差し替えるのは GNSS とヨー精度だけで姿勢は実センサー（CLAUDE.md）。ESKF 未収束なら "ESKF Not ready" が正しい。風の設定 OFF を尊重するのも意図どおり |
| BUG-36 | 本当（範囲は広い） | 10 倍して丸めてから桁を分ける。10° の手前だけでなく x.95〜x.99 がすべて x.9 になっていた |
| BUG-37 | 本当 | GS・TT の赤線を `gnss_speed_track_invalid()` に集約（実機は断線・fix なし・衛星 0、ミラーは送信機の断線・fix、デモとリプレイは衛星 0 だけ） |
| BUG-39 | (2) 本当 / (1) 意図どおり | SW OFF のまま USB では "OVER-DISCHARGED (SW OFF)"（0.00V を出していた）。稼働中のラッチを設定画面に出さないのはコメントどおりの意図 |
| BUG-41 | ページ 1 本当 / ページ 2 誤検出 | ミラー中の GNSS DETAIL 1 は衛星数と fix も受信機自身の値（"RX:"）。ページ 2 は全部受信機自身の値で混ざっていない |
| BUG-42 | 本当 | ミラー中の KF 高度の MSL 判定は機体の fix と KF 高度で。GND/ISA に (RX) |
| BUG-44 | 本当（表示だけ） | 照度センサー無しは減光より先に判定（点灯率はどちらも 5% で同じ） |
| BUG-50 | 本当 | 最後の頂点でも経度のあとのカンマを必須に、空の緯度も弾く（緯度 0 の頂点＝赤道まで伸びる線ができた） |
| BUG-54 | 本当 | `browse_sd()` の早い return でも `loading_sddetail` を落とす |
| BUG-55 | 誤検出 | SdFat 2.3.1 の `getName()` は入らないとき先頭を '\0' にして 0 を返す（未初期化は読まない） |
| BUG-60 | (1) 本当 / (2) 直さない | 0 時をまたいだら日付も 1 日進める（JST の深夜に受信しているときだけ）。時刻が無効なとき前の時刻を持ち続けるのは、0 時に飛ぶよりよいので残す |
| BUG-61 | 本当 | グループ・SF を変えたら受信の組み立てと「別グループを見た」を捨てる（CH は前から捨てていた） |
| BUG-64 | 本当 | 時刻を先頭 0 にそろえる、ファイル名をエスケープし「開く」は番号で呼ぶ（FAT の名前の ' で壊れた）、findLastIndex の代わりのループ |
| BUG-66 | 本当（警告だけ） | 作らなかった LOD を警告する（ドキュメントの簡易の例がこの使い方なので書き込みは止めない） |
| BUG-69 | 本当 | KML の 2 要素の座標を読む。KML の時刻は JST（+09:00）。'Z' で 9 時間ずれていた |
| BUG-70 | 本当 | (1) eskf.py にヨー分散の上限（BUG-21 と一緒に）、(2) iTOW の差を 1 週で割った余りに、(3) merge_flight の長さ 0 と古い列名・無い列 |
| **Low-Medium** | | |
| BUG-58 | (3) 本当 / (1) ほぼ意図どおり / (2) 既知 | (3) 末尾以外で止めたとき（アンダーランの打ち切り・過放電の警報の停止）も pending を再開（`resume_pending_if_idle()`）。(1) 無音の WAV は意図した作りで、前の WAV が入れていたアンプだけ切るようにした。(2) は docs/pons_sound.md §5 で受け入れている振る舞い |
| BUG-72 | (1) 本当 / (2)(3) 誤検出 | (1) dead_test の比力を水平（{-g,0,0}）に（{0,0,g} は機首が真上／真下でロールが定まらず、確認が効いていなかった）。(2) attitude_test/settings.h は冒頭に「使われていない」と明記され、どこも include していない。(3) 不良フレームで t_prev を進めないのはコメントどおりの意図 |
| **Low** | | |
| BUG-01 | 誤検出 | 今の値は `BOOTWATCH_WDT_MS` = 6000・`BOOTWATCH_STUCK_MS` = 1000（リストの 4000・3000 ではない） |
| BUG-05 | 予防（害なし） | 時刻の範囲も確かめる（UBX のチェックサムを通った validTime なら本来範囲内） |
| BUG-11 | 実害なし | 時刻が有効になった最初の 1 回だけ取り、ずれは最大 0.5 秒。BT1 の時間の推定には効かない |
| BUG-13 | 誤検出 | 目的地は名前で保存され、読み込みは名前を引いて有効な番号だけ入れる。auto10k は INTO / AWAY に正規化済み |
| BUG-18 | 本当（診断だけ） | 復旧したら読み出しの間隔とレートの計測を起点から取り直す（gapmax= に途絶えた時間が入っていた） |
| BUG-22 | 誤検出 | 静止判定は生のジャイロで見る（ゼロ点の学習の前提なので、引いた値で見ると循環する）。eskf.py も同じ使い分け |
| BUG-25 | 本当（設定の書き間違い） | 読み込み後に min ≥ max なら両方を既定へ戻してログに残す |
| BUG-27 | 誤検出（コメントだけ直した） | CRC4 は実装済み（`ms5611_crc4()`）。トリム平均の古いコメント（15%）を実際の「上下 2 つずつ」に |
| BUG-30 | 本当（コメント） | 式（前進オイラー）はそのままにして、コメントを `B = [0, dt, 0]` に合わせた（バリオはこの式で調整済み） |
| BUG-34 | 実害なし（揃えた） | 唯一の呼び出し元は現在地を中心に渡すので同じ値。引数の中心で絞るように揃えた |
| BUG-35 | 本当（境界） / 負の剰余は誤検出 | `isOutsideTft()` を `>=` に（240 は画面の外）。`detail_page` は負にならない |
| BUG-43 | 本当 | 引き算して比べる（起動直後の 1 秒と millis の一周で全部が灰色になった） |
| BUG-45 | 誤検出 | `veml_init()` が t0 を取ってから少なくとも待つので空振りしない（しても次の tick で読む） |
| BUG-47 | 本当（ログだけ） | CV ENTER / FULL の印は新しい充電（USB の挿し直し）でだけ捨てる |
| BUG-48 | 予防（害なし） | `now ? now : 1` に揃えた（起動直後の 1ms か millis の一周でしか起きない） |
| BUG-49 | 本当 | 設定の読み込み（Core1）の ERR は `log_sdf()` で直接書く（キューが溢れて起動時の音まで捨てられ得た） |
| BUG-51 | 本当 | `log_sd()` のバッファを本文 256 バイト＋起動秒に（128 バイトで切っていた） |
| BUG-52 | 本当 | 日時が決まる前のファイルの FAT 日時は 2020-01-01（0 年をそのまま通して回り込んだ日付が付いていた） |

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

## 第2部：バグ一覧サマリー表（全72件）

| ID | 機能 | 重要度 | 対象ファイル (行番号) | 概要 |
|---|---|---|---|---|
| **BUG-01** | 1-2 | Low | `bootwatch.cpp` (L357–396) | Core0 起動遅延時、Core1 の診断ダンプ中に 4000ms WDT が発火してログが切れる可能性 |
| **BUG-02** | 1-3 | Medium | `GPS_TFT_map.ino` (L1745–1754) | デモ飛行中に `!is_demo_active()` ガードが無く、仮想高度で実機の気圧基準と KF が汚染される |
| **BUG-03** | 1-4 | Medium | `GPS_TFT_map.ino` (L1826, L2424–2428, L2470) | `imu_exit()`・移動退場時に IMU 案内 3 ページ完了後の `guide_eskf_apply.wav` が止まらない |
| **BUG-04** | 1-6 | Medium | `GPS_TFT_map.ino` (L1874–1883) | `PONS:CONNECT` 受信時に前画面の後始末が無く、再受信で `prev_screen_mode` が上書きされる |
| **BUG-05** | 2-2 | Low | `gnss.cpp` (L998–1005) | `handle_navpvt()` で時刻（`hour`/`min`/`sec`）の範囲検証が無い |
| **BUG-06** | 2-3 | High | `gnss.cpp` (L1820–1824), `power.cpp` (L354–362, L464) | リプレイ終了時に `ubx_pos_valid`・`stored_gs` 等がクリアされず、地上で移動中と誤判定し SW OFF 警告が鳴らない |
| **BUG-07** | 2-3 | High | `gnss.cpp` (L1695, L1772–1775, L2005–2008) | 無線ミラー中に (1) 送信機未Fixの `(0,0)` で Auto10km が復路へ誤遷移、(2) ボート GNSS が 2Hz で `newcourse_arrived` を立て旋回角速度が狂う、(3) 航跡間隔がボートの速度で決まる |
| **BUG-08** | 2-4 | High | `gnss_dbd.cpp` (L370, L703–715, L921) | 屋内（未Fix）で `PONS:DBD,COLD` 等を実行すると最大30分間 `ST_WAIT` に張り付き全コマンドが `BUSY` になる |
| **BUG-09** | 2-4 | Medium | `gnss_dbd.cpp` (L374–386, L951–957) | `abort_op()` が送信途中の UBX フレームを切断し、`ST_WAIT`/`ST_TEST_LOAD` を解除しない |
| **BUG-10** | 2-4 | Medium | `gnss_dbd.cpp` (L779–783, L829–832) | 起動時の `SEC-UNIQID` 再送（5秒周期）より先に `ST_BOOT` の 5000ms 待機が切れて復元がスキップされる |
| **BUG-11** | 2-5 | Low | `rtc_backup.cpp` (L80–89) | `capture_utc()` が `get_gnss_fix_millis()` ではなく現在時刻 `now` を引いて `s_utc0` を算出する |
| **BUG-12** | 3-2 | Medium | `navdata.cpp` (L117), `mysd.cpp` (L1353–1361) | 飛行中の SD 再初期化で `init_destinations()` が呼ばれると `currentdestination` が `0` にリセットされる |
| **BUG-13** | 3-2 | Low | `mysd.cpp` (L361–380), `navdata.cpp` (L50–58, L673) | `currentdestination` の負値チェック漏れ、および `auto10k_status` の範囲外値に対する正規化漏れ |
| **BUG-14** | 3-3 | Medium | `GPS_TFT_map.ino` (L3137) | `update_course_warning()` が `!get_gnss_fix()` を確認せず、GNSS 断線時や 1〜3 衛星未Fix時に警報がリセットされない |
| **BUG-15** | 4-1 | Medium-High | `imu_sch16t.cpp` (L408–409), `attitude.cpp` (L612–650) | `read_sample()` が `imu_feed_gyro` → `imu_feed_accel` の順で呼ぶため、常に1サンプル古い加速度を参照する |
| **BUG-16** | 4-1 | Medium | `imu_sch16t.cpp` (L216–226, L246–267) | 飛行中に SCH16T が無応答になると、60秒ごとの `try_recovery()` が Core0 を最大 1,046ms ブロックする |
| **BUG-17** | 4-1 | Medium | `imu_sch16t.cpp` (L231–238, L315–325, L507) | `set_dry()` と `sch_init()` のレジスタ検証で `frame_integrity_ok()`（CRC8検査）が呼ばれていない＋タイムアウト時の `s_present = false` 契約違反 |
| **BUG-18** | 4-1 | Low | `imu_sch16t.cpp` (L254–262, L520–553) | `try_recovery()` 成功時にポーリング時刻・レートカウンタがリセットされず偽の `gapmax` と低レート警告が出る |
| **BUG-19** | 4-2 | High | `attitude.cpp` (L225–236, L634–644, L707) | SCH16T（FIFOなし）に 64サンプル窓平均 `dt_est_` を適用しており、Core0 ブロック中の回転角が最大 88% ロストし `eskf.py` と乖離する |
| **BUG-20** | 4-2 | High | `attitude.cpp` (L1101–1125) | `attitude_on_gnss_velocity()` の移動中フォールバック初期化に `sAcc` ゲートと速度閾値が無く、地上で1サンプル粗初期化が起きる |
| **BUG-21** | 4-2 | Medium | `attitude.cpp` (L748–760, L782–799, L923–932) | `attitude_on_gyro()` でプロセスノイズ `+ Q` を加算する前に `level_update()` を呼んでおり `eskf.py` と不一致 |
| **BUG-22** | 4-2 | Low | `attitude.cpp` (L688 vs L815–816) | 地上再水平化 `ground_hold` のジャイロノルム判定が生値 `g` を使っており `update_static()`（バイアス除去後 `w`）と不統一 |
| **BUG-23** | 4-3 | High | `attitude.cpp` (L1260–1324), `tas.cpp` (L81–88), `link.cpp` (L1679–1696), `mysd.cpp` (L876–879) | IMU 途絶時（`!attitude_ready()`）に凍結した `Pavg`・`TAS`・`Wind` が有効のまま無線と CSV に出力され続ける |
| **BUG-24** | 4-3 | Medium | `attitude.cpp` (L1398–1419) | `attitude_calibrate_to()`（APPLY）実行時に 10秒平均ピッチ窓 `pitch_avg_reset()` と風 LPF がリセットされない |
| **BUG-25** | 4-3 | Low | `attitude.cpp` (L1293–1305), `mysd.cpp` (L628–642) | `airspeed_min` と `airspeed_max` の大小逆転（`min > max`）ガードが無い |
| **BUG-26** | 4-5 | Medium-High | `airdata.cpp` (L251–253, L365–435, L511–530) | MS5611 の ADC 値 `0`/`0xFFFFFF` 未検査による `NaN` 発生・起動時ラッチリスク、およびウィンドウ未完了時の `last_altitude` 生値上書き |
| **BUG-27** | 4-5 | Low | `airdata.cpp` (L198–218, L380–382, L563–566) | MS5611 PROM CRC4 検証未実装、および `0` 番兵・旧定数コメント残り |
| **BUG-28** | 4-6 | High | `imu.cpp` (L541–560, L668), `src/sound.cpp` (L1215–1222), `link.cpp` (L1653–1658) | 飛行中に MS5611 が途絶するとバリオ KF が発散し、バリオ音が最高音で鳴り続け無線にも発散値が送出される |
| **BUG-29** | 4-6 | High | `imu.cpp` (L541–576), `link.cpp` (L1654) | `vario_source_ok()`・`vario_vspeed_mps()`・`vario_deadband_mps()` にリプレイ分岐が無い＋IMU故障時の機体表示と無線送信 V/S の不一致 |
| **BUG-30** | 4-6 | Low | `imu.cpp` (L25–30, L198–227) | `kf_predict()` のコード（オイラー積分）とコメント記載の状態遷移式（$\frac{1}{2}a\Delta t^2$ 項あり）の不一致 |
| **BUG-31** | 5-1 | High | `display_tft.cpp` (L2951–2959, L3083–3093, L3366) | `draw_notice_2lines()` 等が `backscreen.unloadFont()` 後に `AA_FONT_SMALL` を戻さず、次フレームの `"GS"`, `"Pavg"`, `"TT"` が極小フォントに化ける |
| **BUG-32** | 5-1 | Medium | `display_tft.cpp` (L576–579, L712–729) | 表示4系統間で `eskf_yaw_reliable()` 等のヒステリシス `static bool held` が共有され、また `eskf_display_yaw()` のデモ/ミラー優先順が逆転 |
| **BUG-33** | 5-1 | Medium | `display_tft.cpp` (L1750–1755, L1813) | 実機 ESKF が未収束（`!attitude_ready()`）のときデモモードの姿勢が `"ESKF Not ready"` で潰され、風設定 OFF 時にデモの風が出ない |
| **BUG-34** | 5-1 | Low | `display_tft.cpp` (L1224–1225, L1237) | `draw_ExtraMaps()` / `draw_FlashMaps()` の範囲カリングが引数 `center_lat/lon` ではなく `get_gnss_lat()/lon()` を参照 |
| **BUG-35** | 5-1 | Low | `display_tft.h` (L53–55), `display_tft.cpp` (L4476) | `cord_tft::isOutsideTft()` の `>= BACKSCREEN_SIZE` オフバイワンと `draw_sddetail()` の負ページ剰余 |
| **BUG-36** | 5-2 | Medium | `display_tft.cpp` (L3616–3625) | `draw_header_pitch()` で `9.95° <= \|pitch\| < 10.0°` のときに `+10` ではなく `+9.9` にクランプされる |
| **BUG-37** | 5-2 | Medium | `display_tft.cpp` (L3419, L3790) | GS・TT の赤取り消し線が `get_gnss_numsat() == 0` のみで判定され、衛星 1〜3 個の未Fix時や GNSS 断線時に赤線が引かれない |
| **BUG-38** | 5-3 | High | `display_tft.cpp` (L4671–4765) | `draw_maplist_mode()` でページ0の `COURSE` 6行分がページ総数・開始インデックス計算から漏れ、SD地図6件が画面外へ溢れてスキップされる |
| **BUG-39** | 5-3 | Medium | `display_tft.cpp` (L4824, L4939–4944) | 設定画面の電池状態が `power_overdischarge_boot_latched()` しか見ず稼働中ラッチを見落とす＋SW OFF の USB 給電中に `0.00V` を表示 |
| **BUG-40** | 5-4 | High | `display_pages.cpp` (L201, L613, L717, L798, L829, L1049), `display_tft.cpp` (L2146 等) | `header_footer` で `unloadFont()` なしに `setTextSize(2)` を呼んでおり、`draw_tas_curve()` 閲覧直後だけ全詳細画面のヘッダが巨大フォントに急変する |
| **BUG-41** | 5-4 | Medium | `display_pages.cpp` (L713–821) | 無線ミラー中の `GNSS DETAIL 1/3, 2/3` で送信機の衛星数/Fix と受信機の時刻/スカイプロットが警告なく混在 |
| **BUG-42** | 5-4 | Medium | `display_pages.cpp` (L365–393) | 無線ミラー中の `VARIO DETAIL 1/2` で `_msl_ready` が受信機自身の GNSS を参照し、送信機の MSL 高度に `(AGL)` と誤表示する |
| **BUG-43** | 5-4 | Low | `display_pages.cpp` (L810) | `draw_gnss_page2()` の `millis() - 1000` 符号なしアンダーフロー |
| **BUG-44** | 5-5 | Medium | `backlight.cpp` (L428–555, L614–621) | VEML7700 未接続（`!s_als_ok`）時にも 30秒静止で `s_dimmed = true` になり、表示が `"NO SENSOR"` から `"IDLE DIM"` に書き換わる |
| **BUG-45** | 5-5 | Low | `backlight.cpp` (L349, L566–573) | `backlight_als_setup()` の `delay(ALS_FIRST_READ_MS)` 直後の `millis()` 境界で初回測光が空振りする可能性 |
| **BUG-46** | 5-6 | High | `readiness.cpp` (L50–58, L72–85), `src/button.cpp` (L592) | 起動時に SCH16T の初期化が失敗（`!get_imu_ok()`）しても `RDY_IMU` が立たず「準備完了（READY）」と発声してしまう |
| **BUG-47** | 5-7 | Low | `power.cpp` (L728–796) | 充電中に一瞬 SW OFF → ON すると `CHG START` / `CHG CV ENTER` / `CHG FULL` が古い経過時間で重複ログされる |
| **BUG-48** | 5-7 | Low | `power.cpp` (L480, L514, L559) | タイマー起点保持の `now ? now : 1` ガード漏れ |
| **BUG-49** | 6-1 | Low | `mysd.cpp` (L337, L344, L349, L371, L377, L454, L472) | Core1 の `loadSettings()` 内から `log_sdf()` ではなく Core0 用の `enqueueTask(createLogSdfTask(...))` を呼んでいる |
| **BUG-50** | 6-2 | Medium | `mysd.cpp` (L1049) | `process_mapcsv_line()` で最終頂点 `i == size - 1` の経度・緯度間カンマ欠落チェックがスキップされ、ポリゴン名を緯度として読む |
| **BUG-51** | 6-2 | Low | `mysd.cpp` (L2374–2397) | `log_sd()` の `char logtext[128]` が 256 バイトの `logSdfArgs`（`sdio_fail_desc` 等）を切り詰める |
| **BUG-52** | 6-2 | Low | `mysd.cpp` (L65–76, L2506–2510, L2568–2575) | SdFat の `dateTime()` コールバックが未Fix時や RX モード時に `fileyear == 0` のまま壊れた FAT タイムスタンプを返す |
| **BUG-53** | 6-3 | High | `mysd.cpp` (L1431–1437, L1608–1631, L1691–1706) | SPSC リングバッファの `replay_tail` を Core1 の `init_replay()` が直接 0 に書き換え、Core0 の `replay_pop()` と競合すると 15連ポップする |
| **BUG-54** | 6-4 | Medium | `mysd.cpp` (L2205, L2220) | `browse_sd()` の早期リターンで `loading_sddetail = false` が漏れ、SD エラー時に `SD DETAIL` 画面が永久に `"Stand by ..."` で固まる |
| **BUG-55** | 6-4 | Medium | `mysd.cpp` (L2071, L2233, L2919) | `entry.getName(name, sizeof(name))` の戻り値未確認かつ `name` 未初期化のため、32文字以上のファイル名で未初期化スタック領域を参照 |
| **BUG-56** | 6-5 | High | `src/imulog.cpp` (L127–133, L288–346), `GPS_TFT_map.ino` (L1352) | WebSerial（PC接続）モード中に `imulog_push()` と `imulog_write_buffer()` が止まらず、約1.1秒後に `imuraw/*.bin` を再オープンして書き込み続ける |
| **BUG-57** | 7-1/7-2 | High | `src/sound.cpp` (L1204–1293) | (1) Core0 `update_vario()` 消音時の `delay(3)` 競合で Core1 の警報 WAV のアンプが切られ完全無音化、(2) バリオ発音中に `vario_inhibit=true` になると音が止まらない |
| **BUG-58** | 7-1/7-2 | Low-Medium | `src/sound.cpp` (L574–589, L685, L1004, L1052) | `startPlayWav()` の音量0早期リターン位置によるアンプ切り忘れ・`pending_wav` 消失、無音トーン時のアンプ ON、アンダーラン停止時の `pending_wav` 再開漏れ |
| **BUG-59** | 8-1 | Medium-High | `e220.cpp` (L159–172, L494–531, L754–768), `link.cpp` (L515–534) | ブロッキング版 `e220_wake()` / `set_mode()` が AUX 自己診断 Low パルスを見逃す＋ `link_probe_module()` の不要なモード2往復と RX ゴミ残り |
| **BUG-60** | 8-2 | Medium | `lora_link/link_wire2.h` (L308–320, L363–372) | Wire2 復号器で日界（0時）ロールオーバー時に次の Slot 0 まで前日の日付が出力される＋ `tod == 0xFFFF` 時に過去時刻が残留する |
| **BUG-61** | 8-3 | Medium | `link.cpp` (L1551–1588) | `link_set_group()` / `link_set_radio_profile()` で `s_w2rx` と `s_otherGroup` がリセットされず、切替直後の最大7フレームに旧グループの値が混入 |
| **BUG-62** | 9-1 | High | `tools/flight_viewer/index.html` (L1197) | 飛行 CSV の推定対気速度カラム名 `tas_mps` を `'tas'`（完全一致）で探しているため常に読み落とす |
| **BUG-63** | 9-1 | High | `tools/flight_viewer/index.html` (L753, L2566–2570, L2647, L2732–2805) | (1) ファイル読込後にモーダルが閉じると PING が止まり60秒後に本体が勝手に切断、(2) ヘッダの「PONS切断」ボタンで切断されない、(3) 転送キャンセル/エラー時に `isDataStreaming` がロックする |
| **BUG-64** | 9-1 | Medium | `tools/flight_viewer/index.html` (L1236–1239, L1304, L1348, L2689–2695) | 日跨ぎ・時刻逆転時の負の `t` によるシーク不良、`innerHTML` 未エスケープ、および `findLastIndex` フォールバックバグ |
| **BUG-65** | 9-2 | High | `tools/vectormap/build_vectormap.py` (L384–390, L412, L795–799) | `--pbf` と `--variant hires` 併用時に `VM_PARK`・`VM_BUILDING`・`VM_STREET` が Overpass フォールバックもされず 0 件のまま欠落する |
| **BUG-66** | 9-2 | Medium | `tools/vectormap/build_vectormap.py` (L28, L878–882, L1035–1042) | `--lods 0` 等の部分 LOD 指定実行時に、指定外の LOD タイルが消去された状態で `vectormap_data.cpp` を上書きする |
| **BUG-67** | 9-3 | High | `tools/png2bmp.py` (L1, L38, L96–98), `tools/clean_wav8.py` (L168) | `png2bmp.py` が Pillow 10.0+ で削除された `Image.FLIP_TOP_BOTTOM` を参照してクラッシュ＋ `clean_wav8.py` の `/tmp` ハードコード |
| **BUG-68** | 9-4 | High | `tools/createmovie_csv2mp4.py` (L238, L381, L692), `tools/flight_preprocess.py` (L630) | `createmovie_csv2mp4.py` が旧カラム名 `data['Altitude']` を参照して `KeyError` でクラッシュ、小数秒で `ValueError`、`flight_preprocess.py` の風向 180° 定義差 |
| **BUG-69** | 9-4 | Medium | `tools/kml_to_mapcsv.py` (L19–27), `tools/logcsv2kml4earth.py` (L45), `tools/logcsv2tourkmz.py` (L40, L50) | 2要素座標 KML で `ValueError`、JST 時刻に `'Z'`（UTC）を付与して9時間ズレる問題 |
| **BUG-70** | 9-5 | Medium | `tools/imulog/eskf.py` (L242–243), `decode_imulog.py` (L324–330), `merge_flight.py` (L159, L170) | `eskf.py` のヨー分散上限クランプ欠落、`decode_imulog.py` の土曜朝9時 JST 週ロールオーバー誤警告、`merge_flight.py` のゼロ除算・カラム名エラー |
| **BUG-71** | 9-6 | High | `tools/flightanalysis/plot_phugoid.py` 等3本, `plot_spectrum.py` 等4本 | `GNSS_Altitude`（GPS高度）を気圧高度と誤認して `vs_baro` や気圧ノイズフロアを計算しているバグ、および NumPy 2.0+ での `np.trapz` 削除によるクラッシュ |
| **BUG-72** | 9-7 | Low-Medium | `tools/attitude_test/dead_test.cpp` (L14), `tools/attitude_test/settings.h`, `tools/sch16t_test/sch16t_test.ino` (L1183) | `dead_test.cpp` のセンサー軸加速度 `[0,0,+g]`（機首真下90°）入力、死蔵された古い `attitude_test/settings.h`、`sch16t_test` の初回 `dt` スパイク |

---

## 第3部：カテゴリ別・機能順の詳細バグレポート（全72件）

---

## カテゴリ1：システム基盤・デュアルコア制御・死活監視・入力管理

### [BUG-01] [Low] Core0 の起動が遅延した際、Core1 の診断ダンプ中にハードウェア Watchdog が発火してログが欠損する可能性
- **場所**: `bootwatch.cpp` (L134, L357–396)
- **原因・メカニズム**: `bootwatch_rt_start()`（L134）で有効化されるハードウェア Watchdog は `BOOTWATCH_WDT_MS = 4000ms` で設定され、Core0 が `setup()` 末尾の `bootwatch_boot_done_core0()` に到達するまで一度も feed（`watchdog_update()`）されない。一方、`bootwatch_report_core1()`（L357–367）は `BOOTWATCH_WAIT_CORE0_MS (1000ms)` 経過後にさらに最大 `BOOTWATCH_STUCK_MS (3000ms)` 待機してから `log_sdf()` で SD へ故障状態を書き出す。合計 `1000 + 3000 = 4000ms` となり、Core1 が `log_sdf("BOOT WATCH: core0 stuck...")` を書く直前または最中にハードウェア Watchdog がタイムアウトしてリセットがかかり、HardFault レジスタダンプが SD に残らない場合がある。
- **修正案**: `bootwatch_report_core1()` が `log_sdf()` を書き出す直前に `watchdog_update()` を 1 回呼んで書き込み猶予を確保する、または `BOOTWATCH_WDT_MS` を `6000` に広げる。

### [BUG-02] [Medium] デモ飛行モード中に仮想 GNSS 高度・鉛直速度が実機の気圧基準とバリオ KF を汚染する
- **場所**: `GPS_TFT_map.ino` (`kalman_feed_tick()` L1745–1754)
- **原因・メカニズム**: `kalman_feed_tick()` のガード条件が `if (!getReplayMode() && !link_mirror_active() && get_gnss_fixok() && get_gnss_fixtype() >= 3)` となっており、`!is_demo_active()` が抜けている。実機 GNSS が 3D Fix した状態でデモモードに入ると、`get_gnss_altitude()` はデモ用の疑似高度 `10 + 10 * sin(millis()/5000)`（`gnss.cpp` L2241）を返すため、`imu_kalman_gnss_update()` が呼ばれて実機の気圧基準 `ground_alt_abs` と `_gnss_kf_offset` がデモ高度に書き換えられ、デモ終了後も高度・V/S のズレが残り続ける。
- **修正案**: `GPS_TFT_map.ino` L1745 に `&& !is_demo_active()` を追加する。

### [BUG-03] [Medium] `imu_exit()` および移動開始による自動退場時に、IMU案内3ページ目完了後のガイド音声が停止されない
- **場所**: `GPS_TFT_map.ino` (`screen_moving_exit_tick()` L1825–1829, `imu_exit()` L2424–2428, `imu_guide_next()` L2466–2473)
- **原因・メカニズム**: `imu_guide_next()` で案内最終ページ（ページ2）を抜ける際、`imu_guide_page = -1` に戻した直後に約10秒間の `"wav/guide_eskf_apply.wav"` を `WAV_EXCL_ESKF_GUIDE` 付きで再生開始する（L2470–2471）。しかし `imu_exit()`（L2424）には `createStopWavTask` が一切無く、`screen_moving_exit_tick()`（L1826）も `if (imu_guide_page >= 0)` のときしか止めないため、ページ1に入ってから `Exit` または移動退場すると地図画面に戻った後もガイド音声が鳴り続ける。
- **修正案**: `imu_exit()` と `screen_moving_exit_tick()` の `MODE_IMUDETAIL` 分岐で、`imu_guide_page` の値に関わらず無条件に `enqueueTask(createStopWavTask(WAV_EXCL_ESKF_GUIDE));` と `enqueueTask(createStopWavTask(WAV_EXCL_TAS_GUIDE));` を呼ぶ。

### [BUG-04] [Medium] `PONS:CONNECT` 受信時に遷移元画面のクリーンアップが行われず、再送時に `prev_screen_mode` が `MODE_PCCONNECT` で上書きされる
- **場所**: `GPS_TFT_map.ino` (`handle_serial_command()` L1874–1883)
- **原因・メカニズム**:
  1. `PONS:CONNECT` をすでに `screen_mode == MODE_PCCONNECT` の状態で再受信すると（`index.html` L2921 の自動再接続など）、`prev_screen_mode = screen_mode;` により戻り先が `MODE_PCCONNECT` で上書きされ、`PONS:BYE` やボタン長押しで切断しても元の画面（`MODE_MAP` 等）に戻れなくなる。
  2. `MODE_GNSSDETAIL`（`NAV-SAT` 1Hz 有効中）や `MODE_IMUDETAIL`（ガイド音声再生中）から `PONS:CONNECT` に入った際、`gnss_getposition_mode()` や音声停止などの後始末が行われない。
- **修正案**: `if (screen_mode != MODE_PCCONNECT)` のときだけ前画面の後始末（`screen_moving_exit_tick()` と同様の switch 分岐）と `prev_screen_mode = screen_mode;` を行う。

---

## カテゴリ2：GNSS測位・UBX制御・時刻管理・DBDアシスト

### [BUG-05] [Low] `handle_navpvt()` で時刻フィールド（`hour`/`min`/`sec`）の範囲検証が無い
- **場所**: `gnss.cpp` (`handle_navpvt()` L998–1005)
- **原因・メカニズム**: 日付には `y >= 2025 && y <= 2035 && m >= 1 && m <= 12 && d >= 1 && d <= 31` の範囲検証があるが、時刻（`hour <= 23 && min <= 59 && sec <= 60`）には範囲チェックが無い。
- **修正案**: `ubx_time_valid` の判定に `&& pvt.hour <= 23 && pvt.min <= 59 && pvt.sec <= 60` を追加する。

### [BUG-06] [High] リプレイ終了時に `ubx_pos_valid`・`stored_gs`・`stored_latitude/longitude`・`stored_fixtype` がクリアされず、屋内未Fix時に「測位中・移動中」と誤認し続ける
- **場所**: `gnss.cpp` (`set_replaymode()` L1820–1824), `power.cpp` (L354–362, L464), `GPS_TFT_map.ino` (L1809–1810)
- **原因・メカニズム**: `apply_replay_row()`（L1538–1547）は `ubx_pos_valid = true;` と `stored_gs = row->gs;`、`stored_latitude/longitude` を書き換えるが、リプレイを抜ける `set_replaymode(false)`（L1820–1824）では `ubx_date_valid / ubx_time_valid / ubx_pvt_valid / ubx_gnssFixOK` しか `false` に戻していない。そのため、実 GNSS が未 Fix の場所（屋内など）で飛行中 CSV（`gs >= 2.0 m/s`）をリプレイして終了すると、`gnss_get_own_fix()` が `true` かつ `gs >= 2.0 m/s` を返し続ける。その結果、`power.cpp` の `update_moving()` が地上にもかかわらず「時速 7km 以上で移動中（`s_moving = true`）」と判定し続け、**地上で電源スイッチを切り忘れても `power_sw_off_warn_active()`（SW OFF 警告）が抑制されて鳴らなくなる**。また RX モードの自機ボートマーカー（`draw_rx_self_marker`）もリプレイ最終座標に表示され続ける。
- **修正案**: `set_replaymode()` で `if (was_replay && !replaymode)` のときに `ubx_pos_valid = false; stored_fixtype = 0; gsa_fixtype = 0; stored_gs = 0; stored_numsats = 0; gsa_numsat = 0;` にリセットする。また `power.cpp` L464 でもリプレイ/デモ中は `s_moving = false; s_move_ToggleMs = 0;` にリセットする。

### [BUG-07] [High] 無線ミラーリング中に (1) 送信機が未Fixのとき `(0, 0)` 座標で Auto10km が復路へ誤遷移し、(2) 受信機自身の GNSS が `newcourse_arrived` を 2Hz で立てて旋回角速度を狂わせ、(3) 航跡記録間隔がボートの速度で決まる
- **場所**: `gnss.cpp` (`add_latlon_track()` L1695, `gnss_new_location_arrived()` L1772–1775, `gnss_loop()` L2005–2008), `GPS_TFT_map.ino` (`nav_alarm_tick()` L2981–3047)
- **原因・メカニズム**:
  1. `gnss.cpp` L1772 の `if (link_mirror_active() && link_take_new_frame())` で送信機の Fix 有無（`link_get_fix_ok()`）を確認せずに `new_location_arrived = true;` と `add_latlon_track(get_gnss_lat(), get_gnss_lon());` を実行している。送信機が未 Fix の間は `link_get_lat()/lon()` が `(0.0, 0.0)` を返すため、航跡に `(0, 0)` が積まれ、さらに `nav_alarm_tick()` が現在地 `(0, 0)` と琵琶湖の距離（約 13,400 km > 10.475 km）を計算して即座に `AUTO10K_AWAY` → `AUTO10K_INTO` へ誤遷移し、`destination_change.wav` と `destination_toofar.wav` を鳴らして設定を SD に保存してしまう。
  2. `gnss.cpp` L2005 の `if (!is_demo_active())` に `&& !link_mirror_active()` が無いため、ミラー中もボート自身の GNSS（2Hz）が毎回 `newcourse_arrived = true` を立てる。すると 1Hz でしか更新されない機体方位 `link_get_track()` が 3Hz（ボート 2Hz + 無線 1Hz）で `truetrack_samples[]` に押し込まれ、3回中2回で方位差が 0 になって `degpersecond` が実態より小さく算出される。
  3. `gnss.cpp` L1695 の `add_latlon_track()` が `get_gnss_mps()` ではなく `stored_gs`（ボート自身の速度）を使って `tracklog_interval` を計算しているため、停船中のボートでは飛行中の機体の航跡が常に最長間隔（15秒おき）に間引かれる。
- **修正案**:
  1. `gnss.cpp` L1772 で `if (link_get_fix_ok()) { new_location_arrived = true; add_latlon_track(...); }` とし、`nav_alarm_tick()` L2981 でも `if (new_gnss_info && get_gnss_fix())` とする。
  2. `gnss.cpp` L2005 を `if (!is_demo_active() && !link_mirror_active())` に変更する。
  3. `add_latlon_track()` L1695 の `stored_gs` を `get_gnss_mps()` に変更する。

### [BUG-08] [High] 屋内（3D Fix なし）で `PONS:DBD,COLD` 等を実行すると、最大30分間 `ST_WAIT` に張り付いて以降の全 DBD コマンドが `ERROR,BUSY` で拒否される
- **場所**: `gnss_dbd.cpp` (`restore_done()` L370, `wait_tick()` L703–715, `gnss_dbd_command()` L921)
- **原因・メカニズム**: `restore_done()` で `s_st = ST_WAIT;` に入った後、`wait_tick()` は 3D Fix（`s_ep == EP_NONE`）するかエピソードが30分タイムアウトするまで、たとえパス2の再送が完了した後（`s_rs_pass == 2`）や再送不要と判定した後でも `s_st` を `ST_IDLE` に戻さない。一方 `gnss_dbd_command()` L921 は `const bool busy = (s_st != ST_IDLE) || ...` で判定するため、屋内ベンチテストで一度 `PONS:DBD,COLD` を打つと30分間すべてのコマンドが `PONS:DBD,ERROR,BUSY` になる。
- **修正案**: `wait_tick()` で再送不要と判定した時点（L712）、および `restore_done()` で `s_rs_pass == 2` が完了した時点（L370）で `s_st = ST_IDLE;` に戻す（TTFF 計測自体は `s_ep` と `episode_tick()` が独立して継続するため影響しない）。

### [BUG-09] [Medium] `abort_op()` が `s_tx_end = s_tx_pos` で送信途中の UBX フレームを切断し、直後の UBX コマンドが GNSS 側で破棄される＋ `ST_WAIT`/`ST_TEST_LOAD` が解除されない
- **場所**: `gnss_dbd.cpp` (`abort_op()` L374–386, `gnss_dbd_command()` L951–957)
- **原因・メカニズム**: `gnss_dbd.h` L35–38 のコメントに「1フレームの途中で打ち切ると次のコマンドの先頭バイトが MGA-DBD の残りのペイロードとして食われチェックサム不一致で捨てられる」と明記されているにもかかわらず、`abort_op()` L376 は `gnss_dbd_finish_tx()` ではなく `s_tx_end = s_tx_pos;` で即座に打ち切っている。また `s_st == ST_WAIT` や `ST_TEST_LOAD` のときに `abort_op()` が呼ばれても `s_st = ST_IDLE` に戻らない。さらに L951 で `enqueueTask(createLoadDbdTask())` が失敗した際に `s_st = ST_TEST_LOAD` に遷移させずに `gnss_dbd_loaded_core1(false, ...)` を呼ぶため `s_ep = EP_TEST_DBD` だけが残る。
- **修正案**: `abort_op()` 内で `gnss_dbd_finish_tx()` を呼び、`ST_WAIT` / `ST_TEST_LOAD` も `ST_IDLE`（および `s_ep = EP_NONE`）へリセットする。

### [BUG-10] [Medium] 起動時の `SEC-UNIQID` ポーリング再試行（5秒周期×4回）より先に `ST_BOOT` の `GNSS_DBD_LOAD_WAIT_MS (5000ms)` が切れて復元がスキップされる
- **場所**: `gnss_dbd.cpp` (`gnss_dbd_tick()` L779–783, L829–832)
- **原因・メカニズム**: `setup()` で `gnss_setup()` が完了して最初の `SEC-UNIQID` を送るのは起動後約 1.5〜2.5 秒後であり、1回目の応答を取りこぼした場合の2回目の再送（`now - s_gid_poll_ms >= 5000`、L779）は `now ≈ 6500〜7500ms` まで行われない。しかし `ST_BOOT`（L829–832）は `const bool waiting = now < GNSS_DBD_LOAD_WAIT_MS;`（5000ms）で打ち切って `skip_restore("no GNSS ID")` してしまうため、4回リトライ機構が `ST_BOOT` の DBD 復元には一度も使われない。
- **修正案**: `!s_gid_ok` の待機判定を `!s_gid_fail_logged`（または `now - s_gid_poll_ms` ベース）と連動させる。

### [BUG-11] [Low] `capture_utc()` が GNSS 時刻確定時刻（`get_gnss_fix_millis()`）ではなく現在時刻 `now` を差し引いて `s_utc0` を算出する
- **場所**: `rtc_backup.cpp` (`capture_utc()` L80–89)
- **原因・メカニズム**: `s_utc0 = to_epoch(...) - now / 1000UL;` としているが、デモ飛行中や GNSS 途絶中は `get_gnss_date()/time()` が最後に受信した NAV-PVT の時刻のまま据え置かれるため、デモ終了直後に `capture_utc()` が走ると `(now - get_gnss_fix_millis()) / 1000` 秒だけ `s_utc0` が過去へずれる。
- **修正案**: `now / 1000UL` の代わりに `get_gnss_fix_millis() / 1000UL` を引く。

---

## カテゴリ3：航法計算・目的地管理・コース逸脱警報・ベクタ地図

### [BUG-12] [Medium] 飛行中の SD 再初期化（`try_sd_recovery()` → `setup_sd()`）で `init_destinations()` が呼ばれると、`currentdestination` が一時的に `0` にリセットされる
- **場所**: `navdata.cpp` (`init_destinations()` L117), `mysd.cpp` (`setup_sd()` L1353–1361)
- **原因・メカニズム**: 起動時に SD カードが認識されず、パイロットが手動で目的地を `1`（北パイロン等）に変更して離陸した後、飛行中に `try_sd_recovery()` が SD カードの認識に成功して `setup_sd()` を走らせると、Core1 が `init_destinations()`（L117）で `currentdestination = 0;` を書き込む。
- **修正案**: `init_destinations()` で `if (currentdestination < 0) currentdestination = 0;` とし、既に有効なインデックスが選ばれている場合は `0` に戻さない。

### [BUG-13] [Low] `settings.txt` から読み込む `currentdestination` や `auto10k_status` に負値・範囲外値が入った場合の境界チェック漏れ
- **場所**: `mysd.cpp` (`loadSettings()` L361–380), `navdata.cpp` (`restore_auto10k_status()` L50–58, `nav_update()` L673)
- **原因・メカニズム**: 各所のガードが `currentdestination != -1 && currentdestination < destinations_count` となっているため、`settings.txt` の破損等で `-2` などの負値が入ると配列外参照になる。また `auto10k_status` に `0`/`1` 以外が入ると `AUTO10K_AWAY` にも `AUTO10K_INTO` にも一致せず誘導線と折返し判定が消える。
- **修正案**: `currentdestination >= 0 && currentdestination < destinations_count` に統一し、`restore_auto10k_status()` で `(st == AUTO10K_INTO) ? AUTO10K_INTO : AUTO10K_AWAY` に正規化する。

### [BUG-14] [Medium] `update_course_warning()` が `get_gnss_fix()` を確認しておらず、GNSS 断線時や 1〜3 衛星追跡のみの未Fix時にコース逸脱警報がリセットされない
- **場所**: `GPS_TFT_map.ino` (`update_course_warning()` L3137)
- **原因・メカニズム**: `if (get_gnss_mps() < 2 || get_gnss_numsat() == 0)` だけで判定しており、`!get_gnss_fix()` を見ていない。飛行中に GNSS コネクタが抜けた場合（`stored_gs` と `stored_numsats` は直前の値を保持）や、衛星 1〜3 個で `gnssFixOK == false` かつドップラー速度ノイズが 2m/s 以上出た場合に、`course_warning_index` が 0 にリセットされず誤警報が鳴る。
- **修正案**: L3137 の条件を `if (!get_gnss_fix() || get_gnss_mps() < 2 || get_gnss_numsat() <= 0)` に変更する。

---

## カテゴリ4：IMUドライバ(SCH16T)・姿勢推定ESKF・推定TAS・気圧バリオKF

### [BUG-15] [Medium-High] `imu_sch16t.cpp` の `imu_feed_gyro` → `imu_feed_accel` 呼び出し順序による 1 サンプル遅延・初回サンプル欠落・復旧時の古い加速度参照
- **場所**: `imu_sch16t.cpp` (`read_sample()` L406–409), `imu.cpp` (L134–182), `attitude.cpp` (L612–650)
- **原因・メカニズム**: `imu_sch16t.cpp` の `read_sample()` は 1 回のポーリングでジャイロと加速度を同時に読み出すが、L408–409 で **先に `imu_feed_gyro(g, t_us)` を呼び、後から `imu_feed_accel(a)` を呼んでいる**。`attitude_on_gyro()` は内部で `accel_last_` と `accel_valid_` を参照するため：
  1. 起動後の第 1 サンプル目で `attitude_on_gyro()` が呼ばれた時点では `accel_valid_ == false` のため `static_now_ = false` になり、第 1 サンプルが `acc_sum_` に積算されない。
  2. サンプル $k$ の `attitude_on_gyro(g[k])` が使う `accel_last_` は、同時刻の `a[k]` ではなく **1 サンプル前（4ms 前）の `a[k-1]`** になる（`eskf.py` は同一サンプルの `w[i]` と `acc[i]` をペアにするため乖離が生じる）。
  3. SCH16T が通信途絶から復旧した際（`try_recovery()` → `imu_feed_reset()`）、`attitude.cpp` の `accel_valid_` はリセットされないため、復旧直後の第 1 サンプルで `imu_feed_gyro()` が先に走ると **60 秒以上前の途絶直前の `accel_last_`** を使って伝播・静止判定を行ってしまう。
- **修正案**: `imu_sch16t.cpp` L408–409 の呼び出し順を `imu_feed_accel(a);` → `imu_feed_gyro(g, t_us);` に入れ替える。

### [BUG-16] [Medium] 飛行中に SCH16T が断線・無応答になると、60秒ごとの `try_recovery()` が Core0 を最大 1,046 ms ブロックする
- **場所**: `imu_sch16t.cpp` (`SCH16T_READY_TIMEOUT_MS = 1000`, `wait_chip_ready()` L216–226, `try_recovery()` L246–267, `sch_init()` L280–346)
- **原因・メカニズム**: 飛行中にハーネス抜け等で SCH16T が無応答になった場合、`s_ever_alive == true` のため 60 秒ごとに `try_recovery()` → `sch_init()` が呼ばれ、`wait_chip_ready()`（L216–226）が `SCH16T_READY_TIMEOUT_MS = 1000` ms フルに回り続ける（合計 `2 + 10 + 34 + 1000 = 1,046 ms` の間 Core0 が完全停止）。`airdata_try_recovery()` では事前疎通確認が入っているが、`imu_sch16t.cpp` にはそれが無い。
- **修正案**: `try_recovery()` の冒頭で、リセットや `delay` に入る前に `spi_xfer48(SPI_RD(REG_COMP_ID))` 等を送り、`frame_integrity_ok(r)` が通ることを確認してから `sch_init()` へ進むようにする（応答がなければ数十 µs で即 return する）。

### [BUG-17] [Medium] `imu_sch16t.cpp` の `set_dry()` および `sch_init()` のレジスタ検証読み出しで `frame_integrity_ok()`（CRC8 / IDS）が検査されていない＋ 1秒タイムアウト時の `s_present = false` 契約違反
- **場所**: `imu_sch16t.cpp` (`set_dry()` L231–238, `sch_init()` L315–325, `imu_drv_poll()` L507), `imu_sensor.h` (L31–34)
- **原因・メカニズム**:
  1. `set_dry()`（L232–235）で `REG_CTRL_USER_IF` を Read-Modify-Write する際、読み出した `cur` の CRC を検証せずに `DRY_DRV_EN` ビットを立てて書き戻している。
  2. `sch_init()` L315–325 の設定値読み返し検証と `REG_STAT_SUM == 0xFFFF` 検証で `frame_integrity_ok()` を呼んでいない（MISO High 張り付き `0xFFFFFFFFFFFF` のとき `frame_data16(r) == 0xFFFF` で `stat_ok = true` と誤判定する）。
  3. `imu_sch16t.cpp` L507 で 1 秒タイムアウト時に `s_present = false;` に書き換えており、`imu_sensor.h` L32 の契約（「setup で見つかったか。故障してもここは変わらない」）に違反している。
- **修正案**: `set_dry()` および `sch_init()` の全 SPI 読み出し結果に対して `frame_integrity_ok(r)` を必須条件に加え、`s_present` と `s_alive` を分離する。

### [BUG-18] [Low] `imu_sch16t.cpp` の `try_recovery()` 成功時にポーリング時刻・レート計測カウンタがリセットされず、偽の `gapmax`（~60s）とレート低下警告が出る
- **場所**: `imu_sch16t.cpp` (`try_recovery()` L254–262, `imu_drv_poll()` L520–538, `imu_drv_service_if_due()` L549–553)
- **原因・メカニズム**: `try_recovery()` 成功時に `s_poll_last_us`、`s_hz_last_ms`、`s_gyro_cnt`、`s_accel_cnt` がリセットされないため、復旧直後の最初の `imu_drv_service_if_due()` で `gap = now_us - s_poll_last_us` が途絶期間全体（60 秒以上）を `s_poll_gap_max_us` に記録し、`s_gyro_hz` が `~0.02 Hz` と算出されて偽の `SCH16T read rate too low` 警告が出る。
- **修正案**: `try_recovery()` 成功時に `s_poll_last_us = time_us_32(); s_hz_last_ms = millis(); s_gyro_cnt = s_accel_cnt = 0;` をリセットする。

### [BUG-19] [High] SCH16T（FIFOなしポーリング）に対する BNO085 時代（FIFOバースト前提）の 64 サンプル窓平均 `dt_est_` の不整合と回転角ロスト・`eskf.py` との乖離
- **場所**: `attitude.cpp` (`update_dt_estimate()` L225–236, `attitude_on_gyro()` L634–644, L707) vs `tools/imulog/eskf.py` (L502)
- **原因・メカニズム**:
  - `attitude.cpp` L634–644 では、BNO085 の SH-2 バッチ転送向けに設計された 64 サンプル移動平均 `dt_est_ = span / (DT_WIN - 1)` を使用してクォータニオン積分（L707）および共分散伝播（L730）を行っている。
  - しかし v7.1 の SCH16T はハードウェア FIFO を持たず、ポーリング 1 回につき直近 1 サンプルしか読み出さない（ポーリングが遅れた間のサンプルはレジスタ上書きで消失する）。
  - そのため、地図描画等で Core0 が 40ms ブロックした後に 1 サンプル届いた場合、実際の経過時間は `40ms` であるにもかかわらず、`attitude.cpp` は `dt_est_`（約 `4.6ms`）で `q_` を積分するため、**その 40ms 間の回転角の約 88%（35.4ms 分）が永久に失われる**。
  - さらに、1秒以上の通信途絶からの復帰時や起動直後のギャップでは、`span / 63` が上限ガード `5.0f / IMU_RATE_GYRO_HZ`（20ms、L641）を超えるため、**その後 63 サンプル（約 0.25 秒間）にわたり `dt_est_` の更新が完全に凍結**される。
  - 一方、`tools/imulog/eskf.py` L502 はサンプル間の実時間差 `dt = float(ts[i] - ts[i - 1])` を使っており、機上 C++ 実装とオフライン Python 実装の姿勢軌道が乖離する。
- **修正案**: SCH16T では `eskf.py` と同様に直前サンプルとの実時間差 `dt = (uint32_t)(t_us - prev_gyro_us_) * 1e-6f`（上限 `0.5f` 等でガード）を使用する。

### [BUG-20] [High] `attitude_on_gnss_velocity()` の移動中フォールバック初期化に `sAcc` ゲートと移動速度ゲートが無く、静止中の初回 GNSS fix で未平均の 1 サンプル初期化＆異常速度注入が起きる
- **場所**: `attitude.cpp` (`attitude_on_gnss_velocity()` L1101–1125)
- **原因・メカニズム**: `attitude_on_gnss_velocity()` の L1108–1118 では、`if (!initialized_)` の判定が `if (sAcc_m_s > ESKF_SACC_MAX) return;`（L1123）よりも前にあり、かつ対地速度が移動中かどうかの判定（`sp >= ESKF_YAW_INIT_MIN_SPEED_MPS` 等）も無い。電源投入後に機体に手が触れていて静止 2 秒（`ESKF_STATIC_INIT_US`）が成立する前に GNSS が最初の 3D fix を出すと、たとえ機体が地上で停止中であっても、あるいは `sAcc` が劣悪であっても即座に「移動中フォールバック初期化」が発火し、直近 1 サンプルだけの `accel_last_` で初期姿勢が確定し、本来の静止 2 秒平均初期化（ジャイロ初期バイアス `bg_` の推定を含む）が永久にスキップされる。
- **修正案**: `if (sAcc_m_s <= 0.0f || sAcc_m_s > ESKF_SACC_MAX) return;` を関数の先頭へ移動し、さらに移動中フォールバック初期化（`!initialized_`）は `sp >= ESKF_YAW_INIT_MIN_SPEED_MPS` のときのみ実行する。

### [BUG-21] [Medium] `attitude.cpp` におけるプロセスノイズ `+ Q` の加算位置ミス（`level_update` の後に `+ Q` が足されている）および `wz` 計算タイミングの `eskf.py` との乖離
- **場所**: `attitude.cpp` (L711, L748–760, L767, L782–799, L923–932) vs `tools/imulog/eskf.py` (L214–243, L534–562)
- **原因・メカニズム**: `attitude.cpp` では L748–760 で $\Phi P \Phi^T$ を計算した直後、`+ Q` を足す前（L923–932）の L782–799 に `level_update()` が挿入されている。一方 `eskf.py` では `f.predict()`（`+ Q` まで完了）の後に `f.update_level()` を呼んでおり、共分散の推移が一致しない。また旋回ゲート用の `wz` も、`attitude.cpp` では `q_` 更新前の回転行列 `R` を使い、`eskf.py` では `q` 更新後の `R` を使っている。
- **修正案**: `attitude.cpp` L923–942 の `+ Q` 加算および `P_[2][2]` の上限クランプを L760（`level_update()` より前）へ移動し、`R` の計算タイミングを `eskf.py` と統一する。

### [BUG-22] [Low] `attitude_on_gyro()` の地上再水平化（`ground_hold`）のジャイロノルム判定が生値 `g` を使っており、`update_static()`（バイアス除去後の `w`）と基準が異なる
- **場所**: `attitude.cpp` (L688 vs L815–816)
- **修正案**: 両方ともバイアス除去後の角速度 `w` に統一する。

### [BUG-23] [High] IMU 途絶時（`!attitude_ready()`）に古い平均ピッチ・推定 TAS・風ベクトルが有効なまま残り、無線テレメトリ（`link_push_telemetry`）と飛行 CSV（`createSaveCsvTask`）に凍結値が送出・記録され続ける
- **場所**: `attitude.cpp` (L1260–1269, L1314–1324), `tas.cpp` (L81–88), `link.cpp` (L1679–1696), `mysd.cpp` (L876–879)
- **原因・メカニズム**: 飛行中に IMU が途絶すると `attitude_ready()` は 500ms 後に `false` になるが、`pavg_count_` と `wind_fill_s_` は減らないため、`attitude_pitch_avg_valid()`、`attitude_get_tas()`、`attitude_get_wind()` はすべて `true` を返し続け、途絶直前の凍結した値を返す。その結果、`link_push_telemetry()`（`if (attitude_ready())` の外にある L1679–1696）と `createSaveCsvTask()`（`mysd.cpp` L876–879）は、機体の IMU が死んで自機画面では Pavg / TAS が消えているにもかかわらず、ボート受信機と飛行 CSV に途絶直前の凍結した Pavg・TAS・風ベクトルを送り続ける。
- **修正案**: `attitude_pitch_avg_valid()` と `attitude_get_wind()` の先頭に `if (!attitude_ready()) return false;` を追加し、`link.cpp` L1676–1696 も `if (attitude_ready())` のブロック内に入れる。

### [BUG-24] [Medium] APPLY（`attitude_calibrate_to()`）実行時に平均ピッチ窓（`pitch_avg_reset()`）と風推定 LPF がリセットされず、較正後 10 秒間の平均ピッチと推定 TAS に較正前の未較正値が混入する
- **場所**: `attitude.cpp` (`attitude_calibrate_to()` L1398–1419)
- **原因・メカニズム**: `attitude_calibrate_to()` で機体ゼロ点オフセットを更新した際、`pitch_avg_reset()` が呼ばれていないため、APPLY 直後の 10 秒間、APPLY 前の古いオフセットで計算されたピッチ角と新しいピッチ角が混ざった不正な平均値および推定 TAS が返される。
- **修正案**: `attitude_calibrate_to()` 内で `pitch_avg_reset();` および風推定 LPF のリセットを行う。

### [BUG-25] [Low] `attitude.cpp` の `attitude_set_airspeed_min()` / `attitude_set_airspeed_max()` で `min > max` の逆転ガードが無い
- **場所**: `attitude.cpp` (L1293–1305), `mysd.cpp` (L628–642)
- **修正案**: `attitude_finish_settings_load()` で `if (airspeed_min_ > airspeed_max_)` の入れ替えまたは既定値復元を行う。

### [BUG-26] [Medium-High] `airdata.cpp` の ADC 生値 `0`/`0xFFFFFF` 未検査による `NaN` 発生・起動時グランドレベル恒久ロック、およびウィンドウ未完了サイクルでの `last_altitude` 生値上書き
- **場所**: `airdata.cpp` (`pressure_to_altitude()` L251–253, `ms5611_process_data()` L365–435, `airdata_update()` L511–530)
- **原因・メカニズム**:
  1. MS5611 が変換未完了や I2C ノイズで `0x000000` や `0xFFFFFF` を返すと、`last_pressure < 0` となり `pow(pressure_hpa / sea_level_hpa, 0.1902949f)` が `NaN` を返す。起動後第 1 サンプルでこれを踏むと L370 の `if (!ground_set)` で `ground_alt_abs = NaN; ground_set = true;` が確定し、再起動するまで高度・バリオが不能になる。
  2. L432 の `else { last_altitude = raw_alt; }` がウィンドウ未完了の毎サイクル（24msごと）に走り、`raw_alt_valid == false`（`NaN` やスパイク）のときすら `last_altitude` を未フィルタ生値で上書きしている。
- **修正案**: `D1_raw == 0 || D1_raw == 0xFFFFFF` をエラーとして弾き、`if (!ground_set)` に気圧妥当性チェックを入れ、L432 を `else if (!alt_win_prev_valid && raw_alt_valid) last_altitude = raw_alt;` に修正する。

### [BUG-27] [Low] MS5611 PROM CRC4 検証（AN520）未実装、および `airdata.cpp` の `0` 番兵・旧定数コメント残り
- **場所**: `airdata.cpp` (L198–218, L380–382, L563–566)
- **修正案**: AN520 の CRC4 検証を追加し、`now ? now : 1` の適用とコメントの 250ms への更新を行う。

### [BUG-28] [High] MS5611（気圧計）が飛行中に途絶した際、バリオ KF が加速度バイアスを無制限に積分して発散し、`update_vario()` が誤警報音を鳴らし続け、無線にも発散した V/S を送出する
- **場所**: `imu.cpp` (`vario_source_ok()` L541–560, `imu_update()` L668), `src/sound.cpp` (`update_vario()` L1215–1222), `link.cpp` (`link_push_telemetry()` L1653–1658)
- **原因・メカニズム**: 起動後に MS5611 が途絶（`get_airdata_ok() == false`）すると、`imu_kalman_baro_update()` と GNSS VSI 更新（L341 `if (get_airdata_ok())`）が止まったまま `kf_predict()` だけが回り続け、加速度バイアス残差を積分して `_imu_vspeed` が数秒で発散する。このとき画面の VSI バーは `if (!vario_source_ok())` で消えるが、音声側の `update_vario()` は `vario_source_ok()` をチェックしておらず、`vario_vspeed_mps()` は `get_imu_alive()` しか見ないため、**バリオ音が最大ピッチで鳴りっぱなしになる**。また `link_push_telemetry()` も `get_airdata_ok()` を確認せずに `RHAVE_KFALT | RHAVE_KFVS | RHAVE_PRESS` を送信し続ける。
- **修正案**: `vario_vspeed_mps()` の先頭で `if (!vario_source_ok()) return 0.0f;` をチェックし、`!get_airdata_ok()` の間は `kf_predict()` を止めて `_imu_vspeed = 0.0f` にリセットし、`link_push_telemetry()` でも `if (get_airdata_ok())` を条件にする。

### [BUG-29] [High] リプレイモード中および IMU 故障時の `vario_source_ok()` / `vario_vspeed_mps()` / `vario_deadband_mps()` / `link_push_telemetry()` の不整合
- **場所**: `imu.cpp` (L541–576), `link.cpp` (L1654)
- **原因・メカニズム**: `get_imu_vspeed_raw()` L564 には `if (replay_has_value(RHAVE_KFVS))` 分岐があるが、上位の `vario_source_ok()`・`vario_vspeed_mps()`・`vario_deadband_mps()`（L541–560）にはリプレイ分岐が無い。そのため机上で自機 IMU が未初期化／未接続のときリプレイ中のバリオ音が CSV の `KF_Vspeed` ではなく自機の `get_airdata_vspeed()`（0 m/s）になり、自機 MS5611 が不調だとリプレイ中の VSI バーも消える。また IMU 故障時に機体画面は `get_airdata_vspeed()` を表示するのに `link_push_telemetry()` は `get_imu_vspeed()` を送る不一致がある。
- **修正案**: `vario_source_ok()`・`vario_vspeed_mps()`・`vario_deadband_mps()` に `if (replay_has_value(RHAVE_KFVS))` 分岐を追加し、`link_push_telemetry()` の V/S 送信値を `vario_vspeed_mps()` と整合させる。

### [BUG-30] [Low] `kf_predict()` の状態遷移式（オイラー積分）とコメント記載（$\frac{1}{2}a\Delta t^2$ 項あり）の不一致
- **場所**: `imu.cpp` (L25–30, L198–227)
- **修正案**: `z_pred` に `+ 0.5f * (a_k - kf_x[2]) * dt * dt` を追加するか、コメントをオイラー近似の式に合わせる。

---

## カテゴリ5：TFT画面描画・マルチページUI・環境光調光・Readiness・電源管理

### [BUG-31] [High] `draw_notice_2lines()` 等が `backscreen.unloadFont()` 後に `AA_FONT_SMALL` を再ロードせず、次フレームの `GS` / `Pavg` / `TT` ラベルが極小内蔵フォントに化ける
- **場所**: `display_tft.cpp` (`draw_notice_2lines()` L2951–2959, `draw_link_overlay()` L3083–3093, `draw_gs_track()` L3366)
- **原因・メカニズム**: `backscreen` スプライトは常時 `AA_FONT_SMALL` がロードされている前提で、`draw_gs_track()`（L3366）は先頭で `loadFont()` を呼ばずに `"GS"`, `"Pavg"`, `"TT"` を描く。しかし `draw_notice_2lines()` は末尾（L2958）で `backscreen.unloadFont()` を呼んだまま `AA_FONT_SMALL` を戻さないため、TX 地上待機中（`LINK TX: 10mW PREFLIGHT`）や RX 電波待ち中（`NO SIGNAL`）に次フレームの `"GS"`, `"Pavg"`, `"TT"` が極小の内蔵 GLCD フォントに化ける。また `draw_link_overlay()` の `"DEST DIFFERS FROM SENDER"` 分岐（L3083–3093）もフォントと文字色を復元していない。
- **修正案**: `draw_notice_2lines()` の末尾と `draw_gs_track()` の先頭で `backscreen.loadFont(AA_FONT_SMALL);` を呼び、L3083–3093 でも文字色を `COLOR_BLACK, COLOR_WHITE` に戻す。

### [BUG-32] [Medium] 4系統（実センサー・無線ミラー・リプレイ・デモ）間で `eskf_yaw_reliable()` / `eskf_yaw_high_confidence()` のヒステリシス状態 `static bool held` が共有され、モード切替時に前の系統の状態が混入する
- **場所**: `display_tft.cpp` (L576–579, L712–729)
- **原因・メカニズム**: `eskf_yaw_reliable()` と `eskf_yaw_high_confidence()` が単一の `static bool held = false;` を 4系統すべてで共有しているため、デモ終了直後等に前の系統の `held = true` が持ち越される。また `eskf_display_yaw()`（L576–579）だけ `is_demo_active()` が `link_mirror_active()` より先に判定されており、他関数と優先順位が逆転している。
- **修正案**: 供給元が変わったときに両関数の `held` を `false` にリセットし、`eskf_display_yaw()` の分岐順序を `link_mirror_active()` 最優先に統一する。

### [BUG-33] [Medium] デモ飛行中に実機 ESKF が未初期化（`!attitude_ready()`）だと、姿勢・風推定のデモ表示が `"ESKF Not ready"` で潰される＆風推定の有効判定が自機設定に依存する
- **場所**: `display_tft.cpp` (`draw_eskf_attitude()` L1750–1755, L1812–1823)
- **原因・メカニズム**: L1750 の `if (!getReplayMode() && !link_mirror_active() && get_imu_ok() && !attitude_ready())` に `&& !is_demo_active()` が抜けているため、実機 ESKF が収束する前に DEMO モードを開始すると `"ESKF Not ready"` で早期 return してしまう。また L1813 にも `|| is_demo_active()` が無く、自機の風設定が OFF だとデモの風が表示されない。
- **修正案**: L1750 に `&& !is_demo_active()`、L1813 に `|| is_demo_active()` を追加する。

### [BUG-34] [Low] `draw_ExtraMaps()` と `draw_FlashMaps()` の範囲カリングが引数 `center_lat/lon` ではなく `get_gnss_lat()/lon()` を参照している
- **場所**: `display_tft.cpp` (L1224–1225, L1237)
- **修正案**: `check_within_latlon()` の引数を `center_lat, center_lon` に置き換える。

### [BUG-35] [Low] `cord_tft::isOutsideTft()` の境界判定オフバイワンと、`draw_sddetail()` の負ページ剰余
- **場所**: `display_tft.h` (L53–55), `display_tft.cpp` (L4476)
- **修正案**: `isOutsideTft()` を `>= BACKSCREEN_SIZE` に修正し、`page` の正規化を正の剰余にする。

### [BUG-36] [Medium] `draw_header_pitch()` で `9.95° <= |pitch| < 10.0°` のときに繰り上がり計算がクランプされ `9.9°` に化ける
- **場所**: `display_tft.cpp` (`draw_header_pitch()` L3616–3625)
- **原因・メカニズム**: `a = 9.96f` のとき `a < 10.0f` 分岐に入り、`i = 9`、`f = (int)(0.96f * 10.0f + 0.5f) = 10` となって `if (f > 9) f = 9;` で潰されるため、本来 `+10` と表示すべき `+9.95° 〜 +9.99°` が `+9.9` と表示される。
- **修正案**: `draw_header_tas()` と同様に `long tenths = lroundf(a * 10.0f);` を先に計算し、`if (tenths < 100)` で分岐する。

### [BUG-37] [Medium] `draw_gs_track()` と `draw_header()` の GS・方位の赤取り消し線条件が `get_gnss_numsat() == 0` のみになっており、No-Fix 時や GNSS 断線時に赤線が引かれない
- **場所**: `display_tft.cpp` (`draw_gs_track()` L3419, `draw_header()` L3790)
- **原因・メカニズム**: 衛星を 1〜3 個捕捉しているが未 Fix（`!get_gnss_fix()`）のときや、飛行中に GNSS コネクタが抜けて `!get_gnss_connection()` になったとき（`stored_numsats` は直前の値を保持）、無効値の上に赤取り消し線が引かれない。
- **修正案**: 条件を `if (!get_gnss_fix() || get_gnss_numsat() == 0)` に変更する。

### [BUG-38] [High] `draw_maplist_mode()` でページ0のコース座標6行分がページ総数・SD地図インデックス計算から漏れており、画面外はみ出しと6件スキップが発生する
- **場所**: `display_tft.cpp` (`draw_maplist_mode()` L4671–4765)
- **原因・メカニズム**: ページ0で追加描画される `COURSE` セクション 6行分（60px）が `pagetotal`（L4672）および `sd_start_index` / `sd_rows`（L4761–4764）の計算に含まれていない。そのためページ0の下端6行が `backscreen`（高さ240px）の下へ溢れて消え、ページ1へ進むとその 6 件の SD 地図ファイルがスキップされて閲覧できなくなる。
- **修正案**: `const int course_rows = 6;` を `pagetotal`・ページ0の残行数・ページ1以降の `sd_start_index` に反映する。

### [BUG-39] [Medium] 設定画面（SETTING MODE）およびアイコン色の電池ステータスが起動時ラッチしか見ておらず、また SW OFF 時の `0.00V` をそのまま表示する
- **場所**: `display_tft.cpp` (`battery_status_color()` L4824, `draw_setting_mode()` L4939–4944)
- **原因・メカニズム**: `power_overdischarge_latched()` ではなく `power_overdischarge_boot_latched()` しか見ていないため、稼働中に発火した過放電ラッチが反映されない。また L4943 で `power_sw_off_on_usb()` のチェックが無く `(0.00V)` と表示される。
- **修正案**: `power_overdischarge_latched()` を使い、`power_sw_off_on_usb()` 時は `(SW OFF)` と表示する。

### [BUG-40] [High] `header_footer` スプライトで `unloadFont()` を呼ばずに `setTextSize(2)` で見出しを描画しており、`draw_tas_curve()` を見た直後だけ全詳細画面のヘッダが巨大な内蔵×2フォントに急変する
- **場所**: `display_pages.cpp` (`draw_tas_curve()` L201, L613, L717, L798, L829, L1049), `display_tft.cpp` (L2146, L2323, L2451, L4473, L4555, L4678)
- **原因・メカニズム**: `header_footer` には `AA_FONT_SMALL` がロードされたままになっており、各詳細画面は `unloadFont()` を呼ばずに `setTextSize(2)`（VLW フォント有効時は無視される）でヘッダを描いている。ところが `draw_tas_curve()`（L201）だけが末尾で `header_footer.unloadFont()` を呼んで終わるため、TAS 曲線画面を見た直後に他の詳細画面へ遷移したときだけ、突然ヘッダが巨大な内蔵×2ビットマップフォントに変わる。また `draw_replayselect()`（L4551）の固定幅フォーマット `%-14s%6s` もプロポーショナルな `AA_FONT_SMALL` で描かれるため桁が揃っていない。
- **修正案**: `header_footer` を使う各関数の先頭で、`unloadFont()` または `loadFont(AA_FONT_SMALL)` を明示的に指定して統一する。

### [BUG-41] [Medium] 無線ミラー中に `GNSS DETAIL 1/3`（星座画面）と `2/3`（UBXフレーム画面）で送信機の値と受信機（自機）の値が警告なく混在表示される
- **場所**: `display_pages.cpp` (`draw_gnss_page1()` L713–791, `draw_gnss_page2()` L794–821)
- **原因・メカニズム**: ページ3（L826–842）にはミラー中の明示バナーがあるが、ページ1・2には無く、送信機の衛星数/Fix と受信機自身の時刻/スカイプロット/フレーム経過秒が区別なく同一画面に並ぶ。
- **修正案**: `link_mirror_active()` 時はページ1・2にもミラー中である旨（`MIRROR: sats/fix=TX, skyplot=RX` 等）を表示する。

### [BUG-42] [Medium] 無線ミラー中の `VARIO DETAIL 1/2` で、`_msl_ready` 判定が受信機自身の GNSS を参照しており、送信機の MSL 高度に `(AGL)` と誤表示する
- **場所**: `display_pages.cpp` (`draw_vario_page1()` L365–393)
- **原因・メカニズム**: L378 の `bool _msl_ready = get_imu_gnss_offset_ready();` はミラーを通さず受信機自身の `_gnss_offset_ready` を返す。そのため送信機が 3D Fix して MSL 高度を送ってきていても、受信機が屋根下等で未 Fix だと送信機の KF 高度に `(AGL)` と誤表示され、`Diff` 行も `"--- (waiting GNSS fix)"` になる。また L374 の `GND/ISA` にも `(RX)` 表記が無い。
- **修正案**: `_mirror` 時は `_msl_ready = link_get_fix_ok() && link_has_value(RHAVE_KFALT);` で判定し、`GND/ISA` に `(RX)` を付記する。

### [BUG-43] [Low] `draw_gnss_page2()` の `millis() - 1000` 符号なしアンダーフロー
- **場所**: `display_pages.cpp` (L810)
- **修正案**: `if ((uint32_t)(millis() - get_gnss_nmea_time(i)) > 1000)` に変更する。

### [BUG-44] [Medium] VEML7700 未接続（`!s_als_ok`）時にも 30 秒静止で `s_dimmed = true` に遷移し、モード表示が `"NO SENSOR"` から `"IDLE DIM"` に変わってしまう
- **場所**: `backlight.cpp` (`dim_tick()` L428–555, `backlight_mode_label()` L614–621)
- **原因・メカニズム**: `dim_tick()` に `if (!s_als_ok) return;` が無く、`backlight_mode_label()`（L615）で `if (s_dimmed) return "IDLE DIM";` が `if (!s_als_ok) return "NO SENSOR";` より先に判定されるため、センサー未接続でも 30 秒静止すると表示が `"IDLE DIM"` に書き換わりセンサー未接続が隠蔽される。
- **修正案**: `backlight_mode_label()` で `if (!s_als_ok) return "NO SENSOR";` を最優先にし、`dim_tick()` でも `if (!s_als_ok) return;` する。

### [BUG-45] [Low] `backlight_als_setup()` の `delay(ALS_FIRST_READ_MS)` 直後の `millis()` 境界で初回測光が空振りする可能性
- **場所**: `backlight.cpp` (L349, L566–573)
- **修正案**: `read_als(s_als_t0 + ALS_FIRST_READ_MS)` を呼ぶか `delay(ALS_FIRST_READ_MS + 2)` にする。

### [BUG-46] [High] 起動時に IMU（SCH16T）の初期化が失敗（`get_imu_ok() == false`）した場合、`readiness` がエラーを立てず「準備完了（READY）」と発声してしまう
- **場所**: `readiness.cpp` (`any_setting_red()` L50–58, `evaluate()` L72–85), `src/button.cpp` (`imu_status_color()` L592)
- **原因・メカニズム**: `evaluate()` は `if (get_imu_ok())` の中しか見ず、`any_setting_red()` も `SETTING_IMU_DETAIL` をスキップし、`imu_status_color()` も `!get_imu_ok()` で `COLOR_GRAY` を返す。そのため J6 FPC コネクタの接触不良等で起動時に `imu_setup()` が失敗すると、`RDY_IMU` が一切立たず「準備完了」の音声が鳴ってしまう。
- **修正案**: v7.1 では `attitude_get_rpy_enabled()` が ON のとき `!get_imu_ok()` でも必ず `RDY_IMU` を立て、`imu_status_color()` も `COLOR_RED` を返すようにする。

### [BUG-47] [Low] `power.cpp` の充電ログ（`charge_log_tick()`）で、充電中に一瞬 SW OFF → ON すると `CHG START` / `CHG CV ENTER` / `CHG FULL` が古い経過時間で再出力される
- **場所**: `power.cpp` (`charge_log_tick()` L728–796)
- **修正案**: `s_chg_start_ms == 0` のとき（USB 挿抜時）だけ `s_cv_logged = false; s_full_logged = false;` にリセットする。

### [BUG-48] [Low] `power.cpp` の一部のタイマー起点保持で `now ? now : 1` ガードが漏れている
- **場所**: `power.cpp` (L480, L514, L559)
- **修正案**: 他箇所と同様に `now ? now : 1` に統一する。

---

## カテゴリ6：SDカード管理・Core1タスクキュー・リプレイ・生IMUロガー

### [BUG-49] [Low] Core1 上で動く `loadSettings()` 内から `log_sdf()` ではなく Core0 用の `enqueueTask(createLogSdfTask(...))` を呼んでいる
- **場所**: `mysd.cpp` (`loadSettings()` L337, L344, L349, L371, L377, L454, L472)
- **原因・メカニズム**: `loadSettings()` は Core1 専用関数だが、異常値検出時に `enqueueTask()` でタスクキューに積んでいるため、`settings.txt` に異常行が多いと `loop1()` 開始前に 40 スロットの `taskQueue` が溢れる。
- **修正案**: `log_sdf(...)` を直接呼ぶ。

### [BUG-50] [Medium] `process_mapcsv_line()` で最終頂点（`i == size - 1`）の経度・緯度間のカンマ欠落チェックがスキップされ、不正行でポリゴン名を緯度として読む
- **場所**: `mysd.cpp` (`process_mapcsv_line()` L1049)
- **原因・メカニズム**: 経度の直後のカンマを探す L1045 の後で `if (commaIndex == -1 && i < size - 1)` としているため、最終頂点 `i == size - 1` で経度の後のカンマが欠けているとエラーにならず、`index = commaIndex + 1` が `0` に戻って先頭のポリゴン名を `lat` としてパースしてしまう。
- **修正案**: L1049 を `if (commaIndex == -1)` に修正する。

### [BUG-51] [Low] `log_sd()` の `logtext[128]` バッファによる `log_sdf` メッセージ（最大256バイト）の切り詰め
- **場所**: `mysd.cpp` (`log_sd()` L2374–2397)
- **原因・メカニズム**: `Task::logSdfArgs.buffer` および `log_sdf()` のバッファは 256 バイトあり、`sdio_fail_desc[160]` などを渡しているが、`log_sd()` の `char logtext[128];` で約 118 文字以降が切り捨てられる。
- **修正案**: `logtext` を `char logtext[280];` に拡張する。

### [BUG-52] [Low] SdFat の `dateTime()` コールバックが未初期化（0年0月0日）または初回 GPS Fix 時の固定時刻を返し続ける
- **場所**: `mysd.cpp` (`dateTime()` L65–76, `saveRxCSV()` L2506–2510, `saveCSV()` L2568–2575)
- **原因・メカニズム**: `fileyear..filesecond` は `saveCSV()` の初回書き込み時に一度しかセットされないため、GPS Fix 前のファイルや RX モードで作成される `received/*.csv` では `fileyear == 0` のまま `FAT_DATE(0, 0, 0)` が計算され、アンダーフローした壊れた FAT 日時（2092年等）が記録される。
- **修正案**: `dateTime()` 内で最新の有効日時を参照し、未確定時は `2026-01-01 00:00:00` にフォールバックする。

### [BUG-53] [High] リプレイ用リングバッファにおける Core0 `replay_pop()` と Core1 `init_replay()` の `replay_tail` データ競合
- **場所**: `mysd.cpp` (L1431–1437, `replay_pop()` L1608–1631, `init_replay()` L1691–1706)
- **原因・メカニズム**: SPSC リングバッファの Consumer 側インデックス `replay_tail` を、Core1 の `init_replay()`（L1702–1703）が直接 `replay_head = 0; replay_tail = 0;` に書き換えている。Core0 の `replay_pop()` が実行中にこれと競合すると、Core1 が 0 にリセットした直後に Core0 が `replay_tail = 1` を書き込み、`replay_head == 0, replay_tail == 1` となって空のバッファが「15件データあり」と誤判定され、古いまたは未初期化の `ReplayRow` を 15 個連続でポップしてしまう。
- **修正案**: `init_replay()` と `replay_pop()` のインデックス操作を `rp2040_enter/exit()` で保護するか、`replay_init_seq` 変更時に Core0 側で `replay_tail = replay_head` に同期させる。

### [BUG-54] [Medium] `browse_sd()` の早期リターンで `loading_sddetail = true` が解除されず UI が `"Stand by ..."` で固まる
- **場所**: `mysd.cpp` (`browse_sd()` L2205, L2220)
- **原因・メカニズム**: L2205（`if (page < 0) return false;`）および L2220（`if (!root || !root.isDirectory()) return false;`）の早期リターンで `loading_sddetail = false;` が抜けている。
- **修正案**: 両方の `return false;` の直前に `loading_sddetail = false;` を追加する。

### [BUG-55] [Medium] `entry.getName(name, sizeof(name))` の戻り値未チェックと未初期化スタックバッファ参照
- **場所**: `mysd.cpp` (`browse_replay_files()` L2071, `browse_sd()` L2233, `list_csv_in_dir()` L2919)
- **原因・メカニズム**: `name` が未初期化のローカル配列であり、`entry.getName(name, sizeof(name))` の戻り値をチェックしていないため、SD カード上に 32 文字以上のファイル名が存在すると未初期化スタック領域を参照する。
- **修正案**: `char name[64] = {0}; if (!entry.getName(name, sizeof(name)) || name[0] == '\0') { entry.close(); continue; }` とする。

### [BUG-56] [High] WebSerial（PC接続）モード中に Raw IMU ロガーが SD カードへの書き込み・追記を継続する
- **場所**: `src/imulog.cpp` (`imulog_push()` L127–133, `imulog_write_buffer()` L288–346), `GPS_TFT_map.ino` (`flightlog_tick()` L1352)
- **原因・メカニズム**: `GPS_TFT_map.ino` L2035 で `imulog_set_paused(getReplayMode() || webserial_connected);` を呼んでいるが、`src/imulog.cpp` の `imulog_push()` は `s_paused` を一切チェックしておらず、`flightlog_tick()` L1352 も `!getReplayMode()` しか見ていない。そのため `webserial_prepare_sd()` で一度 `imulog_flush_and_close()` を呼んでファイルを閉じても、約 1.1 秒後に再びバッファが満杯になって `imuraw/*.bin` を再オープンし、WebSerial 転送の裏で SD 書き込みと `sync()` を継続してしまう。
- **修正案**: `imulog_push()` の冒頭に `if (s_paused) return;` を入れ、`imulog_write_buffer()` と `flightlog_tick()` でも `webserial_connected` をチェックする。

---

## カテゴリ7：サウンド・音声再生・バリオ音制御

### [BUG-57] [High] Core0 `update_vario()` と Core1 音声再生の `PIN_AMP_SD` / `PIN_PWMTONE` データ競合による警報無音化・バリオ停止漏れ
- **場所**: `src/sound.cpp` (`update_vario()` L1204–1293)
- **原因・メカニズム**:
  1. バリオ停止エッジ（L1230–1236, L1282–1288）で Core0 が `if (!wav_playing && !sin_playing)` を確認して `delay(3)` で 3ms 待機している最中に、Core1 が警報 WAV やマルチトーンを開始して `setAmplifierState(true)` を実行すると、3ms 後に起床した Core0 が無条件に `setAmplifierState(false)` を呼んでアンプをシャットダウンしてしまい、**その警報 WAV またはトーンが最後まで完全に無音化する**（さらに L1284 には `&& !tone_in_gap` も漏れている）。
  2. L1205 の `if (vario_inhibit) return;` が `prev_vario` の立ち下がり消音処理より前にあるため、バリオ発音中に `vario_inhibit = true` になると PWM が止まらずバリオ音が鳴りっぱなしになる。
- **修正案**: `vario_inhibit` を `should_vario` の条件に統合して立ち下がり処理を必ず通し、`delay(3)` の後にもう一度 `if (!wav_playing && !sin_playing && !tone_in_gap && !vario_mode)` を確認してから `setAmplifierState(false)` を呼ぶ。

### [BUG-58] [Low-Medium] 音声再生ステートマシンのエッジケース（音量0時のアンプ切り忘れ・無音トーン時のアンプ ON・`pending_wav` 再開漏れ）
- **場所**: `src/sound.cpp` (L574–589, L680–688, L999–1010, L1052, L1093, L1104)
- **原因・メカニズム**:
  1. `startPlayWav()` で `if (sound_volume == 0 && min_volume == 0) return;` が既存再生の中断と `wav_playing = true` の後に置かれているため、音量 0 の WAV が割り込むとアンプが ON のまま放置され `pending_wav` も再開されない。
  2. `startNextToneEntry()` / `loop_tone()` が実効音量 0 のときも `setAmplifierState(true)` を呼ぶ。
  3. WAV アンダーラン停止時（L685）と `stop_overdischarge_alert_core1()`（L1004）で `pending_wav` の再開処理が呼ばれていない。
- **修正案**: 音量 0 チェックを `startPlayWav()` 先頭へ移し、トーンでも実効音量 > 0 を確認してアンプを ON にし、全停止経路で `pending_wav` 再開ヘルパーを呼ぶ。

---

## カテゴリ8：LoRa無線リンク (E220-900T22S)・Wire2プロトコル・ミラーリング

### [BUG-59] [Medium-High] E220 のブロッキング復帰（`e220_wake` / `set_mode`）が AUX 自己診断 Low パルスを見逃す問題と、`link_probe_module()` の不要なモード2往復・RX ゴミ残り
- **場所**: `e220.cpp` (`set_mode()` L159–172, `e220_ping()` L494–531, `e220_wake()` L754–768), `link.cpp` (`link_probe_module()` L515–534)
- **原因・メカニズム**:
  1. E220 は mode 3 → mode 0 遷移から数 ms 遅れて AUX を Low に落とすが、ブロッキング版の `e220_wake()` と `set_mode()` は `delay(2)` の直後に `wait_aux_idle()` を呼ぶため、AUX が Low に落ちる前に素通りしてしまう。
  2. `link_probe_module()` は TX スリープ中に不要な `if (tx) e220_wake();` → `e220_ping()` → `if (tx) e220_sleep();` でモードを 2 往復（3→0→3→0→3）させており、また `e220_ping()` の `set_mode(MODE_NORMAL)` 後に `flush_input()` が呼ばれないため UART グリッチバイトが受信バッファに残る。
- **修正案**: `link_probe_module()` の不要な `e220_wake()` / `e220_sleep()` を削除し、`e220_ping()` で `leave_config_mode()`（`flush_input()` 込み）を使い、`e220_wake()` の待機を `E220_WAKE_SETTLE_MS` と整合させる。

### [BUG-60] [Medium] `link_w2_decode()` における日界（0時）ロールオーバー時の日付ズレと、`tod == 0xFFFF`（時刻無効）時の古い時刻残留
- **場所**: `lora_link/link_wire2.h` (`link_w2_decode()` L308–320, L363–372)
- **原因・メカニズム**:
  1. `st->hour` が `23 → 0` にロールオーバーした際、次の slot 0（`W2S_DATE`、最大 7 フレーム後）が届くまでの間、日付が前日のまま時刻だけが `00:00` になり、丸 1 日巻き戻ったタイムスタンプが出力される。
  2. `tod == 0xFFFF`（TX 時刻無効）のとき、`st->acc` に残っていた過去の時刻がそのまま出力され続ける。
- **修正案**: `tod == 0xFFFF` 時は時刻有効フラグを落とし、`st->hour` が `23 → 0` にラップした際は次の `W2S_DATE` が届くまで `st->have_mask &= ~1u;` で日付を一旦保留にするか日付を +1 日進める。

### [BUG-61] [Medium] PONS Link でグループ変更・プロファイル変更時に RX の Wire2 回転枠アキュムレータと「別グループ検出」表示がリセットされない
- **場所**: `link.cpp` (`link_set_radio_ch()` L1551–1569, `link_set_radio_profile()` L1571–1580, `link_set_group()` L1582–1588)
- **原因・メカニズム**: `link_set_group()` と `link_set_radio_profile()` で `link_w2_rx_reset(&s_w2rx)` と `s_haveLastSeq = false`、`s_otherGroup = false` が呼ばれていないため、グループ切替直後の最大 7 フレームに切り替え前のグループの回転スロット値が混入し、またグループを合わせた後も最大 10 秒間 `DIFF GROUP` 警告が消えない。
- **修正案**: `link_set_group()`・`link_set_radio_profile()`・`link_set_radio_ch()` すべてで `link_w2_rx_reset(&s_w2rx); s_haveLastSeq = false; s_otherGroup = false;` を実行する。

---

## カテゴリ9：PC・Web解析ツール・アセット生成・単体テスト群

### [BUG-62] [High] `tools/flight_viewer/index.html` で飛行 CSV の推定対気速度カラム名 `tas_mps` を `'tas'` で探しているため常に読み落とす
- **場所**: `tools/flight_viewer/index.html` (L1197)
- **原因・メカニズム**: ファームウェア（`mysd.cpp` L1484, L2526, L2596）が出力する列名は `tas_mps` だが、`index.html` L1197 は `tas: header.indexOf('tas') >= 0 ? header.indexOf('tas') : -1` と完全一致で検索しているため常に `-1` になり、CSV に記録された `tas_mps` がすべて無視される。
- **修正案**: `tas: header.indexOf('tas_mps') >= 0 ? header.indexOf('tas_mps') : header.indexOf('tas')` に修正する。

### [BUG-63] [High] `tools/flight_viewer/index.html` の WebSerial 通信で (1) モーダルを閉じると PING が止まり60秒後に本体が切断、(2) ヘッダの「PONS切断」ボタンで切断されない、(3) 転送中断/エラー時に `isDataStreaming` がロックする
- **場所**: `tools/flight_viewer/index.html` (L753, L2566–2570, L2647, L2732–2805, L2821)
- **原因・メカニズム**:
  1. `startSerialPing()`（L2734）が `isModalOpen` を条件にしており、ファイル読込完了時に `closeSerialModal()`（L2791）が `stopSerialPing()` を呼ぶため、ファイルを開いて 60 秒経過すると PONS 本体がアイドルタイムアウトして `MODE_PCCONNECT` を抜けてしまう。
  2. 接続時に `#btn-webserial` の表示を `'🔌 PONS切断'` に変える（L2647）のに、`connectWebSerial()`（L2567）は接続中に呼ばれると `openSerialModal()` を開くだけで切断しない。
  3. `isDataStreaming == true` の間は `checkDataEnd()` が `"PONS:DATA_END,"`（末尾 256 文字のみ）しか探さないため、転送中に本体側でキャンセル（`PONS:CANCEL_OK`）やエラー（`PONS:ERROR,...`）が起きると `isDataStreaming` が永久に解除されない。
- **修正案**: 接続中はモーダル開閉に関わらず PING を継続し、ヘッダボタンの動作とラベルを一致させ、`checkDataEnd()` で `PONS:CANCEL_OK` と `PONS:ERROR,` も検出する。

### [BUG-64] [Medium] `tools/flight_viewer/index.html` の日跨ぎ・時刻逆転時の負の `t`、`innerHTML` 未エスケープ、`findLastIndex` フォールバックバグ
- **場所**: `tools/flight_viewer/index.html` (L1236–1239, L1304, L1329–1348, L2689–2695)
- **修正案**: ソート後に `tMin < 0` なら全サンプルを `-tMin` シフトして 0 起点に正規化し、ファイル名表示には `textContent` を使い、`findLastIndex` 非対応環境用のループフォールバックを実装する。

### [BUG-65] [High] `tools/vectormap/build_vectormap.py` で `--pbf` と `--variant hires` を併用すると、`VM_PARK`・`VM_BUILDING`・`VM_STREET` が Overpass フォールバックもされずに 0 件のまま黙って欠落する
- **場所**: `tools/vectormap/build_vectormap.py` (L384–390, L412, L434–480, L795–799)
- **原因・メカニズム**: `fetch_all_from_pbf()`（L412）が `out = {c: [] for c in classes}` で全クラスの空リストを作る一方、`PBF_TAGS`（L384–389）には `VM_WATER`, `VM_MOTORWAY`, `VM_TRUNK`, `VM_RAIL` の 4 種しか定義されていない。そのため `build_lod()` L795 の `elif pbf_cache is not None and cls in pbf_cache:` が `True`（中身は `[]`）になり、L800 の Overpass フォールバックもスキップされて公園・建物・街路が 0 件になる。
- **修正案**: `PBF_TAGS` と `_PbfCollector` に `VM_PARK`, `VM_BUILDING`, `VM_STREET` を追加するか、`out = {c: [] for c in classes if c in PBF_TAGS}` とする。

### [BUG-66] [Medium] `tools/vectormap/build_vectormap.py` で `--lods 0` 等の部分 LOD 指定実行時に、指定外の LOD が空の状態で `vectormap_data.cpp` を上書きする
- **場所**: `tools/vectormap/build_vectormap.py` (L28, L878–882, L1035–1042)
- **修正案**: 部分 LOD 指定時は `--no-write` を案内するか警告を出す。

### [BUG-67] [High] `tools/png2bmp.py` が Pillow 10.0+ で削除された `Image.FLIP_TOP_BOTTOM` を参照してクラッシュする＋ `clean_wav8.py` の `/tmp` ハードコード
- **場所**: `tools/png2bmp.py` (L1, L38, L96–98), `tools/clean_wav8.py` (L168)
- **修正案**: `Image.Transpose.FLIP_TOP_BOTTOM` を使用し、不要な `import requests` を削除し、`clean_wav8.py` で `tempfile` を使用する。

### [BUG-68] [High] `tools/createmovie_csv2mp4.py` が旧カラム名 `data['Altitude']` を参照して `KeyError` でクラッシュ・小数秒で `ValueError`・`flight_preprocess.py` の風向 180° 定義差
- **場所**: `tools/createmovie_csv2mp4.py` (L238, L381, L692), `tools/flight_preprocess.py` (L630)
- **原因・メカニズム**:
  1. `createmovie_csv2mp4.py` L238 が `GNSS_Altitude` ではなく `data['Altitude']` を参照しており現行 CSV でクラッシュする。
  2. 同 L692 の `seconds_to_time()` が `map(int, start_time_str.split(':'))` のため小数秒で `ValueError` になる。
  3. 同 L381 の `if line.startswith('0:SD INIT'):` は SD 初期化が 1 秒以上かかった起動（`1:SD INIT` 等）を見落とす。
  4. `flight_preprocess.py` L630 の `wind_dir` は気象風向（吹いてくる方向 `atan2(-w_e, -w_n)`）で、機上 CSV（吹いていく方向 `atan2f(wind_e_, wind_n_)`）と 180° 逆になっている。
- **修正案**: `GNSS_Altitude` と `Altitude` の両対応、`float` 秒パース、正規表現 `^\d+:SD INIT` マッチに修正する。

### [BUG-69] [Medium] `tools/kml_to_mapcsv.py` の 2D KML クラッシュと、`logcsv2kml4earth.py` / `logcsv2tourkmz.py` のタイムゾーン `'Z'`（UTC vs JST）9時間ズレ
- **場所**: `tools/kml_to_mapcsv.py` (L19–27), `tools/logcsv2kml4earth.py` (L45), `tools/logcsv2tourkmz.py` (L40, L50)
- **修正案**: `kml_to_mapcsv.py` で 2 要素座標 `(lon, lat)` に対応し、KML 出力のタイムスタンプ末尾を `'Z'` から `'+09:00'` に修正する。

### [BUG-70] [Medium] `tools/imulog/eskf.py` のヨー分散上限クランプ欠落、`decode_imulog.py` の GPS 週ロールオーバー誤警告、`merge_flight.py` のゼロ除算・カラムエラー
- **場所**: `tools/imulog/eskf.py` (L242–243), `tools/imulog/decode_imulog.py` (L324–330), `tools/imulog/merge_flight.py` (L159, L170)
- **原因・メカニズム**:
  1. `eskf.py` に `ESKF_YAW_SIGMA_MAX_DEG`（180°）クランプが無い（BUG-05/B-05 参照）。
  2. `decode_imulog.py` L324 の `iTOW` 差分に `% 604800000` が無く、土曜 08:59:42 JST（GPS 週ロールオーバー）を跨ぐセッションで負値になり `NG` と誤判定する。
  3. `merge_flight.py` L159 でセッション長 0 秒のとき `ZeroDivisionError` になり、L170 で `received/*.csv` 等が混ざると `KeyError` になる。
- **修正案**: `eskf.py` に上限クランプを追加し、`decode_imulog.py` で modulo `604800000` を取り、`merge_flight.py` に期間チェックと必須カラムチェックを追加する。

### [BUG-71] [High] `tools/flightanalysis/` の3スクリプトが `GNSS_Altitude`（GPS高度）を気圧高度と誤認して解析しているバグ、および NumPy 2.0+ での `np.trapz` クラッシュ
- **場所**:
  - `tools/flightanalysis/plot_phugoid.py` (L31–33, L174, L188–190, L468–483)
  - `tools/flightanalysis/plot_phugoid_windows.py` (L144, L154–155)
  - `tools/flightanalysis/plot_vertical_raw.py` (L106, L118, L121)
  - `tools/flightanalysis/plot_spectrum.py` (L108), `plot_spectrogram.py` (L117), `plot_lateral_modes.py` (L226), `plot_vertical_accel.py` (L383)
- **原因・メカニズム**:
  1. `plot_phugoid.py` 等 3 本は `d = d.rename(columns={'GNSS_Altitude': 'Altitude'})` とした上で `vs_baro = np.gradient(alt, tg) # 気圧高度の微分` として解析しているが、PONS の `GNSS_Altitude` は GPS 高度であり気圧高度ではない（気圧は `pressure` 列から換算する必要がある）。
  2. `plot_spectrum.py` 等 4 本で使用している `np.trapz` は NumPy 2.0 で削除されており `AttributeError` になる。
- **修正案**: 気圧高度は `plot_pitch_altitude.py`（L100–102）と同様に `d['pressure']` から計算し、積分には `getattr(np, 'trapezoid', getattr(np, 'trapz', None))` を使う。

### [BUG-72] [Low-Medium] `tools/attitude_test/dead_test.cpp` のセンサー軸加速度向き不整合、死蔵された `attitude_test/settings.h`、`sch16t_test.ino` の初回 `dt`
- **場所**: `tools/attitude_test/dead_test.cpp` (L14), `tools/attitude_test/settings.h` (L1–38), `tools/sch16t_test/sch16t_test.ino` (L1183–1196)
- **原因・メカニズム**:
  1. `dead_test.cpp` L14 はセンサー座標系で `a0[3] = {0, 0, 9.80665f}`（機首真下 90° のジンバルロック特異点姿勢）を入力しており、`main.cpp` / `yaw_test.cpp`（水平姿勢 `{-9.80665f, 0, 0}`）と向きが異なる。
  2. `tools/attitude_test/settings.h` は旧定数のまま残された死蔵ファイルで、誤ってインクルードされると本体 `settings.h` と衝突する。
  3. `sch16t_test.ino` の `cmd_gyro_scale()` で最初の数サンプルが `!s.ok` だった場合、最初の有効サンプルの `dt` にそれまでの待ち時間が全て含まれる。
- **修正案**: `dead_test.cpp` の加速度を `{-9.80665f, 0, 0}` に揃え、`attitude_test/settings.h` を削除または `#include "../../settings.h"` に置き換える。

