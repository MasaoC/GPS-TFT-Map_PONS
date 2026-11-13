# CLAUDE.md

> ## このブランチは **v7.1 基板（SCH16T を J6 の FPC コネクタで外付け）専用**
>
> 初代 v7（BNO085 搭載）基板は **`PONSv7`** ブランチ。**2026-10-23（0.995）にブランチごと分けた。**
> それまで両ブランチ共通だったファームから、このブランチでは v7 だけのコード（`PONS_BOARD` の分岐・
> BNO085 のドライバと取り込んだライブラリ）と文書を外した。**基板を切り替えるスイッチはもう無い。**
>
> - **`kicad/` は v7.1 基板のデータ。**
> - **航法・音・無線・画面など基板に関係しない直しは、`PONSv7` へ cherry-pick で渡す**（逆も同じ）。
>   分けたあとはコードが少しずつ違うので、マージではなく 1 コミットずつ取り込み、
>   衝突したら受け取る側の基板に合わせる。
> - **無線（PONS Link）は v7 の機体とも混ぜて使う。** プロトコル（`lora_link/link_proto.h`）と
>   飛行 CSV の列は両ブランチでそろえておくこと。
> - **SCH16T は本体基板で起動と机上確認（Step 3）まで済み（2026-11-13）。まだ飛ばしていない**（`#warning "... DO NOT FLY."`）。
>   残りは [docs/imu_sch16t_bringup.md](docs/imu_sch16t_bringup.md) の Step 4〜5。
>   マウント回転（`IMU_MOUNT_Q*`）は v7.1 で測り直して **v7 と同じ向き**だったので据え置き。
>   **ESKF のゲインとサンプル間隔は 250Hz（4ms）に適合済み**（`tools/attitude_test` で検証済み）。ブリングアップ時は SCH16T の実機データで確認する。
>   **ESKF の dt は直前のサンプルとの実際の時間差**（2026-11-10〜。SCH16T は 1 回読むと 1 サンプルで、まとめて届かない）。
>   それまでの 64 サンプルの平均間隔は BNO085 のまとめ届き向けで、**PONSv7 では正しいので cherry-pick しないこと。**
> - **v7.1 は SCH16T を必ずつなぐ**（2026-11-10 決定）。起動時に見つからないのも、あとで黙ったのも IMU の欠け
>   （本番前チェックの `IMU`・設定画面の赤の ×）。`get_imu_ok()` は通信が 1 秒途絶えると false に変わるので、
>   「載っていない機体」の判定には使えない。これも v7.1 だけの決めごとなので PONSv7 へは持ち込まない。
> - BNO085 用の能力ビット（`imu_caps()`）・融合出力（GRV / RV / LACC）・動的校正（DCD）の
>   コードと画面は **2026-11-03 に実機確認のうえ削除した。** PONSv7 から IMU 周りの直しを
>   cherry-pick するときは、受け取る側（SCH16T）に合わせて読み替える。

PONS v7 — 人力飛行機パイロット向けの GNSS 航法装置ファームウェア。
RP2350B(80QFN) 自作基板 + 2.8" TFT、Arduino（arduino-pico 4.5.1）。

**コメントとドキュメントは日本語で書く。**
ここには「ソースを読んでも分からないこと」だけを置く。仕様の説明は
[README.md](README.md)（開発者向け）と Google Docs の説明書（利用者向け）にある。

---

## ビルド

Arduino IDE で `GPS_TFT_map.ino` を開く。ボードは **Generic RP2350** / Flash **16MB (no FS)**。
必要ライブラリは `TFT_eSPI` / `SdFat`。**SdFat は 2.3.1 以上**（2.3.0 は RP2350 で SDIO の
PIO を確保し損ね、起動時の SD が `err=0x01` で落ちる。2026-11-05。古いと `mysd.cpp` の `#error` で止まる）。
**2.3.1 は Library Manager に無い**（同梱の `pioasm.exe` で Arduino の登録を弾かれている）ので、
GitHub の 2.3.1 を `libraries/SdFat` に手で入れる。古い方を `libraries/` の中に残さないこと。
**ライブラリはスケッチに保存されない**ので、ボード設定と同じく別の PC では入れ直しを確かめる。

**Flash Size は 16MB（no FS）。** 音声を FLASH へ焼いた 0.982 から所要が約 3.7MB に
なり、2MB 指定では `region FLASH overflowed` でリンクが通らない。
**IDE のボード設定はスケッチに保存されない**ので、別の PC や IDE の入れ直し後に必ず踏む。

コンパイル確認だけなら CLI が使える（IDE 同梱の arduino-cli。動作確認済み）:

```sh
"/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli" \
  compile -b rp2040:rp2040:generic_rp2350:flash=16777216_0 --build-path /tmp/ponsbuild .
```

- **TFT_eSPI は User_Setup を書き換えないと映らない。** サンプルは
  [TFT_eSPI/CopySetupFile_TFT_eSPI.h](TFT_eSPI/CopySetupFile_TFT_eSPI.h)。
  パネル種別（ST7789 / ILI9341）の選択もここで、`settings.h` 側には分岐が無い。
- `VECTORMAP_HIRES` は先に `tools/vectormap/build_vectormap.py --variant hires` で
  データ生成が要る（未生成なら `#error` で止まる）。
- **地図の元データとキャッシュ（数 GB）はスケッチのフォルダの外**、Arduino フォルダ直下の `pons_vectormap_data/` に置く
  （`build_vectormap.py` の既定。2026-11-07〜）。**Arduino はビルドのたびにスケッチのフォルダを丸ごと（`.json` も）
  ビルド先へコピーする**ので、`tools/` の中に Overpass の応答（1.4GB）を置くと、ビルド 1 回ごとに 1.4GB 写されて
  ディスクが溢れる（実際に起きた）。`tools/` に大きな `.json` / `.md` / ソースの拡張子のファイルを置かないこと。
- **音声は本体 FLASH に焼いてある。** 原本は [wav/](wav/)（SD ではない。0.982 で移した）。
  差し替えたら `python3 tools/gen_wav_flash.py` で
  [src/flashdata/wav_data.cpp](src/flashdata/wav_data.cpp) を作り直すこと。
  **忘れると古い音のまま黙って焼ける。** 生成物は git に入れてあるので、
  音を触らないなら実行は不要。
- **案内の日本語の画像も本体 FLASH に焼いてある**（IMU/ESKF 画面の入口の 3 枚と、推定 TAS の案内の 1 枚）。
  本体に日本語フォントは無いので、文字ごと画像にしている。原本は [tools/imu_guide/](tools/imu_guide/) と
  [tools/tas_guide/](tools/tas_guide/) の SVG（別の AI に作らせるときの指示文は `tools/imu_guide/prompt.md`）。差し替えたら
  `python3 tools/gen_guide_flash.py --render`（Chrome で等倍の PNG にしてから
  `src/flashdata/imu_guide_data.cpp` / `tas_guide_data.cpp` を作る）。
  **忘れると古い画像のまま黙って焼ける。** 1 画面ごとの音声は `.ino` の `IMU_GUIDE_WAV[]` / `TAS_GUIDE_WAV[]` と順番で 1 対 1
  （TAS の案内の 2 画面目は曲線をファームが描く）。**TAS の案内の音声 2 本（`guide_tas_*.wav`）は Mac の読み上げ（Kyoko）で
  作った仮のもの**（2026-11-07）。

### リリース前チェック（すべて [settings.h](settings.h) 冒頭）

- `RELEASE` — 有効に。無効だと `setup()`（Core0）が `while(!Serial)` で待ち、実機が起動しない。
- `RELEASE_GNSS` — GNSS シミュレーション（`DEBUG_GNSS_SIM_*`）が全部コメントアウト済みか。
- `BUILDDATE` / `BUILDVERSION` — 更新する。README のバージョン記載も合わせる。
- どちらかが抜けていると `#warning NOT RELEASE!` が出る。これが唯一の保険。
- `python3 tools/gen_wav_flash.py --check` — `wav/` と焼き込み済みの音声が
  CRC32 で一致するか。音声を差し替えて生成を忘れていると非 0 で落ちる。
- `python3 tools/gen_guide_flash.py --check` — 案内画像の PNG と焼き込み済みの画像が一致するか。

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
cd tools/attitude_test && make   # 協調旋回 / 水平化とヨー注入 / 重力観測 / IMU 断線時
```

**`attitude.cpp` と `tools/imulog/eskf.py` は同じ数式の二重実装。
片方を直したらもう片方も必ず合わせる。** バリオの KF（`imu.cpp`）と `tools/imulog/vario.py` も同じ関係
（2026-11-11〜。予測の周期・Q・観測・`KF_ACC_GUARD_*` のガード。合わせていない点は vario.py の頭に書いてある）。

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
- **状態を読み上げる音声には排他グループ（`WAV_EXCL_*`）を付ける。** pending は
  「割り込まれた音声の救済」だが、いまの状態を述べる読み上げには害になる。0.984 まで、
  無線モードを RECEIVER へ回す途中で通過した `sender_mode.wav` が pending から復帰し、
  **受信モードにした機体が「送信モード」と読み上げていた**（操作ミスに気づく手段の反転）。
  **無線モードは 3 状態とも読み上げる。OFF を無音に戻さないこと**（0.985〜）。
  一覧と仕組みは [docs/pons_sound.md](docs/pons_sound.md) §3「排他グループ」。
- **pending キューは `min_volume` も運ぶ。** 落とすと `battery_low.wav`
  （優先度 1・最低音量 60）が pending から復帰したときに 60 を失い、
  音量設定 0 の機体で**電池切れの警告が完全に無音になる**
  （優先度 1 は最低なので、他の WAV とぶつかれば必ず pending を通る）。
- **表示の供給元は 3 系統ある。** 実センサー / リプレイ（`getReplayMode()`）/
  無線ミラー（`link_mirror_active()`）。表示項目を足すときは 3 つとも考える。
  **デモ（`is_demo_active()`）は 4 つめ**だが、差し替えるのは GNSS 値とヨー精度だけで、
  姿勢・気圧は実センサーのまま。
  **リプレイ中・ミラー中は SD への記録を止める**（`flightlog_tick()` と
  `try_enque_savecsv()`）。再生した日付のファイルへ現在値を書いて実飛行ログを
  汚さないため。
- **警報を描画関数の中に書かない。** その画面を出しているときしか鳴らなくなる。
  発報は `loop()` の最上位にある `*_tick()` 群へ（`nav_alarm_tick` / `bank_alarm_tick` /
  `batt_alarm_tick` / `ground_check_tick` …）。**ここに描画を混ぜないこと**、
  逆に `draw_current_screen()` 以下に判断を混ぜないこと。
- **USB の有無は `usb_power_present()`（[power.cpp](power.cpp)）だけで判定する。
  VBUS の ADC（GPIO47）を直接読まない。** 2026-10-15、v7 の充電 IC が壊れて VBUS を
  持ち上げ、「USB」と誤判定して**電池低下警告まで止めていた**（VSYS 2.5V）。
  電源経路の異常（`power_fault_active()`）の表示と音だけは**ミラー中・リプレイ中も出す**
  （自機が発熱している話なので「ミラー中に自機の状態を混ぜない」の例外）。
  経緯と全基板の対応は [docs/pons_power.md](docs/pons_power.md)。
- **電池電圧を表示・警告・送信・記録するときは `power_sw_off_on_usb()` を見る。**
  SW を OFF にしたまま USB で動いていると GPIO40 は SW4 で GND に落ちていて、
  `get_input_voltage()` は 0V へ下がる。0V を本物として扱うと、残量 0%・電池低下の誤報
  （自機と受信機）・CSV の 0.00V になる。いまは電池表示・`batt_alarm_tick()`・
  `link_push_telemetry()`（`RHAVE_VOLT` を立てず `LINK_ST_USB_SW_OFF` を立てる。受信機は `s5V`）・
  CSV（空欄）で処理してある。**`usb_power_present()` で代用しないこと** — VBUS が一瞬
  4.3V を割っただけで外れ、その瞬間の 0V が流れる。
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
  ミラー中の原則を裏返した同じ間違いで、基準も同じ
  **「その表示は、いま画面に出している数字の話か」**。0.984 まで受信枠・
  `NO SIGNAL` ポップアップ・艇アイコン・S/R 電池が再生中の画面に出ていた。
  **`link_mode_setting` は書き換えない** — SD に保存される設定なので、デモのために
  OFF を代入すると**翌日そのまま無線が黙って飛ぶ**。
  デモを止める理由は見えにくい: デモは `stored_*` を上書きしないので**CSV には
  残らない**のに、`link_push_telemetry()` は accessor 経由なので**仮想機体が電波に乗る**。
  一覧は [docs/pons_link.md](docs/pons_link.md) §5。
- **電波に載るのは `LinkTelem`（67 バイト）ではなく 32 バイトのフレーム**
  （[lora_link/link_wire2.h](lora_link/link_wire2.h)。2026-10-27〜、どの CH でも）。`LinkTelem` はメモリの上の形で、
  送る直前に詰めて受けたらすぐ戻す。**形式を見分ける仕掛けは印だけ**なので、並びの違う版とは通じない（3 台そろえる）。
  並びを変えたら `LINK_W2_MAGIC` も変え、PONSv7 / PONSv7.1 で並びを同じに保つ
  （debug/v7-noise-test は 2026-11-07 にこの約束から外した。古い `0xA2` のままで、印で弾くので誤読はしない）。
  **印は上位 6 ビットで、下位 2 ビットはグループ（A〜D）**（2026-11-07〜、`0xA4`）。印は必ず
  `link_w2_is_magic()` で比べること — **バイトごと `==` で比べると別グループの電波が「無信号」になり**、
  SD が飛んでグループが A に戻った事故に気づけなくなる（`e220.cpp` の同期探しも同じ）。変えるときは 4 つ飛ばし（`0xA8` …）。
  **GNSS の時刻の助け合いのビーコン（印 `0x58`。[lora_link/link_beacon.h](lora_link/link_beacon.h)。2026-11-13〜）も同じ 32 バイト**で、
  e220.cpp の切り出しは両方を拾う（`link_frame_is_magic` / `link_frame_valid`）。テレメトリの印を変えるときはビーコンの印とぶつけないこと
  （`static_assert` で止まる）。ビーコンは `link_recv_telem()` が `link_assist.cpp` へ渡し、**受信の数・seq・ミラー・NO SIGNAL・
  DUP SENDER・GROUP X FOUND・送信前チェック・チャンネル調査には混ぜない**。
  ゆっくりした項目は 8 発に分けた回転枠で運ぶので、**have ビットの無い目的地・ナビモード・縮尺は
  `link_rx_slot_known()` を見てから使う**（受け始めは 0 のままで、使うと受信機が目的地 0 に書き換えられる。
  縮尺は 2026-11-12 から受信機が合わせなくなり、設定画面に送信機の縮尺を添えるだけだが、そこに 0 ＝最も広い縮尺と嘘が出る）。
  **目的地は番号ではなく `dest_id`（名前と座標の CRC16）で照合する**（`link_sender_dest_local()`）。番号で合わせると、
  3 台の destinations.csv やパイロンの上書きが違うとき**黙って別の場所**に合う。`dest_id` の 0 は「送っていない」。
  アッパーバンド（CH15 / 21 / 27）は休止の決まりに合わせて送る間隔を延ばし、SF11 は使わない（docs/pons_link.md §2）。
- **PONS Link のテレメトリは下り一方向。上りは GNSS の時刻の助け合いのビーコンだけ**（2026-11-13〜。`link_assist.cpp`）。
  大会のルールは「上りの通信路で操縦しない（ラジコン禁止）」で、上りの通信そのものは禁止されていない（2026-11-08 訂正。
  以前ここに「上りの通信路を持たないのがレギュレーション上の要件」と書いていたのは誤り）。上りで機体が変えるのは
  **「GNSS へ時刻を 1 回入れる」ことだけ**（`gnss_dbd_give_time()`）。上りを足すときも、**機体の操作・設定・表示・航法を
  上りから変えられる経路は作らない**こと、**下りのテレメトリ（PONS の本来の役割）に影響させない**こと
  （機体の 1Hz の送信はタイミングも中身も変えず、status に `LINK_ST_TIME_WANTED` を立てるだけ）。
  **別のグループの時刻は使わない**。テレメトリの回転枠 0 の日付と時は **UTC**（飛行 CSV は JST に直して書くが、電波は UTC）。
- **`.ino` の初期化順のコメントは消さない。** `core1_separate_stack` /
  `mutex_init` の位置 / `setup1()` 内の `Serial.println("")` は、
  どれも消すと動かなくなる理由が書いてある。
  **`link_setup()` は起動画面より前、`link_preflight_restart()` は `setup()` の
  末尾**、という並びも同じ種類の約束。前者は起動画面に E220 の結果を出すため、
  後者は送信前チェックの監視窓が `millis()` 基準だから
  （起動画面のアニメーション約 8 秒を挟むと窓が先に過ぎ、雑音を 1 つも取れずに
  NOT MEASURED になる）。読み上げ `link_announce_mode()` も末尾。
  起動音より先に喋らせないため。
  `setup1()` が `setup_sd()` の前に `tft_pio_ready` を待つのも同じ種類。TFT（16bit パラレル）と
  SD（SDIO）はどちらも PIO を使い、別々のコアで同時に確保すると片方が死ぬ（[display_tft.h](display_tft.h)）。
- **ピンの割り当ては `settings.h` の冒頭（「基板」の節）に 1 つだけ置いてある。** ピンを足すときもそこへ。
  v7 と同じ GPIO が逆の意味になっているものが多い（41-44 は SPI1、M0/M1 は GPIO31）ので、
  v7 のコードや文書を持ち込むときは必ずそこと突き合わせる。
- **GPIO42 / GPIO44 を `INPUT_PULLUP` で停めないこと。** v7 では I2C バスを守るための処置だったが、
  v7.1 では SPI1 の SCK / MISO で、**SPI1 が死ぬ**。
- **E220 の M0/M1（GPIO31）は e220.cpp だけが駆動する。** ほかから触ると E220 が mode 3
  （Config/DeepSleep）へ落ちて無線が黙る（v7 で IMU の初期化から触って、飛行中に無線が黙った）。
- **E220 の AUX と mode 3 の扱いは [docs/pons_link.md](docs/pons_link.md) §6 に
  全部ある。触る前に読むこと。** どれも一度ずつ嵌まっていて、症状は決まって
  「**黙って捨てられる。エラーも返らない**」:
  **切替は前だけでなく後も AUX を待つ**（遷移中のセルフチェックに書き込みがぶつかる。
  v7 立ち上げで最も長く嵌まった）/ **起床は「いま High か」ではなく立ち下がりを見る**
  （`e220_send()` は成功を返すのに電波が出ない。実体は `s_wakeSawLow` と
  `E220_WAKE_SETTLE_MS`）/ **mode 3 へ潜るときは `enter_config_mode()` を通す**
  （落とすと最初の 1 バイトが化け、画面が「Module NO RESPONSE」になる）/
  **状態レジスタは Strict Mode でしか読めない**（`0x09` bit6 は不揮発。ver.1 互換の
  `C0 C1 C2 C3 00 02` は**使ってはいけない** — 返事が無く、代わりに**電波が出る**）。
- **GNSS の UART へ書く前に `gnss_dbd_finish_tx()` を呼ぶ。** SD に置いた航法データベース（gnss_dbd.cpp）は、
  Core0 を待たせないようにフレームを UART の送信 FIFO が空いた分だけ少しずつ書いている。書きかけのところへ別のコマンドを
  書くと、**両方とも GNSS に黙って捨てられる**。gnss.cpp の `ubx_send_packet()`（`gnss_ubx_send()`）と NAV-SAT のレート切替、
  .ino のデバッグ中継は呼んである。GNSS へ書く口を足すときはここを通すこと。
  また吸い出しは UART を数秒埋めて NAV-PVT を遅らせるので、**止まっているときだけ**にしてある（`GNSS_DBD_STILL_MS`）。外さないこと。
- **GNSS へ BBR 層で書いた設定は、BT1 が生きている限りファームを書き替えても残る。**
  いま BBR に書いているのは `CFG-ANA-USE_ANA`（AssistNow Autonomous）だけ。やめるときは
  送るのをやめるのではなく「無効」を書き込むこと（`GNSS_USE_ANA` を 0 にしても OFF を送るのはそのため）。
  **起動時の ANA は、最初の NAV-PVT で BT1 の生死が分かってから送る**（`gnss_ana_boot_tick()`。2026-11-13〜）。BT1 が切れていた
  起動（コールドスタート）だけ切るためで（SPG 5.10 はコールドスタートのあと ANA が有効だと QZSS を使わない）、
  `gnss_setup_impl()` の ⑦ で起動時に「有効」を送るように戻すと、**コールドの起動で黙って QZSS を失う**。
  ほかの設定（ボーレート・出力・Airborne）は RAM にだけ書いていて、電源を切れば工場出荷に戻る前提で
  `gnss_setup()` が組んである（毎回 9600 で開いて 38400 へ切り替える）。BBR へ移すならそこも見直す。
  **本体が動いたまま GNSS だけがリセットしても同じく工場出荷に戻る**（NMEA だけで UBX を出さない）。
  こちらは NMEA を読まないので、`gnss_ubx_watchdog()` が「バイトは来るのに UBX が来ない」を見て設定を一式送り直す
  （2026-11-04〜。それまでは再起動するまで位置が戻らなかった）。設定を足すときは `gnss_setup_impl()` の
  `send_config` の側に足すこと（起動時と送り直しの両方で送られる）。
- **log.txt の `grf` 行（GNSS の受信状態の 1 分まとめ。`gnss_log_rf_minute()`）は、debug/v7-noise-test と
  PONSv7.1 で同じ並び・同じ数え方に保つ。** v7 と v7.1 の基板の GNSS の受け方を比べるための行なので、
  片方だけ変えると比べられなくなる（2026-11-04〜）。
- **log.txt の毎分の行（`volt=` / `grf` / `imu gapmax=` / `CHG [..m]:`）と `BOOT PONS` / `GNSS TIME:` / `GNSS FIXED` の形を変えたら、
  Flight Viewer の log.txt の表示（`tools/flight_viewer/index.html` の `logScanLine` / `LOG_CATS`）も合わせる。**
  ビューアは正規表現で拾っているので、読めなくなった値は**グラフと起動のまとめから黙って消える**（2026-11-11〜）。
  v7 の機体も v7.1 のリポジトリのビューアで見るので、両ブランチの行の形をそろえておく。
- **送信したら AUX の立ち下がりで「本当に送ったか」を確かめる。**
  `link_tx_tick()` の `LTX_TXSTART` が AUX の Low（＝動き出した証拠）を待ち、
  `LTX_TXDONE` が High へ戻る（＝送信完了）のを待って mode 3 へ落とす。
  Low が来ないまま `LINK_TX_TXSTART_MS` 経ったら**モジュールに捨てられた**ので
  `noRf` を数える（60 秒ログ `LINK TX: ... noRf=`）。
  **`Sent` は「UART へ書けた回数」でしかない。** 電波が出た証明にはならない。
- **IMU の読み出しを飢えさせない。** SCH16T は**こちらから 250Hz で取りに行く**方式で（FIFO が無い）、
  読み出しが空くとその間のサンプルが抜ける。LPF2 の最小推奨 150Hz を下回ると阻止域のノイズが
  折り返す（settings.h の `IMU_POLL_INTERVAL_US`）。
  **2026-11-11 から読み出しは Core0 のタイマー割り込み**（`SCH16T_READ_IN_IRQ`）で、`loop()` が止まっても抜けない。
  `imu_service_if_due()` は「溜まった分を推定へ渡す」になったが、**長いブロッキング処理の中では引き続き呼ぶこと**
  （地図描画のレコードごと・航跡の点ごと・面の塗りの 16 行ごと・E220 の AUX 待ち・ボーレート切替・起動画面の各待ちに入れてある）。
  渡すのが遅れると推定が遅れ、512ms を超えるとリングが一杯で読めなくなる（60 秒ログの `ovf=`）。`0` にしたときのポーリングでは今までどおり抜ける。
  60 秒ログの `gapmax=` が読み出しの間隔（割り込みならふだん 4ms）、`drain=` が Core0 が受け取りに来られなかった最長時間。
  描き直しのどこで時間がかかっているかは `redraw` 行（GPS_TFT_map.ino の `redraw_prof_*`）。
  **さらに 1 秒受け取らないと「通信途絶」と誤判定して、生きているチップをリセットしに行く**
  （v7 で実際に起き、復旧に失敗してそのまま死んだ）。`imu_drv_poll()` の冒頭に
  ガード（呼び出し間隔が空いた回の判定は見送る）を入れてあるが、ガードに頼らず待ちループ側でも呼ぶこと。
- **IMU の割り込みの優先度は最低のまま。SPI1 をスレッド側から触るときは割り込みを止める。**
  音声の 16kHz のタイマー割り込みも Core0 で走っていて（既定のアラームプールは起動時に Core0 で作られる）、
  同じか上の優先度にすると 1 回 0.3ms ずつ音声を待たせて 250Hz のうなりになる。SPI1 は SCH16T 専用で、
  スレッド側で使うのは `sch_init()`（起動と復旧）だけ。`try_recovery()` が `irq_stop()` してから呼んでいる。
- **生ログ（`src/imulog.h`）の レコード ID を再利用しない。** 退避済みの過去ログは
  id だけでレコードを解釈するので、止めた ID に別の意味を割り当てると
  **古いログが静かに誤デコードされる**（値はもっともらしく出る）。
  0.984 で止めた `IMULOG_ID_MAG 0x03` と、v7 のログにだけ出る `0x04〜0x06`（GAMERV / RV / LINACC）は
  欠番としてコメントを残してある。
  `tools/imulog/decode_imulog.py` の `KINDS` も同じ理由で定義を残す。
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
| [display_tft.cpp](display_tft.cpp) | TFT 描画**だけ**。判断や発報は置かない。飛行中に出るもの（地図・ヘッダ・フッタ・無線の枠） |
| [display_pages.cpp](display_pages.cpp) | 地上で見る詳細画面（VARIO / GNSS DETAIL）。スプライトは display_tft.cpp のものを借りる |
| [attitude.cpp](attitude.cpp) | 姿勢 ESKF。風推定と自動ロールトリムもここ |
| [imu.cpp](imu.cpp) | バリオ用 Kalman filter と推定側のゲッター。**チップ非依存** |
| [imu_sch16t.cpp](imu_sch16t.cpp) | SCH16T ドライバ（0.991 で実装）。**チップ依存はここだけ。** SPI1 の 48bit フレーム・初期化・サンプル引き取り（2026-11-11〜 タイマー割り込みで読んでリングバッファへ積み、`loop()` 側で推定へ渡す）。**本体基板では起動と机上確認まで（Step 2〜3）**で、まだ飛ばしていないので `#warning` で「飛ばすな」が出る。手順は [docs/imu_sch16t_bringup.md](docs/imu_sch16t_bringup.md) |
| [imu_sensor.h](imu_sensor.h) | IMU チップと推定の境界。**feed の口（`imu_feed_gyro` / `imu_feed_accel`）と `imu_drv_*`** |
| [link.cpp](link.cpp) / [e220.cpp](e220.cpp) | 無線。link=テレメトリの意味 / e220=UART の叩き方だけ |
| [link_assist.cpp](link_assist.cpp) | GNSS の時刻の助け合い（LoRa。2026-11-13〜、**実機未確認**）。GNSS が時刻を知らない機体・ボートへ、時刻を持つ仲間が**精度つきで**時刻を渡す（fix の前の RTC の時刻も。精度は NAV-PVT の tAcc で、テレメトリは status の `LINK_ST_TIME_COARSE` / `BAD`、ビーコンは `BC_TACC`。ビーコンは [lora_link/link_beacon.h](lora_link/link_beacon.h)、印 `0x58`）。機体は窓の間だけ送信の合間も mode 0 で聞く。受けた時刻は 2 発で確かめて `gnss_dbd_give_time(..., GNSS_TIME_SRC_LORA)` へ渡すだけ。定数は `settings.h` の `LINK_ASSIST_*`、`settings.txt` の `link_assist`。**v7 と v7.1 で同じファイル**。計画と決めごとは [docs/pons_ttff_assist_plan.md](docs/pons_ttff_assist_plan.md) §4 |
| [power.cpp](power.cpp) | USB 判定・電源経路の異常（充電 IC の故障）の検出と警報・高温での充電の警告・過放電（3.0V 未満）の本体フラッシュ記録と次回起動警告／USB 完全ロック・「SW OFF のまま充電」の検出・充電残量(%)と残り時間推定物理モデル（[docs/pons_charge_estimation.md](docs/pons_charge_estimation.md)） |
| [backlight.cpp](backlight.cpp) | バックライト PWM。輝度スイッチ High で最大、Low で照度センサー VEML7700（i2c0、ケースカバー基板）による自動調光。待機中自動減光（Auto-Dimming: 30 秒静止で 5% へ減光、手仰ぎや振動で即座に復帰）。電池低下のときの 2 段階点滅（判定は `batt_alarm_tick()`） |
| [bootwatch.cpp](bootwatch.cpp) | 起動の見張りと水晶の立ち上がり待ち（2026-11）。起動処理の最初〜 "BOOT PONS" の間だけ watchdog を掛け、固まったら自分で再起動して止まった段階を log.txt に `BOOT WATCH:` で出す。pico-sdk の `runtime_init_clocks()` を写しに置き換え、水晶が STABLE になってから `BOOTWATCH_XOSC_SETTLE_MS`（50ms）待って PLL を起こす（**0 にしないこと**。経緯は [docs/hardware.md](docs/hardware.md) §2「起動時に固まる件」）。**arduino-pico を上げたら元の `runtime_init_clocks()` と見比べること**。`isr_hardfault` も置き換えている |
| [readiness.cpp](readiness.cpp) | 本番前チェック。そろったら「準備完了」、欠けていれば設定画面の右上に項目名。**既存の判定（設定画面の丸の色・地上チェック・送信前チェック）を読むだけで、条件を書き写さない** |
| [rtc_backup.cpp](rtc_backup.cpp) | GNSS のバックアップ電池（BT1）が「あと何時間もつか」の推定（2026-11-08）。通電で足し、電源を切っていた時間で引き、起動時の時計の生死で引き戻す。記録は SD の `rtc_backup.txt`（読み書きは mysd.cpp・Core1）。**v7 と v7.1 で同じファイル** |
| [gnss_dbd.cpp](gnss_dbd.cpp) | GNSS の航法データベース（UBX-MGA-DBD）を SD の `gnss_dbd.bin` に写し、起動時に時計が死んでいた（BT1 が切れていた）ときだけ GNSS へ戻す（2026-11-09。**実機未確認**）。時刻を知らずに断られた分は、GNSS が衛星から時刻を読んだら入れ直す。**外から GNSS へ時刻を入れる口 `gnss_dbd_give_time()` もここ**（2026-11-10。PC の `PONS:TIME`。LoRa の時刻もここへ渡すだけにして、入れる条件を呼び出し側に書かない）。机の上の試験は USB シリアルの `PONS:DBD,SAVE / COLD / COLDONLY / HOT / STATUS / COLD,TIME / COLDONLY,TIME`。SD の読み書きは mysd.cpp・Core1。**v7 と v7.1 で同じファイル** |
| [lora_link/link_proto.h](lora_link/link_proto.h) | PONS Link の通信プロトコル本体（`LinkTelem` 構造体・CRC・無線方式に依存しない設計） |
| [lora_link/link_wire2.h](lora_link/link_wire2.h) | 電波に載せる 32 バイトのフレーム（`LinkTelem` を詰める・戻す）。PONSv7 と並びを同じに保つ |
| [tas.cpp](tas.cpp) | 推定 TAS の表示の判断（2026-11-07）。ヘッダー右の大きい枠に何を出すか（真方位 / 平均ピッチ / 推定 TAS / 地上の横線つき TAS）を、設定・地上と飛行中・**表示の供給元 4 つ**から 1 か所で決める。描画は `tas_header()` を読むだけ。仕様は [docs/pons_tas.md](docs/pons_tas.md) |
| [src/](src/) | 仕様が固まって普段いじらないもの（button / sound / vectormap / imulog / flashdata） |
| [wav/](wav/) | 音声の原本。**FLASH へ焼く元**で、SD には入れない |
| [src/flashdata/wav_data.cpp](src/flashdata/wav_data.cpp) | 焼き込んだ音声の実体。`tools/gen_wav_flash.py` の生成物。手で編集しない |

- **やること・後回しにしていること（v7・v7.1 両方）: [docs/pons_todo.md](docs/pons_todo.md)**。済んだら行を消す。
  1 項目 1〜2 行に保ち、経緯や数字は [docs/pons_todo_notes.md](docs/pons_todo_notes.md) へ
- ナビの考え方: [docs/pons_navigation.md](docs/pons_navigation.md) — 音の鳴り方: [docs/pons_sound.md](docs/pons_sound.md)
- 無線の設計・電波法: [docs/pons_link.md](docs/pons_link.md) — 実機立ち上げ: [docs/pons_link_bringup.md](docs/pons_link_bringup.md)
- GNSS の TTFF を SD の DBD と LoRa の助け合いで縮める計画: [docs/pons_ttff_assist_plan.md](docs/pons_ttff_assist_plan.md)
  （段階 1 の SD の DBD は 2026-11-09、PC から時刻を入れる口は 2026-11-10、無線の時刻の助け合いは 2026-11-13 に実装・どれも実機未確認）
- 充電 IC 焼損（2026-10-15）の対策と、全基板（v3〜v7.1）の電源の形・ファームの対応: [docs/pons_power.md](docs/pons_power.md)
- 充電中のバッテリー残量(%)・残り時間推定 仕様書: [docs/pons_charge_estimation.md](docs/pons_charge_estimation.md)
- SCH16T の実機立ち上げ: [docs/imu_sch16t_bringup.md](docs/imu_sch16t_bringup.md)
  （単独テストの実測値・Step 0〜5 の手順・踏みそうな罠。設計は imu_sch16t_plan.md）
- SD カードの雛形: [sd/](sd/) — レイアウトとログの列の意味は README の「SD カードの構成」
- PC 側ツールの一覧は README の「ツール」節
- git 管理外の生成物: `production/` と `src/flashdata/vectormap_data_hires.cpp`
- ライセンスは 2 本立て。コードは MIT、`vectormap_data.cpp` は ODbL 1.0（OpenStreetMap 由来）。
  地図データに手を入れるときは帰属表示を残す。
