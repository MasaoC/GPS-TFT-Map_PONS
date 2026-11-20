# PONS Web Serial 通信プロトコル仕様書

- **文書バージョン**: 1.0
- **対象基板**: PONS v7 / v7.1 (RP2350B)
- **更新日**: 2026-11-17

---

## 1. 目的と基本原則

本仕様は、PONS本体からSDカードを抜くことなく、PC/Mac/Android等のWebブラウザ（Web Serial API）からUSBケーブル1本で安全かつ手軽にフライトログ（CSV）を吸い上げるためのシリアル通信プロトコルを定義する。

### 最優先事項：SDカードの安全性
1. **ファイルシステムの破壊防止**:
   - PCが直接SDカードのFATテーブルを触る「USBマスストレージ（MSC）」は採用せず、PONSマイコンが介在する「USB CDC（シリアル）通信」とする。
   - 通信中にケーブルが突然抜かれても、SDカードのFAT32は1ミリも破損しない。
2. **Core0 / Core1 排他制御**:
   - PONSの基本ルール（Core0はSDを触らない、SD操作はCore1専用）を厳守する。
   - PC接続中は、Core1のフライトログ新規追記（飛行CSV・受信CSV・姿勢ログ・生IMUログ）を一時停止（flush & close）し、SDカードを「読み出し専用モード」に固定する。

---

## 2. 物理層・通信層仕様

- **インターフェース**: USB Type-C (USB 2.0 Full Speed CDC / 仮想シリアルポート)
- **通信速度**: USB CDCのネイティブ速度（ボーレート設定に依存せず、実効スループット約 800KB〜1MB/s）
- **文字コード**: UTF-8 / ASCII
- **改行コード**: `\r\n` (CRLF) または `\n` (LF)

---

## 3. 状態遷移 (State Machine)

```
       [ 通常動作モード (航法・ロギング) ]
                       │
                       │ 受信: "PONS:CONNECT"
                       ▼
         [ PC接続・データ転送モード (MODE_PCCONNECT) ]
           ・本体液晶に「USB SERIAL LINK ACTIVE / SD Protected」表示
           ・オープン中ログ（CSV, 生IMU, log.txt）を flush & close して一時停止
           ・SDカードを完全読み出し専用に固定
                       │
        ┌──────────────┼──────────────┬──────────────┐
        │ 受信:        │ USBケーブル   │ 本体SW短押し  │ 無通信タイムアウト
        │ "DISCONNECT" │ 抜去検知      │              │ (60秒アイドル)
        ▼              ▼              ▼              ▼
        [ 通常動作モード (MODE_MAP) へ安全復帰・ログ記録自動再開 ]
```

---

## 4. コマンド仕様

すべてのコマンドとレスポンスは `PONS:` プレフィックスで始まる行指向プロトコルとする。

### (1) 接続要求 (CONNECT)
* **PC ➔ PONS**: `PONS:CONNECT`
* **PONS ➔ PC**: `PONS:READY,<board>,<fw_version>,<build_date>`
* **例**:
  ```
  PC:   PONS:CONNECT
  PONS: PONS:READY,Version 7.1,0.999v71,20261029
  ```

* PC 接続の画面のまま `PONS:CONNECT` をもう一度受けても、戻り先（接続前の画面）は上書きしない（2026-11-10〜。ビューアのページを読み直したとき）。

### (2) ファイル一覧要求 (LIST)
SDカード内の飛行CSVファイル（`data/` フォルダ、`received/` フォルダ、およびルートディレクトリ）を走査して一覧を返す。
ルートの `log.txt`（本体の記録）も、あれば最後に 1 行出す（ビューアは種別「記録」で開く。消されずに伸び続ける）。
* **PC ➔ PONS**: `PONS:LIST`
* **PONS ➔ PC**:
  - 各ファイル: `PONS:FILE,<filename>,<size_bytes>,<datetime>`
  - 完了時: `PONS:LIST_END`
* **例**:
  ```
  PC:   PONS:LIST
  PONS: PONS:FILE,data/2026-07-26_0804.csv,1245020,2026-07-26 08:04:12
  PONS: PONS:FILE,data/2026-07-25_0615.csv,450120,2026-07-25 06:15:00
  PONS: PONS:FILE,received/2026-07-26_0804.csv,1180400,2026-07-26 08:04:15
  PONS: PONS:FILE,log.txt,3008531,1980-01-01 00:00:00
  PONS: PONS:LIST_END
  ```

* **全部のファイルの一覧 (LIST,ALL)**（開発用）: `PONS:LIST,ALL`
  - ルートと、その直下のフォルダ（1 段だけ）の**全部のファイル**を同じ `PONS:FILE,...` の行で返す（`imuraw/*.bin` の生 IMU ログ・
    `settings.txt`・`gnss_dbd.bin`・`rtc_backup.txt` など。`data/`・`received/`・`log.txt` もこの中に出る）。隠しファイルと `.` で始まる名前は出さない。
  - **これを知らない古いファーム（1.07 まで）は何も返さない**（エラーも出ない）。Flight Viewer は 4 秒待って 1 行も来なければ
    古いファームとして `PONS:LIST` を頼み直す。
  - 取るのは下の `GET` のまま（`GET` はもとからどのパスでも中身をそのまま送る。バイナリも同じ）。パスは 63 バイトまで。
  ```
  PC:   PONS:LIST,ALL
  PONS: PONS:FILE,settings.txt,912,2026-11-13 08:00:02
  PONS: PONS:FILE,data/2026-11-13_0804.csv,1245020,2026-11-13 08:04:12
  PONS: PONS:FILE,imuraw/20261113.bin,52428800,2026-11-13 09:10:00
  PONS: PONS:FILE,log.txt,3008531,1980-01-01 00:00:00
  PONS: PONS:LIST_END
  ```

### (3) ファイル取得 (GET)
指定したファイルの中身をシリアルへストリーミング送信する（CSV でもバイナリでも、中身はそのまま）。
**中身は `DATA_START` の `<size_bytes>` バイトちょうど**で、そのあとに `\r\n` と `PONS:DATA_END,<crc32>` の行が来る
（Flight Viewer はこの大きさで中身を切り出す。バイナリの中に `PONS:DATA_END,` と同じ並びがあっても終わりにしない）。
* **PC ➔ PONS**: `PONS:GET,<filename>`
* **PONS ➔ PC**:
  - 開始通知: `PONS:DATA_START,<filename>,<size_bytes>`
  - 生データ: *(CSVデータがそのまま流れる)*
  - 終了通知: `PONS:DATA_END,<crc32>`
* **エラー応答**:
  - `PONS:ERROR,FILE_NOT_FOUND` （ファイルが存在しない、またはディレクトリ）
  - `PONS:ERROR,BUSY` （既に別のファイル転送タスクを実行中）。**前の転送が終わってから**返す（すぐ出すと前の転送の中身に混ざる）
  - `PONS:ERROR,SD_NOT_READY` （SDカード未装着または初期化エラー）
  - `PONS:ERROR,TX_TIMEOUT` （送信中に PC 側の受信停止等で 10 秒連続して 1 バイトも送れなかった場合の中断）
* **例**:
  ```
  PC:   PONS:GET,data/2026-07-26_0804.csv
  PONS: PONS:DATA_START,data/2026-07-26_0804.csv,1245020
  PONS: latitude,longitude,gs,TrueTrack,GNSS_Altitude,...
  PONS: 35.295123,136.254120,6.54,342,86.2,...
  ...
  PONS: PONS:DATA_END,A3F5B12C
  ```
* **末尾だけ取る (GETTAIL)**: `PONS:GETTAIL,<バイト数>,<filename>`
  - ファイルがそれより大きければ、末尾のそのバイト数だけを、**最初の改行の次から**（行の途中から始めないように）送る。
    そのときの開始通知は `PONS:DATA_START,<filename>,<送るバイト数>,<送り始めた位置>`（3 つめが付く）。CRC は送った部分だけ。
  - ファイルのほうが小さければ `GET` と同じ（3 つめは付かない）。
  - バイト数を先に置くのは、FAT のファイル名に `,` が使えるため（パスは行の最後まで）。
  - 伸び続ける `log.txt` を、最新の分だけ速く取るためのもの。Flight Viewer は 1MB を超える `log.txt` を、
    ふだん「最新 1MB」で取る（「全部」も選べる）。
  ```
  PC:   PONS:GETTAIL,1048576,log.txt
  PONS: PONS:DATA_START,log.txt,1048530,1960001
  PONS: 2952:GROUND PITCH -79.4 deg ...
  ...
  PONS: PONS:DATA_END,5C01A2E7
  ```
* **生 IMU ログの時刻の範囲だけ取る (GETIMU)**: `PONS:GETIMU,<開始 JST 日内秒>,<終了 JST 日内秒>,<filename>`（2026-11-20〜）
  - 生 IMU ログ（28 バイトのレコード）を 224KB（約 16 秒）おきに覗き、GNSSVEL（id 0x10）の iTOW から JST の日内秒を出して、
    範囲に入る所の前後 1〜2 区切りぶんだけを送る。開始通知は `GETTAIL` と同じく 3 つめ（送り始めた位置）が付く。CRC は送った部分だけ。
  - 受け付けたらすぐ `PONS:GETIMU_ACCEPTED` を返す（時刻探しの間は何も出ないので、古いファームと見分けるため）。
    この印が 12 秒来なければ、Flight Viewer は `GETIMU` を知らないファームとみて `PONS:GET` で全部を取り直す。
  - 範囲に入る記録が無ければ `PONS:ERROR,IMU_RANGE_NOT_FOUND`。秒は 0〜86400、6 桁まで。開始 ≥ 終了は `INVALID_FILENAME`。
  ```
  PC:   PONS:GETIMU,46200,49200,imuraw/20261119.bin
  PONS: PONS:GETIMU_ACCEPTED
  PONS: PONS:DATA_START,imuraw/20261119.bin,41440000,11424000
  ...
  PONS: PONS:DATA_END,1B2C3D4E
  ```
* **ビューアの受信の終わりの見分け**: 中断の印（`PONS:CANCEL_OK` / `PONS:ERROR,`）は**行の頭にあるときだけ**見る
  （`log.txt` の中身に同じ文字列があっても、転送を止めたことにしない）。
* **転送の間（GET を受けてから `DATA_END` / `CANCEL_OK` まで）は、本体は転送以外を 1 行も書かない**。
  PC はその間のバイトを全部ファイルの中身として受けて CRC を取るため。
  - `PONS:READY`（転送中に `CONNECT` し直したとき。前の転送をやめさせてから）・`PONS:BYE`・`PONS:ERROR,BUSY` は、
    転送が終わってから出す。
  - `PONS:PING` の返事は出さない（以前から）。`PONS:DBD` / `PONS:TIME` は転送中は受けない（返事も無い）。
  - 例外: `RELEASE` でないビルドのデバッグ出力。
  - SD のタスクのキューが溢れたときの `ERR: task queue full` も、転送と一覧の間は出さない。数百 MB の生 IMU ログは送るのに何分もかかり、
    その間 Core1 はキューを消費しないので満杯になる。捨てた数は送り終えたあと `log.txt` の `PC XFER: n tasks dropped` に残す
    （その間の `log.txt` の行は失われる）。
* **一覧（`LIST` を受けてから `LIST_END` まで）の間も同じ**。`READY` / `BYE` / `BUSY` / `PONG` は
  `LIST_END` のあとに出す。一覧の最中に来た `CANCEL` の `CANCEL_OK` も `LIST_END` のあと。
* 本体の SD のタスクのキューが満杯で `LIST` / `GET` / `GETTAIL` / `GETIMU` を受けられなかったときは `PONS:ERROR,BUSY` を返す。
  パスが 63 バイトより長いときは `PONS:ERROR,INVALID_FILENAME`。`GETTAIL` のバイト数は 9 桁まで。127 バイトより長い行は改行まで読み捨てる。

### (4) 転送キャンセル (CANCEL)
転送中のファイル読み出しを中断する。
* **PC ➔ PONS**: `PONS:CANCEL`
* **PONS ➔ PC**: `PONS:CANCEL_OK`
* `GET` がまだ始まる前（本体が別の SD の仕事をしていて待っている間）に来た `CANCEL` も効く。そのときは `DATA_START` を出さずに `CANCEL_OK` だけ返す。

### (5) 切断要求 (DISCONNECT)
* **PC ➔ PONS**: `PONS:DISCONNECT`
* **PONS ➔ PC**: `PONS:BYE`
* 転送中に切断した（`DISCONNECT`・本体のボタン）ときは、転送をやめて行の頭に `PONS:CANCEL_OK` を出してから `PONS:BYE`。
* PONSは安全に通常の航法画面（`MODE_MAP`）と各種ロギングへ復帰する。

### (6) GNSS の航法データベースの机の上の試験 (DBD)
* **PC ➔ PONS**: `PONS:DBD,SAVE` / `PONS:DBD,COLD` / `PONS:DBD,COLDONLY` / `PONS:DBD,HOT` / `PONS:DBD,STATUS` /
  `PONS:DBD,COLD,TIME` / `PONS:DBD,COLDONLY,TIME`（2026-11-10〜）
* **PONS ➔ PC**: `PONS:DBD,OK,<コマンド>` / `PONS:DBD,ERROR,<理由>`（`BUSY` / `NO_FIX` / `NO_GNSS_ID` / `NO_TIME` / `REPLAY` / `UNKNOWN` / `DISABLED`）/
  `PONS:DBD,STATUS,state=...,file=...,auto=on|off,time=none|<pc|lora|self>/<kept|wait|ok|refused|noack>(入れた回数)`
* `PONS:CONNECT` は要らない（Arduino IDE のシリアルモニタから打てる）。SD を読み出し専用にする PC 接続モードとは別物。
* `COLD` は GNSS をコールドスタートさせ（UBX-CFG-RST navBbrMask=0xFFFF、GNSS だけの再起動）、SD の `gnss_dbd.bin` を戻す。
  `COLDONLY` は戻さない比べ用、`HOT` はホットスタート、`SAVE` はいま吸い出して書く。結果（測位までの秒数など）は
  `log.txt` の `GNSS DBD TEST ...` に出る。仕組みは `gnss_dbd.cpp` の冒頭と docs/pons_ttff_assist_plan.md §3。
* `COLD` / `COLDONLY` は、リセットの前に**その起動の間 ANA を切る**（2026-11-14〜。本物のコールドの起動と同じ条件。SPG 5.10 は ANA が有効だと
  コールドスタートのあと QZSS を使わないため）。次の普通の起動で ANA は有効に戻る。
* `COLD` / `COLDONLY` / `HOT` を 1 回でも打つと、**再起動するまで自動の吸い出しはしない**（`auto=off`）。`COLDONLY` で消した
  GNSS の薄いデータベースで SD のファイルを上書きして、次の `COLD` の比べ物にならなくなるのを防ぐため。`SAVE` は効く。
* リプレイ中と、抜けた直後（本物の NAV-PVT が届くまで）は `STATUS` 以外を `ERROR,REPLAY` で断る（fix と日時が再生した CSV の値のため）。
* `COLD,TIME` / `COLDONLY,TIME` は `COLD` / `COLDONLY` と同じだが、リセットのあと（`COLD` は DBD より先に）、リセット前に本体が
  覚えた時刻を、PC や LoRa から来た時刻の代わりに入れる（下の (7) と同じ口・同じ精度）。GNSS が時刻を知っていないと `ERROR,NO_TIME`。
  時刻を入れたときに TTFF がどれだけ縮むかを、同じ場所で `COLD` と比べるためのもの。
  `,TIME` の無い試験の間は、PC から時刻が来ても入れない（`PONS:TIME` は `ERROR,TEST`。比べる素の値を汚さない）。

### (7) PC の時刻を GNSS へ入れる (TIME)
* **PC ➔ PONS**: `PONS:TIME,<UNIX 時刻 ms>`（UTC。例 `PONS:TIME,1794193200500`。JavaScript の `Date.now()`）
* **PONS ➔ PC**: `PONS:TIME,OK,<SENT|LATER|NOT_NEEDED|ALREADY>` / `PONS:TIME,ERROR,<理由>`
  （`BAD_FORMAT` / `BAD_TIME` / `GAVE_UP` / `TEST` / `REPLAY` / `DISABLED`）
  - `SENT` GNSS へ入れた / `LATER` まだ入れられない（起動直後で時計の生死が分からない・GNSS がつながっていない・入れ直しの間隔待ち）
    ので覚えておき、入れられるようになったら入れる / `NOT_NEEDED` GNSS は時刻を知っている（ふだんはこれ）/ `ALREADY` もう入れて受け取られた
  - `BAD_TIME` ビルドした日より前か 2100 年以降（PC の時計が戻っている）/ `GAVE_UP` 3 回入れて受け取られなかった（GNSS がリセットするまで入れない）
* **GNSS が時刻を知らない（時計の電池 BT1 が切れていた）ときだけ**、本体が UBX-MGA-INI-TIME_UTC で GNSS へ入れる
  （精度は 2 秒と伝える。`settings.h` の `GNSS_TIME_ACC_PC_MS`）。SD から戻した航法データベースが「時刻が無いので使えない」と
  断られずに済み、断られていた分はすぐ入れ直す。仕組みは `gnss_dbd.cpp` の冒頭と docs/pons_ttff_assist_plan.md §3.6。
* `PONS:CONNECT` は要らない（シリアルモニタからも打てる）。PC 接続の画面のあいだも GNSS は動いている。屋内では fix しないが、
  入れた時刻と受け取られたデータベースは GNSS に残り、つないでいる間は BT1 も充電されるので、電源を切ってもしばらく持つ。
* Flight Viewer は `PONS:READY` を受けるたびに送る（`PONS:LIST` より前。返事は見ない）。
* **PC の時計がずれていると、GNSS の性能はかえって落ちる**（伝えた精度より外れたとき）。最初の fix で、入れた時刻が本当は
  どれだけずれていたかを `log.txt` の `GNSS TIME (pc): given time was +0.08s off at 1st fix (told acc 2.0s)` に残す
  （NAV-PVT の届く遅れのぶん少し + に出る。精度より大きいと ` ** more than acc **`）。ほかの結果も `GNSS TIME (pc): ...` の行。
* LoRa で仲間の機体・ボートからもらう時刻（計画のみ）も、本体の中では同じ口（`gnss_dbd_give_time()`）を通る。

### (8) 本体設定の確認・変更・保存 (SETTINGS_GET / SETTINGS_SET / SETTINGS_SAVE)
PONS Flight Viewer から `settings.txt` の全設定（32 項目）を確認・変更し、SD へ安全に保存するためのコマンド（2026-11-20〜）。`PONS:CONNECT` 接続中のみ受け付ける。

* **取得 (`SETTINGS_GET`)**:
  - **PC ➔ PONS**: `PONS:SETTINGS_GET`
  - **PONS ➔ PC**:
    - 各設定: `PONS:SETTING,<key>,<value>,<in_file>`（`in_file`: `1` = `settings.txt` に記載あり、`0` = 未記載で本体既定値）
    - 目的地候補: `PONS:DEST,<name>`（`extradestinations` の全件）
    - 終了: `PONS:SETTINGS_END`
* **変更適用 (`SETTINGS_SET`)**:
  - **PC ➔ PONS**: `PONS:SETTINGS_SET,<key>:<value>`
  - **PONS ➔ PC**: `PONS:SETTINGS_OK,<key>,<applied_value>` または `PONS:SETTINGS_ERR,<UNKNOWN_KEY|BAD_FORMAT>,<key>`
  - 本体側の既存 `setter` を通してバリデーション・反映し、変化があれば `log.txt` に `PC SETTING <key>: <old> -> <new>` を記録する。
* **SD へ保存 (`SETTINGS_SAVE`)**:
  - **PC ➔ PONS**: `PONS:SETTINGS_SAVE`
  - **PONS ➔ PC**: `PONS:SETTINGS_SAVED,OK` または `PONS:SETTINGS_ERR,<SD_NOT_READY|SAVE_FAILED>`
  - 項目間の整合チェック（`attitude_finish_settings_load()`・`airspeed_min < airspeed_max`）ののち、既存の `saveSettings()`（`settings.tmp` に書き切ってからリネームする原子的保存）で `settings.txt` を更新し、`log.txt` に `PC SETTINGS SAVED ...` を残す。
  - 無線関連（`link_mode` / `link_radio_ch` / `link_radio_prof` / `link_group` / `link_assist`）のハードウェア設定は、本体の再起動後に反映される。

---

## 5. PONS本体側の実装構造（ファームウェア組み込み済み）

本プロトコルは PONS v7.1 ファームウェア（`mysd.cpp`, `GPS_TFT_map.ino`）に組み込み済みです。

1. **Core0 の役割**:
   - `check_serial_commands()`（Core0のメインループ）で `Serial.available()` を常時監視。
   - `PONS:CONNECT` を受信すると、動作モードを `MODE_PC_CONNECT` に切り替え、TFT液晶に「USB SERIAL LINK ACTIVE / SD Protected」の専用画面を表示。
   - コマンドを受信すると、Core1 へのタスク（`TASK_SERIAL_LIST`, `TASK_SERIAL_GET`）としてエンキュー。
   - 転送中は Core0 側で「TRANSFERRING... (ファイル名)」プログレスを表示。
2. **Core1 の役割**:
   - 開いているログ（飛行CSV、受信CSV、生IMUログ、log.txt）は、`LIST` / `GET` の処理の頭（`webserial_prepare_sd()`）で `sync()` & `close()` する。
     生IMUログは `PONS:CONNECT` を受けた次の周回から積むのを止め（`imulog_set_paused()`）、書き出し待ちのバッファも捨てる。
   - SDカードを「完全読み出し専用」に固定。
   - `serial_list_csv_files()`：`data/` ディレクトリおよび `received/` ディレクトリ（PONS Link 受信ログ）を自動走査し、ファイル一覧（名前・バイト数・更新日時）を出力。
   - `serial_stream_file()`：指定された CSV を 512B チャンクで安全に読み出し、CRC32 を算出しながら `Serial.write()` へストリーミング送出。ウォッチドッグタイマーを更新しながら安全に実行。
   - 切断（`PONS:DISCONNECT` またはタイムアウト・ケーブル抜去）時に安全にログ記録を再開。

---

## 6. クライアント実装（PONS Flight Viewer）

本プロトコルに対応した公式 Web クライアントとして、`tools/flight_viewer/index.html` が同梱されています。
Web Serial API 対応ブラウザ（Google Chrome / Microsoft Edge など）で開くだけで、インストール不要・完全ローカル動作で以下の機能を利用できます。

- **ワンクリック接続 & ログ吸い出し**: USB 接続した PONS から自機ログ（`data/`）および受信ログ（`received/`）を一覧取得・高速ダウンロード。
- **PC の時刻を本体へ**: 接続のたびに `PONS:TIME` を送る。本体の GNSS が時刻を知らないとき（BT1 が切れていた）だけ使われる（上の (7)）。
- **つながらないとき**: ポートを開けないとき（Arduino IDE のシリアルモニタ・シリアルプロッタ、別のタブの
  ビューアなどがポートを掴んでいる）と、開けたのに `PONS:READY` が来ないときは、ダイアログに理由と直し方と「🔁 もう一度つなぐ」を出す。
  `PONS:CONNECT` は 2 秒ごとに 6 回まで送り直す
  （USB を挿した直後で本体が起動中でも、起動し終わればつながる）。逆に、ビューアがポートを掴んでいる間は Arduino IDE から
  書き込めないことがあるので、書き込む前にビューアの「🔌 切断」を押す。
- **生 IMU ログなどの取り出し（開発用のタブ）**: 一覧の上のタブ「IMU 生ログ」（`imuraw/`）・「その他」（ほかの全部）を
  開くと `PONS:LIST,ALL` で一覧を取り、「💾 保存」で中身をそのまま PC に保存する（ブラウザのダウンロード。開かない）。
  最初のタブ「フライト・記録」は今までどおりで、各行にも「💾 保存」（開かずに保存）がある。
  受信は**バイトのまま**扱う。CRC も受けたバイトで計算する。
  保存した `imuraw/*.bin` は `tools/imulog/decode_imulog.py` で読む。
- **log.txt の表示**: 一覧の `log.txt` を開くと（手元の log.txt を「CSV / log.txt を開く」やドロップで開いても）、
  起動ごとに分けて、日時・版・最初の測位・エラーと警告の数を並べ、毎分の行（`volt=` / `grf` / `imu gapmax=` / `CHG`）を
  グラフにする（電池・GNSS の C/N0 と衛星数・Fix の割合・CPU 温度・IMU の空き・BT1 の残り・充電の残量・明るさ・jamInd）。
  行は種類（起動・DBD と時刻・GNSS・PC と SD・電源・無線・IMU）と重さ（エラー・警告）で色分けし、検索と絞り込みができる。
  グラフをクリックするとその時刻の行へ飛ぶ。「💾 保存」で受け取った分をファイルに保存する。
  ★ 毎分の行の形を変えたら、ビューアの読み取り（`index.html` の `logScanLine`）も合わせること（読めない値はグラフから黙って消える）。
- **自機ログ / 受信ログ 自動判別**: CSV ヘッダーから種別を自動判定し、機体・追尾ボート双方のアイコン・航跡・テレメトリを同時に再生。
- **航法・姿勢・横滑り計器**:
  - 風の三角形（対気速度・風向風速・対地速度）
  - 人工水平儀（ロール/ピッチ＋Turn & Slip 旋回針＆スリップボール）
  - 横滑り角推定エンジン（`Est.Beta`）
  - 照度センサー・画面輝度（`lux`, `backlight` 列）の HUD 表示

### 接続の保ち方（PING）

- **PING（15 秒ごと）は転送のダイアログが開いている間だけ送る。** ファイルを開くとダイアログが閉じ、本体は 60 秒の
  無通信で PC 接続の画面を抜けて通常動作へ戻る（リプレイを見ている間、本体を PC 接続のまま止めておかないための意図した作り）。
- ダイアログを開き直すと一覧を取り直す。本体が既に戻っていれば `PONS:ERROR,NOT_CONNECTED` が返り、ビューアが自動で
  `PONS:CONNECT` し直して一覧を出す。
- ヘッダーのボタンは接続中「🔌 PONS接続中 (ファイル一覧)」で、押すとダイアログを開く。**切断はダイアログの中の「🔌 切断」**
- 転送の途中で本体が `PONS:CANCEL_OK` や `PONS:ERROR,...` を返したときも転送を終わったことにする
  （受信中は全部ファイルのデータとして溜めるので、拾わないと次のダウンロードと PING が止まったままになる）。
