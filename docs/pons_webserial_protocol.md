# PONS Web Serial 通信プロトコル仕様書

- **文書バージョン**: 1.0
- **対象基板**: PONS v7 / v7.1 (RP2350B)
- **更新日**: 2026-11-10

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
        │ "DISCONNECT" │ 抜去検知      │  / 長押し    │ (5分アイドル)
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
  PONS: PONS:READY,Version 7,0.996,20261103
  ```

* PC 接続の画面のまま `PONS:CONNECT` をもう一度受けても、戻り先（接続前の画面）は上書きしない（2026-11-10〜。ビューアのページを読み直したとき）。

### (2) ファイル一覧要求 (LIST)
SDカード内の飛行CSVファイル（`data/` フォルダ、`received/` フォルダ、およびルートディレクトリ）を走査して一覧を返す。
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
  PONS: PONS:LIST_END
  ```

### (3) ファイル取得 (GET)
指定したCSVファイル（または `imu_replaydata/YYYYMMDD.txt` 等の姿勢ログファイル）の中身をシリアルへストリーミング送信する。
* **PC ➔ PONS**: `PONS:GET,<filename>`
* **PONS ➔ PC**:
  - 開始通知: `PONS:DATA_START,<filename>,<size_bytes>`
  - 生データ: *(CSVデータがそのまま流れる)*
  - 終了通知: `PONS:DATA_END,<crc32>`
* **エラー応答**:
  - `PONS:ERROR,FILE_NOT_FOUND` （ファイルが存在しない、またはディレクトリ）
  - `PONS:ERROR,BUSY` （既に別のファイル転送タスクを実行中）
  - `PONS:ERROR,SD_NOT_READY` （SDカード未装着または初期化エラー）
* **例**:
  ```
  PC:   PONS:GET,data/2026-07-26_0804.csv
  PONS: PONS:DATA_START,data/2026-07-26_0804.csv,1245020
  PONS: latitude,longitude,gs,TrueTrack,GNSS_Altitude,...
  PONS: 35.295123,136.254120,6.54,342,86.2,...
  ...
  PONS: PONS:DATA_END,A3F5B12C
  ```

### (4) 転送キャンセル (CANCEL)
転送中のファイル読み出しを中断する。
* **PC ➔ PONS**: `PONS:CANCEL`
* **PONS ➔ PC**: `PONS:CANCEL_OK`

### (5) 切断要求 (DISCONNECT)
* **PC ➔ PONS**: `PONS:DISCONNECT`
* **PONS ➔ PC**: `PONS:BYE`
* PONSは安全に通常の航法画面（`MODE_MAP` 等）と各種ロギングへ復帰する。

### (6) 生存確認 (PING)
* **PC ➔ PONS**: `PONS:PING`
* **PONS ➔ PC**: `PONS:PONG`
* ファイル転送（GET）の待機中・実行中は応答しない（データの途中に `PONG` が混ざらないように）。
* 無通信が 60 秒続くと PONS は自動で通常の航法画面へ戻るので、PC は待っている間これを送る。
  USB を抜いたとき（給電が切れたとき）も自動で戻る。

### (7) GNSS の航法データベースの机の上の試験 (DBD)（2026-11-09〜）
* **PC ➔ PONS**: `PONS:DBD,SAVE` / `PONS:DBD,COLD` / `PONS:DBD,COLDONLY` / `PONS:DBD,HOT` / `PONS:DBD,STATUS` /
  `PONS:DBD,COLD,TIME` / `PONS:DBD,COLDONLY,TIME`（2026-11-10〜）
* **PONS ➔ PC**: `PONS:DBD,OK,<コマンド>` / `PONS:DBD,ERROR,<理由>`（`BUSY` / `NO_FIX` / `NO_GNSS_ID` / `NO_TIME` / `REPLAY` / `UNKNOWN` / `DISABLED`）/
  `PONS:DBD,STATUS,state=...,file=...,auto=on|off,time=none|<pc|lora|self>/<kept|wait|ok|refused|noack>(入れた回数)`
* `PONS:CONNECT` は要らない（Arduino IDE のシリアルモニタから打てる）。SD を読み出し専用にする PC 接続モードとは別物。
* `COLD` は GNSS をコールドスタートさせ（UBX-CFG-RST navBbrMask=0xFFFF、GNSS だけの再起動）、SD の `gnss_dbd.bin` を戻す。
  `COLDONLY` は戻さない比べ用、`HOT` はホットスタート、`SAVE` はいま吸い出して書く。結果（測位までの秒数など）は
  `log.txt` の `GNSS DBD TEST ...` に出る。仕組みは `gnss_dbd.cpp` の冒頭と docs/pons_ttff_assist_plan.md §3。
* `COLD` / `COLDONLY` / `HOT` を 1 回でも打つと、**再起動するまで自動の吸い出しはしない**（`auto=off`）。`COLDONLY` で消した
  GNSS の薄いデータベースで SD のファイルを上書きして、次の `COLD` の比べ物にならなくなるのを防ぐため。`SAVE` は効く。
* リプレイ中と、抜けた直後（本物の NAV-PVT が届くまで）は `STATUS` 以外を `ERROR,REPLAY` で断る（fix と日時が再生した CSV の値のため）。
* `COLD,TIME` / `COLDONLY,TIME` は `COLD` / `COLDONLY` と同じだが、リセットのあと（`COLD` は DBD より先に）、リセット前に本体が
  覚えた時刻を、PC や LoRa から来た時刻の代わりに入れる（下の (8) と同じ口・同じ精度）。GNSS が時刻を知っていないと `ERROR,NO_TIME`。
  時刻を入れたときに TTFF がどれだけ縮むかを、同じ場所で `COLD` と比べるためのもの。
  `,TIME` の無い試験の間は、PC から時刻が来ても入れない（`PONS:TIME` は `ERROR,TEST`。比べる素の値を汚さない）。

### (8) PC の時刻を GNSS へ入れる (TIME)（2026-11-10〜）
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

---

## 5. PONS本体側の実装構造（ファームウェア組み込み済み）

本プロトコルは PONS v7 / v7.1 ファームウェア（`mysd.cpp`, `GPS_TFT_map.ino`）に組み込み済みです。

1. **Core0 の役割**:
   - `serial_command_service()`（Core0のメインループ）で `Serial.available()` を常時監視。
   - `PONS:CONNECT` を受信すると、動作モードを `MODE_PCCONNECT` に切り替え、TFT液晶に「USB SERIAL LINK ACTIVE / SD Protected」の専用画面を表示。
   - コマンドを受信すると、Core1 へのタスク（`TASK_SERIAL_LIST`, `TASK_SERIAL_GET`）としてエンキュー。
2. **Core1 の役割**:
   - 開いているログ（飛行CSV、受信CSV、生IMUログ、log.txt）は、`LIST` / `GET` の処理の頭（`webserial_prepare_sd()`）で `sync()` & `close()` する。
     生IMUログは `PONS:CONNECT` を受けた次の周回から積むのを止め（`imulog_set_paused()`）、書き出し待ちのバッファも捨てる。
   - SDカードを「完全読み出し専用」に固定。
   - `serial_list_csv_files()`：`data/` ディレクトリ、`received/` ディレクトリ（PONS Link 受信ログ）、およびルートディレクトリを自動走査し、ファイル一覧（名前・バイト数・更新日時）を出力。
   - `serial_send_csv_file()`：指定されたファイルを 512B チャンクで安全に読み出し、CRC32 を算出しながら `Serial.write()` へストリーミング送出。
   - 切断（`PONS:DISCONNECT` またはタイムアウト・ケーブル抜去・ボタン操作）時に安全にログ記録を再開。

---

## 6. クライアント実装（PONS Flight Viewer）

本プロトコルに対応した公式 Web クライアントとして、PONS v7.1 側の `tools/flight_viewer/index.html` を共通利用できます。
Web Serial API 対応ブラウザ（Google Chrome / Microsoft Edge など）で開くだけで、インストール不要・完全ローカル動作で以下の機能を利用できます。

- **ワンクリック接続 & ログ吸い出し**: USB 接続した PONS から自機ログ（`data/`）および受信ログ（`received/`）を一覧取得・高速ダウンロード。旧形式の自機ログ（`data/*.csv` に姿勢列が含まれないもの）を開いた場合は、対応する `imu_replaydata/YYYYMMDD.txt` も自動的に取得・マージします。
- **PC の時刻を本体へ**（2026-11-10〜）: 接続のたびに `PONS:TIME` を送る。本体の GNSS が時刻を知らないとき（BT1 が切れていた）だけ使われる（上の (8)）。
- **自機ログ / 受信ログ 自動判別**: CSV ヘッダーから種別を自動判定し、機体・追尾ボート双方のアイコン・航跡・テレメトリを同時に再生。
- **航法・姿勢・横滑り計器**:
  - 風の三角形（対気速度・風向風速・対地速度）
  - 人工水平儀（ロール/ピッチ＋Turn & Slip 旋回針＆スリップボール）
  - 横滑り角推定エンジン（`Est.Beta`）

