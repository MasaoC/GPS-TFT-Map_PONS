# PONS Web Serial 通信プロトコル仕様書

- **文書バージョン**: 1.0
- **対象基板**: PONS v7 / v7.1 (RP2350B)
- **更新日**: 2026-10-30

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
指定したCSVファイルの中身をシリアルへストリーミング送信する。
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
* PONSは安全に通常の航法画面（`MODE_MAP`）と各種ロギングへ復帰する。

---

## 5. PONS本体側の実装構造（ファームウェア組み込み済み）

本プロトコルは PONS v7.1 ファームウェア（`mysd.cpp`, `GPS_TFT_map.ino`）に組み込み済みです。

1. **Core0 の役割**:
   - `check_serial_commands()`（Core0のメインループ）で `Serial.available()` を常時監視。
   - `PONS:CONNECT` を受信すると、動作モードを `MODE_PC_CONNECT` に切り替え、TFT液晶に「USB SERIAL LINK ACTIVE / SD Protected」の専用画面を表示。
   - コマンドを受信すると、Core1 へのタスク（`TASK_SERIAL_LIST`, `TASK_SERIAL_GET`）としてエンキュー。
   - 転送中は Core0 側で「TRANSFERRING... (ファイル名)」プログレスを表示。
2. **Core1 の役割**:
   - `MODE_PC_CONNECT` 移行時に、オープン中だったすべてのログ（飛行CSV、受信CSV、生IMUログ、log.txt）を直ちに `flush()` & `close()` して追記を一時停止。
   - SDカードを「完全読み出し専用」に固定。
   - `serial_list_csv_files()`：`data/` ディレクトリおよび `received/` ディレクトリ（PONS Link 受信ログ）を自動走査し、ファイル一覧（名前・バイト数・更新日時）を出力。
   - `serial_stream_file()`：指定された CSV を 512B チャンクで安全に読み出し、CRC32 を算出しながら `Serial.write()` へストリーミング送出。ウォッチドッグタイマーを更新しながら安全に実行。
   - 切断（`PONS:DISCONNECT` またはタイムアウト・ケーブル抜去）時に安全にログ記録を再開。

---

## 6. クライアント実装（PONS Flight Viewer）

本プロトコルに対応した公式 Web クライアントとして、`tools/flight_viewer/index.html` が同梱されています。
Web Serial API 対応ブラウザ（Google Chrome / Microsoft Edge など）で開くだけで、インストール不要・完全ローカル動作で以下の機能を利用できます。

- **ワンクリック接続 & ログ吸い出し**: USB 接続した PONS から自機ログ（`data/`）および受信ログ（`received/`）を一覧取得・高速ダウンロード。
- **自機ログ / 受信ログ 自動判別**: CSV ヘッダーから種別を自動判定し、機体・追尾ボート双方のアイコン・航跡・テレメトリを同時に再生。
- **航法・姿勢・横滑り計器**:
  - 風の三角形（対気速度・風向風速・対地速度）
  - 人工水平儀（ロール/ピッチ＋Turn & Slip 旋回針＆スリップボール）
  - 横滑り角推定エンジン（`Est.Beta`）
  - 照度センサー・画面輝度（`lux`, `backlight` 列）の HUD 表示
