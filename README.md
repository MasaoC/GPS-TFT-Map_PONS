# GPS(GNSS)-TFT-map_PONS=Pilot Oriented Navigation System v6
<img width="1312" height="368" alt="ponsv6image" src="https://github.com/user-attachments/assets/1e0ee79d-7a78-437a-a577-ab036f17d5f1" />
<img width="240" height="52" alt="logo" src="https://github.com/user-attachments/assets/ffad4b6f-56ef-4505-8b95-28498c59182f" />


# PONS 概要/Abstract
 * 人力飛行機のパイロットが使いやすいことを目標とした航法装置（ナビゲーションシステム）です。琵琶湖上空を飛行するパイロットの支援をしています。Arduinoを使用し、TFTにGNSSマップや位置・速度・方位の情報を表示します。琵琶湖での鳥人間コンテスト用に特化しています。GPS/GNSS navigation for human-powered aircraft specialized for the Japan International Birdman at Biwako, Japan.
 * 大阪大学albatrossにて使用実績あり（2024追走ボート: v3, 2025追走ボート: v4, 2025機体搭載: v5）。2025年大会優勝。
 * 3Dプリントケース及び基板データ（KiCad)あり。ケースは、PLA_LWが軽量化でオススメです。
 * PONSデバイスの使い方や画面の見方などの詳細は、docsフォルダに説明書があります。このREADMEファイルより説明書の方が情報が新しい場合があります。
 * PONS for HPA = Pilot Oriented Navigation System for Human-powered aircraft = 人力飛行機用パイロット向けナビゲーションシステム
 * 地図データは自作したGoogle earthから出力された緯度経度列の他、Google map APIにて事前ダウンロードしたbmp画像をSDカードから読み込んで表示も可能です。
   * Google map APIを利用した画像データは、各自がAPIを利用登録する必要があります。APIへのアクセスとbmpへの変換は、支援ツール（Python）がtoolsフォルダにあります。
 * 日本地図・琵琶湖（沖島・竹生島・多景島）などは、プログラムに事前登録されています。パイロン座標は2025ルールに合わせてありますが、現実のパイロン位置はズレることがあるようです。

# スペック (v6)
 * 重量: 59.6g (W59 H81 D20 mm)
 * バッテリー: 約4時間（高輝度モード） / 約12時間（低輝度モード）。USB Micro-Bで充電。
 * TFT: 2.8インチ 1000ニト。スイッチによる輝度調整可能。
 * GNSS位置精度: 1.5m、速度精度: 0.05m/s、更新レート: 2Hz

# 推奨機器
## マイコン  Recommended Microcontroller
 * Raspberry Pi Pico 2 (RP2350)

 TFTとの通信では、PicoのPIOを使用し 16 bit Paralel 接続を使う事で高速描画可能になっています。これは、TFT_eSPIライブラリによるものです。

## TFT  Recommended TFT panel. (推奨の動作確認済みTFT)
 * Digikey
   * NHD-2.8-240320AF-CSXP-F-ND (v4,v5,v6:パイロット用で使用)
   * NHD-2.4-240320CF-CSXN#-F-ND (v3:追走ボート用で使用。1000ニト)

## GNSS Module モジュール
 UART NMEA0183 で通信するため、任意のモジュール使用可能。下記のGNSSモジュールで実験済み。ただし、Mediatek or ubloxで、初期化処理が異なるため、settings.hを変更すること。
 * ublox SAM-M10Q (v6:パイロット用。基板表面実装。水平位置精度1.5m、速度精度0.05m/s)
 * Quectel LC86G (v4,v5:パイロット用で使用。LC86GPAMD)
 * ublox M-10Q (v3:追走ボート用で使用)

# 機能 Detail.
 * 基本機能（画面の表示）
    * 画面上部：対地速度(m/s)、航路（MT=Magnetic Track）の表示。2.0m/s未満はグレー表示。
    * 画面中央：地図（SDカード上に登録された緯度経度列およびbmp画像）の表示、MTの針の表示。
      * 旋回角速度表示（3秒平均 deg/sec）。2.0deg/s超えで着色、左右端に表示。
      * 旋回すべき修正方向の表示（赤矢印）。コースズレ15°以上で表示・点滅。
      * コース警報Arc：向かうべき方位(MC)への赤線円弧。10〜60秒放置で音声警報。
      * 過去の航跡は緑色の線で引かれる。最大500地点（約25km分）を表示。
      * 琵琶湖付近にいる場合、タケシマ・北パイロン・西パイロンに向けて自動で線が引かれる。パイロンが表示される。折り返し目安の10.975km円が描かれる。
      * NORTH UP / TRACK UP 切替対応。
    * 画面下部：時刻（JST）/緯度/経度/目的地コース(Magnetic Course)/目的地までの距離（km）/ 補足衛星数 / 電池残量(%)
    * 画面最下部：目的地モード / 目的地名 / SD認識表示
    * 最大GS表示：過去5分および累計の最大対地速度を画面下部に表示。
 * ロガー：緯度/経度/対地速度/時刻をSDカードへ保存（GPS位置捕捉時、1秒に2回）。気圧データも記録。
 * バリオメーター(MS5611)：昇降計。設定画面でボリューム0にすると無音・非表示。
 * 設定画面
    * 目的地の設定（PLATHOME, N_PILON, W_PILON, TAKESHIMA が本体登録済み。SDカードで追加可能。）
    * 向かうIntoモードか、離れるAwayモードか、目的地から10kmをひたすら往復する Auto10km モード(大会用）の選択
    * ボリューム / バリオメーターボリュームの調整
    * NORTH UP（北が上） / TRACK UP（進行方向が上） の設定
    * デモモード（琵琶湖でそれっぽいデータを表示）
    * リプレイモード（SDカードのNMEAデータを再生）
    * GPSのコンステレーション表示（受信中衛星・NMEA原文）
    * 地図データ情報一覧表示 (FLASH + SD mapdata.csv)
    * SDカード内のファイル表示
    * 設定はSDカードに保存・次回起動時に自動読込。NAV画面のscale（拡大率）も保存される。

# 警告音・警報
 * 旋回率 警告音：旋回率が2deg/sを超えるとピピと高音。2.0m/s以下では鳴らない。
 * 航路(TRACK)警報：左右15°範囲を超えて航路が変わるとChime音+「Track！」音声。
 * 目的地自動更新音：Auto 10kmモードで10K AWAY↔10K INTO切替時に4Chime+音声。
 * コース警報：針路のズレが10〜60秒放置されると4Chime+「針路、右(左)に旋回」音声。
 * 電圧低下警報：3.4V未満で音声案内（60秒に1回）。

# ツール (tools/)
 * `tools/csv2kml4earth.py`：複数のフライトログCSVをまとめてKMLに変換（Google Earth用）
 * `tools/mapimage/create_mapimages.py`：Google map API を使ってbmp地図画像を生成

# 目的地の追加方法
 SDカードに `destinations.csv` ファイルを作成し、コンマ区切りで名前（英語）、緯度、経度の順で記述。
 例：`testdata,35.4123,136.1234132`

# 追加地図（独自地図）の作り方
 1. Google EarthでPATHを作成しKMLファイルをダウンロード。PATHの名前は英語で登録。
 2. `kml_to_mapcsv.py` でKMLから `mapdata.csv` を生成。
 3. `mapdata.csv` をSDカードのトップディレクトリに配置。
 * PATH名の1文字目で線の色が決まる: `r`=RED, `o`=ORANGE, `g`=GRAY, `m`=MAGENTA, `c`=CYAN, `b`=BLUE

# WAV音声ファイルの変更
 16kHz、Unsigned 8bit PCM、メタ情報なしのWAVファイルに対応。Audacity推奨。
 使用ファイル名: `track.wav`, `course_left.wav`, `course_right.wav`, `destination_change.wav`, `battery_low.wav`, `destination_toofar.wav`, `opening.wav`, `matane.wav`, `arigato.wav`, `baibai.wav`

# 使用部品 (v6)
| 部品 | 型番 | 備考 |
|------|------|------|
| GNSS | ublox SAM-M10Q | 基板表面実装 |
| TFT | Newhaven NHD-2.8-240320AF-CSXP-F-ND | 2.8インチ |
| バッテリー | マルツ DTP603048 860mAh | 保護回路付き |
| 充電コントローラー | MCP73831 | - |
