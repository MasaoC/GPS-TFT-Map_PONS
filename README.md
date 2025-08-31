# PONS=Pilot Oriented Navigation System v5
<img width="220" alt="ponsv5" src="https://github.com/user-attachments/assets/42279b57-4c61-43eb-a7d4-d60ea7dddddf" />

# GPS_TFT_map 概要/Abstract
 * 人力飛行機のパイロットが使いやすいことを目標とした航法装置（ナビゲーションシステム）です。琵琶湖上空を飛行するパイロットの支援をしています。Arduinoを使用し、TFTにGNSSマップや位置・速度・方位の情報を表示します。琵琶湖での鳥人間コンテスト用に特化しています。GPS/GNSS navigation for human-powered aircraft specialized for the Japan International Birdman at Biwako, Japan.
 * 3Dプリントケース及び基板データ（KiCad)あり。ケースは、PLA_LWが軽量化でオススメです。
 * PONSデバイスの使い方や画面の見方などの詳細は、docsフォルダに説明書があります。このREADMEファイルの方が情報が古い場合があります。
 * PONS for HPA = Pilot Oriented Navigation System for Human-powered aircraft = 人力飛行機用パイロット向けナビゲーションシステム
 * 地図データは自作したGoogle earthから出力された緯度経度列の他、Google map APIにて事前ダウンロードしたbmp画像をSDカードから読み込んで表示も可能です。
   * Google map APIを利用した画像データは、各自がAPIを利用登録する必要があります。APIへのアクセスとbmpへの変換は、支援ツール（Python）がtoolsフォルダにあります。
 * 日本地図・琵琶湖（沖島・竹生島・多景島）などは、プログラムに事前登録されています。パイロン座標は2025ルールに合わせてありますが、現実のパイロン位置はズレることがあるようです。（2025：600m程度）
 * 大阪大学albatross (2024-2025) にて使用実績があります。

# 推奨機器
## マイコン  Recommended Microcontroller
 * Raspberry Pi Pico 2
   
 TFTとの通信では、PicoのPIOを使用し 8 bit Paralel 接続を使う事で高速描画可能になっています。これは、TFT_eSPIライブラリによるものです。

## TFT  Recommended TFT panel. (推奨の動作確認済みTFT)
 * Digikey
   * NHD-2.8-240320AF-CSXP-F-ND (v4,v5:パイロット用で使用)
   * NHD-2.4-240320CF-CSXN#-F-ND (v3:追走ボート用で使用。1000ニト)

## GNSS Module モジュール
 UART NMEA0183 で通信するため、任意のモジュール使用可能。下記のGNSSモジュールで実験済み。ただし、Mediatek or ubloxで、初期化処理が異なるため、settings.hを変更すること。
 * ublox M-10Q (v3:追走ボート用で使用)
 * Quectel LC86G (v4,v5:パイロット用で使用。LC86GPAMD)

# 機能 Detail.
 * 基本機能（画面の表示）
    * 画面上部：対地速度(m/s)、航路（MT=Magnetic Track）の表示。
    * 画面中央：地図（SDカード上に登録された緯度経度列およびbmp画像）の表示、MTの針の表示。
      * 旋回すべき修正方向の表示。
      * 過去の航跡は緑色の線で引かれる。およそ過去30分程度の履歴は表示される。
      * 琵琶湖付近にいる場合、タケシマ・北パイロン・西パイロンに向けて自動で線が引かれる。パイロンが表示される。1km、10km円が描かれる。琵琶湖の輪郭線が表示される。
    * 画面下部：時刻（JST）/緯度/経度/目的地コース(Magnetic Course)/目的地までの距離（km）/ 補足衛星数 / 電圧
    * 画面最下部：目的地モード / 目的地名 / SD認識表示
 * ロガー：緯度/経度/速度(kt)/真航跡/時刻のSDカードへの保存　（GPS位置捕捉時のみ、1秒に1度）
 * 設定画面
    * 目的地の設定
    * 向かうIntoモードか、離れるAwayモードか、目的地から10kmをひたすら往復する Auto10km モード(大会用）の選択
    * デモモード
    * リプレイモード
    * NORTH UP（北が上） / TRACK UP（進行方向が上） の設定
    * GPSのコンステレーション表示（受信中衛星）
    * SDカード内のファイル表示

