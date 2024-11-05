# GPS_TFT_map 概要/Abstract
 * PONSデバイスの使い方や画面の見方などの詳細は、[docsフォルダ](docs)に説明書があります。このREADMEファイルの方が情報が古い場合があります。
   ![image](https://github.com/user-attachments/assets/e5c458c2-ea0e-470b-8ae2-00049514d380)
 * Arduinoを使用して、TFTにGPSマップや位置・速度・方位の情報を表示します。琵琶湖での鳥人間コンテスト用に特化されています。
 * PONS for HPA = Pilot Oriented Navigation System for Human-powered aircraft = 人力飛行機用パイロット向けナビゲーションシステム
 * 人力飛行機用のパイロットが使いやすいナビゲーションシステムです。琵琶湖上空を飛行するパイロットの支援を目的としています。
 * 地図データは、Google earthから出力された緯度経度の他、Google map APIにて事前ダウンロードしたbmp画像をSDカードから読み込んで表示も可能です。
 * 日本地図・琵琶湖（沖島・竹生島・多景島）などがプログラムに事前登録されています。


# 推奨機器
## マイコン  Recommended Microcontroller
下記二ついずれかを推奨。
 * Raspberry Pi Pico / Raspberry Pi Pico 2
   
 TFTとの通信では、PicoのPIOを使用し 16 bit Paralel 接続を使う事で高速描画可能になっています。これは、TFT_eSPIライブラリによるものです。

## TFT  Recommended TFT panel.
 Tested on ILI9341 and ST7789 using TFT_eSPI library.  Aliexpress で安価に購入もできるが、屋外で使用する場合は輝度が高く画質も綺麗なDigikeyにて購入を推奨する。
 * Aliexpress TZT Choice Store
   * Arduino、2.8 "、240x320、spi、tft、pcbアダプター、マイクロsd、il9341、st7789v、5v、3.3v、2.8" 用のLCDシリアルポートモジュール、LEDディスプレイ
 * Digikey (動作確認済みTFT)
   * 757-NHD-2.4-240320CF-BSXV-F-ND
   * NHD-2.4-240320CF-CSXN#-F-ND
   * NHD-2.8-240320AF-CSXP-F-ND  

## GNSS Module モジュール
 UART NMEA0183 で通信するため、任意のモジュール使用可能。下記のGNSSモジュールで実験済み。ただし、Mediatek or ubloxで、初期化処理が異なるため、settings.hを変更すること。
 * GT-502MGG-N https://akizukidenshi.com/catalog/g/g117980/
 * ublox M-10Q
 * Quectel LC86G


# 機能 Detail.
 * 基本機能
    * 画面上部：対地速度、航路（Magnetic Track）の表示。
    * 画面下部：琵琶湖プラットホームまでの距離表示（km）/ 補足衛星数 / 電圧。BAT_LOW と表示される = 3.55V以下。電池交換推奨。表示されてから約30分程度は正常に動作する。
    * 画面最下部：緯度/経度/SD認識 SDカード正常認識＝緑色。SDカードエラー＝赤色。
 * 緯度/経度/速度(kt)/真航跡/時刻のSDカードへの保存　（GPS位置捕捉時のみ、1秒に1度）
 * 過去の航跡は黒色の線で引かれる。おおよそ過去10分の履歴が表示される。
 * SDカードから独自地図のを読み込んで表示する
 * 設定画面
    * 画面の輝度調整
    * デモモード
    * NORTH UP（北が上） / TRACK UP（進行方向が上） の設定
    * GPSのコンステレーション表示（受信中衛星）
 * 琵琶湖付近にいる場合
    * 自動的にプラットホームから離れる方向にピンク色で線が引かれる。
    * タケシマ・北パイロン・西パイロンに向けて自動で線が引かれる。


# SD：Creating map.  地図作成
 * 追加地図（独自地図）の作り方・使い方
 * Google Earthでパスを作り、KMLファイルをダウンロードする。
 * kml_to_mapcsv.py を使って、KMLファイル名を入力すると、自動でmapdata.csvファイルが作成される。
 * 作成された 「mapdata.csv」ファイルをSDカードのルートフォルダに追加する。
 * 捕捉：CSVデータ形式＝名前,point数,lon1,lat1,lon2,lat2,lon3,lat3....(point数分続く）。改行ごとに線が引かれる。
 * 捕捉：名前は英語のみ対応。また1文字目によって線の色が決まる。r:RED,o=ORANGE,g=GRAY,m=MAGENTA,c=CYAN
   * 赤色の線の例：r_warningline,2,135.0,46.0,135.1,46.1
 * 捕捉：描ける線の数を便宜上200に制限している。上限はプログラムで変更可能。

