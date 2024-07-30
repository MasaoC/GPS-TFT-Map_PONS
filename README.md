# GPS_TFT_map 概要/Abstract
 * Show GPS map on TFT and data on OLED using Arduino.  Code are specialized for Birdman rally at Biwako.
 * PONS for HPA = Pilot Oriented Navigation System for Human-powered aircraft
 * 人力飛行機用のパイロットが使いやすいナビゲーションシステムです。Designed to assist pilots while flying over lake Biwa.

# 推奨機器
## マイコン  Recommended Microcontroller
下記二ついずれかを推奨。
 * Waveshare RP2040 zero
 * Rasbery Pi Pico
   
 Picoでは、TFTとの間で16 bit Paralel接続を使う事で約３倍高速描画可能。

## TFT  Recommended TFT panel.
 Tested on ILI9341 and ST7789 using TFT_eSPI library.  Aliexpress で安価に購入もできるが、屋外で使用する場合は輝度が高く画質も綺麗なDigikeyにて購入を推奨する。
 * Aliexpress TZT Choice Store
   * Arduino、2.8 "、240x320、spi、tft、pcbアダプター、マイクロsd、il9341、st7789v、5v、3.3v、2.8" 用のLCDシリアルポートモジュール、LEDディスプレイ
 * Digikey (動作確認済みTFT)
   * 757-NHD-2.4-240320CF-BSXV-F-ND
   * NHD-2.4-240320CF-CSXN#-F-ND

## GPS Module モジュール
 UART NMEA0183 で通信するため、任意のモジュール使用可能。
 * テストではGT-502MGG-N を使用した。https://akizukidenshi.com/catalog/g/g117980/


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

