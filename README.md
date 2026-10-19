> ## ⚠️ 重要なお知らせ:安全のため使用を一旦中止してください(2026-10-15)
>
> 充電IC **MCP73831** の **VDD** / **VBAT** に必要な **4.7µF のコンデンサが実装されていない**不具合が見つかりました。
> 電源スイッチ投入時のチャタリング(接点の瞬間的な ON/OFF 反復)で IC に過電圧が加わり、**ごく稀に IC が故障**します。
> 故障時は **発熱(焦げ臭くなる)** する可能性があります。
> v3 は充電IC が電源スイッチの後ろにあるため、**電源スイッチを OFF にすれば電池からは切り離せます**。
> 異常を感じたら、**電源スイッチを OFF にし、USB ケーブルも抜いてください**(USB からの給電は抜くまで止まりません)。
>
> **改修(コンデンサの追加)が完了するまで、本バージョンの基板の使用を中止してください。**
>
> * **対象バージョン**:v1 を除く全バージョン(v3 / v4 / v5 / v6 / v7 / v7.1。`main` ブランチは v6 と同一のため対象)。
> * **修正方法**:このバージョンは本リポジトリに基板データ(KiCad)が含まれていないため、現物の基板上で
>   MCP73831(充電IC、SOT-23-5 パッケージの小さな IC)を探し、データシート(Microchip DS20001984)の
>   ピン配置(`1=STAT, 2=VSS/GND, 3=VBAT, 4=VDD, 5=PROG`)を確認した上で、**4.7µF のセラミックコンデンサ**
>   (X5R/X7R、耐圧 10V 以上推奨)を **VDD(4番)⟷VSS/GND(2番)間** と **VBAT(3番)⟷VSS/GND(2番)間** の
>   それぞれに後付けで実装してください。ピンの特定に自信が無い場合は、無理に自己判断で改修せず下記へご相談ください。
> * ご不安な場合は masaochiguchi@gmail.com までご連絡ください。

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

## GNSS Module モジュール
 UART NMEA0183 で通信するため、任意のモジュール使用可能。下記のGNSSモジュールで実験済み。ただし、Mediatek or ubloxで、初期化処理が異なるため、settings.hを変更すること。
 * GT-502MGG-N https://akizukidenshi.com/catalog/g/g117980/
 * ublox M-10Q
 * Quectel L86


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

