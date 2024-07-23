# GPS_TFT_map
 Show GPS map on TFT and data on OLED using Arduino.  Code are specialized for Birdman rally at Biwako.

# 推奨マイコン  Recommended Microcontroller
下記二ついずれか。
 * Waveshare RP2040 zero
 * Rasbery Pi PICO
   
 PICOでは、TFTとの間で8 bit or 16 bit Paralel接続を使う事で高速描画可能。

# 推奨TFT  Recommended TFT panel.
 Tested on ILI9341 and ST7789.  Using TFT_eSPI library.

# 概要  Abstract
PONS for HPA = Pilot Oriented Navigation System for Human-powered aircraft
人力飛行機用のパイロットが使いやすいナビゲーションシステムです。Designed to assist pilots while flying over lake Biwa.

# 機能 Detail.
 * 画面上部：対地速度、航路（Magnetic Track）の表示。
 * 画面下部：琵琶湖プラットホームまでの距離表示（km）/ 補足衛星数 / 電圧。BAT_LOW と表示される = 3.55V以下。電池交換推奨。表示されてから約30分程度は正常に動作する。
 * 画面最下部：緯度/経度/SD認識 SDカード正常認識＝緑色。SDカードエラー＝赤色。
 * 緯度/経度/時刻のSDカードへの保存　（GPS位置捕捉時のみ、1秒に1度）
 * SDカードからの独自地図の表示
 * 設定画面
 * 画面の輝度調整
 * デモモード
 * NORTH UP（北が上） / TRACK UP（進行方向が上） の設定
 * GPSのコンステレーション表示（受信中衛星）
 * 琵琶湖付近にいる場合には、自動的にプラットホームから離れる方向にピンク色で線が引かれる。
 * タケシマ・北パイロン・西パイロンに向けて自動で線が引かれる。
 * 航跡は黒色の線で引かれる。


# 地図作成
 * 追加地図（独自地図）の作り方・使い方
 * Google Earthでパスを作り、KMLファイルをダウンロードする。
 * kml_to_mapcsv.py を使って、KMLファイル名を入力すると、自動でmapdata.csvファイルが作成される。
 * 作成された 「mapdata.csv」ファイルをSDカードのルートフォルダに追加する。
 * 捕捉：CSVデータ形式＝名前,point数,lon1,lat1,lon2,lat2,lon3,lat3....(point数分続く）。改行ごとに線が引かれる。
 * 捕捉：Google Earth上でPATHの名前は英語で登録を。またPATH名から地図で引かれる線の色が決まる。r:RED,o=ORANGE,g=GRAY,m=MAGENTA,c=CYAN
   * 例：r_warningarea,
 * 捕捉：描ける線の数を便宜上200に制限している。上限はプログラムで変更可能。

