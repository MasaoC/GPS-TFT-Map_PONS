# Pilot Oriented Navigation System v5
<img width="220" alt="ponsv5" src="https://github.com/user-attachments/assets/42279b57-4c61-43eb-a7d4-d60ea7dddddf" />

# GPS_TFT_map 概要/Abstract
 * PONSデバイスの使い方や画面の見方などの詳細は、[docsフォルダ](docs)に説明書があります。このREADMEファイルの方が情報が古い場合があります。
 * Arduinoを使用して、TFTにGPSマップや位置・速度・方位の情報を表示します。琵琶湖での鳥人間コンテスト用に特化されています。
 * PONS for HPA = Pilot Oriented Navigation System for Human-powered aircraft = 人力飛行機用パイロット向けナビゲーションシステム
 * 人力飛行機用のパイロットが使いやすいナビゲーションシステムです。琵琶湖上空を飛行するパイロットの支援を目的としています。
 * 地図データは、Google earthから出力された緯度経度の他、Google map APIにて事前ダウンロードしたbmp画像をSDカードから読み込んで表示も可能です。
 * 日本地図・琵琶湖（沖島・竹生島・多景島）などがプログラムに事前登録されています。


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

