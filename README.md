# GPS(GNSS)-TFT-map_PONS=Pilot Oriented Navigation System v6
<img width="1312" height="368" alt="ponsv6image" src="https://github.com/user-attachments/assets/1e0ee79d-7a78-437a-a577-ab036f17d5f1" />
<img width="240" height="52" alt="logo" src="https://github.com/user-attachments/assets/ffad4b6f-56ef-4505-8b95-28498c59182f" />


# PONS 概要/Abstract
 * 人力飛行機のパイロットが使いやすいことを目標とした航法装置（ナビゲーションシステム）です。琵琶湖上空を飛行するパイロットの支援をしています。Arduinoを使用し、TFTにGNSSマップや位置・速度・方位の情報を表示します。琵琶湖での鳥人間コンテスト用に特化しています。GPS/GNSS navigation for human-powered aircraft specialized for the Japan International Birdman at Biwako, Japan.
 * 大阪大学albatrossにて使用実績あり（2024追走ボート: v3, 2025追走ボート: v4, 2025機体搭載「白夜」: v5, 2026追走ボート: v6, 2026機体搭載「陽還」: v6 β）。2025年大会優勝。
 * 3Dプリントケース及び基板データ（KiCad)あり。ケースは、PLA_LWが軽量化でオススメです。
 * 最新のソフトウェアバージョンは **0.922**（Build 20260731）です。
 * PONSデバイスの使い方や画面の見方などの詳細は、docsフォルダに説明書（PDF）があります。画面写真付きで説明されているため、READMEより詳しい情報が載っています（ただしPDFのバージョンがソフトウェアより古い場合があります）。
 * PONS for HPA = Pilot Oriented Navigation System for Human-powered aircraft = 人力飛行機用パイロット向けナビゲーションシステム
 * 地図データは自作したGoogle earthから出力された緯度経度列の他、Google map APIにて事前ダウンロードしたbmp画像をSDカードから読み込んで表示も可能です。
   * Google map APIを利用した画像データは、各自がAPIを利用登録する必要があります。APIへのアクセスとbmpへの変換は、支援ツール（Python）がtoolsフォルダにあります。
 * 日本地図・琵琶湖（沖島・竹生島・多景島）などは、プログラムに事前登録されています。パイロン座標は2025ルールに合わせてありますが、現実のパイロン位置はズレることがあるようです。

# スペック (v6)
 * 重量: 59.8g (W59 H81 D20 mm)。v5比 15.5g（21%）軽量化。
 * バッテリー: 約3時間30分（高輝度モード） / 約12時間（低輝度モード）。USB Micro-Bで充電（5V1Aで十分、空から満充電まで2〜3時間）。
 * TFT: 2.8インチ 1000ニト（v5は600ニト）。スイッチによる輝度調整可能。
 * GNSS位置精度: 1.5m、速度精度: 0.05m/s、更新レート: 2Hz、Cold start 23秒
 * 気圧センサー MS5611 搭載。気圧を2HzでSDカードへ記録。

## v6での主な変更点（v5から）
 * GNSSを Quectel LC86G から u-blox SAM-M10Q に変更し、アンテナを裏面から**天面**に移動。感度・精度が体感で分かる程度に向上。
 * 更新レート 1Hz → 2Hz。速度・トラックの更新頻度が2倍。
 * TRACK UP モードのサポート追加（v5まではNORTH UP推奨）。
 * 気圧センサー MS5611 搭載。バリオメーター機能を追加。
 * 過去5分および累計の最大GS表示を追加（主にTF時の確認用）。
 * SDログ項目の追加（気圧、GNSS精度・捕捉数、電圧など）。
 * 画面輝度向上、ブツブツ音ノイズなど細かいバグ修正。

## v6 β試験版（2個中1個）
 * BNO085（モーションセンサー）を内蔵し、デバイスの pitch / roll / yaw を約5HzでSDへ保存。
 * バリオメーターの精度が向上しているため β版でのみバリオメーターを有効化。通常版は精度が不十分なため無効。
 * 重量 62.4g（通常版より約2.6g重い）、バッテリー持続は約3時間（通常版より約30分短い）。
 * 通常版は起動画面で「BNO085:NG」と表示されるが正常。

# 取り扱い上の注意
 * **リチウム電池内蔵のため高温禁止。** 真夏の車内・ダッシュボードへの放置禁止。高温密閉環境での充電は発火の恐れあり。落下や踏みつけなど強い衝撃を与えないこと。
 * **防水ではない。** 雨ざらしにしないこと。多少の水滴は許容だが拭き取ること。
 * **天面付近に金属・カーボン・金属フィルムを配置しないこと。** GNSSアンテナが天面にあるため、信号を受信できず使用不能になる。取り付け位置・固定方法・フェアリング材質に注意。
 * **偏光サングラス禁止**（画面が見えなくなる）。偏光でないサングラスはOK。
 * リチウム電池入りのため廃棄しないこと。不要になったら開発者へ返却してください。

# 外観（スイッチ / LED）
 * 左側面：microSDスロット（押し込むとカチっとラッチ解除）、USB Micro-B（LiPo充電用）、TFTバックライトスイッチ（下）、バッテリー電源スイッチ（上）
   * バックライトは夜間・暗所ではオフに。オフだとバッテリーの保ちが3倍程度（約12時間）になる。
   * USB接続時に電源ONにした場合はUSB電源が使用される。
 * 右側面：操作用トグルスイッチ（短押し / 長押し）、スピーカー
 * 正面：左上LED＝USB接続中(白) / 不具合発生中(赤点灯) / GNSS信号ロスト(赤点滅)、右上LED＝充電インジケーター(赤)、TFT 2.8インチ
 * 天面：内部から天に向けてGNSSアンテナ

# 電池・充電
 * `USB` 表示 = USB接続中でUSBから給電。充電完了は右上の赤LED消灯で確認。
 * 緑 `XX%` = 3.8V以上（50%以上）。マゼンタ = 3.8V未満。赤 = 3.5V以下（約12%以下、充電推奨）。
   * 4.2V=100%、3.4V=0% として電圧をそのまま%表示。高輝度モードでは充電直後でも100%を切るが正常。
 * 赤表示から20分程度で過放電保護が働き電源が落ちる。**その後も電源ONで放置すると保護回路がFailし電池が恒久的に使用不能になる**（この場合は充電を試みず電池交換）。
 * 電圧低下の目安: 高輝度 -0.17〜-0.2V/h、低輝度 -0.06V/h。
 * 右上LEDが点滅 = バッテリー接続不良・エラー。左上LEDの赤常時点灯 = 不具合発生（開発者にコンタクト）。赤点滅 = GNSS信号ロスト。

# 推奨機器
## マイコン  Recommended Microcontroller
 * RP2354B

 TFTとの通信では、TFT_eSPIライブラリ（PicoのPIO）を使用し 16 bit Paralel 接続を使う事で高速描画可能になっています。

## TFT  Recommended TFT panel. (推奨の動作確認済みTFT)
 * Digikey
   * NHD-2.8-240320AF-CSXP-F-ND (v4,v5,v6[Rev1B])
   * NHD-2.4-240320CF-CSXN#-F-ND (v3:追走ボート用で使用)

## GNSS Module モジュール
 v6以降、UBX で通信するため、u-bloxモジュールで使用可能。下記のGNSSモジュールで実験済み。
 v5までは、UART NMEA0183 で通信。ただし Mediatek or ubloxで、初期化処理が異なるため、settings.hを変更すること。
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
      * 30度範囲表示：針の先の三角形（薄い線）が過去の針路から左右15度の範囲を示す。15度を超えるたびに三角形が更新され、警告音+「トラック」音声が鳴り8秒間点滅する。
    * 画面下部：sAcc（速度精度1σ）/ 最大GS（過去5分・累計）と発生時刻 / 時刻（JST）/緯度/経度
    * 画面下部2：目的地コース(Magnetic Course)/目的地までの距離（km、直線距離）/ 捕捉衛星数（10以上で緑）/ 電池残量(%)
    * 画面最下部：目的地モード（FLY INTO / FLY AWAY / 10K INTO / 10K AWAY）/ 目的地名 / SD認識表示（正常=緑、エラー=赤）
    * 状態表示：`Acc: XXm`（水平位置精度1σ、10m以上のとき表示）、`Scanning GNSS/GPS Signal`（起動直後）、`Weak GNSS/GPS Signal`（衛星数0）、`No GNSS connection !!`（GNSS通信不可＝要再起動）、`No map image available.`（該当エリアの地図画像なし）
 * ロガー：緯度/経度/対地速度/時刻をSDカードへ保存（GPS位置捕捉時、1秒に2回）。気圧データも記録。
 * バリオメーター(MS5611)：昇降計。設定画面でボリューム0にすると無音・非表示。デッドバンドは±0.3m/s。通常版は精度が不十分なため無効、β版のみ使用可。
 * 設定画面（NAV画面でスイッチ長押しで入る。赤丸⚫︎は本番推奨設定を満たしていない項目。本番では全て緑丸にすること）
    * 目的地の設定（PLATHOME, N_PILON, W_PILON, TAKESHIMA が本体登録済み。SDカードで追加可能。SDには SHIRAHAMA16 / SHIRAHAMA34 / KASAOKA03 / KASAOKA21 / BUSHITSU 等を初期登録済み。）
    * 向かうIntoモードか、離れるAwayモードか、目的地から10kmをひたすら往復する Auto10km モード(大会用）の選択
    * ボリューム / バリオメーターボリュームの調整
    * NORTH UP（北が上） / TRACK UP（進行方向が上） の設定
    * 地図の拡大率（Map scale）の変更。NAV画面ではボタンのダブルクリックでも変更可能（誤操作防止のためシングルクリックでは変わらない）。通常使用する zoom11 のときアイコンが緑、それ以外は赤。
    * デモモード（琵琶湖でそれっぽいデータを表示）
    * リプレイモード（SDカードのNMEAデータを再生）
    * GPSのコンステレーション表示（受信中衛星・NMEA原文）
    * 地図データ情報一覧表示 (FLASH + SD mapdata.csv)
    * SDカード内のファイル表示
    * 設定はSDカードに保存・次回起動時に自動読込（DEMO/REPLAYモードは保存されない）。NAV画面のscale（拡大率）も保存される。
    * 画面下部にバッテリーの目安持続時間とCPU温度を表示。CPU55度（外気温35度）までは動作確認済み。2026年大会ではCPU最大48度。

# 警告音・警報
 * 旋回率 警告音：旋回率が2deg/sを超えるとピピと高音（v6では音程固定）。2.0m/s以下では鳴らない。
 * 航路(TRACK)警報：左右15°範囲を超えて航路が変わるとChime音+「Track！」音声。
 * 目的地自動更新音：Auto 10kmモードで10K AWAY↔10K INTO切替時に4Chime+音声。切替時に1回のみ。
 * コース警報：針路のズレが10〜60秒放置されると4Chime+「針路、右(左)に旋回」音声。コース警報Arcがピンク線(MC)に到達すると鳴る。
   * 15°未満のズレでは鳴らない。15°で60秒、90°以上で10秒放置が条件。修正操作中は鳴らない。前回警報から30秒以内は再警報しない。GSが2m/s以下でも鳴らない。
 * 電圧低下警報：3.5V未満で音声案内（60秒に1回）。この警告のみ強制的に最低音量60で鳴る。
 * 目的地確認：目的地まで100km以上あると120秒に1回、確認を促す音声。

# ツール (tools/)
 * `tools/logcsv2kml4earth.py`：複数のフライトログCSVをまとめてKMLに変換（Google Earth用）
 * `tools/logcsv2tourkmz.py`：ログCSVをGoogle Earthのツア－(KMZ)に変換
 * `tools/kml_to_mapcsv.py`：Google EarthのKMLから独自地図用 `mapdata.csv` を生成
 * `tools/createmovie_csv2mp4.py`：ログCSVからフライト動画を生成
 * `tools/flight_preprocess.py` / `tools/flightanalysis/`：ログの前処理と飛行解析（スペクトル、横・方向安定性など）のプロット
 * `tools/vectormap/build_vectormap.py`：OpenStreetMap から内蔵ベクタ地図 `vectormap_data.cpp` を生成
 * `tools/png2bmp.py`：PNG画像を起動ロゴ用bmpへ変換

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

# SDカードのログ記録項目
 * ルートフォルダのCSV（例：`2026-03-26_1519.csv`）
   `latitude, longitude, gs, TrueTrack, Altitude, KF_Altitude, KF_Vspeed, pressure, date, time`
   * `Altitude` はGNSSのみ。`KF_*` はKalman filterによる推定（β版は MS5611+BNO085+GNSS、通常版は MS5611+BNO085）。
 * `euler/`フォルダ（例：`euler/20260324.txt`）：β版のみ。`time, roll, pitch, yaw`（BNO085。センサーが内部で傾いているためオフセットあり）
 * `/log.txt`：起動からの秒数、電圧、センサー類の初期化結果やGNSS精度などのイベントログ。

# 大会本番での使い方（Auto 10km モード）
 * 目的地はパイロンではなく **PLATHOME** を選択し、Navigation Mode を **Auto 10km** にする。
 * 自動で FLY AWAY（表示は `10K AWAY`）で始まる。プラットホーム上や上昇中はプラットホームから離れる方角（南東）を指すが正常で、数秒飛べば北西方面の指示に変わる。
 * このモードはパイロンへは誘導せず、**プラットホームから離れる最短距離の方角（マゼンタ線=MC）** に誘導する設計。風は考慮していない。
 * パイロンに向かいたい場合は、画面上の北パイロン・西パイロンへの線とMagnetic Track(MT)を見てパイロット自身が判断する。2025ルールではプラットホームから各パイロンへのMCはそれぞれ 326° と 266°。
 * 目的地から 10.475km 離れると自動更新音が鳴り `10K INTO` に切替。以降はパイロンを旋回したかに関係なくプラットホームへ戻る警告が出るので、**目視で折り返しを終えるまではコース警報を無視する**。
 * 復路で目的地から 1.5km 以内に入ると再び `10K AWAY` に切替。同様に警告を無視して目視で1kmパイロンを探して旋回する。
 * 実際のパイロン位置は目安の 10.975km 円からズレる（2025大会では5km通過で膨らんだ）。6km過ぎたあたりから目視で探すこと。
 * 目的地の `N_PILON` / `W_PILON` は、飛行中に急遽反対側のパイロンへ向かう場合に FLY INTO で直接誘導するためのもの（主にボート側での操作を想定）。

# 使用者（パイロット/ボート）向け留意事項
 * **TFでは目的地の設定を間違えやすい。** 逆に設定すると飛行中に逆向きの旋回指示が出る。風向き変化で逆向きに飛ぶ場合はPONSの設定も変更すること。
   * 例：笠岡の南向きなら `KASAOKA21`+FLY AWAY または `KASAOKA03`+FLY INTO。白浜の北向きなら `SHIRAHAMA16`+FLY INTO または `SHIRAHAMA34`+FLY AWAY。
 * **磁方位と真方位の違い：** NORTH UPモードの画面真上は真北(True 360°)だが、磁方位では008°。画面のコンパス(N/E/S/W)は磁方位を示し、真方位から偏角8度ずれている。磁気コンパスと一致させるためPONSは基本的に磁方位を使用・表示する。
 * **方位の精度は対地速度に依存する。** 移動速度が遅いと方位が大きく変化しやすい。徒歩ではばらつくが、ジョギング(3m/s)以上で安定する。
 * **航路(TRACK)警報が鳴っても、すぐに画面を注視しないこと。** 警告音は航路変化＝姿勢変化を知らせている。まずバンクなど機体姿勢の安定が最優先。姿勢が落ち着いてから修正方向・修正量の確認に使う。
 * 稼働時は捕捉衛星数が10sats以上（緑色）、`sAcc: 0.2m/s` 以下になっていることを確認するとよい。

# 使用部品 (v6)
| 部品 | 型番 | 備考 |
|------|------|------|
| GNSS | ublox SAM-M10Q | 基板表面実装 |
| TFT | Newhaven NHD-2.8-240320AF-CSXP-F-ND | 2.8インチ |
| バッテリー | マルツ DTP603048 860mAh | 1s、過充電・過放電保護回路付き |
| 充電コントローラー | MCP73831 | - |
| 気圧センサー | MS5611 | 気圧を2Hzで記録 |
| モーションセンサー | BNO085 | β版のみ搭載 |

# 着水（水没）後の処理
 1. 分解して取れる水分をすべて拭き取り、ネジのサビを除去する。
 2. バッテリー端子とTFT端子を取り外し、可能な範囲で拭いた後エタノール洗浄する。バッテリーはテープと回路部分を分解洗浄し、テープ保護して戻す。
 3. 乾燥剤を入れたジップロックでTFT内部などの水分を抜く（湿度20%で2〜3日放置推奨）。
 4. DIPスイッチは内部で錆が発生し接触不良になる恐れがあるため交換推奨。

# 開発者向け
 * 電源スイッチ故障時に備え、基板にジャンパーがある（はんだで繋げばON）。GPSバックアップ電源（リチウムボタン電池）故障時のジャンパーもある。
 * 参考動画：[PONS v5 琵琶湖 デモモード（音声誘導の例）](https://www.youtube.com/watch?v=fB5rZf2j_d8) / [PONS v5 Auto10kmモード 折り返し動作確認](https://www.youtube.com/watch?v=gKichWYD1Wk)
 * 問い合わせ：開発者 大阪大学 albatross OB 7期 MasaoC（masaochiguchi@gmail.com / LINE, X: masao_mobile）

# ライセンス
本リポジトリは 2 種類のライセンスで構成されています。

| 対象 | ライセンス |
|------|-----------|
| ソースコード（`*.ino` / `*.cpp` / `*.h` / `tools/*.py`） | MIT License（[LICENSE](LICENSE)） |
| 地図データ（`vectormap_data.cpp`） | ODbL 1.0（[LICENSE.ODbL](LICENSE.ODbL)） |

地図データは OpenStreetMap 由来です。

> Map data (c) OpenStreetMap contributors — https://www.openstreetmap.org/copyright
> Licensed under the [Open Database License (ODbL) v1.0](https://opendatacommons.org/licenses/odbl/1-0/)

`vectormap_data.cpp` は座標列を格納したベクタ形式のため、ODbL における **Derivative Database**
（派生データベース）に該当し、share-alike の対象になります。再配布時は ODbL のままとし、
帰属表示を保持してください。ODbL 4.5(a) の Collective Database 免除により、
ファームウェアのソースコード本体は MIT のままで問題ありません。詳細は [LICENSE.ODbL](LICENSE.ODbL) を参照。

地図データの生成方法は [tools/vectormap/build_vectormap.py](tools/vectormap/build_vectormap.py) を参照してください。
