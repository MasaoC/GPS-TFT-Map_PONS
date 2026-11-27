# TODO（やること・後回しにしていること）

<!--
File    : docs/pons_todo.md
Project : PONS v7.1 / v7.2
Role    : 残っている作業の一覧。1 項目 1〜2 行。経緯・数字・確かめ方の詳細は pons_todo_notes.md。
Updated : 2026/11/27
-->

**済んだら行を消す。増えたら足す。** 1 項目は 1〜2 行。詳しいことは [pons_todo_notes.md](pons_todo_notes.md)（「メモ §n」）か各行のリンク先。
**v7（BNO085）の基板は廃棄する可能性があるので、v7 での確認と v7 へのファームの持ち込みはしない**（2026-11-18、ユーザー）。

並べ方: **§1 が飛ぶ前に必ず済ませるもの**。§2〜§4 は「どこで確かめるか」で分けた（車・屋外／机の上／2 台以上）。

---

## 1. 飛ぶ前に必ず（Step 4 の残り → Step 5）
SCH16T は Step 5 が済むまで飛ばさない（`#warning "... DO NOT FLY."`）→ [sch16t_design.md](sch16t_design.md) §6
- [ ] **方位が決まる前の「加速中」の上限（11/22。実機未確認）**: 11/22 の車で、はうように 8 秒動いたところで上限が切れ、そのあとの加速で
  ピッチ +3.8°・2D 相関も 0 に戻され方位は航跡に（mag=0）。方位が決まる前で GNSS が動いていると言う間は上限を 30 秒に（`ESKF_ACCEL_HOLD_MAX_PREYAW_US`）。
  生ログの再現でピッチのずれ +3.6° → 0.2° 以内、方位は 2.8 m/s で相関（mag 284・航跡と 0.1°）。yaw_test D14。方位が決まった後の発進（20 回）は ±1〜2°・最大 3° で良好
- [ ] **方位の確定（2D 相関）を車で**: 11/22 は 2 回とも航跡に戻った（上の上限と、起動 9 秒で動き出した回）。上を焼いてから、起動して
  `GYRO BIAS: learned` まで止めて置き、止まった所からはっきり加速する。`ESKF YAW INIT:` の `how`・`mag`・`corr-trk` を見る → [pons_yaw_init_plan.md](pons_yaw_init_plan.md) §4
- [ ] **起動地の MSL 高度の決め直し（11/22。実機未確認）**: 11/22 の車で最初の測位のでたらめな高度（vAcc 9.9 m と出て 50 m 以上ずれ）でオフセットが +82.7 m に決まり、
  50 m の外れ値の門で 30 分直らなかった（KF の MSL 高度が 80 m 高い）。決めるときは vAcc 5 m 未満、精度の良い測位で 50 m を超えて外れ続けたら 20 回で決め直す
  （`GNSS_OFFSET_*`、log.txt `GNSS alt offset re-init`）。vario_test §8。次の走行で CSV の GNSS_Altitude と KF_Altitude が数 m 以内か見る
- [ ] **ジャイロの飽和で方位を捨てる（11/27。実機未確認）**: 手で素早く回すと SCH16T が ±327.68 °/s で頭打ちになり、方位が大きく狂うのに精度は良いままだった。
  300 °/s 以上で方位を捨てる（`ESKF_GYRO_SAT_DPS`）。方位が決まったあとに手で素早く回す → 灰色・`YAW ALIGN`、log.txt に `ESKF GYRO SAT` と `imu sat`。
  そのまま発進して `ESKF YAW INIT` で入れ直されるか。`imu sat` の数と `ESKF GYRO SAT` のサンプル数で、チップが飽和を言う角速度も見る。yaw_test G8・G9
- [ ] Step 5 飛行。**立ち会わず、飛んだあとに直せない**ので、直すものは飛ぶ前に決める。
  飛行のログで見ること: GNSS の上下速度（`vsi gnss` の `innov=`＝湖面の反射。食い違いの門は 11/22 の車で働いた `consist=` も）、加速度のガード（`VARIO: IMU accel ignored`）、離陸直後のピッチ

## 2. 車・屋外で確かめる
- [ ] DBD と PC の時刻: BT1 が切れた起動の TTFF、`COLD` / `COLDONLY` の比べ、`given time was ... off`（新しい基板の DBD は 11/18 に保存済み）
  → [pons_ttff_assist_plan.md](pons_ttff_assist_plan.md) §3.5・§3.6
- [ ] 対地 2 m/s で飛ぶときの門（車で動作確認）: 停止後、5 秒で `GROUND` が戻るか（`TAS_LAND_HOLD_MS`）
- [ ] ケイデンスの推定（`tools/imulog/cadence.py`）: 台の上で漕ぐか飛行の生ログで、前後の振動が 180 回/分（3.0Hz）前後に出るか

## 3. 机の上で確かめる
- [ ] **PC がつないでいる間は PC 接続のまま（11/25。実機未確認）**: Re:PONS のダイアログを閉じて 2 分置いても本体が地図へ戻らず生 IMU ログが伸びないか、
  タブを閉じると 3 秒で戻り log.txt に `PC PORT CLOSED (3s)` が出るか
- [ ] **受信のミラー中の充電ポップアップ（11/23。実機未確認）**: 送信機と受信機を一緒に充電 → 受信機に `RX CHG ...` が出るか・箱が機体の表示と重ならないか、
  短押しで消えて USB を挿し直すまで出ないか、受信が切れて自機の画面に戻ると `RX` の無い箱が出るか
- [ ] **USB のまま SW を入れたら「挿し直して」（11/25。実機未確認）**: 実機: 電池 90% 前後で SW OFF のまま USB → SW ON で赤 LED が点かないか・ヘッダーが `REPLUG` か、挿し直すと点くか → [pons_power.md](pons_power.md) §4.1
- [ ] **充電の残り時間の終わり（11/25。実機未確認）**: 4.20V に着いてから 45 分を数え下げる（`CHG_TAIL_MIN`）。`1m (max 16m)` に張り付かないか、
  log.txt の `CHG [..m]` の rem と `CHG FULL` の時刻が合うか → [pons_charge_estimation.md](pons_charge_estimation.md) §4.3
- [ ] **Re:PONS からパイロン座標を変える（11/23。実機未確認）**: 本体設定の「コース（パイロン）」で北パイロンを変えて保存 → SD の `override_pilon_coordinate.csv` と
  log.txt の `PC PILON ... (next boot)` → 再起動で `PILON OVERRIDE: 1 applied` と地図。既定に戻すとファイルが消えるか。
  北パイロンを 7 km の所にすると欄が赤く、保存で確認のダイアログが出てキャンセルで何も書かれないか → [pons_webserial_protocol.md](pons_webserial_protocol.md) §4 (8)
- [ ] **GNSS が抜けたあとの KF 高度（11/27 修正・実機未確認）**: 起動して fix → GNSS のコネクタを抜いて本体を 1〜2 m 上下させ、数分置いても
  高度の表示が抜けた時点の GNSS 高度へ寄っていかないか（凍った高度を 1 秒ごとに入れていた。imu.cpp の imu_kalman_gnss_update を iTOW で 1 回だけに）
- [ ] **地図以外の画面の DIM MODE（11/26。実機未確認）**: 電池で設定画面・GNSS DETAIL・VARIO などを出したまま 30 秒置く → 画面の上に `DIM MODE` の箱が出て、
  ボタン・手かざしで戻ると箱が消えて元の画面に描き直されるか。減光中に設定画面が時間切れで地図へ戻ったときも → [pons_power.md](pons_power.md) §4.3
- [ ] **APPLY した日時の表示（11/27。実機未確認）**: 測位してから APPLY → IMU/ESKF 1/2 の stored offset の下に `applied YYYY/MM/DD HH:MM`（JST）。
  再起動しても残るか（settings.txt の `calib_time`）、前日の較正で橙の警告 2 行が出ているときも最終行が画面の下で切れないか
- [ ] v7.2 基板が届いたら: `PONS_MAINBOARD_REV` = 72 で焼き、満充電で数日〜数週間切ってから起動して `RTC BACKUP CHECK: ok` か、
  充電ポップアップの `Fast GPS <2d, clock 3mo`、回路が動き出す・止まる電圧（約 3.92V / 3.83V）→ [pons_charge_estimation.md](pons_charge_estimation.md)

## 4. 2 台以上で確かめる（無線）
- [ ] **MAX G/S・航跡を受信機の分と送信機の分で別々に（11/23。実機未確認）**: 受信機を持って車で動かし、送信機は止めておく → 送信機の電源を切る→入れる。
  受信中は送信機の MaxGS（0 付近）と送信機の航跡、NO SIGNAL の間は MaxGS が隠れ受信機の航跡に切り替わり、互いに混ざらないか
- [ ] **受信機のヨーが 328 で止まる（11/26 修正・実機未確認・3 台焼き直し）**: 送信機を 320〜360° でゆっくり回し、受信機のヨーと自機アイコン・
  Re:PONS のライブが付いてくるか、received/ の yaw 列が 0〜360° か。LinkTelem のヨーを int16 → uint16 0〜35999 に（LINK_PROTO_VER 9）
- [ ] **受信機のミラー中の DIM MODE（11/26。実機未確認）**: 送信機・受信機とも電池で机に置き 30 秒 → 受信機が減光して `Plane or boat moves (auto)` の箱が出るか、
  log.txt に `enter auto-dim (... mirror sender_gs=)`。送信機を持って歩く（0.6 m/s 以上）・受信機を叩く・送信機の GNSS を抜く・地図を出したまま送信機の電源を切る（NO SIGNAL）で戻るか → [pons_power.md](pons_power.md) §4.3

## 5. 原因を探している不具合・見張り
- [ ] 照度センサーと GNSS が起動時に同時に黙った（13 回中 1 回）。次に起きたら直前の操作と +3V3

## 6. 部品・手作業
- [ ] 22µF の基板で SW OFF＋USB の充電 LED が消えるか、Q4 の逆接保護、VBUS の分圧、リコール基板の後付け
- [ ] 取扱説明書（Google Docs）の更新: SW OFF で充電しない、推定 TAS、DBD、充電ポップアップ、ユーザー LED（fix なしで 1 回点滅）、GNSS 詳細画面の SBAS（赤紫）
- [ ] 推定 TAS の V0・K・上下限の解析値を `settings.txt` へ

## 7. いつか（計画だけ）
- [ ] ESKF にジャイロの感度の誤差を持たせるか（何周も回る旋回の途中で GNSS を切ると、方位のずれが σ の 1.4〜2.9 倍）→ メモ §6
- [ ] GNSS の衛星数・vAcc を PC の再現（vario.py）に使えるようにする → メモ §6
- [ ] SCH16T の健全性（CRC エラー・STAT・飽和）を 60 秒ログへ、セルフテスト → [sch16t_design.md](sch16t_design.md) §8
- [ ] 描き直しを速くする（重ね描き `ovl` 28〜39ms が大きい）
- [ ] リプレイの x20 で方位変化の警告（track.wav）が鳴りすぎる

## 8. 保留（決めて止めているもの）
水晶を推奨品に（費用）・電源 ON 直後のバックライトの一瞬の点灯・2 台をつなぐ端子（次の改版で）・
GNSS の高度で気圧の基準を直す処理を V/S から切り離すか・BeiDou の B1I（使わない）・GNSS 基板の載せ替え（MAX-M10N / DAN-F10N）・SW OFF 直後 1 回だけ電池 0% の表示。理由はメモの §7。
