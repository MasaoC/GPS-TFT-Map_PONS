# TODO（やること・後回しにしていること）

<!--
File    : docs/pons_todo.md
Project : PONS v7 / v7.1
Role    : 残っている作業の一覧（v7.1 と v7 の両方）。1 項目 1〜2 行。経緯・数字・確かめ方の詳細は pons_todo_notes.md。
Updated : 2026/11/13
-->

**済んだら行を消す。増えたら足す。** 1 項目は 1〜2 行にとどめ、詳しいことは
[pons_todo_notes.md](pons_todo_notes.md)（同じ話題の段落）か、各行のリンク先に書く。
v7.1 で確かめたものは、同じ直しが v7 にも入っていれば v7 の基板でも確かめる。

## 1. 次にやる: SCH16T の Step 4 走行（v7.1）
→ [imu_sch16t_bringup.md](imu_sch16t_bringup.md)。Step 5 が済むまで飛ばさない（`#warning "... DO NOT FLY."`）。
- [ ] 車載で走る（**本体をしっかり固定**。旋回・加減速・止まる・GNSS 断を含める）。走ったら log.txt と `imuraw/` の .bin を解析へ
- [ ] 生ログを 250Hz にした（2026-11-13、`SCH16T_IMULOG_DECIM` 1・バッファ 584）: log.txt の `raw ... drop=` が 0 のままか。増えたら 2 に
- [ ] 同じ走行で見る: `ESKF RELEVEL` が出ない（`big=0`）、`VARIO: IMU accel ignored` が出ない、止まっているときバリオが鳴りすぎない、
  `ESKF IMU GAP` が出ず `stale=` が 0 のまま（BUG-60・62 の直し。2026-11-13）
- [ ] 生ログの読み出しの間隔が 4ms でそろっているか（`decode_imulog.py`）
- [ ] BUG-16: SCH16T を抜いたとき 50ms で見切るか（`SCH16T_READY_NOFRAME_MS`）
- [ ] Step 5 飛行

## 2. 実機で確かめること（コードは入っている）
- [ ] 推定 TAS: 見た目・案内の画面と音声・リプレイ・無線 3 台で出るか → [pons_tas.md](pons_tas.md) §8・§9
- [ ] DBD と PC の時刻: `COLD` / `COLDONLY` の比べ、BT1 が切れた起動の TTFF、`given time was ... off` → [pons_ttff_assist_plan.md](pons_ttff_assist_plan.md) §3.5・§3.6
- [ ] LoRa の時刻の助け合い（2026-11-13 実装）: ボートへ・機体へ時刻が渡るか、1Hz の送信が崩れないか、fix の前の RTC の時刻の精度
  （`LINK ASSIST: own time tAcc`）→ 同 §4.7。**`PONS:DBD,COLD` / `COLDONLY` / `HOT` の試験中は外からの時刻を入れない**ので、BT1 が切れた起動で確かめる
- [ ] コールドの起動だけ ANA を切る（2026-11-13）: `GNSS ANA off ...: ack` のあと QZSS が使われるか → 同 §2.5
- [ ] 電源: 細いケーブルの VBUS、USB を抜いたときの %（明るいとき）、充電の残り時間、BT1 の推定、BT1 の記録が `loaded` になるか
  → [pons_power.md](pons_power.md) §6・[pons_charge_estimation.md](pons_charge_estimation.md)
- [ ] 無線: 屋外で到達距離（いま約 130m）と雑音（画面オン・オフで比べる）→ 送信前チェックのしきい値 −95 dBm を見直す
- [ ] 無線: 受信機の設定画面の縮尺 `(S: ..)`、BUG-59 の直しで RX に切り替えて受信できるか（相手の機体が要る）

## 3. 原因を探している不具合（v7.1）
- [ ] E220 を付けてから、USB で起動すると起動中に固まる（5 回中 3 回）。U10 を疑う。E220 あり・なしで 10 回ずつ、VSYS と +3V3 をオシロで
- [ ] 輝度 SW が Low だと真っ暗（約 100 回に 1 回）。次に起きたら `BL DIAG` の `pin_low=` と Q1 のゲート電圧
- [ ] 照度センサーと GNSS が起動時に同時に黙った（13 回中 1 回）。次に起きたら直前の操作と +3V3
- [ ] 見張り: log.txt に `BOOT WATCH: restarted by WATCHDOG`（水晶の件）が出ないか

## 4. 部品・基板・手作業
- [ ] U10 を NCP360 に載せ替える（部品待ち）→ [pons_power.md](pons_power.md) §3・§6
- [ ] 22µF の基板で SW OFF＋USB の充電 LED が消えるか、Q4 の逆接保護、VBUS の分圧、リコール基板の後付け
- [ ] 取扱説明書（Google Docs）の更新: SW OFF で充電しない、推定 TAS、DBD、充電ポップアップ
- [ ] TTFF の段階 0 の測定 → [pons_ttff_assist_plan.md](pons_ttff_assist_plan.md) §7
- [ ] 推定 TAS の案内の音声 2 本の差し替え（いまは仮の音声）、V0・K・上下限の解析値を `settings.txt` へ

## 5. いつか（計画だけ）
- [ ] SCH16T の健全性（CRC エラー・STAT・飽和）を 60 秒ログへ、セルフテスト → [imu_sch16t_plan.md](imu_sch16t_plan.md) §2.7
- [ ] 描き直しを速くする（重ね描き `ovl` 28〜39ms が大きい）
- [ ] リプレイの x20 で方位変化の警告（track.wav）が鳴りすぎる

## 6. 保留（決めて止めているもの）
水晶を推奨品に（費用）・電源 ON 直後のバックライトの一瞬の点灯・2 台をつなぐ端子（次の改版で）・
GNSS の高度で気圧の基準を直す処理を V/S から切り離すか。理由はメモの §7。
