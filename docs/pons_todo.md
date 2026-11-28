# TODO（やること・後回しにしていること）

<!--
File    : docs/pons_todo.md
Project : PONS v7.1 / v7.2
Role    : 残っている作業の一覧。1 項目 1〜2 行。経緯・数字・確かめ方の詳細は pons_todo_notes.md。
Updated : 2026/11/28
-->

**済んだら行を消す。増えたら足す。** 1 項目は 1〜2 行。詳しいことは [pons_todo_notes.md](pons_todo_notes.md)（「メモ §n」）か各行のリンク先。
**v7（BNO085）の基板は廃棄する可能性があるので、v7 での確認と v7 へのファームの持ち込みはしない**（2026-11-18、ユーザー）。

並べ方: **§1 が飛ぶ前に必ず済ませるもの**。§3 は基板到着後の確認、§6〜§8 は部品・文書・計画・保留（§2・§4・§5 は確認完了により削除済み）。

---

## 1. 飛ぶ前に必ず（Step 5）
SCH16T は Step 5 が済むまで飛ばさない → [sch16t_design.md](sch16t_design.md) §6
- [ ] Step 5 飛行（試験飛行・大会）。**立ち会わず、飛んだあとに直せない**ので、直すものは飛ぶ前に決める。
  飛行のログで見ること: GNSS の上下速度（`vsi gnss` の `innov=`＝湖面の反射。食い違いの門は 11/22 の車で働いた `consist=` も）、加速度のガード（`VARIO: IMU accel ignored`）、離陸直後のピッチ

## 3. 机の上で確かめる
- [ ] v7.2 基板が届いたら: `PONS_MAINBOARD_REV` = 72 で焼き、満充電で数日〜数週間切ってから起動して `RTC BACKUP CHECK: ok` か、
  充電ポップアップの `Fast GPS <2d, clock 3mo`、回路が動き出す・止まる電圧（約 3.92V / 3.83V）→ [pons_charge_estimation.md](pons_charge_estimation.md)

## 6. 部品・手作業
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
