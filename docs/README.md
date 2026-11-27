# docs 案内

docs は、利用・製作手順、機能仕様、実装計画と調査メモに分けてあります。
日々の作業項目は [pons_todo.md](pons_todo.md)、その補足は [pons_todo_notes.md](pons_todo_notes.md) を参照してください。

## 導入・製作・運用

| 文書 | 内容 |
|---|---|
| [hardware.md](hardware.md) | 基板の構成、部品調達、組み立て、導通確認 |
| [pons_power.md](pons_power.md) | 電源回路の安全対策とファームウェアの電源保護 |
| [pons_charge_estimation.md](pons_charge_estimation.md) | 充電中の残量・満充電までの時間の推定 |
| [pons_link_bringup.md](pons_link_bringup.md) | 無線機の立ち上げと機体間試験の手順 |

## 機能仕様

| 文書 | 内容 |
|---|---|
| [pons_navigation.md](pons_navigation.md) | 航法、目的地、Auto 10km、コース警報、航跡 |
| [pons_tas.md](pons_tas.md) | 推定 TAS の式、設定、表示、無線伝送 |
| [pons_sound.md](pons_sound.md) | 音声・操作音・バリオ音の優先度と再生仕様 |
| [pons_link.md](pons_link.md) | PONS Link の無線・プロトコル・受信機表示・実測 |
| [pons_webserial_protocol.md](pons_webserial_protocol.md) | Re:PONS と本体の USB シリアル通信 |

## 設計・実装・調査

| 文書 | 内容 |
|---|---|
| [sch16t_design.md](sch16t_design.md) | SCH16T の設定、接続、ドライバ、試験記録 |
| [pons_yaw_init_plan.md](pons_yaw_init_plan.md) | ESKF の初期ヨー推定の方式と評価 |
| [pons_tas_gamma_plan.md](pons_tas_gamma_plan.md) | 推定 TAS の経路角 γ 補正の実装記録と、大会ログでの根拠・残る実機確認 |
| [pons_ttff_assist_plan.md](pons_ttff_assist_plan.md) | GNSS の時刻・航法データベースを助け合う設計と試験 |
| [pons_live_stream_plan.md](pons_live_stream_plan.md) | Re:PONS ライブ配信の決定事項、実装構成、運用メモ |

## 作業管理

| 文書 | 内容 |
|---|---|
| [pons_todo.md](pons_todo.md) | 未完了の作業・実機確認 |
| [pons_todo_notes.md](pons_todo_notes.md) | TODO の背景、ログ値、詳細な確認手順 |

完了した作業は TODO から取り除き、再利用する仕様や手順は各機能文書に残します。
