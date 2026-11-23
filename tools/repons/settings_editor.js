// ============================================================
// File    : tools/repons/settings_editor.js
// Project : PONS v7.1 / v7.2 Re:PONS
// Role    : 本体設定エディタ（settings.txt の確認・変更・差分の確認・TAS 曲線のプレビュー・パイロンの座標）
// Author  : MasaoC (@masao_mobile)
// Updated : 2026-11-23
// ============================================================
//
// 2026-11-23 に index.html から分けた。読む順は index.html の一覧どおり。関数・変数はファイルをまたいで
// グローバルで見えるが、読み込みの最中に動くコードは先に読んだファイルのものしか使えない。

// =========================================================================
// 本体設定エディタ（settings.txt の確認・安全な変更・Diff 確認・TAS 曲線プレビュー）
// =========================================================================
const PONS_SETTING_CATS = [
  { id: 'all',      label: 'すべて' },
  { id: 'basic',    label: '基本・表示' },
  { id: 'nav',      label: 'ナビ・地図' },
  { id: 'link',     label: '無線 (PONS Link)' },
  { id: 'tas',      label: '推定 TAS・風' },
  { id: 'imu',      label: '姿勢・バリオ KF' },
  { id: 'internal', label: '内部状態・較正' },
  { id: 'course',   label: 'コース（パイロン）' },
];

const PONS_TIER_META = {
  normal:   { label: '通常',     cls: 'sbadge-normal',   locked: false },
  caution:  { label: '注意',     cls: 'sbadge-caution',  locked: false },
  expert:   { label: '専門',     cls: 'sbadge-expert',   locked: true  },
  internal: { label: '内部状態', cls: 'sbadge-internal', locked: true  },
};

// mysd.cpp の settings[] 全 32 項目と完全一致する定義（パイロン座標の pilon.* は下の PONS_PILON_SCHEMA で足す）
const PONS_SETTINGS_SCHEMA = [
  // --- 基本・表示 ---
  {
    key: 'volume', label: '音声・効果音の音量', cat: 'basic', tier: 'normal',
    type: 'number', min: 0, max: 100, step: 1, unit: '%', defaultVal: '100',
    desc: '音声ガイドや警告音の全体音量（0 = 消音、100 = 最大）。',
    caution: '0 にするとコース逸脱や地上チェックなどの重要な音声警報も聞こえなくなります。'
  },
  {
    key: 'vario_volume', label: 'バリオメーター音量', cat: 'basic', tier: 'normal',
    type: 'number', min: 0, max: 100, step: 1, unit: '%', defaultVal: '50',
    desc: '上昇・下降音（バリオ音）の音量（0〜100）。',
    caution: '大きすぎると他の音声案内が聞き取りにくくなることがあります。'
  },
  {
    key: 'vario_inhibit', label: 'バリオ機能の完全抑止', cat: 'basic', tier: 'caution',
    type: 'select', defaultVal: 'false',
    options: [{ v: 'false', l: 'false（通常：有効）' }, { v: 'true', l: 'true（完全無効化）' }],
    desc: 'true にするとバリオメーターの動作と音声を完全に無効化します。',
    caution: '通常は false にしてください。true のままだと本体設定からバリオ音量を上げても鳴りません。'
  },
  {
    key: 'auto_brightness', label: 'AUTO 輝度カーブ', cat: 'basic', tier: 'normal',
    type: 'select', defaultVal: 'ECO',
    options: [{ v: 'ECO', l: 'ECO（省電力・既定）' }, { v: 'BRIGHT', l: 'BRIGHT（明るめ）' }],
    desc: '輝度スイッチ Low 時の自動調光カーブ。',
    caution: 'BRIGHT は屋外で見やすくなりますが、バッテリー消費が増えます。'
  },
  // --- ナビ・地図 ---
  {
    key: 'destination', label: '目的地', cat: 'nav', tier: 'normal',
    type: 'dest', defaultVal: 'PLATHOME',
    desc: 'ナビゲーションの対象となる目的地（SD の destination.csv に登録されている名前）。',
    caution: 'destination.csv に存在しない名前は本体側で無視されます。'
  },
  {
    key: 'navigation_mode', label: 'ナビゲーションモード', cat: 'nav', tier: 'normal',
    type: 'select', defaultVal: 'AUTO10K',
    options: [
      { v: 'INTO',    l: 'INTO（目的地へ向かう）' },
      { v: 'AWAY',    l: 'AWAY（出発点から離れる）' },
      { v: 'AUTO10K', l: 'AUTO10K（10km 折返し自動切替）' },
    ],
    desc: '航法計算の方向モード。鳥人間コンテストの折返し飛行は AUTO10K を使用します。',
    caution: 'INTO / AWAY を取り違えると誘導方位が 180° 逆になります。'
  },
  {
    key: 'scaleindex', label: '地図縮尺インデックス', cat: 'nav', tier: 'normal',
    type: 'select', defaultVal: '4',
    options: [
      { v: '0', l: '0（最も広域：横幅 約 1174 km）' },
      { v: '1', l: '1（広域：横幅 約 294 km）' },
      { v: '2', l: '2（中域：横幅 約 73 km）' },
      { v: '3', l: '3（詳細：横幅 約 18.3 km）' },
      { v: '4', l: '4（より詳細：横幅 約 4.6 km・既定）' },
      { v: '5', l: '5（最大拡大：横幅 1.2 km）' },
    ],
    desc: '起動時の地図ズーム段階（0〜5、大きい番号ほど拡大）。飛行中も本体ボタンで切替可能です。',
    caution: ''
  },
  {
    key: 'upward_mode', label: '地図の向き', cat: 'nav', tier: 'normal',
    type: 'select', defaultVal: '0',
    options: [{ v: '0', l: '0：TRACK UP（進行方向が上）' }, { v: '1', l: '1：NORTH UP（北が上）' }],
    desc: '地図画面の上方向を進行方向にするか、真北固定にするかを選びます。',
    caution: ''
  },
  // --- 無線 (PONS Link) ---
  {
    key: 'link_mode', label: 'PONS Link 動作モード', cat: 'link', tier: 'caution', needsReboot: true,
    type: 'select', defaultVal: '0',
    options: [
      { v: '0', l: '0：OFF（無線無効）' },
      { v: '1', l: '1：TX（送信・機体側）' },
      { v: '2', l: '2：RX（受信・追尾ボート側）' },
    ],
    desc: '機体⇄追尾ボート間の LoRa 無線の役割。',
    caution: '【要注意】機体側を 2（受信）にすると自機画面がミラー待機になってしまいます。変更後は本体の再起動が必要です。'
  },
  {
    key: 'link_radio_ch', label: '無線チャネル (CH)', cat: 'link', tier: 'caution', needsReboot: true,
    type: 'select', defaultVal: '6',
    options: [
      { v: '0',  l: 'CH 0' },  { v: '3',  l: 'CH 3' },  { v: '6',  l: 'CH 6（既定）' },
      { v: '9',  l: 'CH 9' },  { v: '12', l: 'CH 12' }, { v: '15', l: 'CH 15（アッパー）' },
      { v: '21', l: 'CH 21（アッパー）' }, { v: '27', l: 'CH 27（アッパー）' },
    ],
    desc: 'E220 LoRa モジュールの通信チャネル。',
    caution: '機体と追尾ボートで必ず同じチャネルに揃えてください。ズレると一切通信できません。'
  },
  {
    key: 'link_radio_prof', label: '無線プロファイル (SF)', cat: 'link', tier: 'caution', needsReboot: true,
    type: 'select', defaultVal: '1',
    options: [
      { v: '0', l: '0：SF7（高速・短距離）' },
      { v: '1', l: '1：SF8（バランス・既定）' },
      { v: '2', l: '2：SF9（中距離）' },
      { v: '3', l: '3：SF10（長距離）' },
      { v: '4', l: '4：SF11（最長距離・低速）' },
    ],
    desc: 'LoRa の拡散率（Spreading Factor）プリセット。大きいほど遠くまで届きますが送信時間が伸びます。',
    caution: '機体と追尾ボートで必ず一致させてください。'
  },
  {
    key: 'link_group', label: '無線グループ (A〜D)', cat: 'link', tier: 'caution', needsReboot: true,
    type: 'select', defaultVal: 'A',
    options: [
      { v: 'A', l: 'A（既定値・工場初期）' },
      { v: 'B', l: 'B' },
      { v: 'C', l: 'C' },
      { v: 'D', l: 'D' },
    ],
    desc: '同じチャネルを使う他機との混信を防ぐグループ識別子。',
    caution: 'A は SD 異常時の既定値なので、普段の運用では B〜D を機体とボートで揃えて使うのが推奨です。'
  },
  {
    key: 'link_assist', label: 'LoRa 時刻アシスト', cat: 'link', tier: 'caution', needsReboot: true,
    type: 'select', defaultVal: 'ON',
    options: [{ v: 'ON', l: 'ON（有効・既定）' }, { v: 'OFF', l: 'OFF（無効）' }],
    desc: 'BT1 電池切れ時などに、LoRa 経由で仲間機・ボートと GNSS 時刻を助け合う機能。',
    caution: '通常は ON にしてください。'
  },
  // --- 推定 TAS・風 ---
  {
    key: 'header_tas', label: 'ヘッダー右上の大枠表示', cat: 'tas', tier: 'caution',
    type: 'select', defaultVal: '0',
    options: [{ v: '0', l: '0：Pavg（平均ピッチ）' }, { v: '1', l: '1：TAS（推定対気速度）' }],
    desc: '画面右上の大きい数値枠に、過去 10 秒平均ピッチを出すか、推定対気速度（青箱）を出すかを選びます。',
    caution: 'TAS は平均ピッチからの推定値です。取付角が 1° ずれると約 0.35〜0.4 m/s ずれます。'
  },
  {
    key: 'wind_estimate', label: '風向・風速推定', cat: 'tas', tier: 'caution',
    type: 'select', defaultVal: '1',
    options: [{ v: '1', l: '1：ON（有効）' }, { v: '0', l: '0：OFF（無効）' }],
    desc: '対地速度ベクトルと推定対気速度ベクトル（TAS）の差から風を推定し、地図に矢印表示します。',
    caution: '下の V0 / K の値が機体と合っていないと、風の推定値もずれます。'
  },
  {
    key: 'airspeed_v0', label: '対気速度モデル V0（ピッチ 0° の巡航速度）', cat: 'tas', tier: 'expert',
    type: 'number', min: 3.0, max: 20.0, step: 0.01, decimals: 2, unit: 'm/s', defaultVal: '7.07',
    desc: '平らな地面に置いた較正姿勢（ピッチ θ = 0°）で定常飛行したときの対気速度 V0。推定 TAS と風推定の両方に使います。',
    caution: '機体・重量・重心・取付角で変わります。1% 変わると全速度域の推定 TAS が 1% ずれます。上のグラフで曲線を確認してください。'
  },
  {
    key: 'airspeed_k', label: '対気速度モデル K（揚力ゼロまでの角度）', cat: 'tas', tier: 'expert',
    type: 'number', min: 3.0, max: 60.0, step: 0.1, decimals: 2, unit: '度', defaultVal: '10.00',
    desc: '揚力係数ではなく「角度」です（K = 取付角 i − 零揚力角 α0L）。θ = 0° から機首を K° 下げると揚力がゼロになる角度。ピッチ 1° あたりの速度変化（dV/dθ ≈ -V0/(2K)）を決めます。',
    caution: '小さくするほどピッチ変化に対する速度変化が急になり、大きすぎると速度変化が鈍くなります。根拠なく大きく変えないでください。'
  },
  {
    key: 'airspeed_min', label: '推定 TAS 下限クランプ', cat: 'tas', tier: 'expert',
    type: 'number', min: 1.0, max: 20.0, step: 0.1, decimals: 2, unit: 'm/s', defaultVal: '5.80',
    desc: '推定対気速度の下限（失速速度付近）。これ未満に張り付くと TAS 箱が灰色になります。',
    caution: '必ず airspeed_min < airspeed_max にしてください（逆転すると本体が既定値 5.80〜9.00 にリセットします）。'
  },
  {
    key: 'airspeed_max', label: '推定 TAS 上限クランプ', cat: 'tas', tier: 'expert',
    type: 'number', min: 1.0, max: 25.0, step: 0.1, decimals: 2, unit: 'm/s', defaultVal: '9.00',
    desc: '推定対気速度の上限（出し得る最大速度）。これ以上に張り付くと TAS 箱が灰色になります。',
    caution: '必ず airspeed_min < airspeed_max にしてください。'
  },
  // --- 姿勢・バリオ KF ---
  {
    key: 'rpy_functions', label: 'Roll / Pitch / Yaw 機能（マスター）', cat: 'imu', tier: 'caution',
    type: 'select', defaultVal: '1',
    options: [{ v: '1', l: '1：ON（有効）' }, { v: '0', l: '0：OFF（無効）' }],
    desc: 'ESKF による姿勢・風・推定 TAS 機能のマスタースイッチ。',
    caution: 'OFF にすると姿勢表示・推定 TAS・風推定・自動ロールトリムがすべて停止します。'
  },
  {
    key: 'auto_roll_trim', label: '直進中の自動ロールトリム', cat: 'imu', tier: 'caution',
    type: 'select', defaultVal: '1',
    options: [{ v: '1', l: '1：ON（有効）' }, { v: '0', l: '0：OFF（無効）' }],
    desc: '定常直進中の微小なロール取り付けズレを自動で学習・補正する機能。',
    caution: 'OFF にすると既存のロールトリム補正量もクリアされます。'
  },
  {
    key: 'pitch_target', label: '較正時の基準ピッチ角 (SET PITCH)', cat: 'imu', tier: 'expert',
    type: 'number', min: -3.5, max: 2.0, step: 0.5, decimals: 1, unit: '度', defaultVal: '0.0',
    desc: '本体画面で APPLY（水平較正）を実行するときに申告する地面・プラットフォームの傾斜角（例: 本番台は -3.5°）。',
    caution: '値を変えただけでは現在の較正値は変わりません（本体で APPLY したときに使われます）。'
  },
  {
    key: 'roll_target', label: '較正時の基準ロール角 (SET ROLL)', cat: 'imu', tier: 'expert',
    type: 'number', min: -3.0, max: 3.0, step: 0.5, decimals: 1, unit: '度', defaultVal: '0.0',
    desc: '本体画面で APPLY を実行するときに申告するロール角（通常は 0.0°）。',
    caution: '通常は 0.0 のままにしてください。'
  },
  {
    key: 'kf_q_vel', label: 'バリオ KF 速度プロセスノイズ (Q_vel)', cat: 'imu', tier: 'expert',
    type: 'number', min: 0.00001, max: 100, step: 0.001, unit: '', defaultVal: '0.02',
    desc: '鉛直速度カルマンフィルタの速度プロセスノイズ（既定 0.02）。',
    caution: '大きすぎると静止時にバリオ（VSI）がふらつき、小さすぎると昇降への追従が遅れます。'
  },
  {
    key: 'kf_q_bias', label: 'バリオ KF バイアスプロセスノイズ (Q_bias)', cat: 'imu', tier: 'expert',
    type: 'number', min: 0.00000001, max: 1, step: 0.0001, unit: '', defaultVal: '0.0005',
    desc: '鉛直加速度バイアス推定のプロセスノイズ（既定 0.0005）。',
    caution: '通常は既定値（0.0005）から変更しないでください。'
  },
  {
    key: 'kf_R', label: 'バリオ KF 気圧高度観測ノイズ (R)', cat: 'imu', tier: 'expert',
    type: 'number', min: 0.001, max: 10000, step: 0.5, unit: 'm²', defaultVal: '20',
    desc: '気圧高度計の観測ノイズ分散（既定 20）。大きいほど気圧ノイズを抑え、IMU 加速度を重視します。',
    caution: '小さすぎると風圧等の気圧揺らぎが VSI に直接乗ります。'
  },
  // --- 内部状態・較正 ---
  {
    key: 'level_roll', label: '水平較正オフセット（ロール）', cat: 'internal', tier: 'internal',
    type: 'number', min: -30, max: 30, step: 0.001, decimals: 3, unit: '度', defaultVal: '0.000',
    desc: '本体の APPLY 操作で自動記録されたマウント基準のロール補正角。',
    caution: '【原則編集禁止】手入力で書き換えると姿勢表示と旋回判定が狂います。実機の APPLY 操作で更新してください。'
  },
  {
    key: 'level_pitch', label: '水平較正オフセット（ピッチ）', cat: 'internal', tier: 'internal',
    type: 'number', min: -30, max: 30, step: 0.001, decimals: 3, unit: '度', defaultVal: '0.000',
    desc: '本体の APPLY 操作で自動記録されたマウント基準のピッチ補正角。',
    caution: '【原則編集禁止】1° 狂うと推定 TAS が約 0.35〜0.4 m/s ずれます。実機の APPLY 操作で更新してください。'
  },
  {
    key: 'roll_trim', label: '自動ロールトリム累積値', cat: 'internal', tier: 'internal',
    type: 'number', min: -5, max: 5, step: 0.001, decimals: 3, unit: '度', defaultVal: '0.000',
    desc: '直進中に自動学習されたロール微補正量（±5° 上限）。',
    caution: '【原則編集禁止】マウントから外したときは自動でリセットされます。'
  },
  {
    key: 'needs_apply', label: '要較正フラグ (needs_apply)', cat: 'internal', tier: 'internal',
    type: 'select', defaultVal: '0',
    options: [{ v: '0', l: '0：較正済み' }, { v: '1', l: '1：較正のやり直しが必要（赤線警告）' }],
    desc: 'マウントから外されたことを検知して立つフラグ。1 の間は画面に CALIBRATE 警告が出ます。',
    caution: '【危険】較正せずに手動で 0 に戻すと、取り付けがずれたまま警告なしで飛行することになります。'
  },
  {
    key: 'calib_date', label: '最終較正日 (YYYYMMDD)', cat: 'internal', tier: 'internal',
    type: 'text', defaultVal: '0',
    desc: '最後に APPLY を実行した JST 日付（0 = 不明）。日をまたいだ取り外し検知に使います。',
    caution: '【原則編集禁止】通常は本体が自動記録します。'
  },
  {
    key: 'auto10k_status', label: 'AUTO10K 折返しフェーズ', cat: 'internal', tier: 'internal',
    type: 'select', defaultVal: 'AWAY',
    options: [{ v: 'AWAY', l: 'AWAY（往路：離陸後〜折返し前）' }, { v: 'INTO', l: 'INTO（復路：折返し後）' }],
    desc: 'AUTO10K モードでの現在の往路／復路状態（飛行中再起動時の復帰用）。',
    caution: '発進前に INTO になっていると最初から逆方向へ誘導されます。通常は AWAY にしてください。'
  },
];

// ---- コースの座標（パイロン）。2026-11-23〜 ----
// settings.txt ではなく SD の override_pilon_coordinate.csv（地点ごとの上書き）を読み書きする（mysd.cpp の pilon.* 項目）。
// 値は「次の起動で使う座標」"緯度,経度"。既定（navdata.h。公式ルール 2026.06.21）と同じにすると上書きを消す。
// ★ 反映は本体の次の起動から（いまの座標は Core0 が読み続けていて、飛行中に変えると無線の目的地の照合もずれる）。
// defaultVal は本体の PONS:PILON_INFO で置き換える（ファームの navdata.h が正）。
const PONS_PILON_SCHEMA = [
  { name: 'PLATHOME',  label: 'プラットホーム',  def: '35.294222,136.254333' },
  { name: 'N_PILON',   label: '北パイロン',       def: '35.377878,136.189907' },
  { name: 'S_PILON',   label: '南パイロン（旧称 西）', def: '35.274220,136.136183' },
  { name: 'PILON_1KM', label: '1km パイロン',     def: '35.297961,136.243624' },
  { name: 'TAKESHIMA', label: '多景島',          def: '35.296584,136.178054' },
];
for (const p of PONS_PILON_SCHEMA) {
  PONS_SETTINGS_SCHEMA.push({
    key: `pilon.${p.name}`, label: `${p.label}の座標`, cat: 'course', tier: 'caution',
    type: 'latlon', defaultVal: p.def,
    desc: `「緯度,経度」（10 進の度、真北基準の WGS84）。毎年の大会ルールで変わったら直す。SD の override_pilon_coordinate.csv の ${p.name} の行に書かれ、既定と同じにすると上書きを消す。`,
    caution: '保存しても本体の次の起動まで使われません（保存したら再起動）。機体・予備機・ボートの 3 台で同じにしないと、無線で目的地が合いません。',
  });
}

const PONS_SCHEMA_MAP = Object.fromEntries(PONS_SETTINGS_SCHEMA.map(s => [s.key, s]));

// "緯度,経度" を数 2 つにする（形が違えば null）
function parseLatLon(raw) {
  const m = String(raw ?? '').trim().match(/^(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)$/);
  if (!m) return null;
  const lat = Number(m[1]), lon = Number(m[2]);
  return (Number.isFinite(lat) && Number.isFinite(lon)) ? { lat, lon } : null;
}
// 2 点間の距離 [m]
function latLonDistM(a, b) {
  const R = 6371000, r = Math.PI / 180;
  const dLat = (b.lat - a.lat) * r, dLon = (b.lon - a.lon) * r;
  const h = Math.sin(dLat / 2) ** 2 + Math.cos(a.lat * r) * Math.cos(b.lat * r) * Math.sin(dLon / 2) ** 2;
  return 2 * R * Math.asin(Math.sqrt(h));
}
function fmtDistM(m) { return m >= 1000 ? `${(m / 1000).toFixed(2)} km` : `${Math.round(m)} m`; }

// ---- パイロン座標の打ち間違いの見張り（2026-11-23〜、ユーザー）----
// 保存の前に確認のダイアログ（confirm）を出す。止めはしない（ルールで本当に変わることがあるため）。
// 琵琶湖の湖面をおおまかに囲む箱（北端 塩津・南端 瀬田・西端 大津・東端 長浜）。箱の外へ BIWAKO_FAR_M を超えて出ていたら
// 「琵琶湖から明らかに遠い」（度の桁・符号・緯度と経度の入れ違いなど）。湖岸の細かい形は見ない。
const BIWAKO_BOX = { latMin: 34.97, latMax: 35.53, lonMin: 135.85, lonMax: 136.29 };
const BIWAKO_FAR_M = 5000;
// 北・南パイロンとプラットホームの距離がこの範囲の外なら確かめる（既定は北 約 10.99 km・南 約 10.95 km。ふつうは約 11 km）
const PILON_PLAT_MIN_M = 10000, PILON_PLAT_MAX_M = 12000;
const PILON_PLAT_CHECK = ['N_PILON', 'S_PILON'];
// 琵琶湖の箱から何 m 外にいるか（箱の中なら 0）
function biwakoOutsideM(ll) {
  const b = BIWAKO_BOX;
  const c = { lat: Math.min(Math.max(ll.lat, b.latMin), b.latMax), lon: Math.min(Math.max(ll.lon, b.lonMin), b.lonMax) };
  return latLonDistM(c, ll);
}
function pilonShortLabel(key) {
  const p = PONS_PILON_SCHEMA.find(q => `pilon.${q.name}` === key);
  return p ? p.label : key;
}
// パイロン（N/S）とプラットホームの、いまの下書きでの距離 [m]。どちらかが読めなければ null
function pilonPlatDistM(name) {
  const plat = parseLatLon(settingsState.draft['pilon.PLATHOME']);
  const p = parseLatLon(settingsState.draft[`pilon.${name}`]);
  const dm = (plat && p) ? latLonDistM(plat, p) : NaN;
  return Number.isFinite(dm) ? dm : null;   // 範囲外の緯度（入力の途中）では NaN になりうる
}
function pilonPlatDistBad(dm) { return dm !== null && (dm < PILON_PLAT_MIN_M || dm >= PILON_PLAT_MAX_M); }
// 保存の前に確かめてほしいこと（文の配列）。変えたパイロンの項目だけを見る。
// 距離は、プラットホームか、その北・南パイロンを変えたときだけ見る（変えていない組は前から使っている値なので言わない）。
function pilonSafetyIssues() {
  const diffs = computeSettingsDiffs().filter(d => d.schema.type === 'latlon');
  const issues = [];
  if (diffs.length === 0) return issues;
  for (const d of diffs) {
    const ll = parseLatLon(d.newVal);
    if (!ll) continue;
    const out = biwakoOutsideM(ll);
    if (out > BIWAKO_FAR_M) issues.push(`${pilonShortLabel(d.schema.key)}（${d.newVal}）が琵琶湖から約 ${fmtDistM(out)} 離れています。`);
  }
  const changed = new Set(diffs.map(d => d.schema.key));
  for (const name of PILON_PLAT_CHECK) {
    const key = `pilon.${name}`;
    if (!changed.has(key) && !changed.has('pilon.PLATHOME')) continue;
    const dm = pilonPlatDistM(name);
    if (pilonPlatDistBad(dm))
      issues.push(`${pilonShortLabel(key)}とプラットホームの距離が ${fmtDistM(dm)} です（${PILON_PLAT_MIN_M / 1000}〜${PILON_PLAT_MAX_M / 1000} km の外。既定は約 11 km）。`);
  }
  return issues;
}

// 設定エディタの状態
const settingsState = {
  loaded: false,
  loading: false,
  unsupported: false,
  saving: false,
  activeCat: 'all',
  expertUnlocked: false,
  // 本体から取得した元値 { key: string }
  orig: {},
  // SD の settings.txt に記載されていたか { key: boolean }
  inFile: {},
  // 編集中（下書き） { key: string }
  draft: {},
  // 未記載項目を既定値のまま settings.txt へ追加する明示フラグ { key: boolean }
  addToFile: {},
  // 直前の保存前のスナップショット（Undo 用）
  lastSavedBackup: null,
  // 目的地候補一覧（PONS:DEST から取得）
  destinations: [],
  // パイロン座標のいま使っている値 { 'pilon.N_PILON': '緯度,経度' }（PONS:PILON_INFO から）
  pilonRuntime: {},
  rxPilonRuntime: {},
  // 本体へ SET を送ったが、まだ SAVE で書き切っていないパイロンの項目（本体の s_pilon_stage に残っている）
  pilonStaged: new Set(),
  // 受信一時バッファ
  rxVals: {},
  rxInFile: {},
  rxDests: [],
  getTimer: null,
  // 保存時の逐次コマンド待ち用 Promise resolver
  pendingReplyResolve: null,
  pendingReplyReject: null,
};

function abortSettingsPending(reason) {
  if (settingsState.getTimer) {
    clearTimeout(settingsState.getTimer);
    settingsState.getTimer = null;
  }
  if (settingsState.loading) {
    settingsState.loading = false;
    const badge = document.getElementById('settings-conn-badge');
    if (badge) {
      badge.textContent = '取得中断';
      badge.style.background = '#fee2e2';
      badge.style.color = '#991b1b';
    }
  }
  if (settingsState.pendingReplyReject) {
    const rej = settingsState.pendingReplyReject;
    settingsState.pendingReplyResolve = null;
    settingsState.pendingReplyReject = null;
    rej(new Error(reason || '通信が中断されました'));
  }
}

function resetSettingsState(closeModals) {
  abortSettingsPending('接続が切断されました');
  settingsState.loaded = false;
  settingsState.unsupported = false;
  settingsState.saving = false;
  settingsState.orig = {};
  settingsState.inFile = {};
  settingsState.draft = {};
  settingsState.addToFile = {};
  settingsState.lastSavedBackup = null;
  settingsState.destinations = [];
  if (closeModals) {
    const dm = document.getElementById('settings-diff-modal');
    const sm = document.getElementById('settings-modal');
    if (dm) dm.classList.remove('active');
    if (sm) sm.classList.remove('active');
  }
}

function openSettingsModal() {
  if (!serialWriter || !ponsReady) {
    alert('先に PONS 本体へ USB 接続してください。');
    return;
  }
  document.getElementById('settings-modal').classList.add('active');
  // 未保存の変更が無ければ、常に本体から最新値を取得する（機体差し替えや本体側操作の取りこぼし防止）
  if (!settingsState.loaded || settingsState.unsupported || computeSettingsDiffs().length === 0) {
    requestPonsSettings(true);
  } else {
    renderSettingsAll();
  }
}

function closeSettingsModal() {
  if (settingsState.saving) return;
  const diffs = computeSettingsDiffs();
  if (diffs.length > 0 && !confirm(`未保存の変更が ${diffs.length} 件あります。画面を閉じますか？（下書きは残ります）`)) {
    return;
  }
  document.getElementById('settings-modal').classList.remove('active');
}

async function requestPonsSettings(skipConfirm = false) {
  if (!serialWriter || !ponsReady) {
    alert('PONS 本体が未接続です。');
    return;
  }
  if (downloadBusy() || listPending || settingsState.saving) {
    alert('現在ほかの通信を実行中です。終わってから再試行してください。');
    return;
  }
  if (!skipConfirm && settingsState.loaded && computeSettingsDiffs().length > 0) {
    if (!confirm('未保存の変更が破棄されます。本体から最新の設定を再取得しますか？')) return;
  }
  settingsState.loading = true;
  settingsState.unsupported = false;
  settingsState.rxVals = {};
  settingsState.rxInFile = {};
  settingsState.rxDests = [];
  settingsState.rxPilonRuntime = {};
  if (settingsState.getTimer) { clearTimeout(settingsState.getTimer); settingsState.getTimer = null; }

  const badge = document.getElementById('settings-conn-badge');
  badge.textContent = '本体から取得中...';
  badge.style.background = '#fef3c7';
  badge.style.color = '#b45309';
  document.getElementById('settings-body').innerHTML =
    '<div style="text-align:center; color:#64748b; padding:40px;">本体から現在の設定を読み込んでいます...</div>';

  await sendSerialLine('PONS:SETTINGS_GET');
  settingsState.getTimer = setTimeout(() => {
    settingsState.getTimer = null;
    if (!settingsState.loading) return;
    settingsState.loading = false;
    settingsState.unsupported = true;
    badge.textContent = '未対応ファーム';
    badge.style.background = '#fee2e2';
    badge.style.color = '#991b1b';
    document.getElementById('settings-body').innerHTML =
      '<div style="text-align:center; color:#b91c1c; padding:36px; line-height:1.7;">' +
      '<b>本体から設定一覧の応答がありませんでした。</b><br>' +
      'この機能（<code>PONS:SETTINGS_GET</code>）は 2026-11-20 以降のファームウェアで対応しています。<br>' +
      '最新ファームウェアを書き込んだあと、「🔄 再取得」を押してください。</div>';
  }, 4000);
}

function handleSettingsSerialLine(line) {
  if (line.startsWith('PONS:SETTING,')) {
    // PONS:SETTING,<key>,<value>,<in_file>
    // value にカンマが含まれる可能性に備えて先頭と末尾で分割
    const rest = line.substring('PONS:SETTING,'.length);
    const firstComma = rest.indexOf(',');
    const lastComma = rest.lastIndexOf(',');
    if (firstComma > 0 && lastComma > firstComma) {
      const key = rest.substring(0, firstComma);
      const val = rest.substring(firstComma + 1, lastComma);
      const inF = rest.substring(lastComma + 1).trim() === '1';
      settingsState.rxVals[key] = val;
      settingsState.rxInFile[key] = inF;
    }
  } else if (line.startsWith('PONS:DEST,')) {
    const name = line.substring('PONS:DEST,'.length).trim();
    if (name) settingsState.rxDests.push(name);
  } else if (line.startsWith('PONS:PILON_INFO,')) {
    // PONS:PILON_INFO,<NAME>,<既定の緯度>,<既定の経度>,<いまの緯度>,<いまの経度>
    const f = line.substring('PONS:PILON_INFO,'.length).split(',');
    if (f.length >= 5) {
      const key = `pilon.${f[0].trim()}`;
      const s = PONS_SCHEMA_MAP[key];
      if (s) s.defaultVal = `${Number(f[1]).toFixed(6)},${Number(f[2]).toFixed(6)}`;   // 本体の navdata.h が正
      settingsState.rxPilonRuntime[key] = `${Number(f[3]).toFixed(6)},${Number(f[4]).toFixed(6)}`;
    }
  } else if (line === 'PONS:SETTINGS_END') {
    if (settingsState.getTimer) { clearTimeout(settingsState.getTimer); settingsState.getTimer = null; }
    settingsState.loading = false;
    settingsState.loaded = true;
    settingsState.unsupported = false;
    settingsState.orig = { ...settingsState.rxVals };
    settingsState.inFile = { ...settingsState.rxInFile };
    settingsState.draft = { ...settingsState.rxVals };
    settingsState.addToFile = {};
    settingsState.destinations = [...settingsState.rxDests];
    settingsState.pilonRuntime = { ...settingsState.rxPilonRuntime };
    settingsState.pilonStaged = new Set();   // 本体も SETTINGS_GET で決めかけを捨てている
    const badge = document.getElementById('settings-conn-badge');
    // パイロン（pilon.*）は settings.txt の項目ではないので「未記載」に数えない（既定の座標で動いているだけ）
    const missingCnt = Object.entries(settingsState.inFile).filter(([k, v]) => !v && !k.startsWith('pilon.')).length;
    badge.textContent = `取得完了（全 ${Object.keys(settingsState.orig).length} 項目 / 未記載 ${missingCnt}）`;
    badge.style.background = '#dcfce7';
    badge.style.color = '#166534';
    renderSettingsAll();
  } else if (line.startsWith('PONS:SETTINGS_OK,') || line.startsWith('PONS:SETTINGS_SAVED,') || line.startsWith('PONS:SETTINGS_ERR,')) {
    if (settingsState.pendingReplyResolve) {
      const r = settingsState.pendingReplyResolve;
      settingsState.pendingReplyResolve = null;
      settingsState.pendingReplyReject = null;
      r(line);
    }
  }
}

function sendSettingCmdAndWait(cmd, timeoutMs = 5000) {
  return new Promise((resolve, reject) => {
    let timer = setTimeout(() => {
      if (settingsState.pendingReplyReject) {
        settingsState.pendingReplyResolve = null;
        settingsState.pendingReplyReject = null;
        reject(new Error(`タイムアウト: ${cmd}`));
      }
    }, timeoutMs);
    settingsState.pendingReplyResolve = (line) => { clearTimeout(timer); resolve(line); };
    settingsState.pendingReplyReject = (err) => { clearTimeout(timer); reject(err); };
    sendSerialLine(cmd).catch(err => {
      clearTimeout(timer);
      settingsState.pendingReplyResolve = null;
      settingsState.pendingReplyReject = null;
      reject(err);
    });
  });
}

function normalizeSettingVal(schema, raw) {
  const s = (raw === undefined || raw === null) ? '' : String(raw).trim();
  if (!schema) return s;
  if (schema.type === 'latlon') {
    const ll = parseLatLon(s);
    return ll ? `${ll.lat.toFixed(6)},${ll.lon.toFixed(6)}` : s;
  }
  if (schema.type === 'number') {
    const n = Number(s);
    if (!Number.isFinite(n)) return s;
    if (typeof schema.decimals === 'number') return n.toFixed(schema.decimals);
    return String(n);
  }
  return s;
}

function isSettingChanged(key) {
  if (!settingsState.inFile[key] && settingsState.addToFile[key]) return true;
  const schema = PONS_SCHEMA_MAP[key];
  const o = normalizeSettingVal(schema, settingsState.orig[key]);
  const d = normalizeSettingVal(schema, settingsState.draft[key]);
  return o !== d;
}

function computeSettingsDiffs() {
  const diffs = [];
  for (const s of PONS_SETTINGS_SCHEMA) {
    if (!(s.key in settingsState.orig)) continue;
    if (isSettingChanged(s.key)) {
      diffs.push({
        schema: s,
        oldVal: settingsState.orig[s.key],
        newVal: normalizeSettingVal(s, settingsState.draft[s.key]),
        wasMissing: !settingsState.inFile[s.key],
      });
    }
  }
  return diffs;
}

function validateAllDrafts() {
  const errors = {};
  const warnings = [];
  for (const s of PONS_SETTINGS_SCHEMA) {
    if (!(s.key in settingsState.draft)) continue;
    const raw = String(settingsState.draft[s.key] ?? '').trim();
    if (/[\r\n]/.test(raw)) {
      errors[s.key] = '改行文字は使用できません';
      continue;
    }
    if (s.type === 'latlon') {
      const ll = parseLatLon(raw);
      if (!ll) { errors[s.key] = '「緯度,経度」の形で入れてください（例: 35.377878,136.189907）'; continue; }
      if (ll.lat < -90 || ll.lat > 90 || ll.lon < -180 || ll.lon > 180) { errors[s.key] = '緯度は −90〜90、経度は −180〜180 です'; continue; }
      if (ll.lat === 0 && ll.lon === 0) { errors[s.key] = '0,0 は使えません'; continue; }
      if (isSettingChanged(s.key)) {
        const def = parseLatLon(s.defaultVal);
        const dm = def ? latLonDistM(def, ll) : 0;
        if (dm > 3000) warnings.push(`${s.label}が既定（navdata.h）から ${fmtDistM(dm)} 離れています。打ち間違い（緯度と経度の入れ違い・桁）でないか確かめてください。`);
      }
      continue;
    }
    if (s.key === 'calib_date' && !/^(0|\d{8})$/.test(raw)) {
      errors[s.key] = '0 または 8 桁の YYYYMMDD（例: 20261120）を入力してください';
      continue;
    }
    if (s.type === 'number') {
      const n = Number(raw);
      if (raw === '' || !Number.isFinite(n)) {
        errors[s.key] = '数値を入力してください';
      } else if (typeof s.min === 'number' && n < s.min) {
        errors[s.key] = `最小値 ${s.min} 未満です`;
      } else if (typeof s.max === 'number' && n > s.max) {
        errors[s.key] = `最大値 ${s.max} を超えています`;
      }
    }
  }
  // 整合性チェック: airspeed_min < airspeed_max
  const vmin = Number(settingsState.draft.airspeed_min);
  const vmax = Number(settingsState.draft.airspeed_max);
  if (Number.isFinite(vmin) && Number.isFinite(vmax) && !(vmin < vmax)) {
    errors.airspeed_min = 'airspeed_min は airspeed_max より小さくしてください';
    errors.airspeed_max = 'airspeed_max は airspeed_min より大きくしてください';
  }
  // 運用警告（保存はできるが注意喚起）
  if (isSettingChanged('link_group') && settingsState.draft.link_group === 'A') {
    warnings.push('無線グループ A は SD 異常時の既定値です。普段の運用では B〜D の使用を推奨します。');
  }
  if (isSettingChanged('link_mode') && settingsState.draft.link_mode === '2') {
    warnings.push('PONS Link が「2：受信（追尾ボート側）」に変更されます。機体本体に書き込んでいないか確認してください。');
  }
  if (isSettingChanged('needs_apply') && settingsState.draft.needs_apply === '0') {
    warnings.push('【危険】needs_apply を手動で 0 にしています。機体の実取り付けと水平較正が一致しているか必ず確認してください。');
  }
  const diffs = computeSettingsDiffs();
  // 打ち間違いの見張り。保存を押すと確認のダイアログでもう一度聞く（commitSettingsToPons）
  for (const msg of pilonSafetyIssues()) warnings.push(`【要確認】${msg}入力の間違いでないか確かめてください。`);
  if (diffs.some(d => d.schema.type === 'latlon')) {
    warnings.push('パイロン座標は保存しても本体の次の起動まで使われません。保存したら本体を再起動してください。機体・予備機・ボートの 3 台で同じ座標にしないと、無線で目的地が合いません。');
  }
  if (diffs.some(d => d.schema.needsReboot)) {
    warnings.push('無線設定（link_*）の変更は、本体を再起動したあとにハードウェアへ反映されます。');
  }
  return { errors, warnings };
}

function toggleExpertUnlock() {
  const cb = document.getElementById('settings-unlock-expert');
  settingsState.expertUnlocked = cb.checked;
  const lbl = document.getElementById('settings-unlock-label');
  lbl.classList.toggle('unlocked', settingsState.expertUnlocked);
  lbl.childNodes[lbl.childNodes.length - 1].textContent = settingsState.expertUnlocked
    ? ' 🔓 上級者・危険項目のロック解除中'
    : ' 🔒 上級者・危険項目のロックを解除';
  renderSettingsBody();
}

function setSettingsCat(catId) {
  settingsState.activeCat = catId;
  renderSettingsAll();
}

function renderSettingsAll() {
  renderSettingsCats();
  renderSettingsBody();
  updateSettingsFooter();
}

function renderSettingsCats() {
  const box = document.getElementById('settings-cats');
  const diffs = computeSettingsDiffs();
  box.innerHTML = PONS_SETTING_CATS.map(c => {
    const cnt = c.id === 'all'
      ? diffs.length
      : diffs.filter(d => d.schema.cat === c.id).length;
    const dot = cnt > 0 ? `<span class="chg-dot">${cnt}</span>` : '';
    return `<button class="scat-btn${c.id === settingsState.activeCat ? ' active' : ''}" onclick="setSettingsCat('${c.id}')">${c.label}${dot}</button>`;
  }).join('');
}

function escHtml(s) {
  return String(s ?? '').replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
}

function renderSettingsBody() {
  const body = document.getElementById('settings-body');
  if (!settingsState.loaded) return;

  const q = (document.getElementById('settings-search').value || '').trim().toLowerCase();
  const onlyChanged = document.getElementById('settings-only-changed').checked;
  const { errors } = validateAllDrafts();

  let html = '';

  // TAS カテゴリまたは「すべて」のときはリアルタイム曲線プレビューを表示
  if ((settingsState.activeCat === 'tas' || settingsState.activeCat === 'all') && !q && !onlyChanged) {
    html += `
          <div class="tas-preview-box">
            <canvas id="tas-curve-canvas" width="320" height="175"></canvas>
            <div style="flex:1; min-width:220px; font-size:0.82rem; color:#334155; line-height:1.55;">
              <div style="font-weight:bold; color:#1e40af; font-size:0.9rem; margin-bottom:4px;">📈 推定 TAS モデル リアルタイムプレビュー</div>
              <div>式: <code>V(θ) = V0 · √(K / (K + θ))</code></div>
              <div id="tas-preview-stats" style="margin-top:6px; font-family:var(--font-mono); background:#f8fafc; padding:6px 8px; border-radius:6px; border:1px solid #e2e8f0;"></div>
              <div style="margin-top:6px; font-size:0.76rem; color:#64748b;">
                青実線＝現在の下書き / 灰点線＝変更前 / 赤点線＝上下限クランプ（min〜max）<br>
                <b>K</b> は「ピッチ 0° から揚力ゼロになるまでの角度」。K が小さいほどピッチ 1° あたりの速度変化が急になります。
              </div>
            </div>
          </div>`;
  }

  const items = PONS_SETTINGS_SCHEMA.filter(s => {
    if (!(s.key in settingsState.orig)) return false;
    if (settingsState.activeCat !== 'all' && s.cat !== settingsState.activeCat) return false;
    if (onlyChanged && !isSettingChanged(s.key)) return false;
    if (q) {
      const hay = `${s.key} ${s.label} ${s.desc} ${s.caution}`.toLowerCase();
      if (!hay.includes(q)) return false;
    }
    return true;
  });

  if (items.length === 0) {
    html += '<div style="text-align:center; color:#94a3b8; padding:32px;">条件に一致する設定項目はありません。</div>';
  } else {
    for (const s of items) {
      const tier = PONS_TIER_META[s.tier];
      const locked = tier.locked && !settingsState.expertUnlocked;
      const changed = isSettingChanged(s.key);
      const missing = !settingsState.inFile[s.key];
      const val = settingsState.draft[s.key] ?? '';
      const origVal = settingsState.orig[s.key] ?? '';
      const err = errors[s.key] || '';

      let ctrlHtml = '';
      const dis = locked ? 'disabled' : '';
      if (s.type === 'select') {
        const opts = s.options.slice();
        if (!opts.some(o => o.v === val)) opts.unshift({ v: val, l: `${val}（現在値）` });
        ctrlHtml = `<select ${dis} onchange="onSettingInput('${s.key}', this.value)">` +
          opts.map(o => `<option value="${escHtml(o.v)}"${o.v === val ? ' selected' : ''}>${escHtml(o.l)}</option>`).join('') +
          `</select>`;
      } else if (s.type === 'dest') {
        const dests = settingsState.destinations.slice();
        if (val && !dests.includes(val)) dests.unshift(val);
        if (dests.length > 0) {
          ctrlHtml = `<select ${dis} onchange="onSettingInput('${s.key}', this.value)">` +
            dests.map(d => `<option value="${escHtml(d)}"${d === val ? ' selected' : ''}>${escHtml(d)}</option>`).join('') +
            `</select>`;
        } else {
          ctrlHtml = `<input type="text" ${dis} value="${escHtml(val)}" oninput="onSettingInput('${s.key}', this.value)">`;
        }
      } else if (s.type === 'number') {
        const minA = typeof s.min === 'number' ? `min="${s.min}"` : '';
        const maxA = typeof s.max === 'number' ? `max="${s.max}"` : '';
        const stepA = typeof s.step === 'number' ? `step="${s.step}"` : 'step="any"';
        ctrlHtml = `<div style="display:flex; align-items:center; gap:6px;">` +
          `<input type="number" ${dis} ${minA} ${maxA} ${stepA} value="${escHtml(val)}" oninput="onSettingInput('${s.key}', this.value)">` +
          (s.unit ? `<span style="font-size:0.8rem; color:#475569; white-space:nowrap;">${escHtml(s.unit)}</span>` : '') +
          `</div>`;
      } else {
        ctrlHtml = `<input type="text" ${dis} value="${escHtml(val)}" oninput="onSettingInput('${s.key}', this.value)">`;
      }

      const rangeText = (s.type === 'number' && typeof s.min === 'number' && typeof s.max === 'number')
        ? `範囲: ${s.min}〜${s.max}` : '';

      html += `
            <div id="scard-${s.key}" class="setting-card tier-${s.tier}${changed ? ' changed' : ''}${locked ? ' locked' : ''}">
              <div>
                <div class="setting-head">
                  <span class="setting-label">${escHtml(s.label)}</span>
                  <span class="setting-key">${escHtml(s.key)}</span>
                  <span class="sbadge ${tier.cls}">${tier.label}</span>
                  ${s.type === 'latlon'
                    ? (missing ? '<span class="sbadge sbadge-missing" title="override_pilon_coordinate.csv に無いので、ファームの既定（navdata.h）の座標で動きます">既定の座標</span>'
                               : '<span class="sbadge sbadge-reboot" title="SD の override_pilon_coordinate.csv で既定（navdata.h）を上書きしています">SD で上書き中</span>')
                    : (missing ? '<span class="sbadge sbadge-missing" title="settings.txt に未記載のため本体の既定値で動作中。保存すると settings.txt に追加されます">未記載（既定値）</span>' : '')}
                  ${s.needsReboot ? '<span class="sbadge sbadge-reboot">要再起動</span>' : ''}
                  <span id="schg-${s.key}" class="sbadge sbadge-changed" style="${changed ? '' : 'display:none;'}">${missing ? '追加対象' : '変更あり'}</span>
                  ${locked ? '<span style="font-size:0.75rem; color:#64748b;">🔒 ロック中</span>' : ''}
                </div>
                <div class="setting-desc">${escHtml(s.desc)}</div>
                ${s.caution ? `<div class="setting-caution">⚠️ ${escHtml(s.caution)}</div>` : ''}
              </div>
              <div class="setting-ctrl">
                ${ctrlHtml}
                <div class="setting-meta">
                  <span>既定: <code>${escHtml(s.defaultVal)}</code>${rangeText ? ' / ' + escHtml(rangeText) : ''}</span>
                  <span style="display:inline-flex; gap:4px;">
                    ${missing && !locked && s.type !== 'latlon' ? `<button onclick="toggleAddMissingSetting('${s.key}')" title="既定値のまま settings.txt へ書き出す対象に追加/解除します">${settingsState.addToFile[s.key] ? '✓ 追加予約' : '＋ 追加'}</button>` : ''}
                    ${!locked ? `<button onclick="resetOneSettingDefault('${s.key}')" title="既定値（${escHtml(s.defaultVal)}）に戻す">既定</button>` : ''}
                  </span>
                </div>
                <div id="sorig-${s.key}" style="font-size:0.74rem; color:#1d4ed8; ${changed ? '' : 'display:none;'}">${(missing && s.type !== 'latlon') ? '未記載 → 保存で settings.txt に追加' : '変更前: <code>' + escHtml(origVal) + '</code>'}</div>
                ${s.type === 'latlon' ? pilonInfoHtml(s, val) : ''}
                <div id="serr-${s.key}" class="setting-err-msg" style="${err ? '' : 'display:none;'}">${err ? '❌ ' + escHtml(err) : ''}</div>
              </div>
            </div>`;
    }
  }

  body.innerHTML = html;
  drawTasPreviewCanvas();
}

// パイロンの項目の下に出す: 既定からの距離・地図のリンク・いま本体が使っている座標（保存済みで再起動待ちなら知らせる）
function pilonInfoHtml(s, val) {
  const ll = parseLatLon(val), def = parseLatLon(s.defaultVal);
  const rt = settingsState.pilonRuntime[s.key];
  const parts = [];
  if (ll && def) {
    const dm = latLonDistM(def, ll);
    parts.push(dm < 0.5 ? '既定と同じ' : `既定から ${fmtDistM(dm)}`);
    parts.push(`<a href="https://www.google.com/maps?q=${ll.lat.toFixed(6)},${ll.lon.toFixed(6)}" target="_blank" rel="noopener">地図で見る</a>`);
  }
  // 打ち間違いの見張り（pilonSafetyIssues と同じ基準。保存の前にも確認のダイアログが出る）
  if (ll) {
    const out = biwakoOutsideM(ll);
    if (out > BIWAKO_FAR_M) parts.push(`<span style="color:#dc2626; font-weight:bold;">琵琶湖から約 ${fmtDistM(out)} 離れています</span>`);
  }
  const pname = s.key.substring('pilon.'.length);
  if (PILON_PLAT_CHECK.includes(pname)) {
    const dm = pilonPlatDistM(pname);
    if (dm !== null) parts.push(pilonPlatDistBad(dm)
      ? `<span style="color:#dc2626; font-weight:bold;">プラットホームから ${fmtDistM(dm)}（${PILON_PLAT_MIN_M / 1000}〜${PILON_PLAT_MAX_M / 1000} km の外）</span>`
      : `プラットホームから ${fmtDistM(dm)}`);
  }
  // いま本体が使っている座標が入力欄と違うときだけ知らせる（保存済みなら再起動で、まだなら保存して再起動で変わる）
  const cur = normalizeSettingVal(s, val);
  const saved = normalizeSettingVal(s, settingsState.orig[s.key]) === cur;
  if (rt && rt !== cur) parts.push(`<span style="color:#b45309;">いま本体が使っているのは <code>${escHtml(rt)}</code>（${saved ? '保存済み。再起動で' : '保存して再起動すると'}上の値に変わる）</span>`);
  return `<div id="spilon-${s.key}" style="font-size:0.74rem; color:#475569;">${parts.join(' / ')}</div>`;
}

function updateSettingCardVisuals(keysToCheck) {
  const { errors } = validateAllDrafts();
  for (const k of keysToCheck) {
    const card = document.getElementById(`scard-${k}`);
    if (!card) continue;
    const chg = isSettingChanged(k);
    const missing = !settingsState.inFile[k];
    const err = errors[k] || '';
    card.classList.toggle('changed', chg);
    const badge = document.getElementById(`schg-${k}`);
    if (badge) {
      badge.style.display = chg ? '' : 'none';
      badge.textContent = missing ? '追加対象' : '変更あり';
    }
    const origEl = document.getElementById(`sorig-${k}`);
    if (origEl) origEl.style.display = chg ? '' : 'none';
    const pEl = document.getElementById(`spilon-${k}`);
    if (pEl && PONS_SCHEMA_MAP[k]) pEl.outerHTML = pilonInfoHtml(PONS_SCHEMA_MAP[k], settingsState.draft[k] ?? '');
    const errEl = document.getElementById(`serr-${k}`);
    if (errEl) {
      errEl.style.display = err ? '' : 'none';
      errEl.textContent = err ? `❌ ${err}` : '';
    }
  }
}

function toggleAddMissingSetting(key) {
  settingsState.addToFile[key] = !settingsState.addToFile[key];
  renderSettingsAll();
}

function onSettingInput(key, val) {
  settingsState.draft[key] = val;
  // airspeed_min / airspeed_max は相互チェックがあるため両方更新
  // パイロンはプラットホームとの距離を北・南の欄に出すので、どれかを変えたら全部を出し直す
  const keys = (key === 'airspeed_min' || key === 'airspeed_max')
    ? ['airspeed_min', 'airspeed_max']
    : key.startsWith('pilon.')
      ? PONS_PILON_SCHEMA.map(p => `pilon.${p.name}`)
      : [key];
  updateSettingCardVisuals(keys);
  renderSettingsCats();
  updateSettingsFooter();
  if (key.startsWith('airspeed_')) {
    drawTasPreviewCanvas();
  }
}

function resetOneSettingDefault(key) {
  const s = PONS_SCHEMA_MAP[key];
  if (!s) return;
  settingsState.draft[key] = s.defaultVal;
  renderSettingsAll();
}

function revertSettingsDraft() {
  settingsState.draft = { ...settingsState.orig };
  settingsState.addToFile = {};
  renderSettingsAll();
}

function undoLastSavedSettings() {
  if (!settingsState.lastSavedBackup) return;
  settingsState.draft = { ...settingsState.lastSavedBackup };
  renderSettingsAll();
}

function updateSettingsFooter() {
  const diffs = computeSettingsDiffs();
  const { errors } = validateAllDrafts();
  const errCount = Object.keys(errors).length;

  const summary = document.getElementById('settings-change-summary');
  summary.textContent = diffs.length > 0 ? `未保存の変更: ${diffs.length} 件` : '';

  const valStat = document.getElementById('settings-validation-status');
  valStat.textContent = errCount > 0 ? `❌ 入力エラーが ${errCount} 件あります（修正してください）` : '';

  const revBtn = document.getElementById('btn-settings-revert');
  revBtn.disabled = (diffs.length === 0);
  revBtn.style.opacity = (diffs.length === 0) ? '0.5' : '1';

  const undoBtn = document.getElementById('btn-settings-undo');
  undoBtn.disabled = !settingsState.lastSavedBackup;
  undoBtn.style.opacity = settingsState.lastSavedBackup ? '1' : '0.5';

  const reviewBtn = document.getElementById('btn-settings-review');
  const canSave = settingsState.loaded && diffs.length > 0 && errCount === 0 && !settingsState.saving;
  reviewBtn.disabled = !canSave;
  reviewBtn.style.opacity = canSave ? '1' : '0.5';
}

function drawTasPreviewCanvas() {
  const cv = document.getElementById('tas-curve-canvas');
  if (!cv) return;
  const ctx = cv.getContext('2d');
  const W = cv.width, H = cv.height;
  ctx.clearRect(0, 0, W, H);

  const v0 = Number(settingsState.draft.airspeed_v0);
  const k  = Number(settingsState.draft.airspeed_k);
  const lo = Number(settingsState.draft.airspeed_min);
  const hi = Number(settingsState.draft.airspeed_max);

  const ov0 = Number(settingsState.orig.airspeed_v0);
  const ok  = Number(settingsState.orig.airspeed_k);

  const padL = 38, padR = 12, padT = 12, padB = 24;
  const pw = W - padL - padR, ph = H - padT - padB;
  const thMin = -6, thMax = 10;
  const vMin = 4.0, vMax = 12.0;

  const xOf = th => padL + ((th - thMin) / (thMax - thMin)) * pw;
  const yOf = v  => padT + (1 - (v - vMin) / (vMax - vMin)) * ph;

  // グリッド
  ctx.strokeStyle = '#e2e8f0';
  ctx.lineWidth = 1;
  for (let th = -6; th <= 10; th += 2) {
    const x = xOf(th);
    ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, H - padB); ctx.stroke();
  }
  for (let v = 4; v <= 12; v += 2) {
    const y = yOf(v);
    ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(W - padR, y); ctx.stroke();
  }
  // θ=0 の縦線
  ctx.strokeStyle = '#94a3b8';
  ctx.beginPath(); ctx.moveTo(xOf(0), padT); ctx.lineTo(xOf(0), H - padB); ctx.stroke();

  // 軸ラベル
  ctx.fillStyle = '#64748b';
  ctx.font = '10px monospace';
  ctx.textAlign = 'center';
  for (let th = -6; th <= 10; th += 4) ctx.fillText(`${th}°`, xOf(th), H - 8);
  ctx.textAlign = 'right';
  for (let v = 4; v <= 12; v += 2) ctx.fillText(`${v}`, padL - 4, yOf(v) + 3);

  // 上下限の赤点線
  ctx.save();
  ctx.setLineDash([4, 3]);
  ctx.strokeStyle = '#ef4444';
  if (Number.isFinite(lo) && lo >= vMin && lo <= vMax) {
    ctx.beginPath(); ctx.moveTo(padL, yOf(lo)); ctx.lineTo(W - padR, yOf(lo)); ctx.stroke();
  }
  if (Number.isFinite(hi) && hi >= vMin && hi <= vMax) {
    ctx.beginPath(); ctx.moveTo(padL, yOf(hi)); ctx.lineTo(W - padR, yOf(hi)); ctx.stroke();
  }
  ctx.restore();

  const drawCurve = (V0, K, color, dashed) => {
    if (!Number.isFinite(V0) || !Number.isFinite(K) || K <= 0) return;
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = dashed ? 1.5 : 2.5;
    if (dashed) ctx.setLineDash([4, 4]);
    ctx.beginPath();
    let started = false;
    for (let th = thMin; th <= thMax; th += 0.2) {
      const d = K + th;
      if (d <= 0.1) continue;
      let v = V0 * Math.sqrt(K / d);
      if (!dashed && Number.isFinite(lo) && Number.isFinite(hi) && lo < hi) {
        v = Math.max(lo, Math.min(hi, v));
      }
      const x = xOf(th), y = Math.max(padT, Math.min(H - padB, yOf(v)));
      if (!started) { ctx.moveTo(x, y); started = true; } else ctx.lineTo(x, y);
    }
    ctx.stroke();
    ctx.restore();
  };

  // 変更前（灰点線）→ 現在（青実線）
  drawCurve(ov0, ok, '#94a3b8', true);
  drawCurve(v0, k, '#2563eb', false);

  // θ=0 点の丸
  if (Number.isFinite(v0) && v0 >= vMin && v0 <= vMax) {
    ctx.fillStyle = '#2563eb';
    ctx.beginPath(); ctx.arc(xOf(0), yOf(v0), 4, 0, Math.PI * 2); ctx.fill();
  }

  const stats = document.getElementById('tas-preview-stats');
  if (stats) {
    if (Number.isFinite(v0) && Number.isFinite(k) && k > 0) {
      const slope = -v0 / (2 * k);
      const vM2 = (k - 2 > 0) ? (v0 * Math.sqrt(k / (k - 2))).toFixed(2) : '--';
      const vP2 = (v0 * Math.sqrt(k / (k + 2))).toFixed(2);
      stats.innerHTML =
        `θ=0°: <b>${v0.toFixed(2)} m/s</b> ｜ 傾き(θ=0): <b>${slope.toFixed(3)} m/s/°</b><br>` +
        `θ=-2°: ${vM2} m/s ｜ θ=+2°: ${vP2} m/s ｜ 範囲: ${Number.isFinite(lo) ? lo.toFixed(2) : '--'}〜${Number.isFinite(hi) ? hi.toFixed(2) : '--'} m/s`;
    } else {
      stats.textContent = 'V0 または K の値が不正です';
    }
  }
}

function openSettingsDiffModal() {
  const diffs = computeSettingsDiffs();
  const { errors, warnings } = validateAllDrafts();
  if (diffs.length === 0 || Object.keys(errors).length > 0) return;

  const warnBox = document.getElementById('diff-warnings');
  if (warnings.length > 0) {
    warnBox.innerHTML = warnings.map(w =>
      `<div style="background:#fffbeb; border:1px solid #fcd34d; color:#92400e; padding:8px 10px; border-radius:6px; font-size:0.83rem; margin-bottom:6px;">⚠️ ${escHtml(w)}</div>`
    ).join('');
  } else {
    warnBox.innerHTML = '';
  }

  const tbody = document.getElementById('diff-table-body');
  tbody.innerHTML = diffs.map(d => {
    const tier = PONS_TIER_META[d.schema.tier];
    return `<tr>
          <td>
            <div style="font-weight:bold;">${escHtml(d.schema.label)}</div>
            <div style="font-size:0.75rem; color:#64748b;">
              <code>${escHtml(d.schema.key)}</code>
              <span class="sbadge ${tier.cls}">${tier.label}</span>
              ${d.wasMissing ? '<span class="sbadge sbadge-missing">新規追加</span>' : ''}
            </div>
          </td>
          <td class="diff-old">${escHtml(d.oldVal)}</td>
          <td class="diff-new">${escHtml(d.newVal)}</td>
          <td style="font-size:0.78rem; color:#475569;">${escHtml(d.schema.caution || '—')}</td>
        </tr>`;
  }).join('');

  const hasDanger = diffs.some(d => d.schema.tier === 'expert' || d.schema.tier === 'internal');
  const ackBox = document.getElementById('diff-danger-ack-box');
  const ackCb = document.getElementById('diff-danger-ack');
  ackBox.style.display = hasDanger ? 'block' : 'none';
  ackCb.checked = false;

  document.getElementById('diff-save-progress').style.display = 'none';
  document.getElementById('btn-diff-cancel').disabled = false;
  updateDiffCommitBtn();
  document.getElementById('settings-diff-modal').classList.add('active');
}

function updateDiffCommitBtn() {
  const ackBox = document.getElementById('diff-danger-ack-box');
  const needAck = ackBox.style.display !== 'none';
  const ok = !needAck || document.getElementById('diff-danger-ack').checked;
  const btn = document.getElementById('btn-diff-commit');
  btn.disabled = !ok || settingsState.saving;
  btn.style.opacity = (!ok || settingsState.saving) ? '0.5' : '1';
}

function closeSettingsDiffModal() {
  if (settingsState.saving) return;
  document.getElementById('settings-diff-modal').classList.remove('active');
}

async function commitSettingsToPons() {
  if (!serialWriter || !ponsReady || settingsState.saving) return;
  const diffs = computeSettingsDiffs();
  if (diffs.length === 0) return;
  // ★ パイロン座標の打ち間違い（琵琶湖から遠い・プラットホームとの距離が 10〜12 km の外）は、保存の直前に確認のダイアログ。
  //   キャンセルなら何も送らない（差分の画面に戻る）。2026-11-23〜（ユーザー）
  const pilonIssues = pilonSafetyIssues();
  if (pilonIssues.length > 0 &&
      !confirm(`パイロン座標が入力の間違いかもしれません。\n\n・${pilonIssues.join('\n・')}\n\nこのまま本体に保存しますか？`)) {
    return;
  }

  settingsState.saving = true;
  const backupBeforeSave = { ...settingsState.orig };
  const prog = document.getElementById('diff-save-progress');
  const cancelBtn = document.getElementById('btn-diff-cancel');
  const commitBtn = document.getElementById('btn-diff-commit');
  prog.style.display = 'block';
  cancelBtn.disabled = true;
  commitBtn.disabled = true;
  commitBtn.style.opacity = '0.5';

  try {
    const adjusted = [];
    // ★ 前の保存が SAVE まで行かずに失敗していると、本体にパイロンの決めかけ（SET 済み）が残っている。そのあと下書きを
    //   元に戻したパイロンは差分に出ないので、そのまま SAVE すると取り消したはずの座標が書かれる。元の値を送り直して打ち消す。
    for (const k of [...settingsState.pilonStaged]) {
      if (diffs.some(d => d.schema.key === k) || !(k in settingsState.orig)) continue;
      const s = PONS_SCHEMA_MAP[k];
      const v = normalizeSettingVal(s, settingsState.orig[k]);
      prog.textContent = `前回の決めかけを打ち消し中: ${k} = ${v} ...`;
      const rep0 = await sendSettingCmdAndWait(`PONS:SETTINGS_SET,${k}:${v}`);
      if (!rep0.startsWith('PONS:SETTINGS_OK,')) throw new Error(`設定 ${k} の打ち消しに失敗しました (${rep0})`);
    }
    for (let i = 0; i < diffs.length; i++) {
      const d = diffs[i];
      prog.textContent = `送信中 (${i + 1} / ${diffs.length}): ${d.schema.key} = ${d.newVal} ...`;
      if (d.schema.type === 'latlon') settingsState.pilonStaged.add(d.schema.key);   // 本体に決めかけが残りうる（失敗に備えて先に記録）
      const rep = await sendSettingCmdAndWait(`PONS:SETTINGS_SET,${d.schema.key}:${d.newVal}`);
      if (!rep.startsWith('PONS:SETTINGS_OK,')) {
        throw new Error(`設定 ${d.schema.key} の反映に失敗しました (${rep})`);
      }
      // PONS:SETTINGS_OK,<key>,<applied_value> を確認し、本体側でクランプ・無視された場合は記録
      const prefix = `PONS:SETTINGS_OK,${d.schema.key},`;
      if (rep.startsWith(prefix)) {
        const applied = rep.substring(prefix.length).trim();
        const wantNorm = normalizeSettingVal(d.schema, d.newVal);
        const gotNorm  = normalizeSettingVal(d.schema, applied);
        if (wantNorm !== gotNorm) {
          adjusted.push(`${d.schema.key}: 送信 ${d.newVal} → 本体実効値 ${applied}`);
        }
      }
    }
    prog.textContent = 'SD カード（settings.txt）へ安全に保存中...';
    const saveRep = await sendSettingCmdAndWait('PONS:SETTINGS_SAVE', 8000);
    if (!saveRep.startsWith('PONS:SETTINGS_SAVED,OK')) {
      throw new Error(`SD 保存に失敗しました (${saveRep})`);
    }

    settingsState.lastSavedBackup = backupBeforeSave;
    settingsState.addToFile = {};
    settingsState.pilonStaged = new Set();   // SAVE で書き切った
    settingsState.saving = false;
    document.getElementById('settings-diff-modal').classList.remove('active');

    const needReboot = diffs.some(d => d.schema.needsReboot);
    let msg = `✅ ${diffs.length} 件の設定を本体に保存しました。\n（変更履歴は SD の log.txt にも記録されました）`;
    if (adjusted.length > 0) {
      msg += `\n\n⚠️ 一部の値が本体側の制約により調整されました:\n・${adjusted.join('\n・')}`;
    }
    if (needReboot) {
      msg += '\n\n⚠️ 無線設定（link_*）を変更したため、反映するには本体を再起動してください。';
    }
    if (diffs.some(d => d.schema.type === 'latlon')) {
      msg += '\n\n⚠️ パイロン座標は SD の override_pilon_coordinate.csv に書きました。本体を再起動すると使われます。機体・予備機・ボートの 3 台を同じにしてください。';
    }
    alert(msg);
    // 本体から最新値を再取得して表示を同期
    await requestPonsSettings(true);
  } catch (err) {
    settingsState.saving = false;
    cancelBtn.disabled = false;
    updateDiffCommitBtn();
    prog.style.color = '#dc2626';
    prog.textContent = `❌ エラー: ${err.message}`;
    alert(`設定の保存中にエラーが発生しました:\n${err.message}`);
  }
}

// ESC キーで設定モーダル／Diff モーダルを安全に閉じる
document.addEventListener('keydown', (e) => {
  if (e.key !== 'Escape') return;
  const diffM = document.getElementById('settings-diff-modal');
  if (diffM && diffM.classList.contains('active')) {
    closeSettingsDiffModal();
    return;
  }
  const setM = document.getElementById('settings-modal');
  if (setM && setM.classList.contains('active')) {
    closeSettingsModal();
  }
});

function exportSettingsBackup(useOrig) {
  if (!settingsState.loaded) return;
  const src = useOrig ? settingsState.orig : settingsState.draft;
  const lines = [];
  for (const s of PONS_SETTINGS_SCHEMA) {
    if (s.key in src) {
      lines.push(`${s.key}:${normalizeSettingVal(s, src[s.key])}`);
    }
  }
  const blob = new Blob([lines.join('\n') + '\n'], { type: 'text/plain;charset=utf-8' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  const stamp = new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-');
  a.href = url;
  a.download = useOrig ? `settings_backup_${stamp}.txt` : `settings_${stamp}.txt`;
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  URL.revokeObjectURL(url);
}

function importSettingsFile(input) {
  const file = input.files && input.files[0];
  input.value = '';
  if (!file) return;
  const reader = new FileReader();
  reader.onload = () => {
    const text = String(reader.result || '');
    let applied = 0;
    let hasLockedChange = false;
    for (const rawLine of text.split(/\r?\n/)) {
      const idx = rawLine.indexOf(':');
      if (idx <= 0) continue;
      const k = rawLine.substring(0, idx).trim();
      const v = rawLine.substring(idx + 1).trim();
      const s = PONS_SCHEMA_MAP[k];
      if (!s || !(k in settingsState.orig)) continue;
      settingsState.draft[k] = v;
      applied++;
      if (PONS_TIER_META[s.tier].locked && isSettingChanged(k)) hasLockedChange = true;
    }
    if (hasLockedChange && !settingsState.expertUnlocked) {
      document.getElementById('settings-unlock-expert').checked = true;
      toggleExpertUnlock();
    }
    renderSettingsAll();
    alert(`ファイルから ${applied} 項目を下書きに読み込みました。\nまだ本体には保存されていません。「📋 変更内容を確認して保存...」で差分を確認してください。`);
  };
  reader.readAsText(file);
}
