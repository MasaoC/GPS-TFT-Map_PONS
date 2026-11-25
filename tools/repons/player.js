// ============================================================
// File    : tools/repons/player.js
// Project : PONS v7.1 / v7.2 Re:PONS
// Role    : 再生の本体。グローバルの状態・初期化・地図（Leaflet）・CSV の読み込み・Est.Beta の推定・
//           サンプルのデータ・補間・アニメーションの更新・航跡・再生の操作・レイアウトとテーマ
// Author  : MasaoC (@masao_mobile)
// Updated : 2026-11-23
// ============================================================
//
// 2026-11-23 に index.html から分けた。読む順は index.html の一覧どおり。関数・変数はファイルをまたいで
// グローバルで見えるが、読み込みの最中に動くコードは先に読んだファイルのものしか使えない。

// ============================================================
// グローバル状態管理
// ============================================================
let flightData = [];       // パース済みデータ行配列
let currentCsvText = "";   // 現在開いているCSVテキストデータ
let currentCsvFilename = "flight_log.csv"; // 現在開いているCSVファイル名
let isPlaying = false;
let playbackSpeed = 1.0;
let currentTimeSec = 0;    // ログ先頭からの相対秒 [0 .. duration]
let totalDurationSec = 0;
let lastAnimFrameTime = null;
let takeoffSec = 0;
let landSec = 0;
let csvClockBaseSec = null;   // CSV の 1 行目の時刻 [JST の日内秒]（time 列が無ければ null）。時間のバーの時刻表示用
let flightAltMin = 0.0;
let flightAltMax = 12.0;
const FLIGHT_SPEED_THRESHOLD = 1.0; // [m/s] 飛行判定しきい値 (1.0 m/s = 3.6 km/h)

// 静止スキップ用ヘルパー
function findNextFlightTime(fromSec) {
  if (!flightData || flightData.length === 0) return -1;
  for (let i = 0; i < flightData.length; i++) {
    if (flightData[i].t >= fromSec && flightData[i].gs >= FLIGHT_SPEED_THRESHOLD) {
      return flightData[i].t;
    }
  }
  return -1;
}

function getFirstFlightTime() {
  if (!flightData || flightData.length === 0) return 0;
  for (let i = 0; i < flightData.length; i++) {
    if (flightData[i].gs >= FLIGHT_SPEED_THRESHOLD) {
      return flightData[i].t;
    }
  }
  return 0;
}

// 動いている区間（G/S >= FLIGHT_SPEED_THRESHOLD）。シークバーの青帯・動き出しの印・飛行解析の区間で共通に使う。
// 静止したら区切る。2 秒未満の瞬間的な静止は 1 つの区間としてつなぐ。
// 戻り値: merged = つないだ区間すべて（青帯）/ segments = 1.5 秒以上動いた区間に #1, #2 … と番号を振ったもの（印と解析）
const MOVE_MERGE_GAP_SEC = 2.0;
const MOVE_MIN_SEG_SEC = 1.5;
// 記録がこれより長く途切れたら、前後が動いていても別の区間にする（2026-11-20。2025 年のログは着水後に
// 402 秒の欠けがあり、前後の行がどちらも G/S ≥ 1 m/s だったので欠けをまたいで 1 つの区間になっていた）。
// トンネルなど GNSS だけが途切れて走り続けている間（11/19 の車で 22〜89 秒）はつなぐので、2 分にしてある
const MOVE_MAX_DATA_GAP_SEC = 120.0;
let movingSegments = [];   // 開いている CSV の segments（parseCSVText のたびに作り直す）

function computeMovingSegments(records) {
  const rawSegs = [];
  let segStart = null;
  for (let i = 0; i < (records ? records.length : 0); i++) {
    const d = records[i];
    const moving = (d.gs !== null && d.gs !== undefined && !isNaN(d.gs) && d.gs >= FLIGHT_SPEED_THRESHOLD);
    // 記録の途切れ: 前の行で区間を閉じる（途切れの中を「動いていた」ことにしない）
    if (segStart !== null && i > 0 && d.t - records[i - 1].t > MOVE_MAX_DATA_GAP_SEC) {
      rawSegs.push({ start: segStart, end: records[i - 1].t, gapAfter: true });
      segStart = null;
    }
    if (moving && segStart === null) {
      segStart = d.t;
    } else if (!moving && segStart !== null) {
      rawSegs.push({ start: segStart, end: d.t });
      segStart = null;
    }
  }
  if (segStart !== null) {
    rawSegs.push({ start: segStart, end: records[records.length - 1].t });
  }
  const merged = [];
  for (const s of rawSegs) {
    const last = merged[merged.length - 1];
    if (last && !last.gapAfter && (s.start - last.end) < MOVE_MERGE_GAP_SEC) {
      last.end = s.end;
      last.gapAfter = !!s.gapAfter;
    } else {
      merged.push({ start: s.start, end: s.end, gapAfter: !!s.gapAfter });
    }
  }
  const segments = merged
    .filter(s => s.end - s.start >= MOVE_MIN_SEG_SEC)
    .map((s, i) => ({ no: i + 1, start: s.start, end: s.end }));
  return { merged, segments };
}
window.computeMovingSegments = computeMovingSegments;

// シークバー上の静止/移動の色分け＆「静止→動き出し」マーカーを描画
function updateSeekBarSegments() {
  const trackBg = document.getElementById('seek-track-bg');
  const markersEl = document.getElementById('seek-markers');
  if (!trackBg || !markersEl) return;

  const skipOn = !!document.getElementById('check-flight-only')?.checked;
  trackBg.classList.toggle('skip-on', skipOn);
  trackBg.innerHTML = '';
  markersEl.innerHTML = '';

  const { merged, segments } = computeMovingSegments(flightData);
  movingSegments = segments;
  if (!flightData || flightData.length < 2 || totalDurationSec <= 0) return;

  // 移動区間の青帯を描画
  for (const s of merged) {
    const leftPct = Math.max(0, Math.min(100, (s.start / totalDurationSec) * 100));
    const widthPct = Math.max(0.25, Math.min(100 - leftPct, ((s.end - s.start) / totalDurationSec) * 100));
    const bar = document.createElement('div');
    bar.className = 'seek-seg-move';
    bar.style.left = `${leftPct}%`;
    bar.style.width = `${widthPct}%`;
    trackBg.appendChild(bar);
  }

  // 「静止→動き出し」ポイントに緑マーカーを配置（1.5 秒以上動いた区間を対象）。
  // 番号は飛行解析の「区間 #n」と同じ（先頭から動いている区間は印を出さないが番号は数える）
  for (const s of segments) {
    // 先頭(0秒)から既に動いている区間は静止からの動き出しではないので出さない。それ以外はすべてマーカー表示
    if (s.start <= 0.5) continue;
    const markIdx = s.no;
    const leftPct = Math.max(0, Math.min(100, (s.start / totalDurationSec) * 100));
    const durSec = Math.max(0, s.end - s.start);
    const mk = document.createElement('div');
    mk.className = 'seek-start-marker';
    mk.style.left = `${leftPct}%`;
    const rec = interpolateAt(s.start);
    const tLabel = (rec && rec.timeStr) ? rec.timeStr.split('.')[0] : formatTime(s.start);
    mk.title = `▶ 動き出し（区間 #${markIdx}）: ${tLabel} (移動 ${formatTime(durSec)}) — クリックでジャンプ`;
    mk.addEventListener('click', (e) => {
      e.stopPropagation();
      currentPlaneTransformYaw = null;
      currentBoatTransformHeading = null;
      currentTimeSec = s.start;
      updateUI(currentTimeSec);
    });
    markersEl.appendChild(mk);
  }
}

// 地図オブジェクト
let map = null;
let trailPolyline = null;
let currentMarker = null;
let boatMarker = null;
let linkPolyline = null;
let mapInitialized = false;
let isReceivedLog = false;
let currentPlaneTransformYaw = null;     // CSS transform用連続ヨー角 (360度跨ぎ時の逆回転防止)
let currentBoatTransformHeading = null;  // CSS transform用連続ボート方位角

// パイロン等の固定目標 (PONS公式 navdata.h より)
const TARGETS = {
  PLATFORM:    { name: "プラットホーム", lat: 35.294222, lon: 136.254333, color: '#ff9800', radius: 8 },
  PILON_1KM:   { name: "1kmパイロン",   lat: 35.297961, lon: 136.243624, color: '#00d2ff', radius: 6 },
  NORTH_PILON: { name: "北パイロン",    lat: 35.377878, lon: 136.189907, color: '#388bfd', radius: 7 },
  SOUTH_PILON: { name: "南パイロン",    lat: 35.274220, lon: 136.136183, color: '#388bfd', radius: 7 },
  TAKESHIMA:   { name: "多景島",        lat: 35.296584, lon: 136.178054, color: '#a3e635', radius: 5 }
};

// ============================================================
// 初期化
// ============================================================
window.addEventListener('DOMContentLoaded', () => {
  initMap();
  setupEventListeners();
  window.addEventListener('resize', handleResize);
  if (window.ResizeObserver) {
    const ro = new ResizeObserver(() => handleResize());
    document.querySelectorAll('.gauge-card, .chart-panel').forEach(el => ro.observe(el));
  }
  handleResize();
  // 初期アニメーションループ開始
  requestAnimationFrame(animationLoop);
});

function setupEventListeners() {
  // ファイル選択
  document.getElementById('file-input').addEventListener('change', (e) => {
    if (e.target.files.length > 0) {
      readCSVFile(e.target.files[0]);
    }
  });

  // ドラッグ＆ドロップ
  const dropZone = document.getElementById('drop-zone');
  window.addEventListener('dragenter', (e) => { e.preventDefault(); dropZone.classList.add('active'); });
  dropZone.addEventListener('dragover', (e) => { e.preventDefault(); });
  dropZone.addEventListener('dragleave', (e) => {
    if (e.relatedTarget === null) dropZone.classList.remove('active');
  });
  dropZone.addEventListener('drop', (e) => {
    e.preventDefault();
    dropZone.classList.remove('active');
    const files = Array.from(e.dataTransfer.files || []);
    if (files.length === 0) return;
    // 暗号化した大会ログ・動画の対応表（.enc）はまとめてパスワードを聞いて開く（olddata_open.js）
    const encFiles = files.filter(f => /\.enc$/i.test(f.name));
    if (encFiles.length > 0 && typeof openEncryptedFiles === 'function') { openEncryptedFiles(encFiles); return; }
    const binFile = files.find(f => /\.bin$/i.test(f.name));
    const textFile = files.find(f => !/\.bin$/i.test(f.name));
    if (textFile) readCSVFile(textFile);
    if (binFile && typeof loadImuBinBytes === 'function') {
      const rd = new FileReader();
      rd.onload = () => loadImuBinBytes(rd.result, binFile.name, { openModal: !textFile, tab: 'imu' });
      rd.readAsArrayBuffer(binFile);
    }
  });

  // シークスライダー
  const slider = document.getElementById('seek-slider');
  slider.addEventListener('input', () => {
    if (totalDurationSec > 0) {
      currentPlaneTransformYaw = null;
      currentBoatTransformHeading = null;
      currentTimeSec = (slider.value / 1000) * totalDurationSec;
      updateUI(currentTimeSec);
    }
  });

  // キーボードショートカット
  window.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
    if (document.getElementById('log-modal').classList.contains('active')) return;   // log.txt を見ている間
    if (e.code === 'Space') {
      e.preventDefault();
      togglePlay();
    } else if (e.code === 'ArrowLeft') {
      e.preventDefault();
      skipTime(-5);
    } else if (e.code === 'ArrowRight') {
      e.preventDefault();
      skipTime(5);
    }
  });

  // 飛行のみ (静止スキップ) 切り替え
  const checkFlightOnly = document.getElementById('check-flight-only');
  if (checkFlightOnly) {
    checkFlightOnly.addEventListener('change', () => {
      updateSeekBarSegments();
      if (checkFlightOnly.checked && totalDurationSec > 0) {
        const cur = interpolateAt(currentTimeSec);
        if (cur && cur.gs < FLIGHT_SPEED_THRESHOLD) {
          const nextT = findNextFlightTime(currentTimeSec);
          if (nextT >= 0) {
            currentTimeSec = nextT;
          } else {
            currentTimeSec = (takeoffSec > 0 ? takeoffSec : getFirstFlightTime());
          }
          updateUI(currentTimeSec);
        }
      }
    });
  }
}

// ============================================================
// 地図初期化 (Leaflet)
// ============================================================
function initMap() {
  // 琵琶湖・彦根周辺を中心に初期化
  map = L.map('map', {
    zoomControl: true,
    attributionControl: true
  }).setView([35.32, 136.21], 11);

  // APIキー完全不要・無制限に利用可能な公式地図レイヤー群
  const baseMaps = {
    "🗾 国土地理院 (標準地図)": L.tileLayer('https://cyberjapandata.gsi.go.jp/xyz/std/{z}/{x}/{y}.png', {
      maxZoom: 18,
      attribution: '&copy; <a href="https://maps.gsi.go.jp/development/ichiran.html" target="_blank">国土地理院</a>'
    }),
    "🛰️ 国土地理院 (写真・航空写真)": L.tileLayer('https://cyberjapandata.gsi.go.jp/xyz/seamlessphoto/{z}/{x}/{y}.jpg', {
      maxZoom: 18,
      attribution: '&copy; <a href="https://maps.gsi.go.jp/development/ichiran.html" target="_blank">国土地理院</a>'
    }),
    "📄 国土地理院 (淡色地図)": L.tileLayer('https://cyberjapandata.gsi.go.jp/xyz/pale/{z}/{x}/{y}.png', {
      maxZoom: 18,
      attribution: '&copy; <a href="https://maps.gsi.go.jp/development/ichiran.html" target="_blank">国土地理院</a>'
    })
  };

  // デフォルトは視認性の高い国土地理院 標準地図
  baseMaps["🗾 国土地理院 (標準地図)"].addTo(map);

  // 右上に地図切り替えコントロールを追加
  L.control.layers(baseMaps, null, { position: 'topright' }).addTo(map);

  // コース基準線 (PONS公式コース)
  // プラットホーム ➔ 北パイロン (10km往路)
  L.polyline([[TARGETS.PLATFORM.lat, TARGETS.PLATFORM.lon], [TARGETS.NORTH_PILON.lat, TARGETS.NORTH_PILON.lon]], {
    color: '#d97706', weight: 2.0, dashArray: '6, 6', opacity: 0.9
  }).addTo(map);

  // プラットホーム ➔ 南パイロン
  L.polyline([[TARGETS.PLATFORM.lat, TARGETS.PLATFORM.lon], [TARGETS.SOUTH_PILON.lat, TARGETS.SOUTH_PILON.lon]], {
    color: '#2563eb', weight: 2.0, dashArray: '6, 6', opacity: 0.9
  }).addTo(map);

  // パイロン固定目標のプロット
  for (const [key, t] of Object.entries(TARGETS)) {
    L.circleMarker([t.lat, t.lon], {
      radius: t.radius || 6,
      color: t.color || '#2563eb',
      fillColor: t.color || '#2563eb',
      fillOpacity: 0.9
    }).bindTooltip(t.name, { permanent: true, direction: 'right', className: 'target-label' }).addTo(map);
  }

  // 航跡ライン (白・水色の地図上でくっきり見える濃い赤色)
  trailPolyline = L.polyline([], { color: '#e11d48', weight: 3.5, opacity: 0.95 }).addTo(map);

  // 現在位置マーカー (PONS本体「士」型 人力飛行機アイコン)
  const planeIcon = L.divIcon({
    className: 'plane-marker-icon',
    html: `
          <svg id="plane-marker" class="plane-marker-svg" width="40" height="40" viewBox="-20 -20 40 40" style="transform: rotate(0deg); overflow: visible;">
            <!-- 主翼 (スパン36px) -->
            <rect x="-18" y="-5" width="36" height="4.2" rx="1.5" fill="#facc15" stroke="#0f172a" stroke-width="1.6"/>
            <!-- 胴体 (全長28px) -->
            <rect x="-2.2" y="-14" width="4.4" height="28" rx="2" fill="#facc15" stroke="#0f172a" stroke-width="1.6"/>
            <!-- 水平尾翼 (スパン15px) -->
            <rect x="-7.5" y="9" width="15" height="3.2" rx="1" fill="#facc15" stroke="#0f172a" stroke-width="1.4"/>
            <!-- 機首ピトー管/前方基準点 (赤ポインタ) -->
            <circle cx="0" cy="-14" r="2.2" fill="#dc2626" stroke="#ffffff" stroke-width="0.8"/>
          </svg>
        `,
    iconSize: [40, 40],
    iconAnchor: [20, 20]
  });
  currentMarker = L.marker([TARGETS.PLATFORM.lat, TARGETS.PLATFORM.lon], { icon: planeIcon }).addTo(map);

  // 受信ログ用ボートマーカー＆通信リンク線
  const boatIcon = L.divIcon({
    className: 'boat-marker-icon',
    html: `
          <svg id="boat-marker" class="boat-marker-svg" width="28" height="28" viewBox="-14 -14 28 28" style="transform: rotate(0deg); overflow: visible;" title="追尾ボート (LoRa受信機)">
            <!-- 船体 (前が尖ったボート平面形) -->
            <path d="M 0,-11 C 5.5,-6 6.5,5 5,11 L -5,11 C -6.5,5 -5.5,-6 0,-11 Z" fill="#ea580c" stroke="#0f172a" stroke-width="1.6"/>
            <!-- 操舵席 / キャビン -->
            <rect x="-3" y="1" width="6" height="5.5" rx="1" fill="#ffffff" stroke="#0f172a" stroke-width="1"/>
            <!-- 船首ポインタ -->
            <circle cx="0" cy="-10" r="1.5" fill="#ffffff"/>
          </svg>
        `,
    iconSize: [28, 28],
    iconAnchor: [14, 14]
  });
  boatMarker = L.marker([TARGETS.PLATFORM.lat, TARGETS.PLATFORM.lon], { icon: boatIcon });
  linkPolyline = L.polyline([], { color: '#ea580c', weight: 2.0, dashArray: '4, 4', opacity: 0.8 });

  mapInitialized = true;
}

// ============================================================
// CSV パース処理
// ============================================================
function readCSVFile(file) {
  if (!file) return;
  // 暗号化した大会ログ（.enc）はパスワードを聞いて開く（olddata_open.js）
  if (/\.enc$/i.test(file.name) && typeof openEncryptedFile === 'function') { openEncryptedFile(file); return; }
  const reader = new FileReader();
  reader.onload = (e) => {
    // log.txt（.txt）は起動ごとの記録の表示へ。それ以外は飛行 CSV として再生する
    if (/\.txt$/i.test(file.name)) openLogViewer(e.target.result, file.name, {});
    else parseCSVText(e.target.result, file.name);
  };
  reader.readAsText(file);
}

function saveCurrentCSV() {
  if (!currentCsvText || flightData.length === 0) {
    alert("保存できるCSVデータがありません。先にCSVファイルを読み込んでください。");
    return;
  }
  // ファイル名の整形（パスが含まれる場合はファイル名部分のみ取得し、拡張子 .csv を保証）
  let baseName = (currentCsvFilename || "flight_log.csv").split('/').pop().trim();
  if (!baseName) baseName = "flight_log.csv";
  if (!baseName.toLowerCase().endsWith('.csv')) {
    baseName += ".csv";
  }

  const blob = new Blob([currentCsvText], { type: "text/csv;charset=utf-8;" });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = baseName;
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

function parseCSVText(text, filename = "フライトログ") {
  const lines = text.split(/\r?\n/).filter(line => line.trim().length > 0);
  if (lines.length < 2) {
    alert("有効なCSVデータが見つかりませんでした。");
    return;
  }

  // ヘッダー行の解析
  const header = lines[0].split(',').map(h => h.trim().toLowerCase());
  const col = {
    lat: header.indexOf('latitude'),
    lon: header.indexOf('longitude'),
    gs: header.indexOf('gs'),
    tt: header.indexOf('truetrack'),
    gnss_alt: header.indexOf('gnss_altitude') >= 0 ? header.indexOf('gnss_altitude') : header.indexOf('altitude'),
    kf_alt: header.indexOf('kf_altitude') >= 0 ? header.indexOf('kf_altitude') : (header.indexOf('kf_alt') >= 0 ? header.indexOf('kf_alt') : header.indexOf('altitude')),
    kf_vs: header.indexOf('kf_vspeed') >= 0 ? header.indexOf('kf_vspeed') : (header.indexOf('kf_vs') >= 0 ? header.indexOf('kf_vs') : header.indexOf('vspeed')),
    press: header.indexOf('pressure'),
    roll: header.indexOf('roll'),
    pitch: header.indexOf('pitch'),
    yaw: header.indexOf('yaw'),
    pavg: header.indexOf('pitch_avg'),
    rtrim: header.indexOf('roll_trim'),
    wspd: header.indexOf('wind_mps'),
    wdir: header.indexOf('wind_dir'),
    beta: header.indexOf('beta_air') >= 0 ? header.indexOf('beta_air') : (header.indexOf('beta') >= 0 ? header.indexOf('beta') : (header.indexOf('est_beta') >= 0 ? header.indexOf('est_beta') : -1)),
    // ファームの列名は tas_mps（mysd.cpp）。header は配列なので indexOf は完全一致で、'tas' では見つからない。
    tas: header.indexOf('tas_mps') >= 0 ? header.indexOf('tas_mps') : header.indexOf('tas'),
    time: header.indexOf('time'),
    date: header.indexOf('date'),
    volt: header.indexOf('voltage'),
    sats: header.indexOf('numsat'),
    lux: header.indexOf('lux'),
    backlight: header.indexOf('backlight'),
    rssi: header.indexOf('rssi'),
    seq: header.indexOf('seq'),
    age_ms: header.indexOf('age_ms'),
    yaw_acc95: header.indexOf('yaw_acc95'),
    own_lat: header.indexOf('own_lat'),
    own_lon: header.indexOf('own_lon'),
    dist_m: header.indexOf('dist_m'),
    slot: header.indexOf('slot')
  };

  // 自機フライトログかボート受信ログかを自動判別
  isReceivedLog = (col.rssi >= 0 || col.dist_m >= 0 || col.own_lat >= 0 || filename.toLowerCase().includes('received'));

  if (col.lat < 0 || col.lon < 0) {
    alert("エラー: latitude または longitude 列が存在しません。");
    return;
  }

  flightData = [];
  let t0_sec = null;

  for (let i = 1; i < lines.length; i++) {
    const parts = lines[i].split(',').map(p => p.trim());
    if (parts.length < header.length / 2) continue;

    const lat = parseFloat(parts[col.lat]);
    const lon = parseFloat(parts[col.lon]);
    if (isNaN(lat) || isNaN(lon) || (lat === 0 && lon === 0)) continue;

    // 時刻・日付パース（日跨ぎ対応）
    let timeStr = col.time >= 0 ? parts[col.time] : `00:00:${i}`;
    let dateStr = col.date >= 0 ? parts[col.date] : "";
    let sec = parseTimeToSeconds(timeStr);
    if (t0_sec === null) t0_sec = sec;
    let rel_sec = sec - t0_sec;
    if (rel_sec < -43200) rel_sec += 86400;

    const gs = (col.gs >= 0 && parts[col.gs] !== '') ? parseFloat(parts[col.gs]) : null;
    const tt = (col.tt >= 0 && parts[col.tt] !== '') ? parseFloat(parts[col.tt]) : null;
    let kf_alt = null;
    if (col.kf_alt >= 0 && parts[col.kf_alt] !== '') kf_alt = parseFloat(parts[col.kf_alt]);
    else if (col.gnss_alt >= 0 && parts[col.gnss_alt] !== '') kf_alt = parseFloat(parts[col.gnss_alt]);
    const kf_vs = (col.kf_vs >= 0 && parts[col.kf_vs] !== '') ? parseFloat(parts[col.kf_vs]) : null;
    const press = (col.press >= 0 && parts[col.press] !== '') ? parseFloat(parts[col.press]) : null;

    // 姿勢・風・横滑り (データ欠損時は 0 ではなく null で保持)
    const roll = (col.roll >= 0 && parts[col.roll] !== '') ? parseFloat(parts[col.roll]) : null;
    const pitch = (col.pitch >= 0 && parts[col.pitch] !== '') ? parseFloat(parts[col.pitch]) : null;
    const yawFromCsv = col.yaw >= 0 && parts[col.yaw] !== '';
    const yaw = yawFromCsv ? parseFloat(parts[col.yaw]) : (tt !== null ? tt : null);   // 方位が無い行は航跡で代用（描画用）
    const pavg = (col.pavg >= 0 && parts[col.pavg] !== '') ? parseFloat(parts[col.pavg]) : null;
    const rtrim = (col.rtrim >= 0 && parts[col.rtrim] !== '') ? parseFloat(parts[col.rtrim]) : null;
    const yaw_acc95 = (col.yaw_acc95 >= 0 && parts[col.yaw_acc95] !== '') ? parseFloat(parts[col.yaw_acc95]) : null;
    const wspd = (col.wspd >= 0 && parts[col.wspd] !== '') ? parseFloat(parts[col.wspd]) : null;
    const wdir = (col.wdir >= 0 && parts[col.wdir] !== '') ? parseFloat(parts[col.wdir]) : null;
    const csvBeta = (col.beta >= 0 && parts[col.beta] !== '') ? parseFloat(parts[col.beta]) : null;
    const csvTas = (col.tas >= 0 && parts[col.tas] !== '') ? parseFloat(parts[col.tas]) : null;

    // システム・自機特有
    const volt = (col.volt >= 0 && parts[col.volt] !== '') ? parseFloat(parts[col.volt]) : null;
    const sats = (col.sats >= 0 && parts[col.sats] !== '') ? parseInt(parts[col.sats]) : null;
    const lux = (col.lux >= 0 && parts[col.lux] !== '') ? parseInt(parts[col.lux]) : null;
    const backlight = (col.backlight >= 0 && parts[col.backlight] !== '') ? parseInt(parts[col.backlight]) : null;

    // 受信ログ特有 (無線通信・ボート位置)
    const rssi = (col.rssi >= 0 && parts[col.rssi] !== '') ? parseFloat(parts[col.rssi]) : null;
    const age_ms = (col.age_ms >= 0 && parts[col.age_ms] !== '') ? parseInt(parts[col.age_ms]) : null;
    const own_lat = (col.own_lat >= 0 && parts[col.own_lat] !== '') ? parseFloat(parts[col.own_lat]) : null;
    const own_lon = (col.own_lon >= 0 && parts[col.own_lon] !== '') ? parseFloat(parts[col.own_lon]) : null;
    const dist_m = (col.dist_m >= 0 && parts[col.dist_m] !== '') ? parseFloat(parts[col.dist_m]) : null;

    flightData.push({
      t: rel_sec,
      timeStr, dateStr,
      lat, lon, gs, tt,
      kf_alt, kf_vs, press,
      roll, pitch, yaw, yawIsTrack: !yawFromCsv, pavg, rtrim, yaw_acc95,
      wspd, wdir,
      csvBeta, csvTas,
      estBeta: (csvBeta !== null && !isNaN(csvBeta)) ? csvBeta : null,
      volt, sats, lux, backlight,
      rssi, age_ms, own_lat, own_lon, dist_m
    });
  }

  if (flightData.length === 0) {
    alert("有効なレコードがありませんでした。");
    return;
  }

  // 現在開いているCSVテキストとファイル名を保持し、保存ボタンを有効化
  currentCsvText = text;
  currentCsvFilename = filename;
  const saveBtn = document.getElementById('btn-save-csv');
  if (saveBtn) {
    saveBtn.disabled = false;
    saveBtn.style.opacity = '1';
    saveBtn.style.cursor = 'pointer';
  }
  const anBtn = document.getElementById('btn-open-analysis');
  if (anBtn) {
    anBtn.disabled = false;
    anBtn.style.opacity = '1';
    anBtn.style.cursor = 'pointer';
  }
  if (typeof resetAnalysisCache === 'function') resetAnalysisCache();

  // 相対時刻でソート
  flightData.sort((a, b) => a.t - b.t);
  // ★ 先頭を 0 にそろえる。時刻は 1 行目からの差なので、1 行目より前の時刻の行があると t が負になり、
  //   シークバーと長さ（最後の t）がその分ずれていた。
  const tMin = flightData[0].t;
  if (tMin !== 0) for (const d of flightData) d.t -= tMin;
  totalDurationSec = flightData[flightData.length - 1].t;
  // 時間のバーに時刻（JST の何時何分何秒）を出すための起点。time 列が無い CSV は経過時間のまま
  csvClockBaseSec = (col.time >= 0) ? parseTimeToSeconds(flightData[0].timeStr) : null;
  if (!Number.isFinite(csvClockBaseSec)) csvClockBaseSec = null;

  // 横滑り角 (Est.Beta) の自動推定
  estimateSideslipBeta(flightData);

  // 高度範囲の自動計算 (フライトデータに完全追従するオートスケール)
  let minA = Infinity, maxA = -Infinity;
  for (const d of flightData) {
    if (d.kf_alt !== null && !isNaN(d.kf_alt)) {
      if (d.kf_alt < minA) minA = d.kf_alt;
      if (d.kf_alt > maxA) maxA = d.kf_alt;
    }
  }
  if (!isFinite(minA) || !isFinite(maxA)) {
    minA = 0;
    maxA = 10;
  }
  flightAltMin = Math.floor(minA - 1);
  flightAltMax = Math.ceil(maxA + 1);
  if (flightAltMax - flightAltMin < 6) {
    flightAltMax = flightAltMin + 6;
  }

  // VSIの自動補完 (もしCSVにVSI列がなく全て0の場合、高度差分から算出)
  let hasVsi = flightData.some(d => d.kf_vs !== null && Math.abs(d.kf_vs) > 0.05);
  if (!hasVsi && flightData.length > 1) {
    for (let i = 1; i < flightData.length; i++) {
      let dt = flightData[i].t - flightData[i - 1].t;
      if (dt > 0.05 && flightData[i].kf_alt !== null && flightData[i - 1].kf_alt !== null) {
        flightData[i].kf_vs = (flightData[i].kf_alt - flightData[i - 1].kf_alt) / dt;
      }
    }
  }

  // 離陸・着水の自動検出 (G/S > 2.0 m/s の区間)
  // ★ 離陸は 2.0（2026-11-22〜。それまで 3.0）。強い向かい風では対地 2〜3 m/s で飛ぶことがあり、離陸が見つからなかった。
  //   本体の「飛んでいる」の門（settings.h の TAS_AIR_GS_MPS など）と同じ。
  let firstFlyIdx = flightData.findIndex(d => d.gs > 2.0);
  // findLastIndex が無いブラウザでも同じ結果にする（以前は代わりに最後の行を使い、着水がいつもファイルの終わりだった）
  let lastFlyIdx = -1;
  for (let k = flightData.length - 1; k >= 0; k--) { if (flightData[k].gs > 2.0) { lastFlyIdx = k; break; } }
  takeoffSec = firstFlyIdx >= 0 ? flightData[firstFlyIdx].t : 0;
  landSec = lastFlyIdx >= 0 ? flightData[lastFlyIdx].t : totalDurationSec;

  const typeTag = isReceivedLog
    ? '<span class="badge" style="background:#ea580c; margin-left:8px;">📡 ボート受信ログ (LoRa)</span>'
    : '<span class="badge" style="background:#0284c7; margin-left:8px;">✈️ 自機フライトログ</span>';
  document.getElementById('flight-title').innerHTML = `${escapeHtml(filename)} <span style="font-size:0.85rem; font-weight:normal; color:var(--text-sub);">(${formatTime(totalDurationSec)})</span> ${typeTag}`;
  const flightOnly = document.getElementById('check-flight-only')?.checked;
  currentTimeSec = flightOnly ? (takeoffSec > 0 ? takeoffSec : getFirstFlightTime()) : 0;

  // 地図の航跡更新
  currentPlaneTransformYaw = null;
  currentBoatTransformHeading = null;
  updateMapTrajectory();
  updateSeekBarSegments();

  // UI初期更新
  updateUI(currentTimeSec);
  console.log(`CSV読込完了: ${flightData.length}点, 全長 ${totalDurationSec.toFixed(1)}s, 離陸 ${takeoffSec.toFixed(1)}s, 着水 ${landSec.toFixed(1)}s`);
  // 大会動画の対応（video_sync.js）。このログに動画があれば小窓を出し、無ければ閉じる
  if (typeof window.onFlightCsvLoaded === 'function') window.onFlightCsvLoaded(filename);
  // 大会ログのデータの注意（olddata_open.js）など、読み終えたことを知りたいものへ
  try { document.dispatchEvent(new CustomEvent('pons-csv-loaded', { detail: { filename } })); } catch (e) { /* 古いブラウザ */ }
}

// ============================================================
// 横滑り角 (Est.Beta) 自動推定エンジン
// 運動方程式: d(Beta)/dt = -Beta/TAU + GAIN * (-r + (g/TAS)*cos(pitch)*sin(roll))
// ============================================================
function estimateSideslipBeta(records) {
  if (!records || records.length === 0) return;

  // CSVに既にbeta列が含まれている場合はそちらを優先
  const hasCsvBeta = records.some(r => r.csvBeta !== null && !isNaN(r.csvBeta));
  if (hasCsvBeta) {
    for (let i = 0; i < records.length; i++) {
      records[i].estBeta = (records[i].csvBeta !== null && !isNaN(records[i].csvBeta)) ? records[i].csvBeta : null;
    }
    return;
  }

  // 姿勢データ（roll, pitch）が存在するか確認。存在しない場合は横滑り角を推定できないため null
  const hasAttitude = records.some(r => r.roll !== null && !isNaN(r.roll) && r.pitch !== null && !isNaN(r.pitch));
  if (!hasAttitude) {
    for (let i = 0; i < records.length; i++) {
      records[i].estBeta = null;
    }
    return;
  }

  const GRAVITY = 9.80; // [m/s^2]
  // ★ TAU・GAIN は 2026 年の機体（陽還）の大会ログに合わせた値（2026-11-22。それまでは 5.0 / 0.25 の仮の値）。
  //   flight_preprocess.py（YouTube の動画の後処理）が風の三角形から求めた β に合わせ込んだ運動方程式の β と、
  //   この式（0.5 秒の行・角速度はならさない）で比べて最良: TAU 5.3 / GAIN 0.73 で RMS 0.70°・相関 0.98（5.0 / 0.25 では
  //   振れ幅が約 1/3 で RMS 2.5°）。2027 年の機体も同じような特性とみてこの値にした（ユーザーの判断）。角速度を 1〜2 秒で
  //   ならしても良くならなかった。CSV に β の列（beta_air など）があれば、この推定は使わずそちらを出す。
  const TAU = 5.3;      // 横滑り減衰時定数 [s] (方向安定性)
  const GAIN = 0.73;    // 協調操作度合い (1.0=無操舵、小さいほど協調が良い)

  // ヨー角の360°アンラップ
  const unwrappedYaw = [];
  let prevY = records[0].yaw ?? 0;
  unwrappedYaw.push(prevY);
  for (let i = 1; i < records.length; i++) {
    const yVal = records[i].yaw !== null ? records[i].yaw : prevY;
    // ★ 正の剰余にそろえてから [-180, 180) にする（2026-11-14、BUG-128）。prevY は巻き戻した累積角なので、右旋回で北を
    //   2 回またぐと yVal - prevY + 540 が負になり、JavaScript の % が負を返して 1 歩で -360° 跳んでいた（Est.Beta が張り付く）
    let diff = ((((yVal - prevY) % 360) + 540) % 360) - 180;
    prevY += diff;
    unwrappedYaw.push(prevY);
  }

  let betaRad = 0.0;
  records[0].estBeta = null;

  for (let i = 1; i < records.length; i++) {
    const r0 = records[i - 1];
    const r1 = records[i];
    const dt = Math.max(0.01, r1.t - r0.t);

    // 姿勢データが欠損、または地上滑走前・着水後 (gs < 2.0 m/s) は横滑り角未定義 (null)
    // ★ CSV に方位（yaw）が無い行も出さない（2026-11-22）。その行の yaw は航跡（TrueTrack）で代用しているので、
    //   ヨー角速度が航跡の変わる速さになる。航跡と機首方位は偏流角（風で 1〜3 m/s・TAS 7 m/s なら旋回中に数十度動く）だけ違い、
    //   方位の無い 2025 年の大会ログや、方位が決まる前の自機のログで、もっともらしいが根拠の無い β が出ていた。
    if (r1.roll === null || r1.pitch === null || r1.gs === null || r1.gs < 2.0 ||
        r1.yawIsTrack || r0.yawIsTrack || r1.yaw === null || isNaN(r1.yaw) || r0.yaw === null || isNaN(r0.yaw)) {
      betaRad = 0.0;
      r1.estBeta = null;
      continue;
    }

    const phi = (r1.roll * Math.PI) / 180;
    const theta = (r1.pitch * Math.PI) / 180;

    const p0 = r0.pitch !== null ? r0.pitch : r1.pitch;
    const thetaDot = ((r1.pitch - p0) * Math.PI / 180) / dt;
    const psiDot = ((unwrappedYaw[i] - unwrappedYaw[i - 1]) * Math.PI / 180) / dt;

    // 機体座標系のヨー角速度 r [rad/s]
    const r_body = -thetaDot * Math.sin(phi) + psiDot * Math.cos(theta) * Math.cos(phi);

    // 推定TAS [m/s]
    const wspd = r1.wspd ?? 0;
    const wdir = r1.wdir ?? 0;
    const yaw = r1.yaw ?? 0;
    let tas = (r1.csvTas !== null && r1.csvTas > 0) ? r1.csvTas :
              Math.max(1.0, r1.gs - Math.cos((yaw - wdir) * Math.PI / 180) * wspd);

    // 旋回・バンクによる横滑り駆動項 drive [rad/s]
    // 協調旋回なら drive = 0 となり Beta は減衰
    const drive = -r_body + (GRAVITY / Math.max(tas, 1.0)) * Math.cos(theta) * Math.sin(phi);

    // 指数積分 (厳密解)
    const decay = Math.exp(-dt / TAU);
    const step = GAIN * TAU * (1.0 - decay);
    betaRad = betaRad * decay + step * drive;

    // 度数法へ変換 (±15°クリップ)
    let betaDeg = (betaRad * 180) / Math.PI;
    betaDeg = Math.max(-15.0, Math.min(15.0, betaDeg));
    r1.estBeta = betaDeg;
  }
}

function parseTimeToSeconds(tStr) {
  if (!tStr) return 0;
  const parts = tStr.split(':');
  if (parts.length === 3) {
    return parseInt(parts[0]) * 3600 + parseInt(parts[1]) * 60 + parseFloat(parts[2]);
  }
  return parseFloat(tStr) || 0;
}

// ============================================================
// サンプルフライト生成 (琵琶湖 プラットホーム〜北旋回〜着水)
// ============================================================
function loadSampleFlight() {
  const rows = ["latitude,longitude,gs,TrueTrack,GNSS_Altitude,KF_Altitude,KF_Vspeed,pressure,voltage,numsat,date,time,lux,backlight,roll,pitch,yaw,pitch_avg,roll_trim,yaw_acc95,wind_mps,wind_dir"];
  const baseLat = TARGETS.PLATFORM.lat;
  const baseLon = TARGETS.PLATFORM.lon;
  const N = 400; // 400サンプル (約200秒)
  const dt = 0.5;

  let curLat = baseLat;
  let curLon = baseLon;
  let curAlt = 10.0; // プラットホーム高さ 10m
  let prevAlt = 10.0;
  let curYaw = 328;  // 北パイロン方向 (北北西 328度)

  for (let i = 0; i < N; i++) {
    const sec = i * dt;
    const hh = String(Math.floor(sec / 3600) + 8).padStart(2, '0');
    const mm = String(Math.floor((sec % 3600) / 60) + 21).padStart(2, '0');
    const ss = (sec % 60).toFixed(2).padStart(5, '0');
    const timeStr = `${hh}:${mm}:${ss}`;

    prevAlt = curAlt;

    // 助走〜離陸〜巡航〜北パイロン旋回〜降下
    let gs = 0, roll = 0, pitch = 0, wind_mps = 1.8, wind_dir = 280;

    if (sec < 10) {
      // プラットホーム上待機
      gs = 0;
      curAlt = 10.0;
    } else if (sec < 20) {
      // 発進・飛び出し
      gs = (sec - 10) * 0.7;
      curAlt = 10.0 - (sec - 10) * 0.4; // 飛び出しの沈み込み
    } else if (sec < 95) {
      // 北上巡航 (10kmコース往路: 北パイロンへ向かう)
      gs = 7.5 + Math.sin(sec * 0.2) * 0.4;
      curYaw = 328;
      curAlt = 6.5 + Math.sin(sec * 0.1) * 0.8;
      roll = Math.sin(sec * 0.4) * 2.0;
      pitch = Math.sin(sec * 0.2) * 1.5;
    } else if (sec < 145) {
      // 北パイロン旋回 (左旋回 180度)
      gs = 6.8;
      curYaw = (328 - (sec - 95) * 3.6 + 360) % 360; // 50秒で180度回る
      roll = -5.5 + Math.sin(sec * 0.5) * 1.0;       // 左バンク
      pitch = 0.5;
      curAlt = 5.0 + Math.sin(sec * 0.15) * 0.5;
    } else if (sec < 185) {
      // 復路南下 (南東方向 148度へ戻る)
      gs = 7.8;
      curYaw = 148;
      roll = Math.sin(sec * 0.3) * 1.5;
      pitch = -0.2;
      curAlt = 4.0 - (sec - 145) * 0.08; // 徐々に降下
    } else {
      // 着水
      gs = Math.max(0, 7.0 - (sec - 185) * 1.5);
      curAlt = Math.max(0, 0.5 - (sec - 185) * 0.1);
      roll = 0;
      pitch = 3.0; // 着水フレア
    }

    // 昇降速度 VSI の計算 [m/s]
    let vs = sec > 0 ? (curAlt - prevAlt) / dt : 0;

    // 位置の積分
    if (gs > 0) {
      const rad = (curYaw * Math.PI) / 180;
      curLat += (Math.cos(rad) * gs * dt) / 111000;
      curLon += (Math.sin(rad) * gs * dt) / (111000 * Math.cos(curLat * Math.PI / 180));
    }

    const press = 1013.25 - curAlt * 0.12;
    const volt = (4.15 - (sec / N) * 0.3).toFixed(2);
    rows.push(`${curLat.toFixed(6)},${curLon.toFixed(6)},${gs.toFixed(2)},${curYaw.toFixed(1)},${curAlt.toFixed(2)},${curAlt.toFixed(2)},${vs.toFixed(2)},${press.toFixed(2)},${volt},14,2026-07-26,${timeStr},3200.0,80,${roll.toFixed(2)},${pitch.toFixed(2)},${curYaw.toFixed(2)},0.20,-0.10,1.2,${wind_mps},${wind_dir}`);
  }

  parseCSVText(rows.join('\n'), "琵琶湖デモフライト_2026.csv");
}

// ============================================================
// サンプル受信ログ生成 (ボート側 PONS が受信した LoRa ログ)
// ============================================================
function loadSampleRxFlight() {
  const rows = ["latitude,longitude,gs,TrueTrack,GNSS_Altitude,KF_Altitude,KF_Vspeed,pressure,voltage,numsat,date,time,rssi,seq,age_ms,roll,pitch,yaw,pitch_avg,roll_trim,yaw_acc95,wind_mps,wind_dir,own_lat,own_lon,dist_m,slot"];
  const baseLat = TARGETS.PLATFORM.lat;
  const baseLon = TARGETS.PLATFORM.lon;
  const N = 400;
  const dt = 0.5;

  let curLat = baseLat;
  let curLon = baseLon;
  let curAlt = 10.0;
  let prevAlt = 10.0;
  let curYaw = 328;

  for (let i = 0; i < N; i++) {
    const sec = i * dt;
    const hh = String(Math.floor(sec / 3600) + 8).padStart(2, '0');
    const mm = String(Math.floor((sec % 3600) / 60) + 21).padStart(2, '0');
    const ss = (sec % 60).toFixed(2).padStart(5, '0');
    const timeStr = `${hh}:${mm}:${ss}`;

    prevAlt = curAlt;

    let gs = 0, roll = 0, pitch = 0, wind_mps = 1.8, wind_dir = 280;

    if (sec < 10) {
      gs = 0;
      curAlt = 10.0;
    } else if (sec < 20) {
      gs = (sec - 10) * 0.7;
      curAlt = 10.0 - (sec - 10) * 0.4;
    } else if (sec < 95) {
      gs = 7.5 + Math.sin(sec * 0.2) * 0.4;
      curYaw = 328;
      curAlt = 6.5 + Math.sin(sec * 0.1) * 0.8;
      roll = Math.sin(sec * 0.4) * 2.0;
      pitch = Math.sin(sec * 0.2) * 1.5;
    } else if (sec < 145) {
      gs = 6.8;
      curYaw = (328 - (sec - 95) * 3.6 + 360) % 360;
      roll = -5.5 + Math.sin(sec * 0.5) * 1.0;
      pitch = 0.5;
      curAlt = 5.0 + Math.sin(sec * 0.15) * 0.5;
    } else if (sec < 185) {
      gs = 7.8;
      curYaw = 148;
      roll = Math.sin(sec * 0.3) * 1.5;
      pitch = -0.2;
      curAlt = 4.0 - (sec - 145) * 0.08;
    } else {
      gs = Math.max(0, 7.0 - (sec - 185) * 1.5);
      curAlt = Math.max(0, 0.5 - (sec - 185) * 0.1);
      roll = 0;
      pitch = 3.0;
    }

    let vs = sec > 0 ? (curAlt - prevAlt) / dt : 0;

    if (gs > 0) {
      const rad = (curYaw * Math.PI) / 180;
      curLat += (Math.cos(rad) * gs * dt) / 111000;
      curLon += (Math.sin(rad) * gs * dt) / (111000 * Math.cos(curLat * Math.PI / 180));
    }

    // ボート（追尾艇）の位置: 機体の後方約 60〜85m を追従
    const boatLagDist = 72.0 + Math.sin(sec * 0.12) * 14.0;
    const boatRad = ((curYaw + 180) * Math.PI) / 180;
    const own_lat = curLat + (Math.cos(boatRad) * boatLagDist) / 111000;
    const own_lon = curLon + (Math.sin(boatRad) * boatLagDist) / (111000 * Math.cos(curLat * Math.PI / 180));
    const dist_m = boatLagDist;

    // 通信ステータス
    const rssi = Math.round(-74 - (dist_m / 80.0) * 8 + (Math.sin(sec * 0.7) * 3));
    const age_ms = Math.round(180 + Math.random() * 40);
    const seq = (i % 256);

    const press = 1013.25 - curAlt * 0.12;
    const volt = (4.15 - (sec / N) * 0.3).toFixed(2);

    rows.push(`${curLat.toFixed(6)},${curLon.toFixed(6)},${gs.toFixed(2)},${curYaw.toFixed(1)},${curAlt.toFixed(2)},${curAlt.toFixed(2)},${vs.toFixed(2)},${press.toFixed(2)},${volt},14,2026-07-26,${timeStr},${rssi},${seq},${age_ms},${roll.toFixed(2)},${pitch.toFixed(2)},${curYaw.toFixed(2)},0.20,-0.10,1.2,${wind_mps},${wind_dir},${own_lat.toFixed(6)},${own_lon.toFixed(6)},${dist_m.toFixed(1)},0`);
  }

  // ★ 名前に大会ログの形（2026-07-26_0804 など日付の入った名前）を使わない。大会の注意（olddata_open.js の notesKey）は
  //   名前の頭、大会動画（video_sync.js の findEntry）は名前の日付で大会ログと見なすので、デモに 2026 年大会の注意と動画が付いた
  //   （2026-11-25）。受信ログの判定は rssi などの列で付く。
  parseCSVText(rows.join('\n'), "琵琶湖デモ受信_2026_boat.csv");
}

// ============================================================
// データ補間エンジン
// ============================================================
function interpolateAt(t) {
  if (flightData.length === 0) return null;
  if (t <= flightData[0].t) return flightData[0];
  if (t >= flightData[flightData.length - 1].t) return flightData[flightData.length - 1];

  // 2分探索
  let low = 0, high = flightData.length - 1;
  while (low <= high) {
    let mid = Math.floor((low + high) / 2);
    if (flightData[mid].t < t) low = mid + 1;
    else high = mid - 1;
  }

  let i0 = Math.max(0, low - 1);
  let i1 = Math.min(flightData.length - 1, low);
  if (i0 === i1) return flightData[i0];

  let d0 = flightData[i0], d1 = flightData[i1];
  let frac = (t - d0.t) / (d1.t - d0.t);

  // 角度の補間 (360度跨ぎ対応、null 対応)
  function lerpAngle(a0, a1, f) {
    if (a0 === null && a1 === null) return null;
    if (a0 === null) return a1;
    if (a1 === null) return a0;
    let diff = (a1 - a0 + 540) % 360 - 180;
    return (a0 + diff * f + 360) % 360;
  }

  function lerp(v0, v1, f) {
    if (v0 === null && v1 === null) return null;
    if (v0 === null) return v1;
    if (v1 === null) return v0;
    return v0 + (v1 - v0) * f;
  }

  return {
    t,
    timeStr: d0.timeStr,
    lat: lerp(d0.lat, d1.lat, frac),
    lon: lerp(d0.lon, d1.lon, frac),
    gs: lerp(d0.gs, d1.gs, frac),
    tt: lerpAngle(d0.tt, d1.tt, frac),
    kf_alt: lerp(d0.kf_alt, d1.kf_alt, frac),
    kf_vs: lerp(d0.kf_vs, d1.kf_vs, frac),
    press: lerp(d0.press, d1.press, frac),
    roll: lerp(d0.roll, d1.roll, frac),
    pitch: lerp(d0.pitch, d1.pitch, frac),
    yaw: lerpAngle(d0.yaw, d1.yaw, frac),
    pavg: lerp(d0.pavg, d1.pavg, frac),
    rtrim: lerp(d0.rtrim, d1.rtrim, frac),
    yaw_acc95: lerp(d0.yaw_acc95, d1.yaw_acc95, frac),
    wspd: lerp(d0.wspd, d1.wspd, frac),
    wdir: lerpAngle(d0.wdir, d1.wdir, frac),
    csvBeta: d0.csvBeta,
    csvTas: d0.csvTas,
    estBeta: lerp(d0.estBeta, d1.estBeta, frac),
    volt: lerp(d0.volt, d1.volt, frac),
    sats: d0.sats,
    lux: d0.lux,
    backlight: d0.backlight,
    rssi: d0.rssi,
    age_ms: d0.age_ms,
    own_lat: lerp(d0.own_lat, d1.own_lat, frac),
    own_lon: lerp(d0.own_lon, d1.own_lon, frac),
    dist_m: lerp(d0.dist_m, d1.dist_m, frac)
  };
}

// ============================================================
// アニメーション＆UI更新ループ
// ============================================================
function animationLoop(timestamp) {
  if (!lastAnimFrameTime) lastAnimFrameTime = timestamp;
  const dt = (timestamp - lastAnimFrameTime) / 1000;
  lastAnimFrameTime = timestamp;

  // ★ 飛行解析を開いている間は再生を進めず、裏の地図・HUD・グラフも描かない（2026-11-20。見えない所を毎フレーム描いて
  //   解析画面のスクロールがかくついていた）。閉じれば続きから再生する（dt はここで捨てるので時間は飛ばない）
  const analysisOpen = typeof window.isAnalysisModalOpen === 'function' && window.isAnalysisModalOpen();
  if (isPlaying && totalDurationSec > 0 && !analysisOpen) {
    currentTimeSec += dt * playbackSpeed;

    const flightOnly = document.getElementById('check-flight-only')?.checked;
    if (flightOnly) {
      const curData = interpolateAt(currentTimeSec);
      if (curData && curData.gs < FLIGHT_SPEED_THRESHOLD) {
        const nextT = findNextFlightTime(currentTimeSec);
        if (nextT >= 0) {
          // 待機・静止区間を検知 ➔ 次の飛行区間開始点へ自動スキップ
          currentTimeSec = nextT;
        } else {
          // これ以降に飛行データなし (着水・着陸完了)
          if (document.getElementById('check-loop').checked) {
            currentTimeSec = (takeoffSec > 0 ? takeoffSec : getFirstFlightTime());
          } else {
            currentTimeSec = landSec;
            isPlaying = false;
            document.getElementById('btn-play').textContent = "▶ 再生";
          }
        }
      }
    }

    if (currentTimeSec > totalDurationSec) {
      if (document.getElementById('check-loop').checked) {
        currentTimeSec = flightOnly ? (takeoffSec > 0 ? takeoffSec : getFirstFlightTime()) : 0;
      } else {
        currentTimeSec = totalDurationSec;
        isPlaying = false;
        document.getElementById('btn-play').textContent = "▶ 再生";
      }
    }
    updateUI(currentTimeSec);
  }

  requestAnimationFrame(animationLoop);
}

function updateUI(t) {
  const data = interpolateAt(t);
  if (!data) return;

  // タイムラインスライダー
  const slider = document.getElementById('seek-slider');
  if (document.activeElement !== slider) {
    slider.value = totalDurationSec > 0 ? (t / totalDurationSec) * 1000 : 0;
  }
  // 時刻（何時何分何秒）で出す。経過時間はマウスを載せたときの説明に（2026-11-20）
  const timeEl = document.getElementById('time-display');
  if (csvClockBaseSec !== null) {
    timeEl.textContent = `${formatClock(csvClockBaseSec + t)} / ${formatClock(csvClockBaseSec + totalDurationSec)}`;
    timeEl.title = `経過 ${formatTime(t)} / ${formatTime(totalDurationSec)}`;
  } else {
    timeEl.textContent = `${formatTime(t)} / ${formatTime(totalDurationSec)}`;
    timeEl.title = 'この CSV には時刻の列が無いので、経過時間を出しています';
  }

  // HUD テキスト更新 (データなし時は --- を表示)
  document.getElementById('hud-press').textContent = (data.press !== null && !isNaN(data.press)) ? `${data.press.toFixed(1)} hPa` : "--- hPa";
  document.getElementById('hud-sats').textContent = (data.sats !== null && !isNaN(data.sats)) ? `${data.sats} sats` : "-- sats";
  document.getElementById('hud-volt').textContent = (data.volt !== null && !isNaN(data.volt)) ? `${data.volt.toFixed(2)} V` : "--.- V";

  // 自機ログと受信ログで右下HUDセルを自動切り替え
  const dynLabel = document.getElementById('hud-label-dynamic');
  const dynVal = document.getElementById('hud-lux');
  if (isReceivedLog) {
    if (dynLabel) dynLabel.textContent = "ボート距離 / LoRa電波";
    const distStr = (data.dist_m !== null && !isNaN(data.dist_m)) ? `${data.dist_m.toFixed(0)}m` : "--m";
    const rssiStr = (data.rssi !== null && !isNaN(data.rssi)) ? `${data.rssi.toFixed(0)}dBm` : "--";
    if (dynVal) dynVal.textContent = `${distStr} (${rssiStr})`;
  } else {
    if (dynLabel) dynLabel.textContent = "照度 / 画面輝度";
    const luxStr = (data.lux !== null && !isNaN(data.lux)) ? `${data.lux} lux` : "-- lux";
    const blStr = (data.backlight !== null && !isNaN(data.backlight)) ? ` (${data.backlight}%)` : "";
    if (dynVal) dynVal.textContent = `${luxStr}${blStr}`;
  }

  document.getElementById('chart-val-gs').textContent = (data.gs !== null && !isNaN(data.gs)) ? data.gs.toFixed(1) : "--.-";
  // 推定TAS: CSV値があれば優先、無ければ風と偏流から簡易推計 (データ欠損時は null)
  let tasEst = null;
  if (data.csvTas !== null && !isNaN(data.csvTas)) {
    tasEst = data.csvTas;
  } else if (data.gs !== null && data.yaw !== null && data.wdir !== null && data.wspd !== null) {
    tasEst = Math.max(0, data.gs - (Math.cos((data.yaw - data.wdir) * Math.PI / 180) * data.wspd));
  }
  document.getElementById('chart-val-tas').textContent = (tasEst !== null && !isNaN(tasEst)) ? tasEst.toFixed(1) : "--.-";

  document.getElementById('chart-val-roll').textContent = (data.roll !== null && !isNaN(data.roll)) ? `${data.roll > 0 ? '+' : ''}${data.roll.toFixed(1)}°` : "--.-°";
  document.getElementById('chart-val-pitch').textContent = (data.pitch !== null && !isNaN(data.pitch)) ? `${data.pitch > 0 ? '+' : ''}${data.pitch.toFixed(1)}°` : "--.-°";
  const betaEl = document.getElementById('chart-val-beta');
  if (betaEl) {
    betaEl.textContent = (data.estBeta !== null && !isNaN(data.estBeta)) ? `${data.estBeta > 0 ? '+' : ''}${data.estBeta.toFixed(1)}°` : "--.-°";
  }
  document.getElementById('chart-val-alt').textContent = (data.kf_alt !== null && !isNaN(data.kf_alt)) ? `${data.kf_alt.toFixed(1)}m` : "--.-m";
  document.getElementById('chart-val-vsi').textContent = (data.kf_vs !== null && !isNaN(data.kf_vs)) ? `${data.kf_vs >= 0 ? '+' : ''}${data.kf_vs.toFixed(2)}` : "--.--";

  // 計器Canvas描画
  drawVectorTriangle(data, tasEst);
  drawArtificialHorizon(data);
  drawSpeedChart(t);
  drawRollPitchChart(t);
  drawAltChart(t);

  // 地図マーカー位置更新
  if (mapInitialized && currentMarker) {
    currentMarker.setLatLng([data.lat, data.lon]);
    const el = document.getElementById('plane-marker');
    if (el) {
      const targetYaw = (data.yaw !== null && !isNaN(data.yaw)) ? data.yaw : 0;
      if (currentPlaneTransformYaw === null) {
        currentPlaneTransformYaw = targetYaw;
      } else {
        // 0/360度跨ぎ時に350度回転せず最短角(-180〜+180度)で滑らかに回転
        const diff = ((targetYaw - currentPlaneTransformYaw) % 360 + 540) % 360 - 180;
        currentPlaneTransformYaw += diff;
      }
      el.style.transform = `rotate(${currentPlaneTransformYaw}deg)`;
    }

    // 受信ログ時の追尾ボートマーカー＆機体間リンク線
    if (isReceivedLog && boatMarker && linkPolyline) {
      if (data.own_lat && data.own_lon && !isNaN(data.own_lat) && !isNaN(data.own_lon) && data.own_lat !== 0) {
        if (!map.hasLayer(boatMarker)) boatMarker.addTo(map);
        boatMarker.setLatLng([data.own_lat, data.own_lon]);

        const boatEl = document.getElementById('boat-marker');
        if (boatEl) {
          const dLat = (data.lat - data.own_lat) * 111000;
          const dLon = (data.lon - data.own_lon) * 111000 * Math.cos(data.lat * Math.PI / 180);
          const boatHeading = (Math.atan2(dLon, dLat) * 180 / Math.PI + 360) % 360;
          if (currentBoatTransformHeading === null) {
            currentBoatTransformHeading = boatHeading;
          } else {
            const diff = ((boatHeading - currentBoatTransformHeading) % 360 + 540) % 360 - 180;
            currentBoatTransformHeading += diff;
          }
          boatEl.style.transform = `rotate(${currentBoatTransformHeading}deg)`;
        }

        if (!map.hasLayer(linkPolyline)) linkPolyline.addTo(map);
        linkPolyline.setLatLngs([[data.lat, data.lon], [data.own_lat, data.own_lon]]);
      } else {
        if (map.hasLayer(boatMarker)) map.removeLayer(boatMarker);
        if (map.hasLayer(linkPolyline)) map.removeLayer(linkPolyline);
      }
    } else {
      if (boatMarker && map.hasLayer(boatMarker)) map.removeLayer(boatMarker);
      if (linkPolyline && map.hasLayer(linkPolyline)) map.removeLayer(linkPolyline);
    }
  }
}

// ============================================================
// 地図航跡ライン更新
// ============================================================
function updateMapTrajectory() {
  if (!mapInitialized || flightData.length === 0) return;
  const latlngs = flightData.map(d => [d.lat, d.lon]);
  trailPolyline.setLatLngs(latlngs);

  let bounds = trailPolyline.getBounds();
  if (isReceivedLog) {
    flightData.forEach(d => {
      if (d.own_lat && d.own_lon && d.own_lat !== 0 && !isNaN(d.own_lat) && !isNaN(d.own_lon)) {
        bounds.extend([d.own_lat, d.own_lon]);
      }
    });
  }
  map.fitBounds(bounds, { padding: [30, 30] });
}

// ============================================================
// 再生コントロール
// ============================================================
function togglePlay() {
  isPlaying = !isPlaying;
  document.getElementById('btn-play').textContent = isPlaying ? "⏸ 一時停止" : "▶ 再生";
  if (isPlaying && totalDurationSec > 0) {
    const flightOnly = document.getElementById('check-flight-only')?.checked;
    if (flightOnly) {
      const cur = interpolateAt(currentTimeSec);
      if (cur && cur.gs < FLIGHT_SPEED_THRESHOLD) {
        const nextT = findNextFlightTime(currentTimeSec);
        if (nextT >= 0) {
          currentTimeSec = nextT;
        } else {
          currentTimeSec = (takeoffSec > 0 ? takeoffSec : getFirstFlightTime());
        }
        updateUI(currentTimeSec);
      }
    }
  }
}

function skipTime(deltaSec) {
  if (totalDurationSec === 0) return;
  currentTimeSec = Math.max(0, Math.min(totalDurationSec, currentTimeSec + deltaSec));
  updateUI(currentTimeSec);
}

function jumpToTakeoff() {
  currentTimeSec = takeoffSec;
  updateUI(currentTimeSec);
}

function jumpToLand() {
  currentTimeSec = landSec;
  updateUI(currentTimeSec);
}

function setPlaybackSpeed(s) {
  playbackSpeed = s;
}

function formatTime(totalSec) {
  const m = Math.floor(totalSec / 60);
  const s = Math.floor(totalSec % 60);
  return `${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
}

// 日内秒 → 時刻 HH:MM:SS（起点は csvClockBaseSec = CSV の 1 行目の時刻）。日を跨いだら 0 時に戻す
function formatClock(daySec) {
  const s = ((Math.floor(daySec) % 86400) + 86400) % 86400;
  const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60), ss = s % 60;
  return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(ss).padStart(2, '0')}`;
}

// ============================================================
// 表示レイアウト＆テーマ切り替え
// ============================================================
function toggleTheme(theme) {
  document.body.className = document.body.className.replace(/theme-\w+/g, '');
  if (theme === 'chromakey') document.body.classList.add('theme-chromakey');
}

function toggleLayout(layout) {
  document.body.className = document.body.className.replace(/layout-\w+/g, '');
  if (layout === 'vertical') document.body.classList.add('layout-vertical');
  setTimeout(() => {
    if (map) map.invalidateSize();
    handleResize();
  }, 200);
}

function handleResize() {
  // 計器用 Canvas (正方形 1:1 解像度フィットで楕円化を恒久防止)
  ['canvas-vector', 'canvas-horizon'].forEach(id => {
    const c = document.getElementById(id);
    if (c) {
      const rect = c.getBoundingClientRect();
      const sz = Math.round(Math.min(rect.width, rect.height) || c.clientHeight || c.clientWidth || 100);
      if (sz > 0 && (c.width !== sz || c.height !== sz)) {
        c.width = sz;
        c.height = sz;
      }
    }
  });
  // チャート用 Canvas の解像度フィット
  ['canvas-chart-speed', 'canvas-chart-rp', 'canvas-chart-alt'].forEach(id => {
    const c = document.getElementById(id);
    if (c && c.parentElement) {
      const w = c.parentElement.clientWidth;
      const h = c.parentElement.clientHeight;
      if (w > 0 && h > 0 && (c.width !== w || c.height !== h)) {
        c.width = w;
        c.height = h;
      }
    }
  });
  if (flightData.length > 0) updateUI(currentTimeSec);
}
