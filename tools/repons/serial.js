// ============================================================
// File    : tools/repons/serial.js
// Project : PONS v7.1 / v7.2 Re:PONS
// Role    : Web Serial の接続と PONS とのやりとり（ファイルの一覧・転送・PONS: の行）。
//           形式は docs/pons_webserial_protocol.md
// Author  : MasaoC (@masao_mobile)
// Updated : 2026-11-25
// ============================================================
//
// 2026-11-23 に index.html から分けた。読む順は index.html の一覧どおり。関数・変数はファイルをまたいで
// グローバルで見えるが、読み込みの最中に動くコードは先に読んだファイルのものしか使えない。

// ============================================================
// Web Serial 接続 & PONS ファイル転送
// ============================================================
let serialPort = null;
let serialReader = null;
let serialWriter = null;
let serialReadLoopActive = false;
let ponsReady = false;           // PONS:READY を受けたか（返事の見張り startReadyWatch が見る）
let lastFailedPort = null;       // 開けなかったポート（「もう一度つなぐ」で選び直さずに開き直す）
let ponsFileList = [];

let isDataStreaming = false;
// ★ 転送の状態は 3 つ（2026-11-13、BUG-79）。isDataStreaming は DATA_START から終わりの印まで（受けたバイトをファイルの中身として貯める）。
//   downloadPending は GET / GETTAIL を送ってから DATA_START が来るまで（本体の Core1 がファイルを開いて位置を合わせている間。
//   ここで中断を押しても以前は PONS:CANCEL を送らず、転送がそのまま続いた）。
//   cancelPending は PONS:CANCEL を送ってから本体の CANCEL_OK（またはエラー・DATA_END）が来るまで。この間も受けたバイトは
//   転送の残りとして捨てる。以前は CANCEL を送った瞬間に isDataStreaming を下ろしていて、USB に残っていた中身
//   （log.txt の行や遅れて来た DATA_START）がコマンドの行として読まれ、すぐ次のファイルを押すと本体の後始末とぶつかった。
let downloadPending = false;
let cancelPending = false;
let cancelWatchTimer = null;
let lastSerialRxMs = 0;            // 最後に何か受けた時刻（中断の後始末が終わらない古いファームの見切りに使う）
// ★ 受信はバイトのまま扱う（2026-11-15〜）。それまでは全部を文字列にしていて、生 IMU ログ（imuraw/*.bin）のような
//   バイナリは UTF-8 として読めないバイトが化けて取れなかった。行（コマンドの返事）は改行までを貯めてから文字列にし、
//   ファイルの中身は DATA_START の言う大きさぶんをそのまま dataBytes に置く。
let rxLine = [];                   // 行の途中のバイト（改行が来るまで）
let dataBytes = null;              // 受けているファイルの中身（Uint8Array。大きさは PONS:DATA_START で分かる）
let dataReceived = 0;              // dataBytes に入れたバイト数
let dataTrailerPending = false;    // 中身を受け終えて、PONS:DATA_END の行を待っている
let currentDownloadMode = "open";  // "open" = 開く（再生・記録の表示）/ "save" = 取って PC に保存するだけ
// 一覧（2026-11-15〜、タブ）。"logs" は今までどおり PONS:LIST（飛行 CSV と log.txt）。ほかのタブを開いたときに
// PONS:LIST,ALL（全部のファイル）を頼む。知らないファーム（2026-11-15 より前）は返事をしないので、4 秒で見切る。
let fileTab = "logs";
let listPending = null;            // 頼んでいる一覧（"basic" / "all"）。LIST_END で null
let listIncoming = [];             // 届いている途中の一覧（LIST_END で ponsFileList へ移す）
let listIsAll = false;             // いまの ponsFileList は LIST,ALL のものか
let listAllUnsupported = false;    // LIST,ALL に返事が無かった（古いファーム）
let listAllTimer = null;
let currentDownloadFile = "";
let currentDownloadSize = 0;
let currentDownloadKind = "csv";   // "csv" = 再生する / "log" = log.txt の表示へ（2026-11-11〜）/ "raw" = 保存だけ / "discard" = 頼んでいないので捨てる
let currentDownloadFrom = 0;       // 末尾だけ取ったとき、送り始めた位置 [B]（PONS:DATA_START の 3 つめ）
let currentDownloadFull = 0;       // 一覧に出ていたファイル全体の大きさ [B]
let currentDownloadVia = "get";    // 1 回で取るときの頼み方 "get" / "gettail" / "getimu"（GETIMU が合わなければ範囲を取り直す）
let downloadStartTime = 0;
let lastProgressUpdate = 0;

// ---- 1MB ずつ取って、合わない塊だけ取り直す（PONS:GETRANGE。2026-11-25〜）----
// ★ USB の転送の途中でバイトが抜けることがある（2026-11-25 の調査。生 IMU ログ 4 本とも 0.002〜0.04%、2 B〜11 KB を数か所。
//   SD のファイルは無事。タブを前に出したままでも抜けた）。全部を 1 回で取ると、1 か所の抜けで全部が「欠落・CRC 不一致」になって
//   いたので、ファイル全体を取るとき（GETTAIL・GETIMU でないとき）は塊ごとに CRC を確かめ、合わない塊だけ頼み直す。
//   形式は docs/pons_webserial_protocol.md の (3)。知らない古いファームは何も返さないので、印 PONS:GETRANGE_ACCEPTED が
//   来なければ PONS:GET に戻す（rangeSupport = false。つなぎ直すまで覚える）。
const RANGE_CHUNK_BYTES = 1024 * 1024;   // 1 つの塊 [B]
const RANGE_MAX_TRIES = 5;               // 1 つの塊を取る回数の上限（越えたら取れた分で先へ進み、最後に知らせる）
const RANGE_STALL_MS = 6000;             // 塊の途中でこれだけ何も来なければ、終わりの印が抜けたとみて頼み直す
const RANGE_ACCEPT_WAIT_MS = 5000;       // 最初の塊で受け付けの印を待つ時間（来なければ古いファーム）
let rangeSupport = null;                 // null = この接続でまだ分からない / true / false
let rangeDl = null;                      // 塊ごとに取っている間の状態（startRangeDownload）
let rangeStallTimer = null;

// 標準 CRC32 (IEEE 802.3, poly 0xEDB88320)
// bytes: Uint8Array（文字列を渡したら UTF-8 にして計算する）
function computeCrc32(bytes) {
  if (!computeCrc32.table) {
    const table = new Uint32Array(256);
    for (let i = 0; i < 256; i++) {
      let c = i;
      for (let j = 0; j < 8; j++) {
        c = (c & 1) ? (0xEDB88320 ^ (c >>> 1)) : (c >>> 1);
      }
      table[i] = c >>> 0;
    }
    computeCrc32.table = table;
  }
  let crc = 0xFFFFFFFF;
  if (typeof bytes === 'string') bytes = new TextEncoder().encode(bytes);
  for (let i = 0; i < bytes.length; i++) {
    crc = (crc >>> 8) ^ computeCrc32.table[(crc ^ bytes[i]) & 0xFF];
  }
  crc = (crc ^ 0xFFFFFFFF) >>> 0;
  return crc.toString(16).toUpperCase().padStart(8, '0');
}

function openSerialModal() {
  document.getElementById('serial-modal').classList.add('active');
}

function closeSerialModal() {
  document.getElementById('serial-modal').classList.remove('active');
}

async function sendSerialLine(str) {
  if (!serialWriter) return;
  const encoder = new TextEncoder();
  await serialWriter.write(encoder.encode(str + "\r\n"));
}

async function connectWebSerial() {
  if (serialPort && serialPort.readable) {
    // すでに接続中ならモーダルを開く。
    // ★ 本体はポートが開いている間は PC 接続の画面のまま（2026-11-25〜。それまではダイアログを閉じて PING が止まると
    //   60 秒で地図へ戻り、記録を再開していた）。本体のボタンで抜けていることはあるので、一覧を取り直し、
    //   戻っていれば PONS:ERROR,NOT_CONNECTED → 自動で CONNECT し直す（handleSerialLine）。
    openSerialModal();
    if (!downloadBusy()) requestFileList();
    return;
  }

  if (!('serial' in navigator)) {
    // ★ iPhone・iPad はどのブラウザも中身が WebKit で、Web Serial が無い（Chrome を入れても同じ）。
    //   「Chrome を使ってください」とだけ出すと、iPhone の Chrome で開いている人には意味が通らないので出し分ける。
    //   iPadOS 13 以降の Safari は Mac を名乗るので、タッチの点数で見分ける。2026-11-23〜（ユーザー）
    const ua = navigator.userAgent || '';
    const isIOS = /iPhone|iPad|iPod/.test(ua) || (/Macintosh/.test(ua) && navigator.maxTouchPoints > 1);
    alert(isIOS
      ? "iPhone・iPad では本体に接続できません（どのブラウザも USB の接続〔Web Serial〕に対応していないため）。\nMac・Windows か Android の Chrome を使ってください。"
      : "お使いのブラウザは本体への USB の接続（Web Serial）に対応していません。\nMac・Windows か Android の Chrome を使ってください。");
    return;
  }

  let port;
  try {
    port = await navigator.serial.requestPort();
  } catch (err) {
    // ポートの選択を取り消した（NotFoundError）。何もしない
    console.warn("Serial port not chosen:", err);
    return;
  }
  await openPonsPort(port);
}

// 選んだポートを開いて PONS:CONNECT を送る。
// ★ 開けなかったとき・返事が無いときは、ダイアログに理由と直し方を出す（2026-11-11〜）。それまでは開けないと
//   コンソールに 1 行出すだけで画面には何も出ず、Arduino IDE のシリアルモニタがポートを掴んでいると「押しても何も起きない」だった。
async function openPonsPort(port) {
  try {
    // 受信バッファを 1MB に拡張して大容量転送時の OS/Web Serial 側ドロップを抑制
    // （16KB だと 1MB/s 転送時に約 16ms 分しかなく、GC やタブ切替等で数 KB 欠落しうる）
    await port.open({ baudRate: 115200, bufferSize: 1024 * 1024 });
  } catch (err) {
    console.warn("Serial open failed:", err);
    serialPort = null;
    lastFailedPort = port;
    openSerialModal();
    updateSerialUIStatus('error', {
      badge: 'ポートを開けません',
      msg: 'このポートを開けませんでした。ほかのアプリが使っていると開けません。\n' +
           '・Arduino IDE のシリアルモニタ（下のタブ）やシリアルプロッタを閉じる\n' +
           '・別のタブやウィンドウの Re:PONS で「🔌 切断」を押す\n' +
           '・📡 ライブの窓を使っていれば、その窓の「切断」を押す（開いたままだと、つなぎ直そうとして取り合います）\n' +
           'それから「🔁 もう一度つなぐ」を押してください。\n' +
           `（ブラウザの言葉: ${err && err.name ? err.name + ': ' : ''}${err && err.message ? err.message : err}）`,
      retry: true,
    });
    return;
  }
  serialPort = port;
  lastFailedPort = null;
  ponsReady = false;
  serialWriter = serialPort.writable.getWriter();
  serialReadLoopActive = true;
  rxLine = [];
  endDownloadUi();
  readSerialLoop();

  openSerialModal();
  updateSerialUIStatus('connecting');

  // ハンドシェイク送信（返事が来るまで見張る）
  setTimeout(() => {
    sendSerialLine("PONS:CONNECT");
  }, 200);
  startReadyWatch();
}

// PONS:READY が来るまで 2 秒ごとに PONS:CONNECT を送り直す（つないだ直後で本体がまだ起動中なら、そのうち返事が来る）。
// 6 秒で途中の案内、14 秒で「返事がありません」にする。ポートは開いたままにする（あとで返事が来れば、そのままつながる）。
let readyWatchTimer = null;
function startReadyWatch() {
  stopReadyWatch();
  let tries = 0;
  readyWatchTimer = setInterval(() => {
    if (ponsReady || !serialPort) { stopReadyWatch(); return; }
    tries++;
    if (tries <= 6) sendSerialLine("PONS:CONNECT").catch(() => {});
    if (tries === 3) {
      updateSerialUIStatus('connecting', { msg: 'PONS からの返事を待っています...（起動の途中なら、終わるまで待ちます）' });
    }
    if (tries >= 7) {
      stopReadyWatch();
      updateSerialUIStatus('error', {
        badge: '返事がありません',
        msg: 'ポートは開きましたが、PONS から返事がありません。\n' +
             '・選んだポートが PONS か（Mac では「usbmodem…」、Windows では「COM…」）\n' +
             '・PONS の電源が入っていて、起動し終わっているか\n' +
             '・Arduino IDE のシリアルモニタなど、ほかのアプリが同じポートを開いていないか\n' +
             'を確かめて、「🔁 もう一度つなぐ」を押してください（違うポートなら「🔌 切断」してから選び直す）。',
        retry: true,
      });
    }
  }, 2000);
}
function stopReadyWatch() {
  if (readyWatchTimer) { clearInterval(readyWatchTimer); readyWatchTimer = null; }
}

// ダイアログの「もう一度つなぐ」。開いていれば CONNECT を送り直し、開けなかったポートは選び直さずに開き直す
async function retryWebSerial() {
  if (serialPort && serialPort.readable) {
    updateSerialUIStatus('connecting');
    sendSerialLine("PONS:CONNECT").catch(() => {});
    startReadyWatch();
    return;
  }
  if (lastFailedPort) await openPonsPort(lastFailedPort);
  else await connectWebSerial();
}

async function disconnectWebSerial() {
  stopSerialPing();
  stopReadyWatch();
  ponsReady = false;
  lastFailedPort = null;
  if (!serialPort) {
    closeSerialModal();
    return;
  }
  try {
    if (isDataStreaming || downloadPending) {
      await sendSerialLine("PONS:CANCEL");
    }
    await sendSerialLine("PONS:DISCONNECT");
  } catch (e) {}

  serialReadLoopActive = false;
  endDownloadUi();
  if (serialReader) {
    try { await serialReader.cancel(); } catch (e) {}
  }
  if (serialWriter) {
    try { serialWriter.releaseLock(); } catch (e) {}
    serialWriter = null;
  }
  if (serialPort) {
    try { await serialPort.close(); } catch (e) {}
    serialPort = null;
  }

  updateSerialUIStatus('disconnected');
  resetSettingsState(true);
  if (typeof resetWipeState === 'function') resetWipeState(true);
  if (typeof resetSdInfo === 'function') resetSdInfo();
  closeSerialModal();
}

function updateSerialUIStatus(state, info = {}) {
  const badge = document.getElementById('serial-status-badge');
  const text = document.getElementById('serial-status-text');
  const verText = document.getElementById('serial-board-ver');
  const headerBtn = document.getElementById('btn-webserial');
  // 本体設定と SD のログの消去（sd_wipe.js）は、つながっている間だけ押せる
  for (const id of ['btn-open-settings', 'btn-open-wipe']) {
    const b = document.getElementById(id);
    if (!b) continue;
    const ok = (state === 'connected');
    b.disabled = !ok;
    b.style.opacity = ok ? '1' : '0.45';
    b.style.cursor = ok ? 'pointer' : 'not-allowed';
  }
  text.style.whiteSpace = 'pre-line';   // 案内は複数行
  text.style.color = '';

  if (state === 'connected') {
    badge.style.background = '#dcfce7';
    badge.style.color = '#15803d';
    badge.textContent = '接続完了';
    text.textContent = 'PONS本体と通信確立中 (SD保護モード有効)';
    verText.textContent = `${info.board || 'PONS v7.1'} (FW: ${info.version || '--'}, Build: ${info.build || '--'})`;
    // ★ 押すと転送のダイアログを開く（切断はダイアログの中の「🔌 切断」）。「切断」と書くと押しても切れない。
    headerBtn.textContent = '🔌 PONS接続中 (ファイル一覧)';
    headerBtn.style.borderColor = 'var(--accent-red)';
  } else if (state === 'connecting') {
    badge.style.background = '#fef3c7';
    badge.style.color = '#b45309';
    badge.textContent = '接続中...';
    text.textContent = info.msg || 'PONSからの応答を待機しています...';
    verText.textContent = '';
  } else if (state === 'error') {
    // 開けない・返事が無い。理由と直し方を出し、「もう一度つなぐ」を置く
    badge.style.background = '#fee2e2';
    badge.style.color = '#b91c1c';
    badge.textContent = info.badge || 'つながりません';
    text.textContent = info.msg || '';
    text.style.color = '#b91c1c';
    verText.innerHTML = info.retry
      ? '<button onclick="retryWebSerial()" style="padding:4px 10px; font-size:0.85rem; white-space:nowrap;">🔁 もう一度つなぐ</button>'
      : '';
    if (!serialPort) {
      headerBtn.textContent = '⚡ PONSに接続 (USB)';
      headerBtn.style.borderColor = 'var(--accent-cyan)';
    }
    return;
  } else {
    badge.style.background = '#e2e8f0';
    badge.style.color = '#475569';
    badge.textContent = '未接続';
    text.textContent = 'PONSに接続してください';
    verText.textContent = '';
    headerBtn.textContent = '⚡ PONSに接続 (USB)';
    headerBtn.style.borderColor = 'var(--accent-cyan)';
  }
}

// 一覧のタブ。dev = 開発用（PONS:LIST,ALL が要る。取って保存するだけ）
const FILE_TABS = [
  { id: 'logs',  label: 'フライト・記録', dev: false, note: '' },
  { id: 'imu',   label: 'IMU 生ログ',     dev: true,
    note: 'imuraw/*.bin（生 IMU ログ。250Hz）。保存したものは tools/imulog/decode_imulog.py で読む。nofixNNN.bin は測位の前の分。' },
  { id: 'other', label: 'その他',         dev: true,
    note: 'ルートとその直下のフォルダの、ほかの全部のファイル（settings.txt・gnss_dbd.bin・rtc_backup.txt・mapdata.csv など）。中身はそのまま保存する。' },
];
// どのタブに出すか。飛行 CSV（data/・received/ の .csv）と log.txt は今までどおり最初のタブ
function fileTabOf(path) {
  if (/^(data|received)\/[^\/]+\.csv$/i.test(path) || isLogPath(path)) return 'logs';
  if (/^imuraw\//i.test(path)) return 'imu';
  return 'other';
}

function setFileTab(id) {
  fileTab = id;
  // 開発用のタブを初めて開いたら、全部の一覧を頼む
  if (id !== 'logs' && !listIsAll && !listAllUnsupported && !listPending && serialWriter && !downloadBusy()) {
    requestFileList();
    return;
  }
  renderFileList();
}

async function requestFileList(forceAll = false) {
  if (!serialWriter || downloadBusy()) return;   // 転送中に一覧を頼むと、一覧が転送のあとに遅れて届くだけなので頼まない
  const all = (forceAll || listIsAll || fileTab !== 'logs') && !listAllUnsupported;
  listPending = all ? 'all' : 'basic';
  listIncoming = [];
  if (listAllTimer) { clearTimeout(listAllTimer); listAllTimer = null; }
  renderFileList();
  await sendSerialLine(all ? "PONS:LIST,ALL" : "PONS:LIST");
  if (all) {
    // ★ PONS:LIST,ALL を知らないファームは何も返さない。1 行も来なければ古いファームとして、ふつうの一覧を取り直す
    listAllTimer = setTimeout(() => {
      listAllTimer = null;
      if (listPending !== 'all' || listIncoming.length > 0) return;
      listAllUnsupported = true;
      listIsAll = false;
      listPending = null;
      if (typeof window.onPonsFileListUpdated === 'function') window.onPonsFileListUpdated();
      requestFileList();
    }, 4000);
  }
}

function renderFileTabs() {
  const box = document.getElementById('serial-file-tabs');
  box.innerHTML = FILE_TABS.map(t => {
    // 開発用のタブの数は、全部の一覧を取ったあとだけ出す
    const n = (!t.dev || listIsAll) ? ponsFileList.filter(f => fileTabOf(f.path) === t.id).length : null;
    const cnt = (n === null || listPending) ? '' : `<span class="cnt">${n}</span>`;
    return `<button class="file-tab${t.id === fileTab ? ' active' : ''}" onclick="setFileTab('${t.id}')">${t.label}${cnt}</button>`;
  }).join('');
  const tab = FILE_TABS.find(t => t.id === fileTab);
  document.getElementById('serial-file-note').textContent = tab ? tab.note : '';
}

function renderFileList() {
  renderFileTabs();
  const tbody = document.getElementById('serial-file-list');
  const msgRow = (txt, color = '#94a3b8') =>
    `<tr><td colspan="5" style="text-align:center; color:${color}; padding:20px; white-space:pre-line;">${txt}</td></tr>`;
  if (listPending) {
    tbody.innerHTML = msgRow('SDカードを走査中...', '#64748b');
    return;
  }
  const tab = FILE_TABS.find(t => t.id === fileTab) || FILE_TABS[0];
  if (tab.dev && !listIsAll) {
    tbody.innerHTML = msgRow(listAllUnsupported
      ? 'この本体のファームは全部のファイルの一覧（PONS:LIST,ALL）を出しません。\n2026-11-15 より後のファームで使えます。'
      : (serialWriter ? '「🔄 再読み込み」を押すと一覧を取ります' : 'PONSを接続すると一覧が表示されます'));
    return;
  }

  // 並べ替えは表示だけで、ponsFileList の添字はそのまま（ボタンは添字で指す）
  const keyOf = {};   // 添字 → flightFileKey()（フライト・記録のタブだけ）
  let order = ponsFileList.map((it, i) => i).filter(i => fileTabOf(ponsFileList[i].path) === tab.id);
  if (order.length === 0) {
    tbody.innerHTML = msgRow(tab.id === 'logs' ? 'フライトログ（*.csv）も log.txt も見つかりませんでした'
                           : tab.id === 'imu'  ? 'imuraw/ にファイルがありません'
                           : 'ほかのファイルはありません');
    return;
  }
  if (tab.dev) {
    // 新しい順（更新日時。時刻を知らずに書いた 1980 年のものは下へ）
    order.sort((a, b) => (ponsFileList[b].date || '').localeCompare(ponsFileList[a].date || '') ||
                         ponsFileList[b].path.localeCompare(ponsFileList[a].path));
  } else {
    // log.txt を先頭に（いちばん新しいことが書いてある）。そのあとは新しい日付が上（2026-11-25〜。本体の REPLAY/DEMO の一覧も新しい順）。
    // 同じ時刻の自機ログと受信ログは data/ → received/ の順。名前に日付の無いものは更新日時の位置（本体はいちばん下に名前の順）、
    // それも分からないものはいちばん下に名前の順
    order.forEach(i => { keyOf[i] = flightFileKey(ponsFileList[i]); });
    order.sort((a, b) => isLogPath(ponsFileList[b].path) - isLogPath(ponsFileList[a].path) ||
                         (keyOf[b] > keyOf[a]) - (keyOf[b] < keyOf[a]) ||
                         ponsFileList[a].path.localeCompare(ponsFileList[b].path));
  }
  const badge = (txt, bg, fg) => `<span style="background:${bg}; color:${fg}; padding:2px 6px; border-radius:4px; font-size:0.75rem;">${txt}</span>`;
  // フライト・記録のタブは日付ごとに見出しの行を挟む（何日のものかを探しやすく）
  const dayCount = {};
  if (!tab.dev) order.forEach(i => {
    if (isLogPath(ponsFileList[i].path)) return;
    const d = keyOf[i].slice(0, 10);
    dayCount[d] = (dayCount[d] || 0) + 1;
  });
  let lastDay = null;
  tbody.innerHTML = order.map((idx) => {
    const item = ponsFileList[idx];
    let dayRow = '';
    if (!tab.dev && !isLogPath(item.path)) {
      const d = keyOf[idx].slice(0, 10);
      if (d !== lastDay) {
        lastDay = d;
        dayRow = `<tr class="file-day"><td colspan="5">${escapeHtml(dayLabel(d))}<span class="cnt">${dayCount[d]} 件</span></td></tr>`;
      }
    }
    const save = `<button class="btn-download sub" onclick="downloadPonsFileIdx(${idx}, 0, 'save')" title="取って、そのまま PC に保存します（開きません）">💾 保存</button>`;
    let typeBadge, btns;
    if (tab.dev) {
      const ext = (item.path.match(/\.([^.\/]+)$/) || [, ''])[1].toUpperCase();
      typeBadge = tab.id === 'imu' ? badge('生 IMU', '#ccfbf1', '#0f766e') : badge(escapeHtml(ext || 'FILE'), '#f1f5f9', '#475569');
      btns = (tab.id === 'imu'
        ? `<button class="btn-download" onclick="downloadPonsFileIdx(${idx}, 0, 'open')" title="生 IMU ログをそのまま解析画面で開きます">📊 解析</button> `
        : '') + save;
    } else if (isLogPath(item.path)) {
      // log.txt: ふだんは最新の分（LOG_TAIL_BYTES）だけ取る。全部は時間がかかる（数 MB になる）
      typeBadge = badge('記録 (log)', '#ede9fe', '#6d28d9');
      const big = item.size > LOG_TAIL_BYTES;
      btns = (big
        ? `<button class="btn-download" onclick="downloadPonsFileIdx(${idx}, ${LOG_TAIL_BYTES})" title="最新の ${logBytes(LOG_TAIL_BYTES)} だけ取って表示します">最新 ${logBytes(LOG_TAIL_BYTES)}</button>
               <button class="btn-download" onclick="downloadPonsFileIdx(${idx}, 0)" title="全部取って表示します（大きいと時間がかかります）" style="opacity:0.8;">全部</button>`
        : `<button class="btn-download" onclick="downloadPonsFileIdx(${idx}, 0)">見る</button>`) + ' ' + save;
    } else {
      typeBadge = item.path.startsWith('received/') ? badge('無線受信', '#fef3c7', '#b45309') : badge('自機ログ', '#e0f2fe', '#0369a1');
      btns = `<button class="btn-download" onclick="downloadPonsFileIdx(${idx})">開く</button> ` + save;
    }
    return dayRow + `
          <tr>
            <td style="font-family:var(--font-mono); font-weight:bold;">${escapeHtml(item.path)}</td>
            <td>${typeBadge}</td>
            <td style="font-family:var(--font-mono);">${logBytes(item.size)}</td>
            <td style="font-family:var(--font-mono); font-size:0.8rem; color:#64748b;">${escapeHtml(item.date)}</td>
            <td style="white-space:nowrap;">${btns}</td>
          </tr>
        `;
  }).join('');
}

// 飛行 CSV を並べる鍵 "YYYY-MM-DD HH:MM"（大きいほど新しい）。名前の日時（記録を始めた JST。YYYY-MM-DD_HHMM.csv・
// 大会ログの ..._fused.csv）を使い、名前に日付が無いものだけ更新日時。どちらも無い・時刻を知らずに書いた 1980 年なら ''。
// ★ 更新日時を先に見ないこと: 名前は始めた時刻、更新日時は終わった時刻で、PC でコピーし直したファイルは更新日時が変わる
function flightFileKey(item) {
  const name = String(item.path || '').split('/').pop();
  const m = name.match(/^(\d{4})-(\d{2})-(\d{2})(?:_(\d{2})(\d{2}))?/);
  if (m) return `${m[1]}-${m[2]}-${m[3]} ${m[4] !== undefined ? m[4] + ':' + m[5] : '--:--'}`;
  const d = String(item.date || '').match(/^(\d{4})-(\d{2})-(\d{2}) (\d{2}):(\d{2})/);
  if (d && parseInt(d[1], 10) >= 2020) return `${d[1]}-${d[2]}-${d[3]} ${d[4]}:${d[5]}`;
  return '';
}

// 見出しの日付 "2026-11-25（水）"。'' は日付の分からないもの
function dayLabel(ymd) {
  const m = String(ymd || '').match(/^(\d{4})-(\d{2})-(\d{2})$/);
  if (!m) return '日付の分からないファイル';
  const dt = new Date(Date.UTC(+m[1], +m[2] - 1, +m[3]));
  // 2026-13-40 のような名前は Date が繰り上げてしまうので、曜日を出さない
  if (dt.getUTCMonth() !== +m[2] - 1 || dt.getUTCDate() !== +m[3]) return ymd;
  return `${ymd}（${'日月火水木金土'[dt.getUTCDay()]}）`;
}

// HTML に文字列を埋め込むときのエスケープ（ファイル名に < & ' などが入っていても壊れないように）
function escapeHtml(str) {
  return String(str).replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
}

// 一覧の「開く」。★ パスを onclick の文字列に埋め込まない（FAT のファイル名には ' が使え、埋め込むと JavaScript が壊れる）
// tail: log.txt の末尾だけ取るバイト数（0 = 全部）。mode: "open" = 開く / "save" = 取って PC に保存するだけ
function downloadPonsFileIdx(idx, tail = 0, mode = 'open') {
  const it = ponsFileList[idx];
  if (it) downloadPonsFile(it.path, it.size, tail, mode);
}

// log.txt（SD のルートの記録）か。開いたあと再生ではなく記録の表示へ回す
function isLogPath(path) { return /(^|\/)log[^\/]*\.txt$/i.test(path || ''); }

let imuRangeFallbackTimer = null;

async function downloadPonsFile(filename, size, tail = 0, mode = 'open', imuRange = null) {
  if (!serialWriter || downloadBusy() || listPending) return;
  if (imuRangeFallbackTimer) { clearTimeout(imuRangeFallbackTimer); imuRangeFallbackTimer = null; }

  currentDownloadFile = filename;
  // 開けるのは飛行 CSV・log.txt・生 IMU (.bin)。ほかは中身をそのまま保存する
  const ftab = fileTabOf(filename);
  currentDownloadKind = ftab === 'imu' ? 'imu' : (ftab !== 'logs' ? 'raw' : (isLogPath(filename) ? 'log' : 'csv'));
  currentDownloadMode = (mode === 'save' || currentDownloadKind === 'raw') ? 'save' : 'open';
  currentDownloadFrom = 0;
  currentDownloadFull = size;
  if (tail > 0 && tail < size) size = tail;   // 進み具合の目安（本当の量は PONS:DATA_START で来る）
  else tail = 0;
  currentDownloadSize = size;
  dataBytes = null;
  dataReceived = 0;
  dataTrailerPending = false;
  downloadStartTime = performance.now();
  lastProgressUpdate = 0;
  // isDataStreaming は DATA_START 受信時に true にする（ヘッダー行混入防止）

  const useImuRange = !!(imuRange && imuRange.jstEnd > imuRange.jstStart && currentDownloadKind === 'imu');
  currentDownloadVia = useImuRange ? 'getimu' : (tail > 0 ? 'gettail' : 'get');

  // プログレスバー表示
  const progBox = document.getElementById('serial-progress-box');
  progBox.style.display = 'flex';
  document.getElementById('serial-progress-filename').textContent = useImuRange
    ? `転送中 (フライト区間抽出): ${filename}`
    : `転送中: ${filename}`;
  document.getElementById('serial-progress-fill').style.width = '0%';
  document.getElementById('serial-progress-stats').textContent = useImuRange ? '本体で時刻検索中...' : '0%';
  document.getElementById('serial-progress-bytes').textContent = useImuRange ? '区間算出中...' : `0 / ${logBytes(size)}`;
  document.getElementById('serial-progress-speed').textContent = '-- KB/s';

  downloadPending = true;   // DATA_START が来るまで（BUG-79）
  if (useImuRange) {
    await sendSerialLine(`PONS:GETIMU,${Math.floor(imuRange.jstStart)},${Math.ceil(imuRange.jstEnd)},${filename}`);
    // 古いファームは PONS:GETIMU を知らず無応答になるため、12秒後に通常 PONS:GET へ自動フォールバック。
    // 2026-11-20〜のファームはすぐ PONS:GETIMU_ACCEPTED を返すので、そこでこのタイマーを止める（時刻探しが長くても送り直さない）
    imuRangeFallbackTimer = setTimeout(() => {
      imuRangeFallbackTimer = null;
      if (!downloadPending || currentDownloadFile !== filename) return;
      document.getElementById('serial-progress-filename').textContent = `転送中 (全件取得): ${filename}`;
      document.getElementById('serial-progress-stats').textContent = '0%';
      currentDownloadVia = 'get';
      sendSerialLine("PONS:GET," + filename).catch(() => {});
    }, 12000);
  } else if (tail > 0) {
    // 末尾だけ: PONS:GETTAIL,<バイト数>,<パス>（2026-11-11〜）
    await sendSerialLine(`PONS:GETTAIL,${tail},${filename}`);
  } else if (rangeSupport !== false) {
    // 全部: 1MB ずつ取って、合わない塊だけ取り直す（2026-11-25〜）
    startRangeDownload(filename, size);
  } else {
    await sendSerialLine("PONS:GET," + filename);
  }
}

// ---- 塊ごとの取り込み（PONS:GETRANGE）----
// 流れ: requestRangeChunk() で 1 つ頼む → DATA_START〜DATA_END は今までの受け口（feedDataBytes）がそのまま受ける →
// finishDownload() が finishRangeChunk() へ回す → 合えば out に足して次の塊、合わなければ同じ塊を頼み直す。
// 本体は塊のたびに PONS:GETRANGE_ACCEPTED を返し、DATA_START の 4 つめにファイル全体の大きさを付ける。
// sub: ファイル全体ではなく [sub.base, sub.base + sub.size) だけを取る（飛行の時刻の範囲 GETIMU が合わなかったときの取り直し）。
//   sub.bytes / sub.note は 1 回目に取れた分とその一言。本体が GETRANGE を知らなければ、それを警告つきで開く（今までどおり）。
function startRangeDownload(filename, fullSize, sub = null) {
  const base = sub ? sub.base : 0;
  const end = sub ? sub.base + sub.size : fullSize;
  let out;
  try {
    out = new Uint8Array(end - base);
  } catch (e) {
    alert(`ファイルが大きすぎて受けられません（${logBytes(end - base)}）。`);
    endDownloadUi();
    return;
  }
  rangeDl = {
    file: filename, kind: currentDownloadKind, mode: currentDownloadMode,
    base, end, sub,                    // 取る範囲 [base, end)（ファイルの中の位置）
    out, filled: 0,                    // out[0..filled) が取れた中身（諦めた塊があると end - base より短い）
    nextFrom: base, len: 0, want: 0,   // 頼んでいる塊の位置（ファイルの中）・頼んだ長さ・本体が送ると言った長さ
    tries: 0, retries: 0, best: null,  // この塊を取った回数・取り直した回数の合計・この塊でいちばん長く取れた分
    gaveUp: [], busyCount: 0, busyWait: false,
    recovering: false, recoverStart: 0, stallBytes: null,
    attemptStart: 0, startTime: performance.now(),
    fallbackTimer: null, retryTimer: null,
  };
  if (!rangeStallTimer) rangeStallTimer = setInterval(rangeStallCheck, 1000);
  requestRangeChunk();
}

function requestRangeChunk() {
  const r = rangeDl;
  if (!r) return;
  // ★ 大きさの分からない（一覧で 0）ファイルは 1 塊ぶん頼む。本体が DATA_START の 4 つめで本当の大きさを返す
  r.len = (r.end > r.nextFrom) ? Math.min(RANGE_CHUNK_BYTES, r.end - r.nextFrom) : RANGE_CHUNK_BYTES;
  r.want = r.len;   // DATA_START で本体の言う長さに直す
  r.recovering = false;
  r.attemptStart = performance.now();
  // 1 つの塊ぶんの受け口（大きさは DATA_START で来る）
  isDataStreaming = false;
  dataTrailerPending = false;
  dataBytes = null;
  dataReceived = 0;
  currentDownloadSize = r.len;
  downloadPending = true;
  if (rangeSupport === null && !r.fallbackTimer) r.fallbackTimer = setTimeout(rangeFallbackToGet, RANGE_ACCEPT_WAIT_MS);
  sendSerialLine(`PONS:GETRANGE,${r.nextFrom},${r.len},${r.file}`).catch(() => {});
}

// 受け付けの印も DATA_START も来ない: PONS:GETRANGE を知らない古いファーム。
// ファイル全体なら全部を 1 回で取る（今までどおり）。GETIMU の取り直しなら、1 回目に取れた分を警告つきで開く（今までどおり）
function rangeFallbackToGet() {
  const r = rangeDl;
  if (!r) return;
  r.fallbackTimer = null;
  if (rangeSupport !== null || !downloadPending || isDataStreaming || cancelPending) return;
  rangeSupport = false;
  if (r.sub) {
    console.warn("PONS: PONS:GETRANGE に返事が無いので（古いファーム）、取り直さずに 1 回目の分を開きます");
    // 頼んだ GETRANGE には本体が答えないので、待たずに閉じる
    endDownloadUi();
    deliverDownload(r.sub.bytes, { name: r.file, kind: r.kind, mode: r.mode, sec: r.sub.sec, from: r.base,
                                   full: r.sub.full, bad: true, note: r.sub.note });
    return;
  }
  console.warn("PONS: PONS:GETRANGE に返事が無いので（古いファーム）、PONS:GET で全部を 1 回で取ります");
  clearRangeState();
  currentDownloadSize = r.end;
  downloadStartTime = performance.now();
  document.getElementById('serial-progress-filename').textContent = `転送中（古いファームなので 1 回で全部）: ${r.file}`;
  sendSerialLine("PONS:GET," + r.file).catch(() => {});
}

function rangeGrowOut(n) {
  const r = rangeDl;
  const grown = new Uint8Array(n);
  grown.set(r.out.subarray(0, r.filled));
  r.out = grown;
}

// DATA_START の 4 つめ（ファイル全体の大きさ）。ファイル全体を取っているなら、一覧のあとに伸びた・縮んだのに合わせる。
// 範囲を取り直しているときは、縮んで範囲が切れたときだけ合わせる
function rangeFileSize(fileSize) {
  const r = rangeDl;
  if (r.sub) {
    if (fileSize < r.end) r.end = Math.max(r.base, fileSize);
    return;
  }
  r.end = fileSize;
  if (fileSize - r.base > r.out.length) rangeGrowOut(fileSize - r.base);
}

// PONS:DATA_END を受けた（塊ごとの取り込みのとき）
function finishRangeChunk(expectedCrc) {
  const r = rangeDl;
  const bytes = dataBytes ? dataBytes.subarray(0, dataReceived) : new Uint8Array(0);
  const want = currentDownloadSize;
  // 塊の受け口を下ろす（進み具合の箱は出したまま）
  isDataStreaming = false;
  dataTrailerPending = false;
  dataBytes = null;
  dataReceived = 0;
  // ★ 中断を送ったあとに届いた終わり: 捨てて終わる（BUG-79 と同じ）
  if (cancelPending) { endDownloadUi(); return; }
  // ★ 止まったとみて頼み直しに入ったあとに遅れて届いた: 使わない（本体の CANCEL_OK を待って頼み直す）
  if (r.recovering) return;
  const crc = computeCrc32(bytes);
  if (bytes.length === want && crc === expectedCrc) {
    rangeAppend(bytes);
    r.nextFrom += bytes.length;
    rangeNext(bytes.length === 0);
    return;
  }
  console.warn(`PONS: GETRANGE ${r.nextFrom}+${want} が合わない（${bytes.length} B・CRC ${crc} / 本体 ${expectedCrc}、${r.tries + 1} 回目）`);
  rangeChunkFailed(bytes);
}

function rangeAppend(bytes) {
  const r = rangeDl;
  if (r.filled + bytes.length > r.out.length) rangeGrowOut(r.filled + bytes.length);
  r.out.set(bytes, r.filled);
  r.filled += bytes.length;
  r.tries = 0;
  r.best = null;
  r.busyCount = 0;
}

// 次の塊へ（終わりなら仕上げる）。atEof: 本体が 0 バイトを返した（ファイルの終わり）
function rangeNext(atEof) {
  const r = rangeDl;
  if (atEof || r.nextFrom >= r.end) { rangeFinish(); return; }
  updateDownloadProgress(true);
  requestRangeChunk();
}

// 塊が足りない・CRC が合わない・途中で止まった。上限までは同じ塊を頼み直し、越えたら取れた分で先へ進む
function rangeChunkFailed(bytes) {
  const r = rangeDl;
  if (!r) return;
  r.tries++;
  r.retries++;
  if (bytes && bytes.length > 0 && (!r.best || bytes.length > r.best.length)) r.best = bytes.slice();
  if (r.tries < RANGE_MAX_TRIES) {
    updateDownloadProgress(true);
    requestRangeChunk();
    return;
  }
  const got = r.best ? r.best.length : 0;
  console.warn(`PONS: GETRANGE ${r.nextFrom}+${r.want} を ${RANGE_MAX_TRIES} 回取っても合わない。取れた ${got} B で先へ進む`);
  r.gaveUp.push({ from: r.nextFrom, len: r.want, got });
  const skip = r.want;
  if (r.best) rangeAppend(r.best);
  r.tries = 0;
  r.best = null;
  r.nextFrom += skip;
  rangeNext(skip === 0);
}

// 全部の塊を取り終えた: 開く・保存する（今までの 1 回で取ったときと同じ先へ）
function rangeFinish() {
  const r = rangeDl;
  const bytes = r.out.subarray(0, r.filled);
  const sec = (performance.now() - r.startTime) / 1000;
  const total = r.end - r.base;
  const nChunks = Math.max(1, Math.ceil(total / RANGE_CHUNK_BYTES));
  const o = { name: r.file, kind: r.kind, mode: r.mode, sec, from: r.base, full: r.sub ? r.sub.full : r.end, bad: false,
              note: (r.sub ? '1 回目は合わず、取り直して ' : '') + `CRC OK（1MB × ${nChunks} 個、取り直し ${r.retries} 回）` };
  if (r.gaveUp.length > 0) {
    const missing = Math.max(0, total - r.filled);
    o.bad = true;
    o.note = `${r.gaveUp.length} 個の塊が ${RANGE_MAX_TRIES} 回取っても合わず、不足 ${missing.toLocaleString()} B`;
    alert(`⚠️ ${r.file}: ${nChunks} 個の塊のうち ${r.gaveUp.length} 個が、${RANGE_MAX_TRIES} 回取り直しても合いませんでした` +
          `（不足 ${missing.toLocaleString()} B）。\n取れた分で${r.mode === 'save' ? '保存します' : '開きます'}。`);
  } else if (r.retries > 0) {
    console.log(`PONS: ${r.file} は取り直し ${r.retries} 回で全部の塊が合った`);
  }
  endDownloadUi();
  deliverDownload(bytes, o);
}

// 1 秒ごと: 塊の途中で何も来なくなった（終わりの印や DATA_START の行が抜けた）ら、本体に中断を送って片付けてから頼み直す
function rangeStallCheck() {
  const r = rangeDl;
  if (!r || cancelPending || r.busyWait || rangeSupport === null) return;
  const now = performance.now();
  if (r.recovering) {
    // CANCEL_OK が来なくても 3 秒で頼み直す
    if (now - r.recoverStart > 3000) { r.recovering = false; rangeChunkFailed(r.stallBytes); }
    return;
  }
  if (now - Math.max(lastSerialRxMs, r.attemptStart) < RANGE_STALL_MS) return;
  console.warn(`PONS: GETRANGE ${r.nextFrom}+${r.want} が途中で止まった（${dataReceived} B 受けたところ）。中断して頼み直す`);
  r.stallBytes = dataBytes ? dataBytes.slice(0, dataReceived) : null;
  isDataStreaming = false;
  dataTrailerPending = false;
  dataBytes = null;
  dataReceived = 0;
  rxLine = [];
  downloadPending = true;   // 中断ボタンが効くように「転送中」のまま
  r.recovering = true;
  r.recoverStart = now;
  sendSerialLine("PONS:CANCEL").catch(() => {});
}

function clearRangeState() {
  if (rangeDl) {
    if (rangeDl.fallbackTimer) clearTimeout(rangeDl.fallbackTimer);
    if (rangeDl.retryTimer) clearTimeout(rangeDl.retryTimer);
  }
  rangeDl = null;
  if (rangeStallTimer) { clearInterval(rangeStallTimer); rangeStallTimer = null; }
}

// 転送が動いているか（GET を送ってから終わりの印まで。中断の後始末を待っている間も含む）
function downloadBusy() { return isDataStreaming || downloadPending || cancelPending; }

// 転送の状態を全部下ろして、進み具合の箱を閉じる（終わり・中断・エラー・切断）
function endDownloadUi() {
  clearRangeState();
  if (imuRangeFallbackTimer) { clearTimeout(imuRangeFallbackTimer); imuRangeFallbackTimer = null; }
  isDataStreaming = false;
  downloadPending = false;
  cancelPending = false;
  dataTrailerPending = false;
  dataBytes = null;
  dataReceived = 0;
  if (cancelWatchTimer) { clearInterval(cancelWatchTimer); cancelWatchTimer = null; }
  document.getElementById('serial-progress-box').style.display = 'none';
}

// 中断（2026-11-13、BUG-79）。★ DATA_START の前でも送る（本体は待っている間の CANCEL も拾う）。
// ★ 状態は本体の CANCEL_OK（またはエラー・DATA_END）を受けるまで下ろさない。それまでに届くバイトは転送の残りとして捨てる。
async function cancelPonsDownload() {
  if (!(isDataStreaming || downloadPending) || cancelPending) return;
  cancelPending = true;
  document.getElementById('serial-progress-filename').textContent = `中断しています: ${currentDownloadFile}`;
  lastSerialRxMs = performance.now();
  // 見切り: CANCEL_OK を返さない古いファーム（DATA_START の前の中断を知らない）でも固まらないように、
  // 3 秒なにも届かなければ終わったことにする
  cancelWatchTimer = setInterval(() => {
    if (cancelPending && performance.now() - lastSerialRxMs > 3000) {
      console.warn("PONS: CANCEL_OK が来ないので転送の終わりとみなす");
      rxLine = [];
      endDownloadUi();
    }
  }, 500);
  try { await sendSerialLine("PONS:CANCEL"); } catch (e) { endDownloadUi(); }
}

let pingTimer = null;

function startSerialPing() {
  stopSerialPing();
  pingTimer = setInterval(async () => {
    const modal = document.getElementById('serial-modal');
    const settingsModal = document.getElementById('settings-modal');
    const wipeOpen = (typeof wipeModalOpen === 'function') && wipeModalOpen();
    const isModalOpen = (modal && modal.classList.contains('active')) ||
                        (settingsModal && settingsModal.classList.contains('active')) || wipeOpen;
    const settingsBusy = (typeof settingsState !== 'undefined') && (settingsState.loading || settingsState.saving);
    const wipeBusyNow = (typeof wipeBusy === 'function') && wipeBusy();
    if (serialPort && serialPort.writable && !downloadBusy() && !settingsBusy && !wipeBusyNow && isModalOpen) {
      try {
        await sendSerialLine("PONS:PING");
      } catch (e) {
        console.warn("Serial PING failed:", e);
      }
    }
  }, 15000);
}

function stopSerialPing() {
  if (pingTimer) {
    clearInterval(pingTimer);
    pingTimer = null;
  }
}

// 受けたバイトを 1 つの Uint8Array の列として、行（コマンドの返事）とファイルの中身に振り分ける（2026-11-15〜）
const serialLineDecoder = new TextDecoder('utf-8');
function feedSerialBytes(u8) {
  let pos = 0;
  while (pos < u8.length) {
    if (isDataStreaming) {
      pos = feedDataBytes(u8, pos);
      continue;
    }
    // 行: 改行までを貯める。PONS:DATA_START の行を handleSerialLine が読むと isDataStreaming が立ち、続きは中身へ回る
    const nl = u8.indexOf(10, pos);
    const end = nl === -1 ? u8.length : nl;
    for (let i = pos; i < end; i++) rxLine.push(u8[i]);
    if (rxLine.length > 8192) rxLine = [];   // 改行の無いごみ（別の機器など）を貯め続けない
    if (nl === -1) break;
    pos = nl + 1;
    const line = serialLineDecoder.decode(Uint8Array.from(rxLine)).replace(/\r$/, '').trim();
    rxLine = [];
    if (line.length > 0) handleSerialLine(line);
  }
}

// ファイルの中身を dataBytes へ。DATA_START の言う大きさぶん受けたら、あとは行（"\r\n" と PONS:DATA_END,<crc>）として読む。
// 戻り値は u8 の読み終えた位置。
function feedDataBytes(u8, pos) {
  const n = Math.min(currentDownloadSize - dataReceived, u8.length - pos);
  dataBytes.set(u8.subarray(pos, pos + n), dataReceived);
  dataReceived += n;
  pos += n;
  if (dataReceived >= currentDownloadSize) {
    isDataStreaming = false;
    dataTrailerPending = true;
    updateDownloadProgress(true);
    return pos;
  }

  // ★ 本体が転送の途中でやめた（PONS:CANCEL_OK / PONS:ERROR,... / 読めなくなって早めに出た PONS:DATA_END）。
  //   受けている間は全部 dataBytes に入るので、ここで拾わないと isDataStreaming が戻らず、以降のダウンロードと PING が止まったままになる。
  //   印は届いた分の末尾 256 バイトの、行の頭にあるときだけ見る（log.txt の中身の文字列やバイナリの偶然で止めないように）。
  //   ★ まず今回届いた分（と手前の 64 バイト）に "PONS:" があるかだけを見る。無ければ何もしない（数十 MB を受けるので軽く）
  let hit = false;
  for (let i = Math.max(0, dataReceived - n - 64), e = dataReceived - 4; i < e; i++) {
    if (dataBytes[i] === 80 && dataBytes[i + 1] === 79 && dataBytes[i + 2] === 78 && dataBytes[i + 3] === 83 && dataBytes[i + 4] === 58) { hit = true; break; }
  }
  if (!hit) {
    updateDownloadProgress(false);
    return pos;
  }
  const from = Math.max(0, dataReceived - 256);
  let tail = '';
  for (let i = from; i < dataReceived; i++) tail += String.fromCharCode(dataBytes[i]);
  const m = tail.match(/\n(PONS:CANCEL_OK\r?\n|PONS:ERROR,[^\n]*\n|PONS:DATA_END,[0-9A-Fa-f]{8}\r?\n)/) ||
            (from === 0 ? tail.match(/^(PONS:CANCEL_OK\r?\n|PONS:ERROR,[^\n]*\n|PONS:DATA_END,[0-9A-Fa-f]{8}\r?\n)/) : null);
  if (m) {
    const mark = from + m.index + (m[0].charCodeAt(0) === 10 ? 1 : 0);   // 印の行の頭
    const rest = dataBytes.slice(mark, dataReceived);                    // 印の行と、そのあとに届いていた行
    // 本体は印の前に "\r\n" を出す。中身はその手前まで
    let len = mark;
    if (len > 0 && dataBytes[len - 1] === 10) len--;
    if (len > 0 && dataBytes[len - 1] === 13) len--;
    dataReceived = len;
    isDataStreaming = false;
    dataTrailerPending = true;   // DATA_END なら受けた分で終わりの処理へ（短いことは finishDownload が言う）
    feedSerialBytes(rest);
    return pos;
  }
  updateDownloadProgress(false);
  return pos;
}

function updateDownloadProgress(force) {
  const now = performance.now();
  if (!force && now - lastProgressUpdate <= 100) return;
  lastProgressUpdate = now;
  // 塊ごとに取っているときは、ファイル全体での進み具合（取れた塊＋いま受けている塊）
  const r = rangeDl;
  const received = r ? r.filled + dataReceived : dataReceived;
  const total = r ? r.end - r.base : currentDownloadSize;
  const pct = total > 0 ? Math.min(100, Math.round((received / total) * 100)) : 0;
  document.getElementById('serial-progress-fill').style.width = pct + '%';
  document.getElementById('serial-progress-stats').textContent = pct + '%' + (r && r.retries > 0 ? `（取り直し ${r.retries}）` : '');
  document.getElementById('serial-progress-bytes').textContent = `${logBytes(received)} / ${logBytes(total)}`;
  const elapsedSec = (now - (r ? r.startTime : downloadStartTime)) / 1000;
  const kbps = elapsedSec > 0 ? Math.round((received / 1024) / elapsedSec) : 0;
  document.getElementById('serial-progress-speed').textContent = `${kbps} KB/s`;
}

// PONS:DATA_END,<crc> を受けた。CRC を確かめて、開く（CSV は再生・log.txt は記録の表示）か、PC に保存する
function finishDownload(expectedCrc) {
  if (rangeDl) { finishRangeChunk(expectedCrc); return; }   // 塊ごとに取っている（1 つの塊の終わり）
  const bytes = dataBytes ? dataBytes.subarray(0, dataReceived) : new Uint8Array(0);
  const wantSize = currentDownloadSize;
  const kind = currentDownloadKind, mode = currentDownloadMode;
  const name = currentDownloadFile;
  const sec = (performance.now() - downloadStartTime) / 1000;
  // ★ 中断を送ったのに最後まで届いた（本体が CANCEL を読む前に送り終えた・古いファーム）。中身は使わずに捨てる（BUG-79）
  const dropped = cancelPending || kind === 'discard';
  endDownloadUi();
  if (dropped) return;

  // CRC32 検証（ファイル本体データのみで計算。本体も送ったバイトだけで計算している）
  const calculatedCrc = computeCrc32(bytes);
  const shortRead = bytes.length < wantSize;
  const crcBad = calculatedCrc !== expectedCrc;
  let note = `CRC OK ${calculatedCrc}`;
  // ★ 飛行の時刻の範囲（GETIMU）が合わなかった: 本体が言った範囲 [from, from + 大きさ) を 1MB ずつ取り直す（2026-11-25〜）。
  //   本体が GETRANGE を知らなければ（rangeFallbackToGet）、この 1 回目の分を今までどおり警告つきで開く
  if ((shortRead || crcBad) && currentDownloadVia === 'getimu' && kind === 'imu' && rangeSupport !== false && wantSize > 0) {
    console.warn(`PONS: GETIMU が合わない（${bytes.length}/${wantSize} B・CRC ${calculatedCrc} / ${expectedCrc}）。範囲を 1MB ずつ取り直す`);
    document.getElementById('serial-progress-box').style.display = 'flex';
    document.getElementById('serial-progress-filename').textContent = `取り直し中（1 回目は合わず、1MB ずつ）: ${name}`;
    startRangeDownload(name, currentDownloadFull, {
      base: currentDownloadFrom, size: wantSize, bytes: bytes.slice(), sec, full: currentDownloadFull,
      note: `欠落 ${bytes.length}/${wantSize} B・CRC 不一致（計算 ${calculatedCrc} / 本体 ${expectedCrc}）。本体のファームが古く取り直せない`,
    });
    return;
  }
  if (shortRead && crcBad) {
    const missing = wantSize - bytes.length;
    console.warn(`Transfer drop & CRC mismatch! Got: ${bytes.length}/${wantSize} (-${missing} B), Calc: ${calculatedCrc}, Expected: ${expectedCrc}`);
    alert(`⚠️ 通信途中でデータが欠落しました（${bytes.length.toLocaleString()} / ${wantSize.toLocaleString()} バイト、不足 ${missing.toLocaleString()} B）。\nCRC32 も一致しません（計算値: ${calculatedCrc} / 受信値: ${expectedCrc}）。\n再度転送をお試しください。`);
    note = `欠落 ${bytes.length}/${wantSize} B・CRC 不一致（計算 ${calculatedCrc} / 本体 ${expectedCrc}）`;
  } else if (shortRead) {
    alert(`⚠️ 転送が途中で終わりました（${bytes.length.toLocaleString()} / ${wantSize.toLocaleString()} バイト）。\n本体が SD を読めなくなった可能性があります。`);
    note = `途中まで ${bytes.length} / ${wantSize} B`;
  } else if (crcBad) {
    console.warn(`CRC mismatch! Calc: ${calculatedCrc}, Expected: ${expectedCrc}`);
    alert(`⚠️ CRC32チェックサムが一致しません（計算値: ${calculatedCrc} / 受信値: ${expectedCrc}）。\n通信中に一部データが破損した可能性があります。`);
    note = `CRC 不一致（計算 ${calculatedCrc} / 本体 ${expectedCrc}）`;
  } else {
    console.log(`✅ CRC32 検証成功: ${calculatedCrc}`);
  }

  deliverDownload(bytes, { name, kind, mode, sec, note, bad: shortRead || crcBad,
                           from: currentDownloadFrom, full: currentDownloadFull });
}

// 取れた中身を開く・保存する（1 回で取ったときと塊ごとに取ったときで同じ）。
// o: { name, kind, mode, sec, note（保存の一言・解析画面の警告）, bad（欠落・CRC 不一致）, from / full（log.txt の位置） }
function deliverDownload(bytes, o) {
  const name = o.name;
  // 保存だけ（開発用のタブと、各行の「💾 保存」）。ダイアログは開いたまま（続けて取れるように）
  if (o.mode === 'save') {
    saveBytesAsFile(bytes, name);
    document.getElementById('serial-file-note').textContent =
      `💾 保存: ${name}（${bytes.length.toLocaleString()} B、${o.sec.toFixed(1)} 秒、${o.note}）`;
    return;
  }

  // 生 IMU ログ (.bin) はバイナリのまま解析エンジンへ渡す
  if (o.kind === 'imu') {
    closeSerialModal();
    const targetTab = window._pendingImuAnalysisTab || 'imu';
    window._pendingImuAnalysisTab = null;
    if (typeof loadImuBinBytes === 'function') {
      // 欠落・CRC 不一致でも開く（デコーダが区切りを探し直す）。ただし解析画面に警告を出し続ける
      loadImuBinBytes(bytes, name || 'imulog.bin', {
        openModal: true, tab: targetTab,
        transferWarning: o.bad ? `USB 転送: ${o.note}` : null
      });
    }
    return;
  }

  // log.txt は記録の表示へ、CSV はパースして再生へ
  const fileData = new TextDecoder('utf-8').decode(bytes);
  if (o.kind === 'log') {
    closeSerialModal();
    openLogViewer(fileData, name || "log.txt",
                  { from: o.from, full: Math.max(o.full, o.from + bytes.length) });
    return;
  }
  parseCSVText(fileData, name || "flight_log.csv");
  closeSerialModal();
}

// 受けたバイトをそのままファイルに保存する（ブラウザのダウンロード。名前はパスの最後の部分）
function saveBytesAsFile(bytes, path) {
  const base = (path || 'pons_file.bin').split('/').pop().trim() || 'pons_file.bin';
  const url = URL.createObjectURL(new Blob([bytes], { type: 'application/octet-stream' }));
  const a = document.createElement('a');
  a.href = url; a.download = base;
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  setTimeout(() => URL.revokeObjectURL(url), 10000);
}

async function readSerialLoop() {
  try {
    while (serialPort && serialPort.readable && serialReadLoopActive) {
      serialReader = serialPort.readable.getReader();
      try {
        while (true) {
          const { value, done } = await serialReader.read();
          if (done) break;
          if (!value) continue;
          lastSerialRxMs = performance.now();
          feedSerialBytes(value);
        }
      } finally {
        serialReader.releaseLock();
        serialReader = null;
      }
    }
  } catch (err) {
    if (serialReadLoopActive) {
      console.warn("Serial read loop error:", err);
      disconnectWebSerial();
    }
  }
}

function handleSerialLine(line) {
  console.log("PONS <<", line);
  // 行の頭にごみ（改行の無いデバッグ出力など）が付いていても DATA_START は拾う（拾い損ねると中身が全部「行」として流れる）
  const ds = line.indexOf("PONS:DATA_START,");
  if (ds > 0) line = line.substring(ds);

  if (line.startsWith("PONS:READY")) {
    ponsReady = true;
    stopReadyWatch();
    // 一覧はつなぎ直すたびに取り直す（別の本体・書き替えたファームかもしれないので、LIST,ALL が使えるかも確かめ直す）
    listIsAll = false;
    listAllUnsupported = false;
    listPending = null;
    rangeSupport = null;   // PONS:GETRANGE が使えるかも確かめ直す（書き替えたファームかもしれない）
    resetSettingsState(false);
    if (typeof resetSdInfo === 'function') resetSdInfo();   // 別の本体かもしれない（一覧の終わりで本体が出し直す）
    if (typeof resetWipeState === 'function') resetWipeState(false);
    const parts = line.split(',');
    updateSerialUIStatus('connected', {
      board: parts[1] || 'PONS v7.1',
      version: parts[2] || '--',
      build: parts[3] || '--'
    });
    startSerialPing();
    // PC の時刻を渡す（本体の GNSS が時刻を知らない＝時計の電池 BT1 が切れていたときだけ、本体が GNSS へ入れて
    // 測位を早める。知っていれば本体は何もしない。返事 PONS:TIME,... は見ない。docs/pons_webserial_protocol.md の (7)）。
    // ★ 一覧より先に送る（一覧の途中に返事の行が混ざらないように）。Date.now() は UTC の UNIX 時刻 [ms]。
    sendSerialLine("PONS:TIME," + Date.now());
    // 接続直後にファイル一覧を自動取得
    requestFileList();
  } else if (line.startsWith("PONS:PONG")) {
    // 生存確認応答
    console.log("PONS PONG received");
  } else if (line.startsWith("PONS:FILE,")) {
    // PONS:FILE,<path>,<size>,<datetime>
    // ★ 後ろから切る（FAT のファイル名には , が使える。日時とバイト数には , が入らない）
    const parts = line.split(',');
    if (parts.length >= 4) {
      listIncoming.push({
        path: parts.slice(1, parts.length - 2).join(','),
        size: parseInt(parts[parts.length - 2], 10) || 0,
        date: parts[parts.length - 1]
      });
    }
  } else if (line === "PONS:LIST_END") {
    // ★ 全部の一覧かどうかは中身でも見る（LIST,ALL の返事が見切りのあとに遅れて届いたとき）
    const gotAll = listPending === 'all' || listIncoming.some(f => fileTabOf(f.path) !== 'logs');
    if (listAllTimer) { clearTimeout(listAllTimer); listAllTimer = null; }
    ponsFileList = listIncoming;
    listIncoming = [];
    listPending = null;
    listIsAll = gotAll;
    if (gotAll) listAllUnsupported = false;
    renderFileList();
    if (typeof window.onPonsFileListUpdated === 'function') window.onPonsFileListUpdated();
  } else if (line === "PONS:GETIMU_ACCEPTED") {
    // 本体が GETIMU を受け付けた（2026-11-20〜のファーム）。時刻探しが 12 秒を超えても PONS:GET を送り直さない
    // （送り直すと、転送が終わったあとに BUSY が返ってアラートが出ていた）
    if (imuRangeFallbackTimer) { clearTimeout(imuRangeFallbackTimer); imuRangeFallbackTimer = null; }
    if (downloadPending) document.getElementById('serial-progress-stats').textContent = '本体で時刻検索中（受付済み）...';
  } else if (line === "PONS:GETRANGE_ACCEPTED") {
    // 本体が PONS:GETRANGE を受け付けた（2026-11-25〜のファーム）。PONS:GET に戻さない
    rangeSupport = true;
    if (rangeDl && rangeDl.fallbackTimer) { clearTimeout(rangeDl.fallbackTimer); rangeDl.fallbackTimer = null; }
  } else if (line.startsWith("PONS:DATA_START,")) {
    if (imuRangeFallbackTimer) { clearTimeout(imuRangeFallbackTimer); imuRangeFallbackTimer = null; }
    // PONS:DATA_START,<path>,<size>[,<from>]。★ パスに , が入りうるので、パスは送ったものを使い、数は後ろから読む。
    // 末尾だけ（GETTAIL）のときだけ <from> が付く（mysd.cpp）。
    const body = line.substring("PONS:DATA_START,".length);
    let nums;
    if (currentDownloadFile && body.startsWith(currentDownloadFile + ",")) {
      nums = body.substring(currentDownloadFile.length + 1).split(',');
    } else {
      const parts = body.split(',');   // 送ったパスと違う（ないはず）: 以前どおり前から読む
      currentDownloadFile = parts[0] || currentDownloadFile;
      nums = parts.slice(1);
    }
    const sz = parseInt(nums[0], 10);
    if (Number.isFinite(sz) && sz >= 0) currentDownloadSize = sz;
    currentDownloadFrom = parseInt(nums[1], 10) || 0;   // 末尾だけのとき、送り始めた位置
    // 塊ごとに取っているとき（PONS:GETRANGE）: 4 つめがファイル全体の大きさ。一覧のあとで変わっていたら合わせる
    if (rangeDl && downloadPending) {
      rangeSupport = true;
      if (rangeDl.fallbackTimer) { clearTimeout(rangeDl.fallbackTimer); rangeDl.fallbackTimer = null; }
      const fs = parseInt(nums[2], 10);
      if (Number.isFinite(fs) && fs >= 0) rangeFileSize(fs);
      rangeDl.want = currentDownloadSize;
    }
    // 頼んでいないのに始まった（ないはず）: 中身は受けて捨てる（受けないと中身が行として流れる）
    if (!downloadPending && !cancelPending) currentDownloadKind = 'discard';
    downloadPending = false;
    // ここから先のバイトは、この大きさぶんを中身として受ける（feedDataBytes）
    try {
      dataBytes = new Uint8Array(currentDownloadSize);
    } catch (e) {
      alert(`ファイルが大きすぎて受けられません（${logBytes(currentDownloadSize)}）。`);
      dataBytes = null;
      cancelPending = true;   // 届く分は捨てる。本体へ中断を送り、3 秒なにも来なくなったら終わりにする
      lastSerialRxMs = performance.now();
      sendSerialLine("PONS:CANCEL").catch(() => {});
      if (!cancelWatchTimer) cancelWatchTimer = setInterval(() => {
        if (cancelPending && performance.now() - lastSerialRxMs > 3000) { rxLine = []; endDownloadUi(); }
      }, 500);
      return;
    }
    dataReceived = 0;
    dataTrailerPending = false;
    if (currentDownloadSize > 0) isDataStreaming = true;
    else dataTrailerPending = true;   // 空のファイル: すぐ DATA_END が来る
  } else if (line.startsWith("PONS:DATA_END,")) {
    if (!dataTrailerPending) return;
    finishDownload(line.substring("PONS:DATA_END,".length).trim().toUpperCase());
  } else if (line === "PONS:CANCEL_OK") {
    // 塊の途中で止まったので中断を送った（rangeStallCheck）: 本体が片付いたので、同じ塊を頼み直す
    if (rangeDl && rangeDl.recovering && !cancelPending) {
      rangeDl.recovering = false;
      rangeChunkFailed(rangeDl.stallBytes);
      return;
    }
    endDownloadUi();
  } else if (line === "PONS:BYE") {
    updateSerialUIStatus('disconnected');
    resetSettingsState(true);
    if (typeof resetWipeState === 'function') resetWipeState(true);
    if (typeof resetSdInfo === 'function') resetSdInfo();
  } else if (line.startsWith("PONS:SETTING,") || line.startsWith("PONS:DEST,") || line.startsWith("PONS:PILON_INFO,") ||
             line === "PONS:SETTINGS_END" || line.startsWith("PONS:SETTINGS_OK,") ||
             line.startsWith("PONS:SETTINGS_SAVED,") || line.startsWith("PONS:SETTINGS_ERR,")) {
    handleSettingsSerialLine(line);
  } else if (line.startsWith("PONS:WIPE_") || line.startsWith("PONS:TRASH_") || line.startsWith("PONS:SDINFO,")) {
    // SD の整理と空き容量（sd_wipe.js。PONS:WIPE_FILE / WIPE_KEEP / TRASH_FILE / WIPE_SCAN_END / WIPE_PROGRESS /
    // WIPE_DONE / TRASH_EMPTY_DONE / TRASH_RESTORE_DONE / WIPE_ERR、一覧の終わりの PONS:SDINFO）
    if (typeof handleWipeSerialLine === 'function') handleWipeSerialLine(line);
  } else if (line.startsWith("PONS:ERROR,")) {
    if (imuRangeFallbackTimer) { clearTimeout(imuRangeFallbackTimer); imuRangeFallbackTimer = null; }
    const errMsg = line.substring(11);
    const failedFile = currentDownloadFile;
    const failedSize = currentDownloadFull;
    // 塊ごとに取っている途中の BUSY: 本体が前の塊の後始末（DATA_END のあと、タスクを終えるまで）を終える前に次を頼んだ。
    // 少し待って同じ塊を頼み直す（1 つの塊で 30 回まで。取り直しの回数には数えない）
    if (errMsg === 'BUSY' && rangeDl && !cancelPending && rangeDl.busyCount < 30) {
      const r = rangeDl;
      r.busyCount++;
      r.busyWait = true;
      r.retryTimer = setTimeout(() => {
        r.retryTimer = null;
        r.busyWait = false;
        if (rangeDl === r && !cancelPending) requestRangeChunk();
      }, 100);
      return;
    }
    if (errMsg === 'IMU_RANGE_NOT_FOUND') {
      endDownloadUi();
      if (failedFile && confirm(`生 IMU ログ（${failedFile}）内に、このフライト時刻の GNSS 記録が見つかりませんでした。\nファイル全体（${logBytes(failedSize)}）を取得しますか？`)) {
        downloadPonsFile(failedFile, failedSize, 0, 'open', null);
      }
      return;
    }
    if (errMsg === 'BUSY') {
      alert("PONSが現在別のタスクを実行中です。少し待ってから再試行してください。");
    } else if (errMsg === 'NOT_CONNECTED') {
      console.warn("PONS: NOT_CONNECTED received. Auto-reconnecting...");
      sendSerialLine("PONS:CONNECT");
    } else if (errMsg === 'FILE_NOT_FOUND') {
      alert("指定されたファイルがSDカード上に見つかりませんでした。");
    } else if (errMsg === 'SD_NOT_READY') {
      alert("SDカードが認識されていないかエラー状態です。PONS本体を確認してください。");
    } else if (errMsg === 'TX_TIMEOUT') {
      alert("送信タイムアウト: PC 側での受信が長時間止まったため、本体が転送を中断しました。\nタブを開いたまま再度お試しください。");
    } else {
      alert("PONSエラー: " + errMsg);
    }
    // 一覧を待っていたなら、待つのをやめる（NOT_CONNECTED は READY のあとで取り直す）
    if (listPending) {
      listPending = null;
      if (listAllTimer) { clearTimeout(listAllTimer); listAllTimer = null; }
      renderFileList();
    }
    abortSettingsPending(`PONSエラー: ${errMsg}`);
    if (typeof abortWipePending === 'function') abortWipePending(`PONSエラー: ${errMsg}`);
    endDownloadUi();
  }
}
