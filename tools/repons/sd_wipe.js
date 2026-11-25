// ============================================================
// File    : tools/repons/sd_wipe.js
// Project : PONS v7.1 / v7.2 Re:PONS
// Role    : 本体の SD の整理（2026-11-25〜）。ログをゴミ箱（trash/）へ移す・ゴミ箱から戻す・ゴミ箱を空にする・SD の空き容量。
//           どれを扱うか・番号とパスワードの照合・直前の数え直しは本体がする（src/sd_wipe_rules.h・mysd.cpp）。
//           ここは本体が出した一覧を見せ、確認をとって、番号・パスワード・一覧の指紋を本体へ渡すだけ。
//           形式は docs/pons_webserial_protocol.md §4 (10)
// Author  : MasaoC (@masao_mobile)
// Updated : 2026-11-25
// ============================================================
//
// ★ パスワードはここに書かない・覚えない（照合は本体。人が打つ手間が「誤って消さない」の仕掛け）。入力欄は送ったらすぐ空にする。
// ★ 移す・空にするときは、いま画面に出している一覧の件数と指紋（本体が PONS:WIPE_SCAN_END で出した値）をそのまま送る。
//   本体は直前に数え直し、1 つでも違えば何もしない（WIPE_ERR,CHANGED）。ここで一覧を作り直したり指紋を計算したりしない。
// ★ 本体の画面の番号は 1 回頼むと（当たり外れにかかわらず）使い切られる。頼んだあとは必ず調べ直して新しい番号を出させる。

// ゴミ箱へ移す種類（本体の SDWIPE_CAT_* と同じビット）
const WIPE_CATS = [
  { bit: 1, label: '自機の飛行ログ', where: 'data/YYYY-MM-DD_HHMM.csv（0.960 までのファームがルートに書いたものも）' },
  { bit: 2, label: '受信ログ',       where: 'received/YYYY-MM-DD_HHMM.csv' },
  { bit: 4, label: '生 IMU ログ',    where: 'imuraw/YYYYMMDD.bin・nofixNNN.bin' },
  { bit: 8, label: '本体の記録',     where: 'log.txt（移したあとは、本体が新しく書き始める）' },
];
const WIPE_ALL = 0x0F;
const WIPE_PASSWORD_MAX = 32;          // 本体の SDWIPE_PASSWORD_MAX（バイト）
const WIPE_MAX_WRONG = 3;              // 本体の SDWIPE_MAX_WRONG（続けて違えると再起動まで受けない）
const WIPE_SCAN_TIMEOUT_MS = 20000;    // 一覧の返事を待つ長さ（知らないファームは何も返さない）
const WIPE_EXEC_SILENT_MS = 15000;     // 動かしている間、本体から何も来なくなってから諦めるまで（進み具合は 0.25 秒ごとに来る）

const wipeState = {
  mask: WIPE_ALL,
  phase: 'idle',        // idle / scanning / ready / executing / done / error
  op: null,             // 動かしているもの: 'wipe'（ゴミ箱へ移す）/ 'empty'（空にする）/ 'restore'（戻す）
  opBatch: '',          // 戻している回
  scan: null,           // 最後に受け終えた一覧 { mask, files, keep, count, fp, trash, tcount, tfp, pw, stopped }
  incoming: null,       // 受けている途中の一覧
  rescanQueued: false,  // 一覧を受けている間にチェックを変えた
  refreshFileList: false, // 動かし終えた: この画面の一覧を取り直したあとで、ファイルの一覧（PONS:LIST）も取り直す
  timer: null,
  waitTimer: null,      // ファイルの転送・一覧が終わるのを待っている
  lastRxMs: 0,
  progress: { done: 0, total: 0 },
  message: '',
  messageKind: '',      // error / ok / info
};
// SD の空き容量（PONS:LIST と PONS:WIPE_SCAN の PONS:SDINFO 行）。null = まだ分からない / 'err' = 本体が数えられなかった
let ponsSdInfo = null;

function wipeModalOpen() {
  const m = document.getElementById('wipe-modal');
  return !!(m && m.classList.contains('active'));
}
// 本体とやりとりしている最中か（PING を止める・ほかの操作を待たせる）
function wipeBusy() { return wipeState.phase === 'scanning' || wipeState.phase === 'executing'; }

function wipeClearTimer() {
  if (wipeState.timer) { clearInterval(wipeState.timer); clearTimeout(wipeState.timer); wipeState.timer = null; }
  if (wipeState.waitTimer) { clearTimeout(wipeState.waitTimer); wipeState.waitTimer = null; }
}

// 一覧が欠けずにそろっているか（止められた・行が抜けたものは使わない）
function wipeScanComplete(s) {
  return !!s && !s.stopped && s.files.length === s.count && s.trash.length === s.tcount;
}
// 移す・空にするの前提（一覧が今の選び方のもので、欠けていない。パスワードが使える）
function wipeScanUsable() {
  const s = wipeState.scan, ph = wipeState.phase;
  return wipeScanComplete(s) && (ph === 'ready' || ph === 'error' || ph === 'done') &&
         s.mask === (wipeState.mask || WIPE_ALL) && s.pw !== 'NONE' && s.pw !== '0';
}
function wipeAuthFilled() {
  return /^\d{4}$/.test(document.getElementById('wipe-code').value.trim()) && document.getElementById('wipe-password').value.length > 0;
}

function openWipeModal() {
  if (!serialWriter || !ponsReady) { alert('PONS に接続してから開いてください。'); return; }
  if (downloadBusy() || listPending) { alert('ファイルの転送・一覧の取得が終わってから開いてください。'); return; }
  if (typeof settingsState !== 'undefined' && (settingsState.loading || settingsState.saving)) {
    alert('本体設定の読み込み・保存が終わってから開いてください。');
    return;
  }
  wipeState.mask = WIPE_ALL;
  wipeState.scan = null;
  wipeState.message = '';
  wipeState.messageKind = '';
  wipeState.phase = 'idle';
  wipeState.op = null;
  document.getElementById('wipe-password').value = '';
  document.getElementById('wipe-code').value = '';
  document.getElementById('wipe-modal').classList.add('active');
  startWipeScan();
}

function closeWipeModal() {
  if (wipeState.phase === 'executing') {
    alert('本体が SD を動かしている途中です。終わるまで待ってください（止めるときは本体のボタンを押す）。');
    return;
  }
  wipeClearTimer();
  wipeState.phase = 'idle';
  wipeState.scan = null;
  wipeState.incoming = null;
  wipeState.rescanQueued = false;
  document.getElementById('wipe-password').value = '';
  document.getElementById('wipe-code').value = '';
  document.getElementById('wipe-modal').classList.remove('active');
}

// 「🔄 調べ直す」（知らせを消してから）
function rescanWipe() {
  wipeState.message = '';
  startWipeScan();
}

function setWipeMask(bit, on) {
  wipeState.mask = on ? (wipeState.mask | bit) : (wipeState.mask & ~bit);
  wipeState.message = '';
  startWipeScan();
}

// 本体に一覧を頼む（移す種類は選んだものだけ。ゴミ箱の中身と空き容量はいつも）。受けている途中なら、受け終えてから頼み直す
async function startWipeScan() {
  if (wipeState.phase === 'executing') return;
  if (wipeState.phase === 'scanning') { wipeState.rescanQueued = true; renderWipe(); return; }
  if (!serialWriter) return;
  // ★ ファイルの転送・一覧の最中に頼むと、本体は終わってから BUSY を返す（警告が出る）。終わるのを待ってから頼む
  if (downloadBusy() || listPending) {
    if (!wipeState.waitTimer) wipeState.waitTimer = setTimeout(() => { wipeState.waitTimer = null; if (wipeModalOpen()) startWipeScan(); }, 300);
    renderWipe();
    return;
  }
  // 何も選んでいなくても、ゴミ箱の中身を見るために頼む（本体の mask は 1〜15 なので 15 で頼み、移すほうは見せない）
  const askMask = wipeState.mask || WIPE_ALL;
  wipeState.scan = null;
  wipeState.phase = 'scanning';
  wipeState.incoming = { files: [], keep: [], trash: [] };
  wipeState.rescanQueued = false;
  renderWipe();
  wipeClearTimer();
  wipeState.timer = setTimeout(() => {
    wipeState.timer = null;
    if (wipeState.phase !== 'scanning') return;
    wipeState.phase = 'error';
    wipeState.incoming = null;
    wipeSetMessage('error', '本体から一覧の返事がありません。この機能の無いファーム（2026-11-25 より前）かもしれません。');
  }, WIPE_SCAN_TIMEOUT_MS);
  try {
    await sendSerialLine(`PONS:WIPE_SCAN,${askMask}`);
  } catch (e) {
    wipeClearTimer();
    wipeState.phase = 'error';
    wipeSetMessage('error', `送れませんでした: ${e && e.message ? e.message : e}`);
  }
}

function wipeSetMessage(kind, text) {
  wipeState.messageKind = kind;
  wipeState.message = text;
  renderWipe();
}

// PONS:SDINFO,<空き KB>,<全体 KB>,<FAT の種類> / PONS:SDINFO,ERR（一覧の終わりごとに来る）
function handleSdInfoLine(line) {
  const p = line.split(',');
  if (p[1] === 'ERR') ponsSdInfo = 'err';
  else {
    const free = parseInt(p[1], 10), total = parseInt(p[2], 10);
    if (Number.isFinite(free) && Number.isFinite(total) && total > 0) {
      ponsSdInfo = { freeKB: free, totalKB: total, fat: parseInt(p[3], 10) || 0 };
    }
  }
  renderSdInfo();
}
// つなぎ直した（別の本体かもしれない）
function resetSdInfo() { ponsSdInfo = null; renderSdInfo(); }

// KB → "12.3 GB" / "512 MB"（logBytes は MB までなので、SD の大きさには GB を使う）
function sdKbText(kb) {
  if (kb >= 1024 * 1024) return `${(kb / 1024 / 1024).toFixed(1)} GB`;
  if (kb >= 1024) return `${Math.round(kb / 1024)} MB`;
  return `${kb} KB`;
}
function sdInfoText() {
  if (ponsSdInfo === 'err') return 'SD の空き容量: 数えられませんでした';
  if (!ponsSdInfo) return '';
  const i = ponsSdInfo;
  const fat = { 12: 'FAT12', 16: 'FAT16', 32: 'FAT32', 64: 'exFAT' }[i.fat] || '';
  const usedPct = Math.round((1 - i.freeKB / i.totalKB) * 100);
  return `SD 空き ${sdKbText(i.freeKB)} / ${sdKbText(i.totalKB)}（${usedPct}% 使用${fat ? '・' + fat : ''}）`;
}
// ファイルの一覧のダイアログ（serial-sd-free）と、整理の画面（wipe-sd-free）の両方へ
function renderSdInfo() {
  const txt = sdInfoText();
  const a = document.getElementById('serial-sd-free');
  if (a) a.textContent = txt;
  const box = document.getElementById('wipe-sd-free');
  if (!box) return;
  if (!ponsSdInfo || ponsSdInfo === 'err') {
    box.innerHTML = `<span class="wipe-sd-text">${escapeHtml(txt || 'SD の空き容量を数えています...')}</span>`;
    return;
  }
  const usedPct = Math.max(0, Math.min(100, (1 - ponsSdInfo.freeKB / ponsSdInfo.totalKB) * 100));
  const col = usedPct >= 90 ? '#dc2626' : usedPct >= 75 ? '#d97706' : '#0284c7';
  box.innerHTML = `<span class="wipe-sd-text">${escapeHtml(txt)}</span>
    <div class="wipe-sd-bar"><div style="width:${usedPct.toFixed(1)}%; background:${col};"></div></div>`;
}

// serial.js の handleSerialLine から PONS:WIPE_ / PONS:TRASH_ / PONS:SDINFO の行が来る
function handleWipeSerialLine(line) {
  if (line.startsWith('PONS:SDINFO,')) { handleSdInfoLine(line); return; }
  wipeState.lastRxMs = performance.now();
  const parts = line.split(',');
  const head = parts[0];
  if (head === 'PONS:WIPE_FILE') {
    // PONS:WIPE_FILE,<種類>,<バイト数>,<パス>（パスに , が入りうるので最後にまとめる）
    if (!wipeState.incoming) return;
    wipeState.incoming.files.push({ cat: parseInt(parts[1], 10) || 0, size: parseInt(parts[2], 10) || 0, path: parts.slice(3).join(',') });
  } else if (head === 'PONS:WIPE_KEEP') {
    if (!wipeState.incoming) return;
    wipeState.incoming.keep.push({ size: parseInt(parts[1], 10) || 0, path: parts.slice(2).join(',') });
  } else if (head === 'PONS:TRASH_FILE') {
    if (!wipeState.incoming) return;
    wipeState.incoming.trash.push({ size: parseInt(parts[1], 10) || 0, path: parts.slice(2).join(',') });
  } else if (head === 'PONS:WIPE_SCAN_END') {
    // PONS:WIPE_SCAN_END,<mask>,<件数>,<指紋>,<ゴミ箱の件数>,<ゴミ箱の指紋>,<NONE|あと何回>,<OK|STOPPED>
    wipeClearTimer();
    const inc = wipeState.incoming || { files: [], keep: [], trash: [] };
    wipeState.incoming = null;
    if (wipeState.phase !== 'scanning') return;   // 閉じたあとに遅れて届いた
    wipeState.scan = {
      mask: parseInt(parts[1], 10) || 0,
      count: parseInt(parts[2], 10) || 0,
      fp: (parts[3] || '').trim().toUpperCase(),
      tcount: parseInt(parts[4], 10) || 0,
      tfp: (parts[5] || '').trim().toUpperCase(),
      pw: (parts[6] || '').trim(),
      stopped: (parts[7] || '').trim() === 'STOPPED',
      files: inc.files,
      keep: inc.keep,
      trash: inc.trash,
    };
    wipeState.phase = 'ready';
    // 受けている間にチェックを変えた・受けた一覧が選んだものと違う → 頼み直す（何も選んでいないときは 15 で頼んでいる）
    if (wipeState.rescanQueued || wipeState.scan.mask !== (wipeState.mask || WIPE_ALL)) { startWipeScan(); return; }
    if (wipeState.refreshFileList) { wipeState.refreshFileList = false; requestFileList(); }   // 動かし終えたあと（wipeFinish）
    if (wipeState.scan.stopped) wipeSetMessage('error', '一覧の途中で止まりました（切断・本体のボタン）。「🔄 調べ直す」を押してください。');
    else if (!wipeScanComplete(wipeState.scan)) {
      // 行が抜けた（USB の取りこぼし）。見せている一覧と本体の数が違うまま動かさせない
      wipeSetMessage('error', '一覧の行が欠けました。「🔄 調べ直す」を押してください。');
    } else renderWipe();
  } else if (head === 'PONS:WIPE_PROGRESS') {
    wipeState.progress = { done: parseInt(parts[1], 10) || 0, total: parseInt(parts[2], 10) || 0 };
    renderWipe();
  } else if (head === 'PONS:WIPE_DONE') {
    // PONS:WIPE_DONE,<移した数>,<移せなかった数>,<OK|STOPPED>,<回の名前>
    const moved = parseInt(parts[1], 10) || 0, failed = parseInt(parts[2], 10) || 0;
    const stopped = (parts[3] || '').trim() === 'STOPPED';
    const batch = (parts[4] || '').trim();
    let msg = `${moved} 件をゴミ箱へ移しました${batch && batch !== '-' ? `（${batchLabel(batch)}）` : ''}。ゴミ箱から元に戻せます。空き容量は「ゴミ箱を空にする」まで増えません。`;
    if (failed > 0) msg += ` ${failed} 件は移せませんでした（読み出し専用の印など）。`;
    if (stopped) msg += ' 途中で止めました（切断・本体のボタン）。';
    wipeFinish(failed > 0 || stopped ? 'error' : 'ok', msg);
  } else if (head === 'PONS:TRASH_EMPTY_DONE') {
    // PONS:TRASH_EMPTY_DONE,<消した数>,<消せなかった数>,<OK|STOPPED>
    const del = parseInt(parts[1], 10) || 0, failed = parseInt(parts[2], 10) || 0;
    const stopped = (parts[3] || '').trim() === 'STOPPED';
    let msg = `ゴミ箱の ${del} 件を消しました。`;
    if (failed > 0) msg += ` ${failed} 件は消せませんでした（読み出し専用の印など）。`;
    if (stopped) msg += ' 途中で止めました（切断・本体のボタン）。';
    ponsSdInfo = null;   // 空き容量が変わった（一覧を取り直すと本体が数え直す）
    renderSdInfo();
    wipeFinish(failed > 0 || stopped ? 'error' : 'ok', msg);
  } else if (head === 'PONS:TRASH_RESTORE_DONE') {
    // PONS:TRASH_RESTORE_DONE,<戻した数>,<名前を変えて戻した数>,<残した数>,<OK|STOPPED>
    const restored = parseInt(parts[1], 10) || 0, renamed = parseInt(parts[2], 10) || 0, left = parseInt(parts[3], 10) || 0;
    const stopped = (parts[4] || '').trim() === 'STOPPED';
    let msg = `${batchLabel(wipeState.opBatch)} の ${restored + renamed} 件を元の場所へ戻しました。`;
    if (renamed > 0) msg += ` そのうち ${renamed} 件は同じ名前があったので、名前の最後に _r${(wipeState.opBatch || '').slice(0, 3)} を付けました（log.txt など）。`;
    if (left > 0) msg += ` ${left} 件はゴミ箱に残しました（本体が書いた名前でないもの・戻せなかったもの）。`;
    if (stopped) msg += ' 途中で止めました（切断・本体のボタン）。';
    wipeFinish(left > 0 || stopped ? 'error' : 'ok', msg);
  } else if (head === 'PONS:WIPE_ERR') {
    wipeClearTimer();
    const code = (parts[1] || '').trim();
    const wasExec = wipeState.phase === 'executing';
    const op = wipeState.op;
    wipeState.phase = 'error';
    wipeState.op = null;
    wipeState.incoming = null;
    const what = op === 'empty' ? '何も消していません' : op === 'restore' ? '何も戻していません' : '何も動かしていません';
    const msgs = {
      SD_NOT_READY: 'SD カードが読めません。本体を確認してください。',
      REPLAY: '本体がリプレイ中なので移せません（再生中のファイルを開いています）。本体の設定の REPLAY/DEMO で OFF にしてから、もう一度。',
      NO_PASSWORD: '本体にパスワードが設定されていないので、使えません。ファームの settings.h の PONS_SD_WIPE_PASSWORD を決めて焼き直してください。',
      LOCKED: `パスワードを ${WIPE_MAX_WRONG} 回続けて間違えたので、本体を再起動するまで受け付けません。`,
      NO_CODE: `本体の画面の番号が切れていました（5 分）。${what}。新しい番号を出したので、本体の画面を見て入れ直してください。`,
      BAD_CODE: `本体の画面の番号が違います。${what}。つないでいる本体は合っていますか？ 新しい番号を出したので、本体の画面を見て入れ直してください。`,
      CHANGED: `確かめたあとで SD の中身が変わったので、${what}。一覧を取り直したので、もう一度確かめてください。`,
      TRASH_FULL: 'ゴミ箱の回が 999 に達したので移せません。先に「ゴミ箱を空にする」をしてください。',
      TRASH_FAILED: 'ゴミ箱のフォルダを作れませんでした（SD に trash という名前のファイルがある？）。',
      NOT_FOUND: 'その回はゴミ箱にありません（ほかで動かした？）。一覧を取り直しました。',
      BAD_ARG: '本体がコマンドの形を受け付けませんでした（Re:PONS とファームの版の違い？）。',
    };
    if (code === 'BAD_PASSWORD') {
      const left = parseInt(parts[2], 10);
      wipeSetMessage('error', `パスワードが違います。${what}。${Number.isFinite(left) ? `あと ${left} 回間違えると、本体を再起動するまで受け付けません。` : ''}新しい番号を出したので、本体の画面を見て入れ直してください。`);
    } else {
      wipeSetMessage('error', msgs[code] || `本体のエラー: ${code}`);
    }
    // ★ 番号は使い切られた（移す・空にするとき）。番号の欄を空にして調べ直し、新しい番号を出させる
    if (op === 'wipe' || op === 'empty') document.getElementById('wipe-code').value = '';
    if (wasExec) startWipeScan();
  }
}

// 移す・空にする・戻すが終わった: 知らせを出し、この画面の一覧を残りで出し直し（知らせは残る）、そのあとでファイルの一覧も取り直す。
// ★ 2 つを続けて頼まない（一覧の最中に頼むと本体が BUSY を返す）
function wipeFinish(kind, msg) {
  wipeClearTimer();
  const op = wipeState.op;
  wipeState.phase = 'done';
  wipeState.op = null;
  if (op === 'wipe' || op === 'empty') document.getElementById('wipe-code').value = '';   // 番号は使い切られた
  wipeSetMessage(kind, msg + ' 本体の log.txt にも残しました。');
  wipeState.refreshFileList = true;
  startWipeScan();
}

// serial.js の PONS:ERROR,... から（BUSY・NOT_CONNECTED など）。頼んでいた一覧・操作をやめたことにする
function abortWipePending(msg) {
  if (!wipeBusy()) return;
  wipeClearTimer();
  const wasExec = wipeState.phase === 'executing';
  wipeState.phase = 'error';
  wipeState.op = null;
  wipeState.incoming = null;
  wipeSetMessage('error', (wasExec ? '頼めませんでした（何も動かしていないはずです。本体の log.txt の PC SD の行で確かめられます）。' : '一覧を取れませんでした。') + ` ${msg || ''}`);
}

// つなぎ直した（READY）・切った（BYE・切断）
function resetWipeState(closeModal) {
  wipeClearTimer();
  wipeState.incoming = null;
  wipeState.rescanQueued = false;
  wipeState.op = null;
  if (closeModal) {
    wipeState.phase = 'idle';
    wipeState.scan = null;
    const pw = document.getElementById('wipe-password');
    if (pw) pw.value = '';
    const cd = document.getElementById('wipe-code');
    if (cd) cd.value = '';
    const m = document.getElementById('wipe-modal');
    if (m) m.classList.remove('active');
    return;
  }
  if (wipeModalOpen()) {
    wipeState.phase = 'idle';
    startWipeScan();
  }
}

// 移す・空にする・戻すを本体へ頼む（共通の後半）
async function wipeSend(op, line, total, batch) {
  wipeState.phase = 'executing';
  wipeState.op = op;
  wipeState.opBatch = batch || '';
  wipeState.progress = { done: 0, total };
  wipeState.message = '';
  renderWipe();
  wipeState.lastRxMs = performance.now();
  wipeClearTimer();
  // 本体から何も来なくなったら諦める（進み具合は 0.25 秒ごとに来る）
  wipeState.timer = setInterval(() => {
    if (wipeState.phase !== 'executing') { wipeClearTimer(); return; }
    if (performance.now() - wipeState.lastRxMs > WIPE_EXEC_SILENT_MS) {
      wipeClearTimer();
      wipeState.phase = 'error';
      wipeState.op = null;
      wipeSetMessage('error', '本体から返事が来なくなりました。どこまで動いたかは、つなぎ直して一覧か本体の log.txt（PC SD の行）で確かめてください。');
    }
  }, 1000);
  try {
    await sendSerialLine(line);
  } catch (e) {
    wipeClearTimer();
    wipeState.phase = 'error';
    wipeState.op = null;
    wipeSetMessage('error', `送れませんでした（何も動かしていません）: ${e && e.message ? e.message : e}`);
  }
}

// 番号とパスワードを確かめて取り出す（だめなら知らせて null）
function wipeTakeAuth() {
  const codeIn = document.getElementById('wipe-code');
  const pwIn = document.getElementById('wipe-password');
  const code = codeIn.value.trim(), password = pwIn.value;
  if (!/^\d{4}$/.test(code)) { wipeSetMessage('error', '本体の画面に出ている 4 桁の番号を入れてください。'); codeIn.focus(); return null; }
  if (!password) { wipeSetMessage('error', 'パスワードを入れてください。'); pwIn.focus(); return null; }
  if (new TextEncoder().encode(password).length > WIPE_PASSWORD_MAX) { wipeSetMessage('error', `パスワードは ${WIPE_PASSWORD_MAX} バイトまでです。`); return null; }
  if (password !== password.trim() || /[\r\n]/.test(password)) {
    wipeSetMessage('error', 'パスワードの前後に空白は使えません（本体がコマンドの行の後ろの空白を落とすため）。'); return null;
  }
  return { code, password };
}
function wipeClearPassword() { document.getElementById('wipe-password').value = ''; }

// PC の日時 "YYYYMMDD-HHMM"（ゴミ箱の回の名前。本体は測位の前に時刻を知らないことがある）
function wipeStamp() {
  const d = new Date(), z = n => String(n).padStart(2, '0');
  return `${d.getFullYear()}${z(d.getMonth() + 1)}${z(d.getDate())}-${z(d.getHours())}${z(d.getMinutes())}`;
}

// 「ゴミ箱へ移す」
async function executeWipe() {
  const s = wipeState.scan;
  if (!wipeScanUsable() || wipeState.mask === 0 || s.count === 0 || !serialWriter) return;
  if (downloadBusy() || listPending) { wipeSetMessage('error', 'ファイルの転送・一覧の取得が終わってから押してください。'); return; }
  const auth = wipeTakeAuth();
  if (!auth) return;
  const bytes = s.files.reduce((a, f) => a + f.size, 0);
  const cats = WIPE_CATS.filter(c => s.mask & c.bit).map(c => `・${c.label}: ${s.files.filter(f => f.cat === c.bit).length} 件`).join('\n');
  if (!confirm(`本体の SD の ${s.count} 件（${logBytes(bytes)}）をゴミ箱（trash/）へ移します。\n\n${cats}\n\n` +
               'ゴミ箱からは元に戻せます。空き容量は「ゴミ箱を空にする」まで増えません。\nよろしいですか？')) return;
  wipeClearPassword();   // 送ったら残さない
  await wipeSend('wipe', `PONS:WIPE,${s.mask},${s.count},${s.fp},${auth.code},${wipeStamp()},${auth.password}`, s.count);
}

// 「ゴミ箱を空にする」（ここだけが本当に消す）
async function executeTrashEmpty() {
  const s = wipeState.scan;
  if (!wipeScanUsable() || s.tcount === 0 || !serialWriter) return;
  if (downloadBusy() || listPending) { wipeSetMessage('error', 'ファイルの転送・一覧の取得が終わってから押してください。'); return; }
  const auth = wipeTakeAuth();
  if (!auth) return;
  const bytes = s.trash.reduce((a, f) => a + f.size, 0);
  const batches = trashBatches(s.trash);
  if (!confirm(`ゴミ箱の ${s.tcount} 件（${logBytes(bytes)}・${batches.length} 回分）を完全に消します。\n\n` +
               '★ 元に戻せません。要るログは先に PC へ保存してください。\n消してよければ OK を押してください。')) return;
  wipeClearPassword();
  await wipeSend('empty', `PONS:TRASH_EMPTY,${s.tcount},${s.tfp},${auth.code},${auth.password}`, s.tcount);
}

// 「元に戻す」（1 回分。番号・パスワードは要らない＝何も消さない）
async function executeTrashRestore(idx) {
  const s = wipeState.scan;
  if (!wipeScanComplete(s) || wipeBusy() || !serialWriter) return;
  if (downloadBusy() || listPending) { wipeSetMessage('error', 'ファイルの転送・一覧の取得が終わってから押してください。'); return; }
  const b = trashBatches(s.trash)[idx];
  if (!b || !b.name) return;
  if (!confirm(`ゴミ箱の ${batchLabel(b.name)} の ${b.files.length} 件を元の場所へ戻します。\n` +
               `同じ名前のファイルがあるもの（log.txt など）は上書きせず、名前の最後に _r${b.name.slice(0, 3)} を付けて戻します。\nよろしいですか？`)) return;
  await wipeSend('restore', `PONS:TRASH_RESTORE,${b.name}`, b.files.length, b.name);
}

// ゴミ箱の中身を回ごとにまとめる（trash/<回>/<元のパス>。trash/ 直下のファイルは name = '' の組）。新しい回が上
function trashBatches(trash) {
  const map = new Map();
  for (const f of trash) {
    const segs = f.path.split('/');
    const name = segs.length >= 3 ? segs[1] : '';
    if (!map.has(name)) map.set(name, { name, files: [], bytes: 0 });
    const b = map.get(name);
    b.files.push({ size: f.size, path: f.path, rel: segs.length >= 3 ? segs.slice(2).join('/') : segs.slice(1).join('/') });
    b.bytes += f.size;
  }
  return [...map.values()].sort((a, b) => b.name.localeCompare(a.name));
}
// "003_20261125-1430" → "No.3（2026-11-25 14:30）"
function batchLabel(name) {
  if (!name) return '回の外';
  const m = String(name).match(/^(\d{3})(?:_(\d{4})(\d{2})(\d{2})-(\d{2})(\d{2}))?$/);
  if (!m) return name;
  return `No.${parseInt(m[1], 10)}` + (m[2] ? `（${m[2]}-${m[3]}-${m[4]} ${m[5]}:${m[6]}）` : '');
}

function renderWipe() {
  if (!document.getElementById('wipe-modal')) return;
  renderSdInfo();
  const s = wipeState.scan;
  const ph = wipeState.phase;
  const busy = wipeBusy();
  const scanOk = ph !== 'scanning' && wipeScanComplete(s);
  const showMove = scanOk && wipeState.mask !== 0 && s.mask === wipeState.mask;

  // ① 移す種類ごとのチェックと件数
  document.getElementById('wipe-cats').innerHTML = WIPE_CATS.map(c => {
    const on = (wipeState.mask & c.bit) !== 0;
    let cnt = '<span class="wipe-cnt off">移さない</span>';
    if (on) {
      if (showMove) {
        const fs = s.files.filter(f => f.cat === c.bit);
        cnt = `<span class="wipe-cnt">${fs.length} 件・${logBytes(fs.reduce((a, f) => a + f.size, 0))}</span>`;
      } else cnt = '<span class="wipe-cnt off">…</span>';
    }
    return `<label class="wipe-cat${on ? ' on' : ''}">
        <input type="checkbox" ${on ? 'checked' : ''} ${ph === 'executing' ? 'disabled' : ''} onchange="setWipeMask(${c.bit}, this.checked)">
        <span class="wipe-cat-name">${escapeHtml(c.label)}</span>
        <span class="wipe-cat-where">${escapeHtml(c.where)}</span>
        ${cnt}
      </label>`;
  }).join('');

  const sum = document.getElementById('wipe-summary');
  if (wipeState.mask === 0) sum.innerHTML = '移す種類を選んでください。';
  else if (ph === 'scanning' || !s) sum.innerHTML = '<span style="color:#64748b;">本体の SD を調べています...</span>';
  else if (showMove) {
    const bytes = s.files.reduce((a, f) => a + f.size, 0);
    sum.innerHTML = s.count === 0
      ? '<span style="color:#15803d; font-weight:bold;">移すものはありません。</span>'
      : `ゴミ箱へ移すもの: <b class="wipe-big">${s.count} 件</b>・<b>${logBytes(bytes)}</b>`;
  } else sum.innerHTML = '';

  const fileRow = (p, size) => `<div class="wipe-file"><span>${escapeHtml(p)}</span><span>${logBytes(size)}</span></div>`;
  const LIMIT = 400;
  const listHtml = (arr, key) => arr.length === 0 ? '<div class="wipe-file muted">なし</div>'
    : arr.slice(0, LIMIT).map(f => fileRow(f[key], f.size)).join('') + (arr.length > LIMIT ? `<div class="wipe-file muted">ほか ${arr.length - LIMIT} 件</div>` : '');
  const files = showMove ? s.files : [];
  const keep = scanOk ? s.keep : [];
  document.getElementById('wipe-files-count').textContent = showMove ? `（${files.length} 件）` : '';
  document.getElementById('wipe-keep-count').textContent = scanOk ? `（${keep.length} 件）` : '';
  document.getElementById('wipe-files').innerHTML = listHtml(files, 'path');
  document.getElementById('wipe-keep').innerHTML = listHtml(keep, 'path');

  // ② ゴミ箱
  const tbox = document.getElementById('wipe-trash');
  const tsum = document.getElementById('wipe-trash-summary');
  if (!scanOk) {
    tsum.innerHTML = ph === 'scanning' || !s ? '<span style="color:#64748b;">調べています...</span>' : '';
    tbox.innerHTML = '';
  } else if (s.tcount === 0) {
    tsum.innerHTML = '<span style="color:#15803d;">ゴミ箱は空です。</span>';
    tbox.innerHTML = '';
  } else {
    const batches = trashBatches(s.trash);
    tsum.innerHTML = `ゴミ箱: <b>${s.tcount} 件</b>・<b>${logBytes(s.trash.reduce((a, f) => a + f.size, 0))}</b>（${batches.length} 回分）`;
    tbox.innerHTML = batches.map((b, i) => `
      <div class="trash-batch">
        <div class="trash-batch-head">
          <span class="trash-batch-name">${escapeHtml(batchLabel(b.name))}</span>
          <span class="trash-batch-cnt">${b.files.length} 件・${logBytes(b.bytes)}</span>
          ${b.name ? `<button onclick="executeTrashRestore(${i})" ${busy ? 'disabled' : ''} title="この回のファイルを元の場所へ戻します（上書きしません）">↩️ 元に戻す</button>`
                   : '<span class="trash-batch-cnt">（空にするときに消えます）</span>'}
        </div>
        <details class="wipe-list"><summary>中身</summary><div>${listHtml(b.files, 'rel')}</div></details>
      </div>`).join('');
  }

  // 番号とパスワード
  const pwNote = document.getElementById('wipe-pw-note');
  const pwInput = document.getElementById('wipe-password');
  const codeInput = document.getElementById('wipe-code');
  let pwBlocked = false;
  if (s && s.pw === 'NONE') {
    pwBlocked = true;
    pwNote.innerHTML = '<span class="wipe-err">本体にパスワードが設定されていないので、移す・空にするは使えません（ファームの settings.h の PONS_SD_WIPE_PASSWORD）。戻すことはできます。</span>';
  } else if (s && s.pw === '0') {
    pwBlocked = true;
    pwNote.innerHTML = '<span class="wipe-err">パスワードを続けて間違えたので、本体を再起動するまで受け付けません。戻すことはできます。</span>';
  } else {
    const left = s && /^\d+$/.test(s.pw) ? parseInt(s.pw, 10) : WIPE_MAX_WRONG;
    pwNote.textContent = '番号は PC 接続中の本体の画面に出ています（5 分で切れ、1 回使うと新しくなります）。照合は本体がします（Re:PONS はパスワードを知りません）。' +
      (left < WIPE_MAX_WRONG ? `あと ${left} 回間違えると、本体を再起動するまで受け付けません。` : `${WIPE_MAX_WRONG} 回続けて間違えると、本体を再起動するまで受け付けません。`);
  }
  pwInput.disabled = pwBlocked || ph === 'executing';
  codeInput.disabled = pwBlocked || ph === 'executing';

  // 進み具合
  const prog = document.getElementById('wipe-progress');
  if (ph === 'executing') {
    const p = wipeState.progress;
    const pct = p.total > 0 ? Math.min(100, Math.round(p.done * 100 / p.total)) : 0;
    const verb = wipeState.op === 'empty' ? '消しています' : wipeState.op === 'restore' ? '戻しています' : 'ゴミ箱へ移しています';
    prog.style.display = 'flex';
    document.getElementById('wipe-progress-fill').style.width = pct + '%';
    document.getElementById('wipe-progress-text').textContent = `本体が${verb}... ${p.done} / ${p.total}（止めるときは本体のボタンを押す）`;
  } else prog.style.display = 'none';

  // 知らせ
  const msg = document.getElementById('wipe-message');
  msg.textContent = wipeState.message;
  msg.className = 'wipe-message' + (wipeState.messageKind ? ' ' + wipeState.messageKind : '');
  msg.style.display = wipeState.message ? 'block' : 'none';

  // ボタン
  const authOk = wipeAuthFilled();
  const usable = wipeScanUsable();
  const btnMove = document.getElementById('btn-wipe-exec');
  const canMove = usable && showMove && s.count > 0 && authOk;
  btnMove.disabled = !canMove;
  btnMove.style.opacity = canMove ? '1' : '0.45';
  btnMove.textContent = showMove && s.count > 0 ? `🗑 ${s.count} 件をゴミ箱へ移す...` : '🗑 ゴミ箱へ移す...';
  const btnEmpty = document.getElementById('btn-trash-empty');
  const canEmpty = usable && s.tcount > 0 && authOk;
  btnEmpty.disabled = !canEmpty;
  btnEmpty.style.opacity = canEmpty ? '1' : '0.45';
  btnEmpty.textContent = scanOk && s.tcount > 0 ? `🔥 ゴミ箱を空にする（${s.tcount} 件を完全に消す）...` : '🔥 ゴミ箱を空にする...';
  document.getElementById('btn-wipe-rescan').disabled = busy;
}

// 入力のたびに押せる・押せないを出し直す（index.html の oninput）
function onWipeAuthInput() { renderWipe(); }
// パスワードの欄で Enter = 「ゴミ箱へ移す」（確認のダイアログは出る。空にするは押して選ぶ）
function onWipePasswordKey(ev) {
  if (ev.key === 'Enter') { ev.preventDefault(); if (!document.getElementById('btn-wipe-exec').disabled) executeWipe(); }
}
