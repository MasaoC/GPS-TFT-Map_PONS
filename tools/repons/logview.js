// ============================================================
// File    : tools/repons/logview.js
// Project : PONS v7.1 / v7.2 Re:PONS
// Role    : log.txt の表示（起動ごとに分け、毎分の行をグラフに、行を種類と重さで色分けして絞り込む）。
//           行の形はファームの log_sd() に合わせる（logScanLine / LOG_CATS）
// Author  : MasaoC (@masao_mobile)
// Updated : 2026-11-23
// ============================================================
//
// 2026-11-23 に index.html から分けた。読む順は index.html の一覧どおり。関数・変数はファイルをまたいで
// グローバルで見えるが、読み込みの最中に動くコードは先に読んだファイルのものしか使えない。

// ============================================================
// log.txt の表示（2026-11-11〜）
//   SD の log.txt（1 行 = "<起動からの秒>:<本文>"。ファームの log_sd()、mysd.cpp）を起動ごとに分け、
//   毎分の行（volt= / grf / imu gapmax= / CHG）をグラフにし、行を種類と重さで色分けして絞り込めるようにする。
//   PONS から PONS:GETTAIL / PONS:GET で受けたものと、手元の log.txt を開いたものの両方をここで見る。
//   ★ 行の形はファームの都合で変わるので、読めない行は「その他」として残し、捨てない。
// ============================================================
const LOG_TAIL_BYTES = 1024 * 1024;   // 一覧の「最新」で取る量（log.txt は消されずに伸び続けるので、ふだんは末尾だけ）
const LOG_ROW_H = 20;                 // 行の高さ [px]（CSS の .lrow と同じ。見えている行だけ描く）
const LOG_CATS = [
  { id: 'boot',  label: '起動',       color: '#475569', re: /^(BOOT|SD INIT|SETTINGS|SETUP DONE|AIRSPEED MODEL|DESTINATIONS|PILON|Initial airdata)/ },
  { id: 'time',  label: 'DBD・時刻',  color: '#7c3aed', re: /^(GNSS DBD|GNSS TIME|GNSS ORB|GNSS ID|GNSS VER|RTC BACKUP)/ },
  { id: 'gnss',  label: 'GNSS',       color: '#0284c7', re: /^(GNSS|grf|GRF|WARN:sat|ACC hAcc|NAV)/ },
  { id: 'pc',    label: 'PC・SD',     color: '#64748b', re: /^(PC |USB DISCONNECTED|SD |REPLAY|ERR REPLAY|DEMO)/ },
  { id: 'power', label: '電源・充電', color: '#16a34a', re: /^(volt=|CHG|BATT|Battery|POWER|PWR|CHARGE|USB|BACKLIGHT|SW )/ },
  { id: 'link',  label: '無線',       color: '#ea580c', re: /^(LINK|E220)/ },
  { id: 'imu',   label: 'IMU・姿勢',  color: '#0d9488', re: /^(imu|raw |rate |eskf|cal |quat|SCH16T|BNO085|MS5611|VARIO|GROUND|BANK|ESKF|IMU|ATT|ROLL TRIM|WIND|redraw)/ },
  { id: 'other', label: 'その他',     color: '#94a3b8', re: /./ },
];
// 重さ（行の色）。目安の正規表現なので、ファームの言い回しが変わればここに足す。
const LOG_SEV_ERR  = /(^ERR\b|\bERROR\b|FAIL|\b[Ff]ault|\bFAULT|stuck|broken|not written|OVERDISCHARGE|NO RESPONSE|SD_NOT_READY)/;
const LOG_SEV_WARN = /(WARN|Battery low|MUTED|REQUIRED|refused|no answer|too big|not saved|aborted|TIMEOUT|timeout|no RF|restarted|TOO HOT|CHARGE HOT:|NO ACK|NOT MEASURED|not restored \((?!rtc alive)|\*\* more than acc)/;
// グラフ（毎分の行から）。点が 2 つ以上ある系列のあるものだけ出す。
const LOG_CHARTS = [
  { title: '電池 [V]', series: [['volt', '#16a34a', '測った電圧'], ['cell', '#2563eb', '推定セル電圧（充電中）']], band: 'usb', bandLabel: '青い帯 = USB' },
  { title: 'GNSS C/N0 [dB-Hz]', series: [['cno4', '#0284c7', '強い 4 衛星'], ['cnoall', '#7dd3fc', '全体']] },
  { title: 'GNSS 衛星数', series: [['used', '#0284c7', '使用'], ['trk', '#94a3b8', '追尾']], ymin: 0 },
  { title: 'GNSS Fix の割合 [%]', series: [['fixpct', '#16a34a', '1 分のうち']], ymin: 0, ymax: 100 },
  { title: 'CPU 温度 [°C]', series: [['cpu', '#ea580c', 'CPU']] },
  { title: 'IMU 読み出しの最大の空き [ms]', series: [['gap', '#0d9488', 'gapmax（読み出しの間隔）'], ['drain', '#f59e0b', 'drain（推定へ渡すまでの遅れ）']], ymin: 0 },
  { title: '地図の描き直しの最長 [ms]', series: [['rdmax', '#475569', '全体'], ['rdmap', '#0d9488', '地図'], ['rdtrack', '#16a34a', '航跡'], ['rdpush', '#7c3aed', '転送']], ymin: 0 },
  { title: 'BT1（GNSS の時計の電池）の残り [h]', series: [['bk', '#7c3aed', '推定']], ymin: 0 },
  { title: '充電の残量 [%]', series: [['soc', '#2563eb', '充電中の推定']], ymin: 0, ymax: 100 },
  { title: '画面の明るさ [%]', series: [['bl', '#d97706', 'バックライト']], ymin: 0, ymax: 100 },
  { title: 'GNSS 妨害の指標 jamInd（1 分の最大）', series: [['jmax', '#dc2626', 'jmax']], ymin: 0 },
];
let logState = null;

function logCatOf(msg) {
  for (let i = 0; i < LOG_CATS.length; i++) if (LOG_CATS[i].re.test(msg)) return i;
  return LOG_CATS.length - 1;
}
function logSevOf(msg) { return LOG_SEV_ERR.test(msg) ? 2 : (LOG_SEV_WARN.test(msg) ? 1 : 0); }
function logPad(n) { return String(n).padStart(2, '0'); }
function logPush(c, key, s, v) {
  if (!Number.isFinite(v)) return;
  (c.series[key] || (c.series[key] = [])).push([s, v]);
}

// 起動ごとに分ける。区切りは「行頭の秒が巻き戻った」か、起動の始めの数秒に 2 回目の SD INIT / BOOT PONS が来たとき
// （起動して数秒で落ちたとき、次の起動も 0 秒から始まって巻き戻らないため）。
function parseLogText(text) {
  const raw = text.split(/\r?\n/);
  const lines = [];
  const sessions = [];
  let cur = null, prevS = -1;
  const newSess = (partial) => {
    cur = {
      idx: sessions.length, firstLine: lines.length, lastLine: lines.length, partial: partial,
      s0: null, s1: 0, ver: '', build: '', board: '', imu: '', baseUtc: null, baseJst: null,
      ttff: null, fixTag: '', nErr: 0, nWarn: 0, hasBoot: false, hasSdInit: false,
      series: {}, marks: [], lastVal: {},
    };
    sessions.push(cur);
  };
  for (const r of raw) {
    if (!r) continue;
    const m = /^(\d{1,7}):(.*)$/.exec(r);
    const s = m ? parseInt(m[1], 10) : Math.max(prevS, 0);   // 秒の無い行（壊れた行）は前の行の秒
    const msg = m ? m[2] : r;
    const isSd = /^SD INIT\b/.test(msg), isBoot = /^BOOT PONS /.test(msg);
    if (!cur) {
      // ファイルの頭。末尾だけ取ったときは、起動の途中から始まる
      newSess(!(m && s <= 2 && (isSd || isBoot || /^BOOT WATCH/.test(msg))));
    } else if (m && (s < prevS || (s <= 2 && ((isSd && cur.hasSdInit) || (isBoot && cur.hasBoot))))) {
      newSess(false);
    }
    if (m) prevS = s;
    const sev = logSevOf(msg);
    const li = lines.length;
    lines.push({ s: s, msg: msg, cat: logCatOf(msg), sev: sev, sess: cur.idx });
    cur.lastLine = li;
    if (cur.s0 === null) cur.s0 = s;
    cur.s1 = s;
    if (sev === 2) { cur.nErr++; cur.marks.push({ s: s, c: '#dc2626' }); }
    else if (sev === 1) cur.nWarn++;
    if (isSd) cur.hasSdInit = true;
    logScanLine(cur, s, msg);
  }
  return { lines: lines, sessions: sessions };
}

// 1 行から、起動のまとめとグラフの値を拾う
function logScanLine(c, s, msg) {
  let m, k;
  if ((m = /^BOOT PONS (\S+) build (\d+) board=(.*?) imu=(\S+)/.exec(msg))) {
    c.hasBoot = true; c.ver = m[1]; c.build = m[2]; c.board = m[3]; c.imu = m[4];
    // 2026-11-12〜 末尾にコンパイルした日時（同じ build のまま焼き直したビルドを区別するため）
    if ((k = / compiled (\d{4}-\d\d-\d\d \d\d:\d\d)/.exec(msg))) c.compiled = k[1];
    return;
  }
  if (c.baseUtc === null && (m = /^GNSS TIME: (\d{4})-(\d\d)-(\d\d) (\d\d):(\d\d):(\d\d) UTC/.exec(msg))) {
    c.baseUtc = Date.UTC(+m[1], +m[2] - 1, +m[3], +m[4], +m[5], +m[6]) / 1000 - s;
    return;
  }
  if (/^GNSS FIXED\b/.test(msg)) {
    c.marks.push({ s: s, c: '#16a34a' });   // 測位し直したときも縦の線を出す
    if (c.ttff === null) {
      // 今のファームは "... 35.2s after boot rtc=lost dbd=..."。古いファームは秒が無いので、行の秒（起動からの秒）で代える
      if ((m = /([\d.]+)s after boot ?(.*)$/.exec(msg))) { c.ttff = parseFloat(m[1]); c.fixTag = (m[2] || '').trim(); }
      else if (!c.partial || c.s0 === 0) { c.ttff = s; c.fixTag = '（行の時刻から）'; }
    }
    return;
  }
  if ((m = /^volt=([\d.]+)V(.*)$/.exec(msg))) {
    const rest = m[2];
    logPush(c, 'volt', s, parseFloat(m[1]));
    if ((k = /usb=(\d)/.exec(rest))) logPush(c, 'usb', s, +k[1]);
    if ((k = /bk=([\d.]+)h/.exec(rest))) logPush(c, 'bk', s, parseFloat(k[1]));
    if ((k = /bl=(\d+)%/.exec(rest))) logPush(c, 'bl', s, +k[1]);
    if ((k = /cpu=(-?[\d.]+)C/.exec(rest))) logPush(c, 'cpu', s, parseFloat(k[1]));
    // 時刻（分まで）。GNSS TIME の行が無い起動の目安（分の真ん中とみなす）
    if (c.baseJst === null && (k = /(\d\d):(\d\d) JST/.exec(rest))) c.baseJst = (+k[1]) * 3600 + (+k[2]) * 60 + 30 - s;
    return;
  }
  if ((m = /^grf n=(\d+) .*?jmax=(\d+) .*?cno=([\d.]+)\/([\d.]+) trk=([\d.]+) used=([\d.]+) fix=(\d+)%/.exec(msg))) {
    logPush(c, 'jmax', s, +m[2]);
    logPush(c, 'cno4', s, parseFloat(m[3])); logPush(c, 'cnoall', s, parseFloat(m[4]));
    logPush(c, 'trk', s, parseFloat(m[5])); logPush(c, 'used', s, parseFloat(m[6]));
    logPush(c, 'fixpct', s, +m[7]);
    return;
  }
  if ((m = /^imu gapmax=(\d+)ms/.exec(msg))) {
    logPush(c, 'gap', s, +m[1]);
    // 2026-11-11〜（IMU をタイマー割り込みで読む）: 推定へ渡すまでの最長。無い行（それより前のファーム）は gap だけ
    if ((k = /drain=(\d+)ms/.exec(msg))) logPush(c, 'drain', s, +k[1]);
    return;
  }
  if ((m = /^redraw n=\d+ max=(\d+)ms .*?map=(\d+) .*?track=(\d+) .*?push=(\d+)/.exec(msg))) {
    logPush(c, 'rdmax', s, +m[1]); logPush(c, 'rdmap', s, +m[2]);
    logPush(c, 'rdtrack', s, +m[3]); logPush(c, 'rdpush', s, +m[4]);
    return;
  }
  if ((m = /^CHG (?:\[\d+m\]|START): .*?cell=([\d.]+)V soc=(\d+)%/.exec(msg))) {
    logPush(c, 'cell', s, parseFloat(m[1])); logPush(c, 'soc', s, +m[2]);
  }
  if (/^CHG (START|UNPLUG)/.test(msg)) c.marks.push({ s: s, c: '#2563eb' });
  if (/^BOOT WATCH/.test(msg)) c.marks.push({ s: s, c: '#dc2626' });
  if ((m = /^LINK TX: mod=\S+ .*?sent=(\d+) noRf=(\d+)/.exec(msg))) c.lastVal.ltx = { sent: +m[1], noRf: +m[2] };
  if ((m = /^LINK RX: mod=\S+ .*?rx=(\d+) miss=(\d+)/.exec(msg))) c.lastVal.lrx = { rx: +m[1], miss: +m[2] };
}

// 時刻の表示。GNSS TIME の行があれば JST の日付と秒まで、無ければ毎分の行の「HH:MM JST」からの目安（~ を付ける）
function logClock(c, s) {
  if (c.baseUtc !== null) {
    const d = new Date((c.baseUtc + s + 9 * 3600) * 1000);
    return `${logPad(d.getUTCMonth() + 1)}/${logPad(d.getUTCDate())} ${logPad(d.getUTCHours())}:${logPad(d.getUTCMinutes())}:${logPad(d.getUTCSeconds())}`;
  }
  if (c.baseJst !== null) {
    const t = (((c.baseJst + s) % 86400) + 86400) % 86400;
    return `~${logPad(Math.floor(t / 3600))}:${logPad(Math.floor(t / 60) % 60)}:${logPad(t % 60)}`;
  }
  return '';
}
function logClockShort(c, s) {   // グラフの目盛り（HH:MM。分からなければ起動からの分）
  if (c.baseUtc !== null || c.baseJst !== null) {
    const base = c.baseUtc !== null ? c.baseUtc + 9 * 3600 : c.baseJst;
    const t = (((base + s) % 86400) + 86400) % 86400;
    return `${logPad(Math.floor(t / 3600))}:${logPad(Math.floor(t / 60) % 60)}`;
  }
  return `+${Math.round(s / 60)}分`;
}
function logStartText(c) {
  const s = c.s0 || 0;
  if (c.baseUtc !== null) {
    const d = new Date((c.baseUtc + s + 9 * 3600) * 1000);
    return `${d.getUTCFullYear()}-${logPad(d.getUTCMonth() + 1)}-${logPad(d.getUTCDate())} ${logPad(d.getUTCHours())}:${logPad(d.getUTCMinutes())} JST`;
  }
  if (c.baseJst !== null) return logClock(c, s).substring(0, 6) + ' JST（日付不明）';
  return '時刻不明';
}
function logRel(s) {
  const h = Math.floor(s / 3600), m = Math.floor(s / 60) % 60, x = s % 60;
  return h > 0 ? `${h}:${logPad(m)}:${logPad(x)}` : `${m}:${logPad(x)}`;
}
function logDur(sec) {
  if (sec < 60) return `${sec}秒`;
  if (sec < 3600) return `${Math.round(sec / 60)}分`;
  return `${Math.floor(sec / 3600)}時間${Math.round(sec / 60) % 60}分`;
}
function logBytes(n) { return n >= 1048576 ? (n / 1048576).toFixed(1) + ' MB' : Math.max(1, Math.round(n / 1024)) + ' KB'; }

function openLogViewer(text, name, info = {}) {
  const parsed = parseLogText(text || '');
  if (parsed.lines.length === 0) { alert('log.txt に行がありませんでした。'); return; }
  logState = {
    text: text, name: name || 'log.txt', from: info.from || 0, full: info.full || 0,
    lines: parsed.lines, sessions: parsed.sessions, sel: parsed.sessions.length - 1,
    catOff: {}, view: [], hl: -1, r0: -1, r1: -1, chartsHidden: false,
  };
  document.getElementById('log-title-name').textContent = logState.name;
  const part = logState.from > 0 ? `（全体 ${logBytes(logState.full || logState.from + text.length)} のうち最新の分）` : '';
  document.getElementById('log-info').textContent =
    `${logBytes(text.length)}${part} · ${parsed.lines.length.toLocaleString()} 行 · 起動 ${parsed.sessions.length} 回`;
  document.getElementById('log-search').value = '';
  document.getElementById('log-only-bad').checked = false;
  document.getElementById('log-all-sess').checked = false;
  logSetupOnce();
  document.getElementById('log-modal').classList.add('active');
  logRenderSessions();
  logSelectSession(logState.sel);
}

function closeLogViewer() {
  document.getElementById('log-modal').classList.remove('active');
  document.getElementById('log-tip').style.display = 'none';
}

function saveLogText() {
  if (!logState) return;
  let base = (logState.name || 'log.txt').split('/').pop() || 'log.txt';
  if (logState.from > 0) base = base.replace(/\.txt$/i, '') + '_tail.txt';
  const blob = new Blob([logState.text], { type: 'text/plain;charset=utf-8' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url; a.download = base;
  document.body.appendChild(a); a.click(); document.body.removeChild(a);
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

let logSetupDone = false;
function logSetupOnce() {
  if (logSetupDone) return;
  logSetupDone = true;
  const list = document.getElementById('log-list');
  let pending = false;
  list.addEventListener('scroll', () => {
    if (pending) return;
    pending = true;
    requestAnimationFrame(() => { pending = false; logRenderRows(false); });
  });
  window.addEventListener('keydown', (e) => {
    if (e.key === 'Escape' && document.getElementById('log-modal').classList.contains('active')) closeLogViewer();
  });
  let rt = null;
  window.addEventListener('resize', () => {
    if (!logState || !document.getElementById('log-modal').classList.contains('active')) return;
    clearTimeout(rt);
    rt = setTimeout(() => { logRenderCharts(); logRenderRows(true); }, 150);
  });
}

// 左の起動の一覧（新しい順）
function logRenderSessions() {
  const st = logState;
  const box = document.getElementById('log-sessions');
  let html = '';
  for (let i = st.sessions.length - 1; i >= 0; i--) {
    const c = st.sessions[i];
    let badges = '';
    if (c.partial) badges += '<span class="log-badge lb-part">途中から</span>';
    if (c.ttff !== null) badges += `<span class="log-badge lb-fix">測位 ${c.ttff < 100 ? c.ttff.toFixed(1) : Math.round(c.ttff)}秒</span>`;
    else if (c.s1 > 120 && !c.partial) badges += '<span class="log-badge lb-nofix">測位なし</span>';
    if (/rtc=lost/.test(c.fixTag)) badges += '<span class="log-badge lb-lost">時計切れ</span>';
    if (c.nErr) badges += `<span class="log-badge lb-err">✖${c.nErr}</span>`;
    if (c.nWarn) badges += `<span class="log-badge lb-warn">⚠${c.nWarn}</span>`;
    const ver = c.ver ? `${escapeHtml(c.ver)}` : (c.partial ? '' : '版の行なし');
    html += `<div class="log-sess${i === st.sel ? ' active' : ''}" onclick="logSelectSession(${i})">
            <div><span class="ls-when">#${i + 1} ${escapeHtml(logStartText(c))}</span></div>
            <div class="ls-sub">${logDur(c.s1)}${ver ? ' · ' + ver : ''}</div>
            <div>${badges}</div>
          </div>`;
  }
  box.innerHTML = html;
}

function logSelectSession(i) {
  const st = logState;
  if (!st || i < 0 || i >= st.sessions.length) return;
  st.sel = i;
  st.hl = -1;
  document.querySelectorAll('#log-sessions .log-sess').forEach((el, k) => {
    el.classList.toggle('active', st.sessions.length - 1 - k === i);
  });
  logRenderSummary();
  logRenderCharts();
  logRenderChips();
  document.getElementById('log-list').scrollTop = 0;
  logApplyFilter();
}

// 選んだ起動のまとめ
function logRenderSummary() {
  const st = logState, c = st.sessions[st.sel], ser = c.series;
  const cards = [];
  const card = (k, v, extra = '', onclick = '') =>
    cards.push(`<div class="log-card${onclick ? ' clickable' : ''}"${onclick ? ` onclick="${onclick}"` : ''}><div class="lc-k">${k}</div><div class="lc-v">${v}</div>${extra ? `<div class="lc-k">${extra}</div>` : ''}</div>`);
  const last = a => a[a.length - 1][1];
  const avg = a => a.reduce((x, p) => x + p[1], 0) / a.length;
  const min = a => a.reduce((x, p) => Math.min(x, p[1]), Infinity);
  const max = a => a.reduce((x, p) => Math.max(x, p[1]), -Infinity);
  card('起動', escapeHtml(logStartText(c)),
       c.ver ? escapeHtml(`${c.ver} build ${c.build}${c.compiled ? ` (${c.compiled})` : ''} · ${c.board} · ${c.imu}`) : (c.partial ? 'ログの途中から（起動の行は前の部分）' : '版の行なし'));
  card('記録の長さ', (c.partial ? '〜' : '') + logDur(c.s1), `${(c.lastLine - c.firstLine + 1).toLocaleString()} 行`);
  if (c.ttff !== null) card('最初の測位', `${c.ttff.toFixed(1)} 秒`, escapeHtml(c.fixTag) || '');
  else card('最初の測位', 'なし', c.partial ? 'この部分には無い' : '');
  if (ser.volt) card('電池', `${ser.volt[0][1].toFixed(2)} → ${last(ser.volt).toFixed(2)} V`, `最低 ${min(ser.volt).toFixed(2)} V（毎分の値）`);
  if (ser.cpu) card('CPU 温度', `最高 ${max(ser.cpu).toFixed(1)} °C`, `平均 ${avg(ser.cpu).toFixed(1)} °C`);
  if (ser.fixpct) card('GNSS', `Fix ${avg(ser.fixpct).toFixed(0)} %`,
       `C/N0 ${ser.cno4 ? avg(ser.cno4).toFixed(1) : '--'} dB-Hz · 使用 ${ser.used ? avg(ser.used).toFixed(1) : '--'} 衛星（平均）`);
  if (ser.gap) card('IMU 読み出しの空き', `最大 ${max(ser.gap)} ms`, '数十 ms を超えたら Core0 の詰まりを疑う');
  if (ser.bk) card('BT1 の残り（推定）', `${last(ser.bk).toFixed(1)} h`, '最後の毎分の行');
  if (c.lastVal.ltx) card('無線（送信）', `sent ${c.lastVal.ltx.sent}`, `noRf ${c.lastVal.ltx.noRf}（モジュールに捨てられた数）`);
  if (c.lastVal.lrx) card('無線（受信）', `rx ${c.lastVal.lrx.rx}`, `miss ${c.lastVal.lrx.miss}`);
  card('エラー / 警告', `<span style="color:#b91c1c">✖ ${c.nErr}</span> / <span style="color:#b45309">⚠ ${c.nWarn}</span>`,
       'クリックでエラーと警告だけを表示', 'logShowBadOnly()');
  document.getElementById('log-summary').innerHTML = cards.join('');
}

function logShowBadOnly() {
  document.getElementById('log-only-bad').checked = true;
  logApplyFilter();
  document.getElementById('log-toolbar').scrollIntoView({ block: 'start', behavior: 'smooth' });
}

function toggleLogCharts() {
  logState.chartsHidden = !logState.chartsHidden;
  document.getElementById('log-charts-toggle').textContent = logState.chartsHidden ? '表示' : '隠す';
  logRenderCharts();
}

// ---- グラフ ----
function logRenderCharts() {
  const st = logState, c = st.sessions[st.sel];
  const box = document.getElementById('log-charts');
  box.innerHTML = '';
  if (st.chartsHidden) return;
  const cfgs = LOG_CHARTS.filter(cfg => cfg.series.some(([k]) => (c.series[k] || []).length >= 2));
  if (cfgs.length === 0) {
    box.innerHTML = '<div style="font-size:0.8rem; color:#94a3b8;">この起動には毎分の記録（volt= / grf など）がありません（1 分たたずに切れた・途中から取った など）。</div>';
    return;
  }
  for (const cfg of cfgs) {
    const div = document.createElement('div');
    div.className = 'log-chart';
    const leg = cfg.series.filter(([k]) => c.series[k])
      .map(([k, col, lab]) => `<span class="lch-leg"><span style="color:${col}">━</span> ${lab}</span>`).join('');
    div.innerHTML = `<div class="lch-title">${cfg.title}${leg}${cfg.bandLabel && c.series[cfg.band] ? `<span class="lch-leg">${cfg.bandLabel}</span>` : ''}</div>`;
    const cv = document.createElement('canvas');
    div.appendChild(cv);
    box.appendChild(div);
    cv._cfg = cfg;
    cv._hover = null;
    cv.addEventListener('mousemove', (e) => logChartHover(cv, e));
    cv.addEventListener('mouseleave', () => { cv._hover = null; logDrawChart(cv); document.getElementById('log-tip').style.display = 'none'; });
    cv.addEventListener('click', (e) => { const s = logChartS(cv, e); if (s !== null) logJumpTo(s); });
    logDrawChart(cv);
  }
}

const LOG_PAD = { l: 44, r: 10, t: 6, b: 18 };
function logChartRange(c) {
  const t0 = c.s0 || 0;
  return [t0, Math.max(c.s1, t0 + 60)];
}
function logChartS(cv, e) {
  const c = logState.sessions[logState.sel];
  const r = cv.getBoundingClientRect();
  const x = e.clientX - r.left;
  const [t0, t1] = logChartRange(c);
  const w = r.width - LOG_PAD.l - LOG_PAD.r;
  if (x < LOG_PAD.l || x > LOG_PAD.l + w) return null;
  return Math.round(t0 + (x - LOG_PAD.l) / w * (t1 - t0));
}
function logNiceStep(range, n) {
  const raw = range / n, p = Math.pow(10, Math.floor(Math.log10(raw)));
  const f = raw / p;
  return (f < 1.5 ? 1 : f < 3 ? 2 : f < 7 ? 5 : 10) * p;
}
function logDrawChart(cv) {
  const cfg = cv._cfg, c = logState.sessions[logState.sel];
  const dpr = window.devicePixelRatio || 1;
  const W = cv.clientWidth, H = cv.clientHeight;
  if (cv.width !== Math.round(W * dpr) || cv.height !== Math.round(H * dpr)) { cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr); }
  const ctx = cv.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, W, H);
  const [t0, t1] = logChartRange(c);
  const pw = W - LOG_PAD.l - LOG_PAD.r, ph = H - LOG_PAD.t - LOG_PAD.b;
  const X = t => LOG_PAD.l + (t - t0) / (t1 - t0) * pw;
  let lo = Infinity, hi = -Infinity;
  for (const [k] of cfg.series) for (const p of (c.series[k] || [])) { if (p[1] < lo) lo = p[1]; if (p[1] > hi) hi = p[1]; }
  if (!Number.isFinite(lo)) return;
  if (cfg.ymin !== undefined) lo = Math.min(lo, cfg.ymin);
  if (cfg.ymax !== undefined) hi = Math.max(hi, cfg.ymax);
  if (hi - lo < 1e-6) { hi += 1; lo -= 1; }
  if (cfg.ymin === undefined || cfg.ymax === undefined) {
    const pad = (hi - lo) * 0.08;
    if (cfg.ymin === undefined) lo -= pad;
    if (cfg.ymax === undefined) hi += pad;
  }
  const Y = v => LOG_PAD.t + (1 - (v - lo) / (hi - lo)) * ph;
  // USB の帯（毎分の行は、その前の 1 分の様子）
  if (cfg.band && c.series[cfg.band]) {
    ctx.fillStyle = 'rgba(37, 99, 235, 0.10)';
    for (const [s, v] of c.series[cfg.band]) if (v) ctx.fillRect(X(Math.max(t0, s - 60)), LOG_PAD.t, Math.max(1, X(s) - X(Math.max(t0, s - 60))), ph);
  }
  // 目盛り
  ctx.strokeStyle = '#e2e8f0'; ctx.fillStyle = '#64748b'; ctx.lineWidth = 1;
  ctx.font = '10px -apple-system, BlinkMacSystemFont, sans-serif';
  const ys = logNiceStep(hi - lo, 4);
  ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
  for (let v = Math.ceil(lo / ys) * ys; v <= hi + 1e-9; v += ys) {
    const y = Math.round(Y(v)) + 0.5;
    ctx.beginPath(); ctx.moveTo(LOG_PAD.l, y); ctx.lineTo(LOG_PAD.l + pw, y); ctx.stroke();
    ctx.fillText(Math.abs(ys) < 1 ? v.toFixed(ys < 0.1 ? 2 : 1) : String(Math.round(v)), LOG_PAD.l - 4, y);
  }
  const xs = logNiceStep(t1 - t0, Math.max(2, Math.floor(pw / 90)));
  const xstep = xs < 60 ? 60 : Math.round(xs / 60) * 60;   // 分単位
  ctx.textAlign = 'center'; ctx.textBaseline = 'top';
  for (let t = Math.ceil(t0 / xstep) * xstep; t <= t1; t += xstep) {
    const x = Math.round(X(t)) + 0.5;
    ctx.beginPath(); ctx.moveTo(x, LOG_PAD.t); ctx.lineTo(x, LOG_PAD.t + ph); ctx.stroke();
    if (x - LOG_PAD.l < 16 || LOG_PAD.l + pw - x < 16) continue;   // 端の目盛りの字は縦軸の字と重なるので出さない
    ctx.fillText(logClockShort(c, t), x, LOG_PAD.t + ph + 3);
  }
  // 出来事の縦線（測位・エラー・USB）
  for (const mk of c.marks) {
    if (mk.s < t0 || mk.s > t1) continue;
    ctx.strokeStyle = mk.c; ctx.globalAlpha = 0.55; ctx.setLineDash([3, 3]);
    const x = Math.round(X(mk.s)) + 0.5;
    ctx.beginPath(); ctx.moveTo(x, LOG_PAD.t); ctx.lineTo(x, LOG_PAD.t + ph); ctx.stroke();
  }
  ctx.setLineDash([]); ctx.globalAlpha = 1;
  // 系列（2 分以上あいたら線を切る）
  for (const [k, col] of cfg.series) {
    const pts = c.series[k];
    if (!pts || pts.length === 0) continue;
    ctx.strokeStyle = col; ctx.fillStyle = col; ctx.lineWidth = 1.6;
    ctx.beginPath();
    let prev = null;
    for (const [s, v] of pts) {
      const x = X(s), y = Y(v);
      if (prev === null || s - prev > 150) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      prev = s;
    }
    ctx.stroke();
    if (pts.length <= 40) for (const [s, v] of pts) { ctx.beginPath(); ctx.arc(X(s), Y(v), 2, 0, Math.PI * 2); ctx.fill(); }
  }
  // マウスの位置
  if (cv._hover !== null) {
    const x = Math.round(X(cv._hover)) + 0.5;
    ctx.strokeStyle = '#0f172a'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(x, LOG_PAD.t); ctx.lineTo(x, LOG_PAD.t + ph); ctx.stroke();
  }
  ctx.strokeStyle = '#cbd5e1'; ctx.strokeRect(LOG_PAD.l + 0.5, LOG_PAD.t + 0.5, pw, ph);
}
function logChartHover(cv, e) {
  const s = logChartS(cv, e);
  const tip = document.getElementById('log-tip');
  if (s === null) { cv._hover = null; logDrawChart(cv); tip.style.display = 'none'; return; }
  cv._hover = s;
  logDrawChart(cv);
  const c = logState.sessions[logState.sel];
  let txt = `${logClock(c, s) || ''}  (起動 +${logRel(s)})`;
  for (const [k, , lab] of cv._cfg.series) {
    const pts = c.series[k];
    if (!pts) continue;
    let best = null;
    for (const p of pts) if (best === null || Math.abs(p[0] - s) < Math.abs(best[0] - s)) best = p;
    if (best && Math.abs(best[0] - s) <= 90) txt += `\n${lab}: ${best[1]}`;
  }
  tip.textContent = txt + '\nクリックでこの時刻の行へ';
  tip.style.display = 'block';
  tip.style.left = Math.min(window.innerWidth - 240, e.clientX + 14) + 'px';
  tip.style.top = (e.clientY + 14) + 'px';
}

// ---- 絞り込みと行の一覧 ----
function logRenderChips() {
  const st = logState, c = st.sessions[st.sel];
  const n = new Array(LOG_CATS.length).fill(0);
  for (let i = c.firstLine; i <= c.lastLine; i++) n[st.lines[i].cat]++;
  document.getElementById('log-chips').innerHTML = LOG_CATS.map((cat, i) =>
    `<span class="log-chip${st.catOff[i] ? ' off' : ''}" onclick="logToggleCat(${i})"><span class="dot" style="background:${cat.color}"></span>${cat.label} ${n[i]}</span>`).join(' ');
}
function logToggleCat(i) {
  logState.catOff[i] = !logState.catOff[i];
  logRenderChips();
  logApplyFilter();
}
function logFilterChanged() { if (logState) logApplyFilter(); }

function logApplyFilter() {
  const st = logState;
  const q = (document.getElementById('log-search').value || '').trim().toLowerCase();
  const onlyBad = document.getElementById('log-only-bad').checked;
  const all = document.getElementById('log-all-sess').checked;
  let a = 0, b = st.lines.length - 1;
  if (!all) { const c = st.sessions[st.sel]; a = c.firstLine; b = c.lastLine; }
  const view = [];
  for (let i = a; i <= b; i++) {
    const ln = st.lines[i];
    if (st.catOff[ln.cat]) continue;
    if (onlyBad && ln.sev === 0) continue;
    if (q && ln.msg.toLowerCase().indexOf(q) === -1) continue;
    view.push(i);
  }
  st.view = view;
  st.q = q;
  st.all = all;
  document.getElementById('log-list-inner').style.height = (view.length * LOG_ROW_H) + 'px';
  document.getElementById('log-count').textContent =
    `${view.length.toLocaleString()} / ${(b - a + 1).toLocaleString()} 行${all ? '（すべての起動）' : ''}`;
  logRenderRows(true);
}

function logMsgHtml(msg, q) {
  let h = escapeHtml(msg).replace(/([A-Za-z_][\w.]*)=/g, '<span class="kk">$1=</span>');
  if (q && !/[<>&"']/.test(q)) {
    const re = new RegExp('(' + q.replace(/[.*+?^${}()|[\]\\]/g, '\\$&') + ')(?![^<]*>)', 'ig');
    h = h.replace(re, '<mark>$1</mark>');
  }
  return h;
}

function logRenderRows(force) {
  const st = logState;
  if (!st) return;
  const list = document.getElementById('log-list');
  const top = list.scrollTop, h = list.clientHeight || 400;
  const i0 = Math.max(0, Math.floor(top / LOG_ROW_H) - 15);
  const i1 = Math.min(st.view.length, Math.ceil((top + h) / LOG_ROW_H) + 15);
  if (!force && i0 === st.r0 && i1 === st.r1) return;
  st.r0 = i0; st.r1 = i1;
  let html = '';
  for (let k = i0; k < i1; k++) {
    const li = st.view[k], ln = st.lines[li], c = st.sessions[ln.sess], cat = LOG_CATS[ln.cat];
    html += `<div class="lrow sev${ln.sev}${li === st.hl ? ' hl' : ''}" style="top:${k * LOG_ROW_H}px">` +
      `<span class="lt">${st.all ? '#' + (ln.sess + 1) + ' ' : ''}${logClock(c, ln.s)}</span>` +
      `<span class="ls">+${logRel(ln.s)}</span>` +
      `<span class="lc" style="background:${cat.color}">${cat.label}</span>` +
      `<span class="lm">${logMsgHtml(ln.msg, st.q)}</span></div>`;
  }
  document.getElementById('log-list-inner').innerHTML = html;
}

// グラフをクリックした時刻の行へ（絞り込みの中で、その時刻以降の最初の行）
function logJumpTo(s) {
  const st = logState;
  if (!st) return;
  if (st.all) { document.getElementById('log-all-sess').checked = false; logApplyFilter(); }
  let k = st.view.findIndex(li => st.lines[li].sess === st.sel && st.lines[li].s >= s);
  if (k === -1) k = st.view.length - 1;
  if (k < 0) return;
  st.hl = st.view[k];
  const list = document.getElementById('log-list');
  list.scrollIntoView({ block: 'nearest', behavior: 'smooth' });
  list.scrollTop = Math.max(0, k * LOG_ROW_H - list.clientHeight / 3);
  logRenderRows(true);
}
