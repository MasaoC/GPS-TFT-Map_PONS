// ============================================================
// File    : tools/flight_viewer/video_sync.js
// Project : PONS v7 / v7.1 Flight Viewer
// Role    : 大会ログに対応する YouTube 動画（限定公開）を小窓に出し、ビューアの再生に合わせて動かす
// Author  : Claude (2026-11-20)
// Updated : 2026-11-20
// ============================================================
//
// ★ 動画の URL は公開できないので、ログと同じパスワードで暗号化した tools/olddata/videos.json.enc にだけ置く
//   （olddata_open.js がログを開くときに同じパスワードで復号して setMap() へ渡す）。形は:
//     { "2026-07-26_0804_fused.csv": { "url": "https://youtu.be/...", "sync": { "video_sec": 12.5, "jst": "08:20:40.0" } }, ... }
//   sync は「動画の video_sec 秒目が JST の jst」。無ければ「ここで同期」で合わせる（合わせた値はこのブラウザに覚え、
//   videos.json に書き写せる形で出す）。
// 同期のしかた: ビューアが主、動画が従。250ms ごとにビューアの時刻から動画の時刻を出して、
//   再生中はずれが 0.5 秒（速度倍）を超えたらシーク、止まっているときはその場面に合わせる。
//   ビューアが止まっているときに動画を動かした（シーク・再生）ら、ビューアをそちらへ合わせる。
// YouTube の埋め込みは file:// では断られることがある（公開ページか http で開く）。そのときは「YouTube で開く」を出す。

(function (global) {
  'use strict';

  const VERSION = '2026.11.20.17';
  const TICK_MS = 250;
  const LS_PREFIX = 'pons_video_offset_';

  let videoMap = null;          // 復号した videos.json
  let cur = null;               // { csvName, entry, videoId, offsetSec|null }
  let player = null, playerReady = false, ytPromise = null;
  let tickTimer = null;
  let ourCmdUntil = 0;          // 自分で play/pause/seek した直後は状態の変化を「人の操作」とみなさない
  let lastExpected = null, lastSeekTarget = null, lastViewerT = null;
  let large = false;

  // ------------------------------------------------------------
  // 対応表
  // ------------------------------------------------------------
  function setMap(map) {
    videoMap = (map && typeof map === 'object') ? map : null;
  }

  function findEntry(csvName) {
    if (!videoMap || !csvName) return null;
    const base = String(csvName).replace(/^.*[\\/]/, '');
    if (videoMap[base]) return { key: base, entry: videoMap[base] };
    // 名前が違っても（手元で戻したファイルなど）日付が同じなら使う
    const d = base.match(/(20\d\d-\d\d-\d\d)/);
    if (d) {
      for (const k of Object.keys(videoMap)) if (k.includes(d[1])) return { key: k, entry: videoMap[k] };
    }
    return null;
  }

  function youtubeId(url) {
    const s = String(url || '').trim();
    if (/^[\w-]{11}$/.test(s)) return s;
    try {
      const u = new URL(s);
      if (/(^|\.)youtu\.be$/.test(u.hostname)) return u.pathname.slice(1).split('/')[0] || null;
      if (/(^|\.)youtube(-nocookie)?\.com$/.test(u.hostname)) {
        if (u.searchParams.get('v')) return u.searchParams.get('v');
        const m = u.pathname.match(/\/(embed|shorts|live|v)\/([\w-]{11})/);
        if (m) return m[2];
      }
    } catch (e) { /* 下で null */ }
    return null;
  }

  function parseJst(s) {
    const m = String(s || '').match(/(\d+):(\d+):([\d.]+)/);
    return m ? (+m[1]) * 3600 + (+m[2]) * 60 + parseFloat(m[3]) : NaN;
  }

  function fmtClock(sec) {
    if (!Number.isFinite(sec)) return '--:--:--';
    const s = ((sec % 86400) + 86400) % 86400;
    const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60), ss = s - h * 3600 - m * 60;
    return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${ss.toFixed(1).padStart(4, '0')}`;
  }

  function fmtVideo(sec) {
    if (!Number.isFinite(sec)) return '--:--';
    const m = Math.floor(sec / 60), s = sec - m * 60;
    return `${m}:${s.toFixed(1).padStart(4, '0')}`;
  }

  function lsGet(k) { try { return localStorage.getItem(k); } catch (e) { return null; } }
  function lsSet(k, v) { try { if (v === null) localStorage.removeItem(k); else localStorage.setItem(k, v); } catch (e) { /* 覚えられないだけ */ } }

  // 動画の秒 = ビューアの JST + offset
  function initialOffset(csvKey, entry) {
    const saved = parseFloat(lsGet(LS_PREFIX + csvKey));
    if (Number.isFinite(saved)) return { off: saved, src: 'このブラウザで合わせた値' };
    const sy = entry && entry.sync;
    if (sy && Number.isFinite(+sy.video_sec) && Number.isFinite(parseJst(sy.jst))) {
      return { off: (+sy.video_sec) - parseJst(sy.jst), src: 'videos.json の sync' };
    }
    return { off: null, src: '未同期' };
  }

  // ------------------------------------------------------------
  // 小窓
  // ------------------------------------------------------------
  function injectStyle() {
    if (document.getElementById('vs-style')) return;
    const st = document.createElement('style');
    st.id = 'vs-style';
    st.textContent = `
      #pons-video-panel { position: fixed; right: 12px; bottom: 84px; z-index: 2500; width: 384px; max-width: calc(100vw - 24px);
        background: #0f172a; color: #e2e8f0; border-radius: 10px; box-shadow: 0 12px 32px rgba(0,0,0,0.4); overflow: hidden;
        font-size: 0.74rem; }
      #pons-video-panel.large { width: 640px; }
      #pons-video-panel.min #vs-body { display: none; }
      body.an-modal-open #pons-video-panel { visibility: hidden; }
      #vs-head { display: flex; align-items: center; gap: 6px; padding: 5px 8px; background: #1e293b; }
      #vs-title { font-weight: 700; flex: 1; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
      #pons-video-panel button { background: #334155; color: #e2e8f0; border: 1px solid #475569; border-radius: 4px;
        padding: 2px 6px; font-size: 0.72rem; cursor: pointer; box-shadow: none; }
      #pons-video-panel button:hover { background: #475569; }
      #vs-frame-wrap { position: relative; width: 100%; aspect-ratio: 16 / 9; background: #000; }
      #vs-frame-wrap > div, #vs-frame-wrap iframe { position: absolute; inset: 0; width: 100%; height: 100%; }
      #vs-ctrl { display: flex; flex-wrap: wrap; gap: 4px; align-items: center; padding: 5px 8px; }
      #vs-status { padding: 0 8px 6px 8px; color: #94a3b8; font-family: var(--font-mono, monospace); line-height: 1.45; }
      #vs-status b { color: #fbbf24; }
      #vs-copy { width: 100%; box-sizing: border-box; margin-top: 3px; font-family: var(--font-mono, monospace); font-size: 0.7rem;
        background: #020617; color: #a5f3fc; border: 1px solid #334155; border-radius: 4px; padding: 3px 5px; }
      #vs-msg { position: absolute; inset: 0; display: none; align-items: center; justify-content: center; text-align: center;
        padding: 12px; color: #fde68a; background: rgba(2,6,23,0.85); line-height: 1.5; }
    `;
    document.head.appendChild(st);
  }

  function buildPanel() {
    injectStyle();
    let p = document.getElementById('pons-video-panel');
    if (p) return p;
    p = document.createElement('div');
    p.id = 'pons-video-panel';
    p.innerHTML = `
      <div id="vs-head">
        <span id="vs-title">▶ 大会動画</span>
        <button id="vs-min" title="小さくたたむ／戻す">▁</button>
        <button id="vs-size" title="大きさを切り替える">⤢</button>
        <button id="vs-close" title="閉じる（このログを開き直すとまた出る）">✕</button>
      </div>
      <div id="vs-body">
        <div id="vs-frame-wrap"><div id="vs-player"></div><div id="vs-msg"></div></div>
        <div id="vs-ctrl">
          <button id="vs-sync" title="動画とログで同じ場面を出してから押すと、その 2 つを同じ時刻として合わせる">⇆ ここで同期</button>
          <button data-adj="-5">−5s</button><button data-adj="-0.5">−0.5s</button>
          <button data-adj="0.5">+0.5s</button><button data-adj="5">+5s</button>
          <button id="vs-reset" title="このブラウザで合わせた値を消して、videos.json の sync（無ければ未同期）に戻す">↺</button>
          <button id="vs-follow" title="いまの動画の場面にログの再生位置を合わせる">動画→ログ</button>
          <button id="vs-open" title="YouTube で開く（いまの場面から）">YouTube</button>
        </div>
        <div id="vs-status"></div>
      </div>`;
    document.body.appendChild(p);
    p.querySelector('#vs-close').onclick = () => removeVideo();
    p.querySelector('#vs-min').onclick = () => p.classList.toggle('min');
    p.querySelector('#vs-size').onclick = () => { large = !large; p.classList.toggle('large', large); };
    p.querySelector('#vs-sync').onclick = syncHere;
    p.querySelector('#vs-follow').onclick = followVideo;
    p.querySelector('#vs-reset').onclick = resetOffset;
    p.querySelector('#vs-open').onclick = openOnYouTube;
    p.querySelectorAll('[data-adj]').forEach(b => { b.onclick = () => adjustOffset(parseFloat(b.dataset.adj)); });
    return p;
  }

  function showMsg(html) {
    const m = document.getElementById('vs-msg');
    if (!m) return;
    m.innerHTML = html || '';
    m.style.display = html ? 'flex' : 'none';
  }

  // ------------------------------------------------------------
  // YouTube プレーヤー
  // ------------------------------------------------------------
  function loadYT() {
    if (global.YT && global.YT.Player) return Promise.resolve();
    if (!ytPromise) {
      ytPromise = new Promise((resolve, reject) => {
        const prev = global.onYouTubeIframeAPIReady;
        global.onYouTubeIframeAPIReady = () => { if (typeof prev === 'function') prev(); resolve(); };
        const s = document.createElement('script');
        s.src = 'https://www.youtube.com/iframe_api';
        s.onerror = () => { ytPromise = null; reject(new Error('YouTube の読み込みに失敗しました（オフライン？）')); };
        document.head.appendChild(s);
      });
    }
    return ytPromise;
  }

  async function setVideo(csvKey, entry) {
    const id = youtubeId(entry && entry.url);
    if (!id) { console.warn('video_sync: URL から動画 ID を取れません', csvKey); removeVideo(); return; }
    const o = initialOffset(csvKey, entry);
    cur = { csvKey, entry, videoId: id, offsetSec: o.off, offsetSrc: o.src };
    lastExpected = lastSeekTarget = lastViewerT = null;
    const p = buildPanel();
    p.querySelector('#vs-title').textContent = `▶ 大会動画（${csvKey.replace(/_fused\.csv$|\.csv$/, '')}）`;
    showMsg('');
    if (player) { try { player.destroy(); } catch (e) { /* 作り直す */ } player = null; playerReady = false; }
    const wrap = p.querySelector('#vs-frame-wrap');
    if (!wrap.querySelector('#vs-player')) { const d = document.createElement('div'); d.id = 'vs-player'; wrap.prepend(d); }
    try {
      await loadYT();
    } catch (e) {
      showMsg(`${e.message}<br>「YouTube」ボタンで別のタブに開けます。`);
      startTick();
      return;
    }
    const vars = { playsinline: 1, rel: 0, modestbranding: 1, controls: 1 };
    if (location.protocol === 'http:' || location.protocol === 'https:') vars.origin = location.origin;
    player = new global.YT.Player('vs-player', {
      videoId: id,
      playerVars: vars,
      events: {
        onReady: () => { playerReady = true; tick(true); },
        onStateChange: onPlayerState,
        onError: (ev) => {
          const why = (ev.data === 101 || ev.data === 150) ? '埋め込みが許可されていない動画です（YouTube Studio の「埋め込みを許可」）。'
            : (ev.data === 100 ? '動画が見つかりません（削除・非公開？）。'
              : (location.protocol === 'file:' ? 'このページを手元のファイルとして開いていると、YouTube が埋め込みを断ることがあります。公開ページか http で開いてください。'
                : `YouTube のエラー ${ev.data}。`));
          showMsg(`${why}<br>「YouTube」ボタンで別のタブに開けます。`);
        }
      }
    });
    startTick();
  }

  function removeVideo() {
    if (tickTimer) { clearInterval(tickTimer); tickTimer = null; }
    if (player) { try { player.destroy(); } catch (e) { /* 消す */ } }
    player = null; playerReady = false; cur = null;
    const p = document.getElementById('pons-video-panel');
    if (p) p.remove();
  }

  function startTick() {
    if (!tickTimer) tickTimer = setInterval(() => tick(false), TICK_MS);
  }

  function viewer() {
    return typeof global.getFlightViewerPlayback === 'function' ? global.getFlightViewerPlayback() : null;
  }

  function expectedVideoSec(pb) {
    if (!cur || cur.offsetSec === null || !pb || pb.clockBaseSec === null || pb.clockBaseSec === undefined) return null;
    return pb.clockBaseSec + pb.currentTimeSec + cur.offsetSec;
  }

  function cmd(fn) { ourCmdUntil = performance.now() + 900; fn(); }

  function rateFor(speed) {
    const rates = (player && player.getAvailablePlaybackRates) ? player.getAvailablePlaybackRates() : [0.25, 0.5, 1, 1.5, 2];
    return rates.includes(speed) ? speed : null;
  }

  function tick(force) {
    updateStatus();
    if (!player || !playerReady || !cur) return;
    const pb = viewer();
    if (!pb || !pb.hasData) return;
    const analysisOpen = typeof global.isAnalysisModalOpen === 'function' && global.isAnalysisModalOpen();
    const st = player.getPlayerState();      // 1 再生 / 2 停止 / 3 読み込み中 / 5 頭出し
    const exp = expectedVideoSec(pb);
    if (exp === null) {                        // 未同期: 動画は人が自由に動かす
      lastViewerT = pb.currentTimeSec;
      return;
    }
    const dur = player.getDuration ? player.getDuration() : 0;
    const vt = player.getCurrentTime ? player.getCurrentTime() : 0;

    // ビューアが止まっている間に人が動画を動かした → ビューアをそちらへ
    if (!pb.isPlaying && !force && performance.now() > ourCmdUntil && lastSeekTarget !== null &&
        lastViewerT !== null && Math.abs(pb.currentTimeSec - lastViewerT) < 0.05 && Math.abs(vt - lastSeekTarget) > 1.0) {
      const t = vt - cur.offsetSec - pb.clockBaseSec;
      if (t >= 0 && t <= pb.totalDurationSec && typeof global.seekFlightViewerTo === 'function') {
        global.seekFlightViewerTo(t, false);
        lastSeekTarget = vt; lastViewerT = t;
        return;
      }
    }

    if (exp < 0 || (dur > 0 && exp > dur)) {
      if (st === 1) cmd(() => player.pauseVideo());
      lastViewerT = pb.currentTimeSec;
      return;
    }

    if (pb.isPlaying && !analysisOpen) {
      const rate = rateFor(pb.playbackSpeed);
      if (rate !== null) {
        if (player.getPlaybackRate && player.getPlaybackRate() !== rate) player.setPlaybackRate(rate);
        if (st !== 1 && st !== 3) cmd(() => player.playVideo());
        if (Math.abs(vt - exp) > 0.5 * Math.max(1, rate)) { cmd(() => player.seekTo(exp, true)); lastSeekTarget = exp; }
      } else {
        // 動画で出せない速さ（4 倍など）: 止めたまま 1 秒ごとに場面だけ合わせる
        if (st === 1) cmd(() => player.pauseVideo());
        if (lastSeekTarget === null || Math.abs(exp - lastSeekTarget) > 1.0) { cmd(() => player.seekTo(exp, true)); lastSeekTarget = exp; }
      }
    } else {
      if (st === 1) cmd(() => player.pauseVideo());
      if (force || lastSeekTarget === null || Math.abs(exp - lastSeekTarget) > 0.15) {
        cmd(() => player.seekTo(exp, true));
        lastSeekTarget = exp;
      }
    }
    lastExpected = exp;
    lastViewerT = pb.currentTimeSec;
  }

  // 人が動画の再生・一時停止を押したら、ビューアも合わせる（同期しているときだけ）
  function onPlayerState(ev) {
    if (!cur || cur.offsetSec === null || performance.now() < ourCmdUntil) return;
    const pb = viewer();
    if (!pb || !pb.hasData || typeof global.setFlightViewerPlaying !== 'function') return;
    if (ev.data === 1 && !pb.isPlaying) {
      const vt = player.getCurrentTime();
      const t = vt - cur.offsetSec - pb.clockBaseSec;
      if (t >= 0 && t <= pb.totalDurationSec && typeof global.seekFlightViewerTo === 'function') global.seekFlightViewerTo(t, false);
      global.setFlightViewerPlaying(true);
    } else if (ev.data === 2 && pb.isPlaying) {
      global.setFlightViewerPlaying(false);
    }
  }

  // ------------------------------------------------------------
  // 同期の操作
  // ------------------------------------------------------------
  function setOffset(off, src) {
    if (!cur) return;
    cur.offsetSec = off;
    cur.offsetSrc = src;
    lsSet(LS_PREFIX + cur.csvKey, off === null ? null : String(off));
    lastSeekTarget = null;
    tick(true);
  }

  function syncHere() {
    const pb = viewer();
    if (!cur || !player || !playerReady || !pb || pb.clockBaseSec === null) {
      alert('動画とログの両方が出ているときに押してください（time 列の無い CSV は合わせられません）。');
      return;
    }
    const vt = player.getCurrentTime();
    setOffset(vt - (pb.clockBaseSec + pb.currentTimeSec), 'このブラウザで合わせた値');
  }

  // このブラウザで覚えた値を消して videos.json の sync に戻す（videos.json を直したあと古い値が残らないように）
  function resetOffset() {
    if (!cur) return;
    lsSet(LS_PREFIX + cur.csvKey, null);
    const o = initialOffset(cur.csvKey, cur.entry);
    cur.offsetSec = o.off;
    cur.offsetSrc = o.src;
    lastSeekTarget = null;
    tick(true);
  }

  function adjustOffset(d) {
    if (!cur) return;
    if (cur.offsetSec === null) { alert('先に「ここで同期」で合わせてください。'); return; }
    // +0.5s = 動画をログより 0.5 秒先に進める
    setOffset(cur.offsetSec + d, 'このブラウザで合わせた値');
  }

  function followVideo() {
    const pb = viewer();
    if (!cur || cur.offsetSec === null || !player || !playerReady || !pb) return;
    const t = player.getCurrentTime() - cur.offsetSec - pb.clockBaseSec;
    if (t < 0 || t > pb.totalDurationSec) { alert('いまの動画の場面は、このログの時間の外です。'); return; }
    global.seekFlightViewerTo(t, false);
    lastSeekTarget = null;
  }

  function openOnYouTube() {
    if (!cur) return;
    let t = 0;
    if (player && playerReady) t = Math.floor(player.getCurrentTime());
    else { const e = expectedVideoSec(viewer()); if (e !== null && e > 0) t = Math.floor(e); }
    global.open(`https://www.youtube.com/watch?v=${encodeURIComponent(cur.videoId)}${t > 0 ? `&t=${t}s` : ''}`, '_blank', 'noopener');
  }

  // ★ 250ms ごとに呼ぶので innerHTML で作り直さない（書き写し用の欄が選べなくなる）。骨組みは 1 回だけ作って文字を差し替える
  function updateStatus() {
    const el = document.getElementById('vs-status');
    if (!el || !cur) return;
    if (!el.querySelector('#vs-line1')) {
      el.innerHTML = '<div id="vs-line1"></div><div id="vs-line2"></div>' +
        '<input id="vs-copy" readonly title="videos.json に書き写すとき用（クリックで全選択）">';
      el.querySelector('#vs-copy').addEventListener('click', (e) => e.target.select());
    }
    const l1 = el.querySelector('#vs-line1'), l2 = el.querySelector('#vs-line2'), cp = el.querySelector('#vs-copy');
    const pb = viewer();
    if (cur.offsetSec === null) {
      l1.innerHTML = '<b>未同期</b>: 動画とログで同じ場面（スタートの合図など）を出して「⇆ ここで同期」を押してください。';
      l2.textContent = '';
      cp.style.display = 'none';
      return;
    }
    const exp = expectedVideoSec(pb);
    const vt = (player && playerReady) ? player.getCurrentTime() : NaN;
    const dur = (player && playerReady && player.getDuration) ? player.getDuration() : 0;
    // videos.json に書き写す形（動画の 0 秒目に当たる JST を書く）
    const syncJst = -cur.offsetSec;
    const syncText = `"sync": { "video_sec": 0, "jst": "${fmtClock(syncJst)}" }`;
    const out = exp !== null && (exp < 0 || (dur > 0 && exp > dur));
    l1.innerHTML = `同期: 動画 0:00 = ${fmtClock(syncJst)}（${cur.offsetSrc}）${out ? ' <b>（この時刻は動画の範囲外）</b>' : ''}`;
    l2.textContent = `ログ ${pb && pb.clockBaseSec !== null ? fmtClock(pb.clockBaseSec + pb.currentTimeSec) : '--'} ↔ 動画 ${fmtVideo(vt)}`;
    cp.style.display = '';
    if (cp.value !== syncText) cp.value = syncText;
  }

  // ------------------------------------------------------------
  // ビューアから呼ばれる
  // ------------------------------------------------------------
  global.onFlightCsvLoaded = function (csvName) {
    const hit = findEntry(csvName);
    if (hit) setVideo(hit.key, hit.entry);
    else removeVideo();
  };

  global.PonsVideo = {
    setMap,
    hasMap: () => !!videoMap,
    count: () => (videoMap ? Object.keys(videoMap).length : 0),
    attachCurrent() {
      const pb = viewer();
      if (pb && pb.hasData) global.onFlightCsvLoaded(pb.filename);
    },
    youtubeId
  };
  global.PONS_VIDEO_SYNC_VERSION = VERSION;
})(window);
