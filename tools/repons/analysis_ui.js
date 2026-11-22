// ============================================================
// File    : tools/repons/analysis_ui.js
// Project : PONS v7 / v7.1 Re:PONS
// Role    : 飛行解析モーダル UI・インタラクティブグラフ描画・本体シーク連携
// ============================================================

(function (global) {
  'use strict';

  const DSP = global.PonsAnalysisDSP;
  const IMU = () => global.PonsImuAnalysis;

  const state = {
    initialized: false,
    open: false,
    activeTab: 'accel',
    // 解析する飛行区間（ctx.segments の添字）。動いている区間ごとに解析し、停止中は解析しない（2026-11-20）。
    // 区切り方はシークバーの青帯・動き出しの印と同じ（index.html の computeMovingSegments）
    segIdx: null,
    rangeStartFrac: 0,   // 区間の中の開始・終了（0..1）
    rangeEndFrac: 1,
    // タブ別オプション
    accelSmoothSec: 1.5,
    accelYSpan: 'auto', // 'auto' | '0.1' | '0.25' | '0.5' | '1.0' | '2.0'
    imuDirectGSmooth: 0.25,
    scatterMode: 'instab', // 'instab' | 'pitch_alt'
    scatterX: 'time',      // 'time' | 'altitude' | 'tas'
    scatterColor: 'altitude', // 'altitude' | 'tas' | 'time'
    scatterWinSec: 45,
    specChannel: 'pitch',
    phugoidPMin: 2.5,
    phugoidPMax: 10.0,
    phugoidSelectedWin: 0,
    lateralThresh: 1.5,
    compareData: null,     // { name, records, takeoff, land }
    // 生 IMU ログ (.bin)
    imuDecoded: null,
    imuMatch: null,
    imuTransferWarning: null,   // USB 転送の欠落・CRC 不一致（あれば解析画面に出し続ける）
    cadencePreset: 'hpa',
    cadenceBandLow: 2.2,
    cadenceBandHigh: 4.0,
    cadenceMinSnr: 8.0,
    cadenceAxis: 'fwd',
    hfChannel: 'az',
    lastSeekSec: null,
    // 描画済みチャートのヒットテスト登録
    hitTargets: [],
    // キャッシュ
    cacheKey: '',
    cache: {}
  };

  // ------------------------------------------------------------
  // スタイル & モーダル DOM 注入
  // ------------------------------------------------------------
  function injectStylesAndDom() {
    if (state.initialized) return;
    state.initialized = true;

    const style = document.createElement('style');
    style.textContent = `
      #analysis-modal {
        display: none;
        position: fixed;
        inset: 0;
        z-index: 2600;
        /* ★ backdrop-filter（すりガラス）は使わない（2026-11-20）。裏の地図ごと毎フレームぼかし直すので
           解析画面のスクロールがかくついていた。裏の画面は開いている間 visibility:hidden で描かせない（下の body.an-modal-open） */
        background: rgba(15, 23, 42, 0.82);
        padding: 14px;
        align-items: center;
        justify-content: center;
      }
      #analysis-modal.open { display: flex; }
      .an-shell {
        width: min(1380px, 98vw);
        height: min(92vh, 960px);
        background: #ffffff;
        border-radius: 12px;
        box-shadow: 0 24px 60px rgba(0, 0, 0, 0.35);
        display: flex;
        flex-direction: column;
        overflow: hidden;
        color: #0f172a;
      }
      .an-header {
        background: #f8fafc;
        border-bottom: 1px solid #e2e8f0;
        padding: 10px 16px;
        display: flex;
        align-items: center;
        justify-content: space-between;
        gap: 10px;
        flex-wrap: wrap;
      }
      .an-header-left {
        display: flex;
        align-items: center;
        gap: 10px;
        flex-wrap: wrap;
      }
      .an-title {
        font-weight: 700;
        font-size: 1.05rem;
        display: flex;
        align-items: center;
        gap: 6px;
      }
      .an-subtitle {
        font-size: 0.78rem;
        color: #64748b;
        font-family: var(--font-mono, monospace);
      }
      .an-header-actions {
        display: flex;
        align-items: center;
        gap: 6px;
        flex-wrap: wrap;
      }
      .an-tabs {
        display: flex;
        gap: 4px;
        padding: 8px 14px 0 14px;
        background: #f1f5f9;
        border-bottom: 1px solid #cbd5e1;
        overflow-x: auto;
      }
      .an-tab {
        padding: 7px 13px;
        border: 1px solid transparent;
        border-bottom: none;
        border-radius: 8px 8px 0 0;
        background: transparent;
        color: #475569;
        font-size: 0.84rem;
        font-weight: 600;
        cursor: pointer;
        box-shadow: none;
        white-space: nowrap;
      }
      .an-tab:hover { background: #e2e8f0; color: #0f172a; }
      .an-tab.active {
        background: #ffffff;
        color: #0284c7;
        border-color: #cbd5e1;
        margin-bottom: -1px;
        padding-bottom: 8px;
      }
      .an-seek-banner {
        display: none;
        background: #e0f2fe;
        border-bottom: 1px solid #bae6fd;
        color: #0369a1;
        padding: 6px 16px;
        font-size: 0.82rem;
        align-items: center;
        justify-content: space-between;
        gap: 10px;
      }
      .an-seek-banner.show { display: flex; }
      .an-seg-bar {
        background: #fff7ed;
        border-bottom: 1px solid #fed7aa;
        padding: 7px 16px 8px 16px;
        display: flex;
        align-items: center;
        gap: 8px;
        flex-wrap: wrap;
        font-size: 0.82rem;
        color: #7c2d12;
      }
      .an-seg-label { font-weight: 700; }
      .an-seg-bar select {
        padding: 3px 7px;
        border: 1px solid #fdba74;
        border-radius: 5px;
        font-size: 0.82rem;
        font-family: var(--font-mono, monospace);
        font-weight: 700;
        background: #fff;
        color: #9a3412;
        max-width: 100%;
      }
      .an-seg-nav { padding: 2px 8px; font-size: 0.78rem; }
      .an-seg-count { color: #9a3412; font-size: 0.78rem; }
      .an-seg-strip {
        position: relative;
        flex-basis: 100%;
        height: 16px;
        border-radius: 4px;
        overflow: hidden;
        background: repeating-linear-gradient(-45deg, #e2e8f0, #e2e8f0 4px, #f1f5f9 4px, #f1f5f9 8px);
        box-shadow: inset 0 0 0 1px #cbd5e1;
      }
      .an-seg-blk {
        position: absolute;
        top: 0;
        bottom: 0;
        min-width: 2px;
        background: #7dd3fc;
        cursor: pointer;
      }
      .an-seg-blk:hover { background: #38bdf8; }
      .an-seg-blk.sel {
        background: #fdba74;
        box-shadow: inset 0 0 0 2px #ea580c;
        z-index: 1;
      }
      .an-seg-sub {
        position: absolute;
        top: 3px;
        bottom: 3px;
        background: #ea580c;
        border-radius: 2px;
        pointer-events: none;
        z-index: 2;
      }
      .an-seg-no {
        position: absolute;
        top: 0;
        bottom: 0;
        display: flex;
        align-items: center;
        justify-content: center;
        font-size: 0.66rem;
        font-weight: 700;
        color: #0c4a6e;
        pointer-events: none;
        z-index: 3;
        overflow: hidden;
        white-space: nowrap;
      }
      .an-seg-no.sel { color: #fff; text-shadow: 0 0 2px #7c2d12; }
      .an-range-bar {
        background: #ffffff;
        border-bottom: 1px solid #e2e8f0;
        padding: 6px 16px;
        display: flex;
        align-items: center;
        gap: 14px;
        flex-wrap: wrap;
        font-size: 0.8rem;
        color: #334155;
      }
      .an-range-bar label {
        display: inline-flex;
        align-items: center;
        gap: 6px;
        font-weight: 500;
      }
      .an-range-time {
        font-family: var(--font-mono, monospace);
        font-weight: 700;
        color: #0f172a;
        min-width: 58px;
      }
      .an-body {
        flex: 1;
        overflow-y: auto;
        overscroll-behavior: contain;
        padding: 14px 16px 20px 16px;
        background: #f8fafc;
        display: flex;
        flex-direction: column;
        gap: 12px;
      }
      .an-body-inner {
        display: flex;
        flex-direction: column;
        gap: 12px;
      }
      /* 区間の帯と範囲のバー。本文と一緒にスクロールして隠れる（上に固定すると表示の場所が足りない） */
      .an-range-card {
        flex: 0 0 auto;
        border: 1px solid #fed7aa;
        border-radius: 8px;
        overflow: hidden;
      }
      .an-range-card .an-range-bar { border-bottom: none; }
      /* グラフの箱ごとに描き直しを閉じ込める（スクロール・ツールチップで他の箱を描き直させない） */
      .an-chart-box, .an-card, .an-controls { contain: layout paint; }
      /* 解析画面を開いている間は、裏の本体（地図・HUD・グラフ）を描かせない */
      body.an-modal-open > header,
      body.an-modal-open > main,
      body.an-modal-open > footer { visibility: hidden; }
      .an-controls {
        background: #ffffff;
        border: 1px solid #e2e8f0;
        border-radius: 8px;
        padding: 9px 12px;
        display: flex;
        align-items: center;
        gap: 14px;
        flex-wrap: wrap;
        font-size: 0.82rem;
      }
      .an-controls label {
        display: inline-flex;
        align-items: center;
        gap: 6px;
        color: #334155;
        font-weight: 500;
      }
      .an-controls select, .an-controls input[type="number"] {
        padding: 3px 7px;
        border: 1px solid #cbd5e1;
        border-radius: 5px;
        font-size: 0.82rem;
        background: #fff;
      }
      .an-cards {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(175px, 1fr));
        gap: 8px;
      }
      .an-card {
        background: #ffffff;
        border: 1px solid #e2e8f0;
        border-radius: 8px;
        padding: 9px 12px;
      }
      .an-card-label {
        font-size: 0.73rem;
        color: #64748b;
        margin-bottom: 3px;
      }
      .an-card-val {
        font-size: 1.08rem;
        font-weight: 700;
        font-family: var(--font-mono, monospace);
        color: #0f172a;
      }
      .an-card-sub {
        font-size: 0.72rem;
        color: #64748b;
        margin-top: 2px;
      }
      .an-chart-box {
        background: #ffffff;
        border: 1px solid #e2e8f0;
        border-radius: 8px;
        padding: 10px 12px;
        display: flex;
        flex-direction: column;
        gap: 6px;
      }
      .an-chart-head {
        display: flex;
        justify-content: space-between;
        align-items: center;
        gap: 8px;
        flex-wrap: wrap;
      }
      .an-chart-title {
        font-size: 0.86rem;
        font-weight: 700;
        color: #1e293b;
      }
      .an-chart-hint {
        font-size: 0.74rem;
        color: #64748b;
      }
      .an-chart-box canvas {
        width: 100%;
        display: block;
        cursor: crosshair;
      }
      .an-grid-2 {
        display: grid;
        grid-template-columns: 1fr 1fr;
        gap: 12px;
      }
      @media (max-width: 900px) {
        .an-grid-2 { grid-template-columns: 1fr; }
      }
      .an-table {
        width: 100%;
        border-collapse: collapse;
        font-size: 0.8rem;
        font-family: var(--font-mono, monospace);
      }
      .an-table th, .an-table td {
        padding: 6px 9px;
        border-bottom: 1px solid #e2e8f0;
        text-align: left;
      }
      .an-table th { background: #f1f5f9; color: #334155; font-family: inherit; }
      .an-table tbody tr { cursor: pointer; }
      .an-table tbody tr:hover { background: #e0f2fe; }
      .an-note {
        font-size: 0.78rem;
        color: #475569;
        background: #f1f5f9;
        border-left: 3px solid #0284c7;
        padding: 8px 11px;
        border-radius: 4px;
        line-height: 1.5;
      }
      .an-guide {
        background: #f8fafc;
        border: 1px solid #e2e8f0;
        border-left: 3px solid #0284c7;
        border-radius: 6px;
        padding: 6px 10px;
        font-size: 0.75rem;
        color: #475569;
        line-height: 1.48;
      }
      .an-guide summary {
        cursor: pointer;
        font-weight: 700;
        color: #334155;
        user-select: none;
        list-style: none;
        display: flex;
        align-items: center;
        gap: 6px;
      }
      .an-guide summary::-webkit-details-marker { display: none; }
      .an-guide summary::before {
        content: '▸';
        font-size: 0.7rem;
        color: #64748b;
        transition: transform 0.15s;
      }
      .an-guide[open] summary::before { transform: rotate(90deg); }
      .an-guide-grid {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(240px, 1fr));
        gap: 6px 14px;
        margin-top: 5px;
        padding-top: 5px;
        border-top: 1px dashed #cbd5e1;
      }
      .an-guide-item b {
        color: #0f172a;
        margin-right: 4px;
      }
      .an-error {
        padding: 20px;
        text-align: center;
        color: #b91c1c;
        background: #fef2f2;
        border: 1px solid #fecaca;
        border-radius: 8px;
        font-weight: 600;
      }
      #an-tooltip {
        display: none;
        position: fixed;
        z-index: 2700;
        pointer-events: none;
        background: rgba(15, 23, 42, 0.92);
        color: #f8fafc;
        padding: 6px 9px;
        border-radius: 6px;
        font-size: 0.76rem;
        font-family: var(--font-mono, monospace);
        line-height: 1.45;
        box-shadow: 0 6px 18px rgba(0,0,0,0.25);
        white-space: pre;
      }
    `;
    document.head.appendChild(style);

    const modal = document.createElement('div');
    modal.id = 'analysis-modal';
    modal.innerHTML = `
      <div class="an-shell">
        <div class="an-header">
          <div class="an-header-left">
            <span class="an-title">📊 フライト解析</span>
            <span class="an-subtitle" id="an-file-info"></span>
          </div>
          <div class="an-header-actions">
            <button id="an-btn-kml" title="Google Earth 用 KML 航跡ファイルを書き出します">🗺️ KML出力</button>
            <button id="an-btn-png" title="現在表示中の解析グラフを PNG 画像として保存します">🖼️ PNG保存</button>
            <button id="an-btn-close">✕ 閉じる</button>
          </div>
        </div>
        <div class="an-tabs" id="an-tabs">
          <button class="an-tab active" data-tab="accel">⚡ 鉛直G (荷重倍数)</button>
          <button class="an-tab" data-tab="raw">📈 縦の生データ</button>
          <button class="an-tab" data-tab="scatter">🔵 散布図・不安定さ</button>
          <button class="an-tab" data-tab="spectrum">🌊 スペクトル・スペクトログラム</button>
          <button class="an-tab" data-tab="phugoid">〰️ フゴイド解析</button>
          <button class="an-tab" data-tab="lateral">🌀 横モード (ダッチロール)</button>
          <button class="an-tab" data-tab="imu">🚴 生IMU・ケイデンス</button>
        </div>
        <div class="an-seek-banner" id="an-seek-banner">
          <span id="an-seek-text"></span>
          <button id="an-btn-close-goto" class="btn-primary" style="padding:3px 10px; font-size:0.78rem;">🗺️ 閉じて地図で確認</button>
        </div>
        <div class="an-body" id="an-body">
          <div class="an-range-card">
          <div class="an-seg-bar" id="an-seg-bar">
            <span class="an-seg-label">🛫 解析中の飛行区間:</span>
            <button id="an-seg-prev" class="an-seg-nav" title="前の飛行区間">◀</button>
            <select id="an-seg-select" title="動いている区間（静止で区切る。シークバーの青帯・動き出しの印と同じ分け方）"></select>
            <button id="an-seg-next" class="an-seg-nav" title="次の飛行区間">▶</button>
            <span id="an-seg-count" class="an-seg-count"></span>
            <div class="an-seg-strip" id="an-seg-strip" title="ログ全体。青: 動いている区間 / 橙: 解析中の区間 / 灰: 停止中（解析しない）。クリックで区間を選ぶ"></div>
          </div>
          <div class="an-range-bar" id="an-range-bar">
            <label>区間内の開始:
              <input type="range" id="an-range-start" min="0" max="98" step="0.5" value="0" style="width:150px;">
              <span class="an-range-time" id="an-range-start-lbl">--:--:--</span>
            </label>
            <label>終了:
              <input type="range" id="an-range-end" min="2" max="100" step="0.5" value="100" style="width:150px;">
              <span class="an-range-time" id="an-range-end-lbl">--:--:--</span>
            </label>
            <span id="an-range-dur-lbl" style="color:#64748b; font-family:var(--font-mono, monospace);"></span>
            <button id="an-range-reset" style="padding:3px 9px; font-size:0.76rem;">↺ 区間全体に戻す</button>
          </div>
          </div>
          <div id="an-body-inner" class="an-body-inner"></div>
        </div>
      </div>
    `;
    document.body.appendChild(modal);

    const tip = document.createElement('div');
    tip.id = 'an-tooltip';
    document.body.appendChild(tip);

    // イベント結線
    modal.addEventListener('click', (e) => {
      if (e.target === modal) closeAnalysisModal();
    });
    document.getElementById('an-btn-close').addEventListener('click', closeAnalysisModal);
    document.getElementById('an-btn-close-goto').addEventListener('click', () => {
      const targetSec = state.lastSeekSec;
      closeAnalysisModal();
      if (targetSec !== null && global.seekReponsTo) {
        requestAnimationFrame(() => global.seekReponsTo(targetSec, true));
      }
    });
    document.getElementById('an-btn-kml').addEventListener('click', exportKmlFromModal);
    document.getElementById('an-btn-png').addEventListener('click', exportActiveCanvasesPng);
    document.getElementById('an-seg-select').addEventListener('change', (e) => {
      const idx = parseInt(e.target.value, 10);
      if (Number.isInteger(idx)) selectSegment(idx);
    });
    document.getElementById('an-seg-prev').addEventListener('click', () => {
      if (state.segIdx !== null && state.segIdx > 0) selectSegment(state.segIdx - 1);
    });
    document.getElementById('an-seg-next').addEventListener('click', () => {
      const n = getSegments(getViewerContext()).length;
      if (state.segIdx !== null && state.segIdx < n - 1) selectSegment(state.segIdx + 1);
    });

    let rangeRenderTimer = null;
    const onRangeInput = () => {
      const sEl = document.getElementById('an-range-start');
      const eEl = document.getElementById('an-range-end');
      let sf = parseFloat(sEl.value) / 100;
      let ef = parseFloat(eEl.value) / 100;
      if (sf > ef - 0.02) {
        if (document.activeElement === sEl) { sf = Math.max(0, ef - 0.02); sEl.value = (sf * 100).toFixed(1); }
        else { ef = Math.min(1, sf + 0.02); eEl.value = (ef * 100).toFixed(1); }
      }
      state.rangeStartFrac = sf;
      state.rangeEndFrac = ef;
      updateRangeBarUI(getViewerContext());
      if (rangeRenderTimer) clearTimeout(rangeRenderTimer);
      rangeRenderTimer = setTimeout(() => { rangeRenderTimer = null; renderActiveTab(); }, 60);
    };
    document.getElementById('an-range-start').addEventListener('input', onRangeInput);
    document.getElementById('an-range-end').addEventListener('input', onRangeInput);
    document.getElementById('an-range-reset').addEventListener('click', () => {
      state.rangeStartFrac = 0;
      state.rangeEndFrac = 1;
      renderActiveTab();
    });

    document.getElementById('an-tabs').addEventListener('click', (e) => {
      const btn = e.target.closest('.an-tab');
      if (!btn) return;
      state.activeTab = btn.dataset.tab;
      document.querySelectorAll('#an-tabs .an-tab').forEach(b => b.classList.toggle('active', b === btn));
      renderActiveTab();
    });

    document.addEventListener('keydown', (e) => {
      if (e.key === 'Escape' && state.open) closeAnalysisModal();
    });
    // スクロール中はツールチップを消す（位置が古いまま残るのと、スクロール中の描き直しを減らすため）
    document.getElementById('an-body').addEventListener('scroll', hideTooltip, { passive: true });
  }

  // ------------------------------------------------------------
  // 本体とのデータ連携ヘルパー
  // ------------------------------------------------------------
  function getViewerContext() {
    const ctx = global.getReponsAnalysisContext ? global.getReponsAnalysisContext() : null;
    return ctx || { flightData: [], takeoffSec: 0, landSec: 0, totalDurationSec: 0, currentTimeSec: 0, filename: '' };
  }

  // ------------------------------------------------------------
  // 飛行区間（動いている区間ごと。停止中は解析しない）
  // ------------------------------------------------------------
  function getSegments(ctx) { return (ctx && ctx.segments) || []; }

  function segIndexAt(segs, t) {
    for (let i = 0; i < segs.length; i++) if (t >= segs[i].start && t <= segs[i].end) return i;
    return -1;
  }

  function longestSegIndex(segs) {
    let best = 0;
    for (let i = 1; i < segs.length; i++) {
      if (segs[i].end - segs[i].start > segs[best].end - segs[best].start) best = i;
    }
    return best;
  }

  // 解析中の区間。未選択・範囲外なら最長の区間にする。区間が無ければ null
  function getSelectedSegment(ctx) {
    const segs = getSegments(ctx);
    if (segs.length === 0) return null;
    if (state.segIdx === null || state.segIdx < 0 || state.segIdx >= segs.length) state.segIdx = longestSegIndex(segs);
    return segs[state.segIdx];
  }

  function selectSegment(idx) {
    state.segIdx = idx;
    state.rangeStartFrac = 0;
    state.rangeEndFrac = 1;
    state.phugoidSelectedWin = 0;
    renderActiveTab();
  }

  function getBaseRange(ctx) {
    const seg = getSelectedSegment(ctx);
    if (seg) return { baseStart: seg.start, baseEnd: seg.end };
    return { baseStart: 0, baseEnd: ctx.totalDurationSec };
  }

  function fmtDur(sec) {
    if (!Number.isFinite(sec)) return '--';
    if (sec < 60) return `${sec.toFixed(0)}秒`;
    const m = Math.floor(sec / 60), s = Math.floor(sec % 60);
    return `${m}分${String(s).padStart(2, '0')}秒`;
  }

  // 区間の選択欄（◀ ▶・プルダウン）とログ全体の帯（どの区間を解析中か）
  let segPickerSig = '';
  function renderSegmentPicker(ctx) {
    const sel = document.getElementById('an-seg-select');
    const strip = document.getElementById('an-seg-strip');
    const cnt = document.getElementById('an-seg-count');
    const prev = document.getElementById('an-seg-prev');
    const next = document.getElementById('an-seg-next');
    if (!sel || !strip) return;
    const segs = getSegments(ctx);
    const cur = getSelectedSegment(ctx);

    const sig = `${ctx.filename}|${segs.map(s => `${s.start}-${s.end}`).join(',')}`;
    if (sig !== segPickerSig) {
      segPickerSig = sig;
      sel.innerHTML = segs.length === 0
        ? '<option value="">動いている区間がありません</option>'
        : segs.map((s, i) => `<option value="${i}">#${s.no}  ${fmtSec(s.start, ctx)}〜${fmtSec(s.end, ctx)}  (${fmtDur(s.end - s.start)})</option>`).join('');
    }
    sel.disabled = segs.length === 0;
    if (cur) sel.value = String(state.segIdx);
    if (prev) prev.disabled = !cur || state.segIdx <= 0;
    if (next) next.disabled = !cur || state.segIdx >= segs.length - 1;
    if (cnt) cnt.textContent = segs.length > 0 ? `${segs.length} 区間中 ${state.segIdx + 1} 番目（停止中は解析しません）` : '';

    // ログ全体の帯: 灰 = 停止中、青 = 動いている区間、橙 = 解析中の区間、濃い橙 = 区間内で絞った解析範囲
    strip.innerHTML = '';
    const total = ctx.totalDurationSec;
    if (!(total > 0)) return;
    const pct = (t) => Math.max(0, Math.min(100, (t / total) * 100));
    segs.forEach((s, i) => {
      const blk = document.createElement('div');
      blk.className = 'an-seg-blk' + (i === state.segIdx ? ' sel' : '');
      blk.style.left = `${pct(s.start)}%`;
      blk.style.width = `${Math.max(0.15, pct(s.end) - pct(s.start))}%`;
      blk.title = `区間 #${s.no}: ${fmtSec(s.start, ctx)}〜${fmtSec(s.end, ctx)} (${fmtDur(s.end - s.start)})${i === state.segIdx ? ' — 解析中' : ' — クリックで解析'}`;
      blk.addEventListener('click', () => { if (i !== state.segIdx) selectSegment(i); });
      strip.appendChild(blk);
      // 幅があれば番号を書く
      if (pct(s.end) - pct(s.start) > 2.5) {
        const lab = document.createElement('div');
        lab.className = 'an-seg-no' + (i === state.segIdx ? ' sel' : '');
        lab.style.left = blk.style.left;
        lab.style.width = blk.style.width;
        lab.textContent = `#${s.no}`;
        strip.appendChild(lab);
      }
    });
    if (cur && (state.rangeStartFrac > 0 || state.rangeEndFrac < 1)) {
      const { tStart, tEnd } = getActiveRange(ctx);
      const sub = document.createElement('div');
      sub.className = 'an-seg-sub';
      sub.style.left = `${pct(tStart)}%`;
      sub.style.width = `${Math.max(0.15, pct(tEnd) - pct(tStart))}%`;
      strip.appendChild(sub);
    }
  }

  function getActiveRange(ctx) {
    const { baseStart, baseEnd } = getBaseRange(ctx);
    const dur = Math.max(1, baseEnd - baseStart);
    return {
      tStart: baseStart + dur * state.rangeStartFrac,
      tEnd: baseStart + dur * state.rangeEndFrac
    };
  }

  function updateRangeBarUI(ctx) {
    const { tStart, tEnd } = getActiveRange(ctx);
    const sEl = document.getElementById('an-range-start');
    const eEl = document.getElementById('an-range-end');
    const sLbl = document.getElementById('an-range-start-lbl');
    const eLbl = document.getElementById('an-range-end-lbl');
    const dLbl = document.getElementById('an-range-dur-lbl');
    if (sEl && document.activeElement !== sEl) sEl.value = (state.rangeStartFrac * 100).toFixed(1);
    if (eEl && document.activeElement !== eEl) eEl.value = (state.rangeEndFrac * 100).toFixed(1);
    if (sLbl) sLbl.textContent = fmtSec(tStart, ctx);
    if (eLbl) eLbl.textContent = fmtSec(tEnd, ctx);
    if (dLbl) dLbl.textContent = `(${((tEnd - tStart) / 60).toFixed(1)}分 / ${(tEnd - tStart).toFixed(0)}秒)`;
    renderSegmentPicker(ctx);
  }

  function fmtSec(sec, ctx) {
    if (!Number.isFinite(sec)) return '--:--';
    const d = ctx && ctx.flightData && ctx.flightData.length > 0 ? findNearestRecord(ctx.flightData, sec) : null;
    if (d && d.timeStr) return d.timeStr.split('.')[0];
    const m = Math.floor(sec / 60);
    const s = Math.floor(sec % 60);
    return `${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
  }

  function fmtRelMin(sec, tStart) {
    const dt = sec - tStart;
    const m = Math.floor(dt / 60);
    const s = Math.floor(Math.abs(dt % 60));
    return `${m}:${String(s).padStart(2, '0')}`;
  }

  function fmtNum(v, digits = 2, suffix = '') {
    return Number.isFinite(v) ? `${v.toFixed(digits)}${suffix}` : '---';
  }

  function findNearestRecord(records, tSec) {
    let lo = 0, hi = records.length - 1;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (records[mid].t < tSec) lo = mid + 1;
      else hi = mid;
    }
    return records[lo];
  }

  function seekTo(tSec) {
    const ctx = getViewerContext();
    const clamped = Math.max(0, Math.min(ctx.totalDurationSec, tSec));
    if (global.seekReponsTo) global.seekReponsTo(clamped, true);
    state.lastSeekSec = clamped;
    const banner = document.getElementById('an-seek-banner');
    const txt = document.getElementById('an-seek-text');
    if (banner && txt) {
      txt.textContent = `📍 ${fmtSec(clamped, ctx)} (経過 ${clamped.toFixed(1)}s) へシークしました`;
      banner.classList.add('show');
    }
    // 縦線カーソル更新のため再描画
    renderActiveTab();
  }

  // ------------------------------------------------------------
  // モーダル開閉 & キャッシュ管理
  // ------------------------------------------------------------
  function openAnalysisModal(initialTab) {
    injectStylesAndDom();
    const ctx = getViewerContext();
    const hasCsv = ctx.flightData && ctx.flightData.length >= 5;
    if (!hasCsv && !state.imuDecoded) {
      alert('先に飛行 CSV ファイル（またはデモログ／生 IMU ログ .bin）を読み込んでください。');
      return;
    }
    const key = `${ctx.filename}_${ctx.flightData.length}_${ctx.totalDurationSec.toFixed(1)}_${state.imuDecoded ? state.imuDecoded.filename : ''}`;
    if (state.cacheKey !== key) {
      state.cacheKey = key;
      state.cache = {};
      state.rangeStartFrac = 0;
      state.rangeEndFrac = 1;
      state.phugoidSelectedWin = 0;
      state.segIdx = null;
      if (state.imuDecoded && IMU()) {
        state.imuMatch = IMU().matchSessionToFlight(state.imuDecoded, ctx.flightData);
      }
    }
    // 再生位置が動いている区間の中なら、その区間を解析する（シークバーの動き出しの印で飛んでから開けばその区間）。
    // 停止中なら前に選んだ区間のまま（初めてなら最長の区間）
    {
      const segs = getSegments(ctx);
      const curIdx = segIndexAt(segs, ctx.currentTimeSec);
      if (curIdx >= 0 && curIdx !== state.segIdx) {
        state.segIdx = curIdx;
        state.rangeStartFrac = 0;
        state.rangeEndFrac = 1;
        state.phugoidSelectedWin = 0;
      }
    }
    if (initialTab) {
      state.activeTab = initialTab;
    } else if (!hasCsv && state.imuDecoded) {
      state.activeTab = 'imu';
    }
    document.querySelectorAll('#an-tabs .an-tab').forEach(b => b.classList.toggle('active', b.dataset.tab === state.activeTab));
    state.open = true;
    const modal = document.getElementById('analysis-modal');
    modal.classList.add('open');
    document.body.classList.add('an-modal-open');   // 裏の本体を描かせない（軽量化）
    document.getElementById('an-seek-banner').classList.remove('show');
    updateHeaderInfo(ctx);
    renderActiveTab();
  }

  function loadImuBinBytes(bytesOrBuf, filename, opts) {
    if (!IMU()) { alert('analysis_imulog.js が読み込まれていません。'); return; }
    const decoded = IMU().decodeImuBin(bytesOrBuf, filename);
    if (decoded.error) {
      alert('生 IMU ログの読み込みに失敗しました:\n' + decoded.error);
      return;
    }
    state.imuDecoded = decoded;
    state.imuTransferWarning = (opts && opts.transferWarning) || null;
    const ctx = getViewerContext();
    state.imuMatch = IMU().matchSessionToFlight(decoded, ctx.flightData);
    state.cache = {};
    const anBtn = document.getElementById('btn-open-analysis');
    if (anBtn) { anBtn.disabled = false; anBtn.style.opacity = '1'; anBtn.style.cursor = 'pointer'; }
    if (opts && opts.openModal) {
      openAnalysisModal(opts.tab || 'imu');
    } else if (state.open) {
      renderActiveTab();
    }
  }

  function closeAnalysisModal() {
    state.open = false;
    const modal = document.getElementById('analysis-modal');
    if (modal) modal.classList.remove('open');
    document.body.classList.remove('an-modal-open');
    hideTooltip();
  }

  function resetAnalysisCache() {
    state.cacheKey = '';
    state.cache = {};
  }

  function updateHeaderInfo(ctx) {
    const { tStart, tEnd } = getActiveRange(ctx);
    const el = document.getElementById('an-file-info');
    if (el) {
      const seg = getSelectedSegment(ctx);
      el.textContent = seg
        ? `${ctx.filename} | 区間 #${seg.no}: ${fmtSec(tStart, ctx)}〜${fmtSec(tEnd, ctx)} (${fmtDur(tEnd - tStart)})`
        : `${ctx.filename} | 動いている区間なし`;
    }
    updateRangeBarUI(ctx);
  }

  // ------------------------------------------------------------
  // Canvas 描画ユーティリティ & ツールチップ
  // ------------------------------------------------------------
  function setupCanvas(canvas, cssHeight) {
    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    const w = Math.max(300, Math.floor(rect.width || 900));
    const h = cssHeight;
    canvas.width = Math.floor(w * dpr);
    canvas.height = Math.floor(h * dpr);
    canvas.style.height = `${h}px`;
    const ctx2d = canvas.getContext('2d');
    ctx2d.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx2d.clearRect(0, 0, w, h);
    ctx2d.fillStyle = '#ffffff';
    ctx2d.fillRect(0, 0, w, h);
    return { g: ctx2d, w, h };
  }

  function showTooltip(clientX, clientY, text) {
    const tip = document.getElementById('an-tooltip');
    if (!tip) return;
    tip.textContent = text;
    tip.style.display = 'block';
    const x = Math.min(window.innerWidth - tip.offsetWidth - 12, clientX + 14);
    const y = Math.min(window.innerHeight - tip.offsetHeight - 12, clientY + 14);
    tip.style.left = `${Math.max(8, x)}px`;
    tip.style.top = `${Math.max(8, y)}px`;
  }

  function hideTooltip() {
    const tip = document.getElementById('an-tooltip');
    if (tip) tip.style.display = 'none';
  }

  function attachInteractive(canvas, handler) {
    canvas.onmousemove = (e) => {
      const r = canvas.getBoundingClientRect();
      const x = e.clientX - r.left;
      const y = e.clientY - r.top;
      const info = handler.onHover ? handler.onHover(x, y) : null;
      if (info) showTooltip(e.clientX, e.clientY, info);
      else hideTooltip();
    };
    canvas.onmouseleave = hideTooltip;
    canvas.onclick = (e) => {
      const r = canvas.getBoundingClientRect();
      const x = e.clientX - r.left;
      const y = e.clientY - r.top;
      if (handler.onClick) handler.onClick(x, y);
    };
  }

  // 共通の軸・グリッド描画
  function drawAxes(g, pad, w, h, xMin, xMax, yMin, yMax, opts) {
    const pw = w - pad.l - pad.r;
    const ph = h - pad.t - pad.b;
    const xToPx = (v) => pad.l + ((v - xMin) / (xMax - xMin || 1)) * pw;
    const yToPx = (v) => pad.t + ph - ((v - yMin) / (yMax - yMin || 1)) * ph;

    g.strokeStyle = '#e2e8f0';
    g.lineWidth = 1;
    g.font = '11px -apple-system, BlinkMacSystemFont, sans-serif';
    g.fillStyle = '#64748b';

    // Y グリッド
    const yTicks = opts.yTicks || 5;
    g.textAlign = 'right';
    g.textBaseline = 'middle';
    for (let i = 0; i <= yTicks; i++) {
      const v = yMin + (yMax - yMin) * (i / yTicks);
      const py = yToPx(v);
      g.beginPath(); g.moveTo(pad.l, py); g.lineTo(pad.l + pw, py); g.stroke();
      g.fillText(opts.yFmt ? opts.yFmt(v) : v.toFixed(1), pad.l - 6, py);
    }

    // X グリッド
    const xTicks = opts.xTicks || 6;
    g.textAlign = 'center';
    g.textBaseline = 'top';
    for (let i = 0; i <= xTicks; i++) {
      const v = xMin + (xMax - xMin) * (i / xTicks);
      const px = xToPx(v);
      g.beginPath(); g.moveTo(px, pad.t); g.lineTo(px, pad.t + ph); g.stroke();
      g.fillText(opts.xFmt ? opts.xFmt(v) : v.toFixed(1), px, pad.t + ph + 5);
    }

    // 外枠
    g.strokeStyle = '#94a3b8';
    g.strokeRect(pad.l, pad.t, pw, ph);

    // 軸ラベル
    if (opts.yLabel) {
      g.save();
      g.translate(13, pad.t + ph * 0.5);
      g.rotate(-Math.PI / 2);
      g.textAlign = 'center';
      g.fillStyle = '#334155';
      g.font = 'bold 11px sans-serif';
      g.fillText(opts.yLabel, 0, 0);
      g.restore();
    }
    if (opts.xLabel) {
      g.textAlign = 'center';
      g.textBaseline = 'bottom';
      g.fillStyle = '#334155';
      g.font = 'bold 11px sans-serif';
      g.fillText(opts.xLabel, pad.l + pw * 0.5, h - 2);
    }
    return { xToPx, yToPx, pw, ph };
  }

  function drawCurrentCursor(g, pad, h, xToPx, xMin, xMax, curT) {
    if (curT < xMin || curT > xMax) return;
    const px = xToPx(curT);
    g.save();
    g.strokeStyle = '#0284c7';
    g.lineWidth = 1.5;
    g.setLineDash([4, 3]);
    g.beginPath();
    g.moveTo(px, pad.t);
    g.lineTo(px, h - pad.b);
    g.stroke();
    g.restore();
  }

  // Viridis 風カラーマップ (0..1)
  function viridisColor(t) {
    const x = Math.max(0, Math.min(1, t));
    const stops = [
      [68, 1, 84],
      [59, 82, 139],
      [33, 145, 140],
      [94, 201, 98],
      [253, 231, 37]
    ];
    const pos = x * (stops.length - 1);
    const i = Math.min(stops.length - 2, Math.floor(pos));
    const f = pos - i;
    const r = Math.round(stops[i][0] + f * (stops[i + 1][0] - stops[i][0]));
    const g = Math.round(stops[i][1] + f * (stops[i + 1][1] - stops[i][1]));
    const b = Math.round(stops[i][2] + f * (stops[i + 1][2] - stops[i][2]));
    return `rgb(${r},${g},${b})`;
  }

  // ------------------------------------------------------------
  // 各タブの「目的・見方・活用」ガイド（tools/flightanalysis/*.py の設計意図を凝縮）
  // ------------------------------------------------------------
  const TAB_GUIDES = {
    accel: {
      title: 'この分析について（鉛直G・荷重倍数）',
      purpose: '飛行CSV単体には加速度列が含まれないため、CSVのみのときは KF_Vspeed の1階差分（積分の逆操作）から鉛直G・最大荷重倍数を推定し、桁設計や突風・旋回時の実荷重を把握するため。',
      lookAt: '「瞬間G」だけでなく「平滑窓長」を変えたときの最大Gの変化と、地上静止時のノイズフロア（S/N比）。上位ピーク行クリックで発生地点を地図で確認。',
      useFor: '最大Gは必ず平均化窓（例: 0.5s / 1.5s）とセットで引用する。2Hz CSVのため1Hz以上の瞬間衝撃は原理的に見えない（高帯域の直接計測には250Hz生IMUログ .bin を使用）。'
    },
    raw: {
      title: 'この分析について（縦の生データ）',
      purpose: 'フゴイド解析などの帯域フィルタでは消えてしまう、高度のゆっくりしたドリフト・トリム変化・連続的な上昇/降下をありのまま確認するため。',
      lookAt: '高度・V/S・ピッチ/ロール・G/S/TAS の4段を時間軸で縦に揃え、特定イベント（離陸直後・旋回・フレア等）をズームして前後関係を読む。',
      useFor: '他のタブで見つけた異常ピークや揺れ区間の「一次確認」として使い、機体姿勢と速度・高度の因果を素直に追う。'
    },
    scatter: {
      title: 'この分析について（散布図・不安定さ相関）',
      purpose: '「だんだん不安定になった」というパイロット体感の原因が、経過時間（風・疲労）か、ピッチ低下（桁のT曲げ変形等）かを切り分けるため。',
      lookAt: '窓内の直線トレンドを除いた残差RMS（揺れの大きさ）と、経過時間・平均ピッチの相関係数 r。ピッチ vs 高度/TAS ではトリム推移を見る。',
      useFor: '相関係数が高い組み合わせを見つけたら、外れ値の点をクリックして現場の時刻・地図・風向と照合し、構造・空力・気象のどれが主因か絞る。'
    },
    spectrum: {
      title: 'この分析について（スペクトル・スペクトログラム）',
      purpose: '揺れの「大きさ」だけでなく「どの周期（速さ）の揺れが、いつ強かったか」を定量化するため。',
      lookAt: '上段のパワースペクトルでピーク周期を読み、スペクトログラムと下段の4周期帯RMSでその揺れがいつ強まったかを追う。区間の一部だけを見たいときは上の「区間内の開始／終了」で絞る。',
      useFor: '数秒周期が増えたなら機体固有振動・乱流、数十秒周期なら操縦やサーマル等と切り分け、次機設計や操縦改善の根拠にする。'
    },
    phugoid: {
      title: 'この分析について（フゴイド解析）',
      purpose: '高度と速度のエネルギー交換で起きる縦の長周期振動（フゴイド）が実飛行で出ているか、周期 T と減衰比 ζ を特定するため。',
      lookAt: '周期が古典値 π√2·V/g に近いだけでなく、①ピッチと非KF V/Sが同位相(0°付近)、②高度とG/Sが逆位相(±180°付近)の2条件が揃う区間をバーで探す。',
      useFor: '重心位置・尾翼容積・トリム設定による縦の動安定性評価に使う。G/S代用時は水平突風と区別しにくい点に注意。'
    },
    lateral: {
      title: 'この分析について（横・方向モード：ダッチロール／スパイラル）',
      purpose: '上反角やラダー諸元の違いが、横・方向の癖（ダッチロールの周期/減衰、スパイラルのゆっくりした傾き発散）として現れているか検証するため。',
      lookAt: '意図した旋回（薄赤帯）を除いた直進区間で、GPS旋回角速度 r と IMUロール角の2本のACFが同じ周期 T・減衰比 ζ を示すか裏取りする。',
      useFor: '「比較用CSVを追加」で前年機や別設定フライトと並べ、上反角・垂直尾翼・ラダー効きの変更が狙い通り効いたかを評価する。'
    },
    imu: {
      title: 'この分析について（250Hz 生IMU・ケイデンス・高周波振動）',
      purpose: '250Hz生IMUログ（imuraw/*.bin）の前後加速度から踏み込み振動を拾ってパイロットのケイデンス[RPM]を推定し、あわせて1〜125Hzの高周波振動・機体共振を調べるため。',
      lookAt: '白色化スペクトログラムで細い帯が続くか、「比のスペクトル」で ×1.0 に対し ×0.5（クランク1回転）や ×2.0 が強すぎないか（取り違え防止）を確認する。',
      useFor: 'ペダリング出力の維持・疲労によるケイデンス低下の把握、ペラ回転や桁振動の周波数特定に使う。USB接続中は同日の .bin をワンクリックで本体から直接取得できる。'
    }
  };

  function renderGuideBox(tabKey) {
    const g = TAB_GUIDES[tabKey];
    if (!g) return '';
    return `
      <details class="an-guide" open>
        <summary>💡 ${g.title}</summary>
        <div class="an-guide-grid">
          <div class="an-guide-item"><b>目的:</b>${g.purpose}</div>
          <div class="an-guide-item"><b>見方:</b>${g.lookAt}</div>
          <div class="an-guide-item"><b>活用・注意:</b>${g.useFor}</div>
        </div>
      </details>
    `;
  }

  // ------------------------------------------------------------
  // タブ描画ディスパッチャ
  // ------------------------------------------------------------
  function renderActiveTab() {
    const ctx = getViewerContext();
    updateHeaderInfo(ctx);
    // 区間の帯・範囲のバーは #an-body の頭に固定で置き、描き直すのはその下（#an-body-inner）だけ
    const body = document.getElementById('an-body-inner');
    if (!body) return;
    hideTooltip();

    // ガイド枠を先に配置してからタブ本体を描画（スクロールバー出現による Canvas 幅ズレを防止）
    body.innerHTML = `${renderGuideBox(state.activeTab)}<div id="an-tab-content" style="display:flex; flex-direction:column; gap:12px;"></div>`;
    const slot = document.getElementById('an-tab-content');

    // 停止中は解析しない。動いている区間が 1 つも無い CSV は、生 IMU 単独のタブ以外は出さない
    const hasCsv = ctx.flightData && ctx.flightData.length >= 5;
    if (state.activeTab !== 'imu' && !getSelectedSegment(ctx)) {
      slot.innerHTML = hasCsv
        ? `<div class="an-error">この CSV には動いている区間（G/S が ${ctx.speedThreshold || 1.0} m/s 以上で 1.5 秒以上）がありません。停止中は解析しません。</div>`
        : `<div class="an-error">飛行 CSV を読み込むと、このタブの解析ができます。</div>`;
      return;
    }

    switch (state.activeTab) {
      case 'accel':    renderAccelTab(slot, ctx); break;
      case 'raw':      renderRawTab(slot, ctx); break;
      case 'scatter':  renderScatterTab(slot, ctx); break;
      case 'spectrum': renderSpectrumTab(slot, ctx); break;
      case 'phugoid':  renderPhugoidTab(slot, ctx); break;
      case 'lateral':  renderLateralTab(slot, ctx); break;
      case 'imu':      renderImuTab(slot, ctx); break;
    }
  }

  // ============================================================
  // タブ 1: 鉛直加速度 (G) & 荷重倍数
  // ============================================================
  function renderAccelTab(body, ctx) {
    const { tStart, tEnd } = getActiveRange(ctx);
    const ck = `accel_${tStart}_${tEnd}_${state.accelSmoothSec}`;
    const res = state.cache[ck] || (state.cache[ck] = DSP.computeVerticalAccel(ctx.flightData, tStart, tEnd, state.accelSmoothSec));

    if (res.error) {
      body.innerHTML = `<div class="an-error">${res.error}</div>`;
      return;
    }

    // 生 IMU が読み込まれて同期済みなら 250Hz 直接計測 G も算出
    let imuG = null;
    if (state.imuMatch && state.imuMatch.synced && IMU()) {
      const ick = `imug_${state.imuMatch.sessionIndex}_${tStart}_${tEnd}_${state.imuDirectGSmooth}`;
      const r = state.cache[ick] || (state.cache[ick] = IMU().computeImuDirectG(state.imuMatch.session, state.imuMatch.toFlightSec, tStart, tEnd, state.imuDirectGSmooth));
      if (!r.error) imuG = r;
    }

    body.innerHTML = `
      ${!imuG ? renderImuQuickBanner(ctx, 'accel') : renderImuWarnings()}
      <div class="an-controls">
        <label>CSV平滑窓長 [秒]:
          <input type="number" id="an-accel-smooth" min="0.5" max="5" step="0.5" value="${state.accelSmoothSec}" style="width:65px;">
        </label>
        <label>縦軸レンジ:
          <select id="an-accel-yrange">
            <option value="auto" ${state.accelYSpan === 'auto' ? 'selected' : ''}>自動 (データに合わせる)</option>
            <option value="0.1" ${state.accelYSpan === '0.1' ? 'selected' : ''}>±0.10 G (0.90–1.10G)</option>
            <option value="0.25" ${state.accelYSpan === '0.25' ? 'selected' : ''}>±0.25 G (0.75–1.25G)</option>
            <option value="0.5" ${state.accelYSpan === '0.5' ? 'selected' : ''}>±0.50 G (0.50–1.50G)</option>
            <option value="1.0" ${state.accelYSpan === '1.0' ? 'selected' : ''}>±1.00 G (0.00–2.00G)</option>
            <option value="2.0" ${state.accelYSpan === '2.0' ? 'selected' : ''}>±2.00 G (-1.0–3.0G)</option>
          </select>
        </label>
        ${imuG ? `
          <label style="color:#15803d; font-weight:bold;">生IMU平滑窓 [秒]:
            <input type="number" id="an-imug-smooth" min="0.02" max="2.0" step="0.05" value="${state.imuDirectGSmooth}" style="width:65px;">
          </label>
          <span style="color:#15803d; font-weight:600;">✅ ${imuG.fs.toFixed(0)}Hz 直接計測G 重ね表示中</span>
        ` : `<span style="color:#64748b;">元データ間隔: 約 ${res.medDt.toFixed(2)} 秒 (${(1 / res.medDt).toFixed(1)} Hz) — グラフまたは下のピーク行をクリックでシーク</span>`}
      </div>
      <div class="an-cards">
        <div class="an-card">
          <div class="an-card-label">最大 正G (CSV 瞬間 / 平滑)</div>
          <div class="an-card-val" style="color:#dc2626;">${fmtNum(res.maxPos.n, 2, ' G')} <span style="font-size:0.82rem; color:#64748b;">/ ${fmtNum(res.maxPosSmooth.n, 2, ' G')}</span></div>
          <div class="an-card-sub">発生時刻: ${fmtSec(res.maxPos.t, ctx)}</div>
        </div>
        <div class="an-card">
          <div class="an-card-label">最大 負G (CSV 瞬間 / 平滑)</div>
          <div class="an-card-val" style="color:#0284c7;">${fmtNum(res.maxNeg.n, 2, ' G')} <span style="font-size:0.82rem; color:#64748b;">/ ${fmtNum(res.maxNegSmooth.n, 2, ' G')}</span></div>
          <div class="an-card-sub">発生時刻: ${fmtSec(res.maxNeg.t, ctx)}</div>
        </div>
        ${imuG ? `
          <div class="an-card" style="border-left:3px solid #16a34a;">
            <div class="an-card-label">250Hz 生IMU 直接 正G (瞬間・振動込み / ${imuG.smoothSec.toFixed(2)}s平滑)</div>
            <div class="an-card-val" style="color:#15803d;">${fmtNum(imuG.maxPosRaw.n, 2, ' G')} <span style="font-size:0.82rem; color:#64748b;">/ ${fmtNum(imuG.maxPosSmooth.n, 2, ' G')}</span></div>
            <div class="an-card-sub">発生時刻: ${fmtSec(imuG.maxPosRaw.t, ctx)}（瞬間値は機体・ペラの振動を含む。荷重の評価は平滑値で）</div>
          </div>
          <div class="an-card" style="border-left:3px solid #16a34a;">
            <div class="an-card-label">250Hz 生IMU 直接 負G (瞬間・振動込み / ${imuG.smoothSec.toFixed(2)}s平滑)</div>
            <div class="an-card-val" style="color:#0284c7;">${fmtNum(imuG.maxNegRaw.n, 2, ' G')} <span style="font-size:0.82rem; color:#64748b;">/ ${fmtNum(imuG.maxNegSmooth.n, 2, ' G')}</span></div>
            <div class="an-card-sub">発生時刻: ${fmtSec(imuG.maxNegRaw.t, ctx)}</div>
          </div>
        ` : `
          <div class="an-card">
            <div class="an-card-label">飛行中 鉛直加速度 RMS</div>
            <div class="an-card-val">${fmtNum(res.flightAzRms, 3, ' m/s²')}</div>
            <div class="an-card-sub">±${fmtNum(res.flightAzRms / DSP.G0, 3, ' G')}</div>
          </div>
          <div class="an-card">
            <div class="an-card-label">地上静止ノイズフロア (S/N)</div>
            <div class="an-card-val">${Number.isFinite(res.noiseFloorRms) ? fmtNum(res.noiseFloorRms, 3, ' m/s²') : '静止区間なし'}</div>
            <div class="an-card-sub">${Number.isFinite(res.snr) ? `S/N 比: ${res.snr.toFixed(2)} (±${res.noiseFloorG.toFixed(3)} G)` : 'G/S < 1.5 m/s の区間から自動算出'}</div>
          </div>
        `}
      </div>
      <div class="an-chart-box">
        <div class="an-chart-head">
          <span class="an-chart-title">荷重倍数 n [G] ${imuG ? '（CSV KF_Vspeed 差分 ＋ 生IMU 250Hz 直接計測）' : '= 1 + (dV_z/dt) / g'}</span>
          <span class="an-chart-hint">薄青: CSV生 / 濃紺: CSV ${res.smoothSec.toFixed(1)}s平滑${imuG ? ` / 緑: 生IMU ${imuG.smoothSec.toFixed(2)}s直接G` : ''} / 赤点: ピーク</span>
        </div>
        <canvas id="an-cv-accel"></canvas>
      </div>
      <div class="an-chart-box">
        <div class="an-chart-head">
          <span class="an-chart-title">上位 G ピークイベント (クリックで該当時刻へシーク)</span>
        </div>
        <table class="an-table">
          <thead>
            <tr><th>#</th><th>時刻</th><th>瞬間 G</th><th>平滑 G</th><th>鉛直加速度</th><th>高度</th><th>G/S</th><th>ロール</th></tr>
          </thead>
          <tbody id="an-accel-peaks"></tbody>
        </table>
      </div>
    `;

    bindImuQuickBannerEvents(ctx, 'accel');
    document.getElementById('an-accel-smooth').addEventListener('change', (e) => {
      state.accelSmoothSec = Math.max(0.5, Math.min(5, parseFloat(e.target.value) || 1.5));
      renderActiveTab();
    });
    document.getElementById('an-accel-yrange').addEventListener('change', (e) => {
      state.accelYSpan = e.target.value;
      renderActiveTab();
    });
    const imugEl = document.getElementById('an-imug-smooth');
    if (imugEl) imugEl.addEventListener('change', (e) => {
      state.imuDirectGSmooth = Math.max(0.02, Math.min(2.0, parseFloat(e.target.value) || 0.25));
      renderActiveTab();
    });

    const tbody = document.getElementById('an-accel-peaks');
    res.peaks.forEach((p, idx) => {
      const tr = document.createElement('tr');
      tr.innerHTML = `
        <td>${idx + 1}</td>
        <td><b>${fmtSec(p.t, ctx)}</b></td>
        <td style="color:${p.nRaw >= 1 ? '#dc2626' : '#0284c7'}; font-weight:bold;">${p.nRaw.toFixed(2)} G</td>
        <td>${p.nSmooth.toFixed(2)} G</td>
        <td>${p.azRaw >= 0 ? '+' : ''}${p.azRaw.toFixed(2)} m/s²</td>
        <td>${fmtNum(p.alt, 1, ' m')}</td>
        <td>${fmtNum(p.gs, 1, ' m/s')}</td>
        <td>${fmtNum(p.roll, 1, '°')}</td>
      `;
      tr.addEventListener('click', () => seekTo(p.t));
      tbody.appendChild(tr);
    });

    // 描画
    const cv = document.getElementById('an-cv-accel');
    const { g, w, h } = setupCanvas(cv, 290);
    const pad = { l: 54, r: 18, t: 14, b: 32 };
    let span;
    if (state.accelYSpan === 'auto') {
      let yMinD = 1, yMaxD = 1;
      for (let i = 0; i < res.nRaw.length; i++) {
        if (res.nRaw[i] < yMinD) yMinD = res.nRaw[i];
        if (res.nRaw[i] > yMaxD) yMaxD = res.nRaw[i];
      }
      if (imuG) {
        for (let i = 0; i < imuG.plotSm.length; i++) {
          if (imuG.plotSm[i] < yMinD) yMinD = imuG.plotSm[i];
          if (imuG.plotSm[i] > yMaxD) yMaxD = imuG.plotSm[i];
        }
      }
      span = Math.max(0.05, Math.max(yMaxD - 1, 1 - yMinD) * 1.15);
    } else {
      span = parseFloat(state.accelYSpan) || 0.25;
    }
    const yMin = 1 - span, yMax = 1 + span;

    const { xToPx, yToPx, pw, ph } = drawAxes(g, pad, w, h, tStart, tEnd, yMin, yMax, {
      yLabel: '荷重倍数 [G]',
      xLabel: '時刻',
      yFmt: v => v.toFixed(2),
      xFmt: v => fmtSec(v, ctx)
    });

    // 枠内クリッピング（固定レンジ時に範囲外スパイクが軸外へはみ出さないように）
    g.save();
    g.beginPath();
    g.rect(pad.l, pad.t, pw, ph);
    g.clip();

    // 1G 基準線
    g.strokeStyle = '#94a3b8';
    g.lineWidth = 1;
    g.setLineDash([5, 4]);
    g.beginPath(); g.moveTo(pad.l, yToPx(1)); g.lineTo(pad.l + pw, yToPx(1)); g.stroke();
    g.setLineDash([]);

    // 生 1 階微分
    g.strokeStyle = 'rgba(14, 165, 233, 0.35)';
    g.lineWidth = 1;
    g.beginPath();
    for (let i = 0; i < res.t.length; i++) {
      const px = xToPx(res.t[i]), py = yToPx(res.nRaw[i]);
      if (i === 0) g.moveTo(px, py); else g.lineTo(px, py);
    }
    g.stroke();

    // 生 IMU 直接計測 G (緑)
    if (imuG) {
      g.strokeStyle = 'rgba(22, 163, 74, 0.85)';
      g.lineWidth = 1.6;
      g.beginPath();
      for (let i = 0; i < imuG.plotT.length; i++) {
        const px = xToPx(imuG.plotT[i]), py = yToPx(imuG.plotSm[i]);
        if (i === 0) g.moveTo(px, py); else g.lineTo(px, py);
      }
      g.stroke();
    }

    // CSV 平滑線
    g.strokeStyle = '#0f172a';
    g.lineWidth = 1.8;
    g.beginPath();
    for (let i = 0; i < res.t.length; i++) {
      const px = xToPx(res.t[i]), py = yToPx(res.nSmooth[i]);
      if (i === 0) g.moveTo(px, py); else g.lineTo(px, py);
    }
    g.stroke();

    // ピークマーカー
    g.fillStyle = '#dc2626';
    for (const p of res.peaks) {
      g.beginPath();
      g.arc(xToPx(p.t), yToPx(p.nRaw), 3.8, 0, Math.PI * 2);
      g.fill();
    }
    g.restore();

    drawCurrentCursor(g, pad, h, xToPx, tStart, tEnd, ctx.currentTimeSec);

    attachInteractive(cv, {
      onHover: (mx) => {
        if (mx < pad.l || mx > pad.l + pw) return null;
        const tHover = tStart + ((mx - pad.l) / pw) * (tEnd - tStart);
        let lo = 0, hi = res.t.length - 1;
        while (lo < hi) { const mid = (lo + hi) >> 1; if (res.t[mid] < tHover) lo = mid + 1; else hi = mid; }
        return `時刻: ${fmtSec(res.t[lo], ctx)}\n瞬間 G: ${res.nRaw[lo].toFixed(2)} G\n平滑 G: ${res.nSmooth[lo].toFixed(2)} G\nクリックでシーク`;
      },
      onClick: (mx) => {
        if (mx < pad.l || mx > pad.l + pw) return;
        const tClick = tStart + ((mx - pad.l) / pw) * (tEnd - tStart);
        seekTo(tClick);
      }
    });
  }

  // ============================================================
  // タブ 2: 縦の生データ (plot_vertical_raw.py 準拠)
  // ============================================================
  function renderRawTab(body, ctx) {
    const { tStart: vStart, tEnd: vEnd } = getActiveRange(ctx);

    body.innerHTML = `
      <div class="an-chart-box">
        <div class="an-chart-head">
          <span class="an-chart-title">縦運動 4段同期ビュー（高度 / V/S / ピッチ・ロール / G/S・TAS）</span>
          <span class="an-chart-hint">上部の開始・終了スライダーで範囲拡大できます / グラフクリックでシーク</span>
        </div>
        <canvas id="an-cv-raw"></canvas>
      </div>
    `;

    const slice = ctx.flightData.filter(d => d.t >= vStart && d.t <= vEnd);
    const cv = document.getElementById('an-cv-raw');
    const { g, w, h } = setupCanvas(cv, 540);
    const padL = 58, padR = 18;
    const pw = w - padL - padR;
    const panels = [
      { label: '高度 [m]', series: [{ name: 'KF Alt', color: '#0284c7', fn: d => d.kf_alt }] },
      { label: 'V/S [m/s]', series: [{ name: 'KF V/S', color: '#16a34a', fn: d => d.kf_vs }], zero: true },
      { label: '姿勢 [deg]', series: [{ name: 'Pitch', color: '#7c3aed', fn: d => d.pitch }, { name: 'Roll', color: '#dc2626', fn: d => d.roll }], zero: true },
      { label: '速度 [m/s]', series: [{ name: 'G/S', color: '#0f172a', fn: d => d.gs }, { name: 'TAS', color: '#ea580c', fn: d => d.csvTas }] }
    ];

    const rowH = (h - 28) / panels.length;
    const xToPx = (t) => padL + ((t - vStart) / (vEnd - vStart || 1)) * pw;

    panels.forEach((p, idx) => {
      const top = 8 + idx * rowH;
      const ph = rowH - 18;
      let yMin = Infinity, yMax = -Infinity;
      for (const s of p.series) {
        for (const d of slice) {
          const v = s.fn(d);
          if (v !== null && Number.isFinite(v)) {
            if (v < yMin) yMin = v;
            if (v > yMax) yMax = v;
          }
        }
      }
      if (!Number.isFinite(yMin)) { yMin = -1; yMax = 1; }
      const margin = Math.max(0.5, (yMax - yMin) * 0.12);
      yMin -= margin; yMax += margin;
      const yToPx = (v) => top + ph - ((v - yMin) / (yMax - yMin || 1)) * ph;

      // グリッド
      g.strokeStyle = '#e2e8f0'; g.lineWidth = 1;
      g.font = '10.5px sans-serif'; g.fillStyle = '#64748b'; g.textAlign = 'right'; g.textBaseline = 'middle';
      for (let k = 0; k <= 3; k++) {
        const v = yMin + (yMax - yMin) * (k / 3);
        const py = yToPx(v);
        g.beginPath(); g.moveTo(padL, py); g.lineTo(padL + pw, py); g.stroke();
        g.fillText(v.toFixed(1), padL - 5, py);
      }
      if (p.zero && yMin < 0 && yMax > 0) {
        g.strokeStyle = '#cbd5e1'; g.setLineDash([4, 3]);
        g.beginPath(); g.moveTo(padL, yToPx(0)); g.lineTo(padL + pw, yToPx(0)); g.stroke();
        g.setLineDash([]);
      }
      g.strokeStyle = '#94a3b8'; g.strokeRect(padL, top, pw, ph);

      // 系列描画
      for (const s of p.series) {
        g.strokeStyle = s.color; g.lineWidth = 1.4;
        g.beginPath();
        let started = false;
        for (const d of slice) {
          const v = s.fn(d);
          if (v === null || !Number.isFinite(v)) { started = false; continue; }
          const px = xToPx(d.t), py = yToPx(v);
          if (!started) { g.moveTo(px, py); started = true; } else g.lineTo(px, py);
        }
        g.stroke();
      }

      // 凡例・ラベル
      g.textAlign = 'left'; g.textBaseline = 'top'; g.font = 'bold 11px sans-serif';
      let lx = padL + 8;
      for (const s of p.series) {
        g.fillStyle = s.color;
        g.fillText(`■ ${s.name}`, lx, top + 4);
        lx += 68;
      }
      g.fillStyle = '#475569'; g.textAlign = 'right';
      g.fillText(p.label, padL + pw - 6, top + 4);

      // X 軸目盛り (最下段のみ)
      if (idx === panels.length - 1) {
        g.textAlign = 'center'; g.textBaseline = 'top'; g.fillStyle = '#64748b'; g.font = '11px sans-serif';
        for (let k = 0; k <= 6; k++) {
          const tv = vStart + (vEnd - vStart) * (k / 6);
          g.fillText(fmtSec(tv, ctx), xToPx(tv), top + ph + 5);
        }
      }
    });

    // 現在位置縦線
    if (ctx.currentTimeSec >= vStart && ctx.currentTimeSec <= vEnd) {
      g.strokeStyle = '#0284c7'; g.lineWidth = 1.5; g.setLineDash([4, 3]);
      const px = xToPx(ctx.currentTimeSec);
      g.beginPath(); g.moveTo(px, 8); g.lineTo(px, h - 22); g.stroke();
      g.setLineDash([]);
    }

    attachInteractive(cv, {
      onHover: (mx) => {
        if (mx < padL || mx > padL + pw || slice.length === 0) return null;
        const tHover = vStart + ((mx - padL) / pw) * (vEnd - vStart);
        const d = findNearestRecord(slice, tHover);
        return `時刻: ${fmtSec(d.t, ctx)}\nAlt: ${fmtNum(d.kf_alt, 1, 'm')} / V/S: ${fmtNum(d.kf_vs, 2, 'm/s')}\nPitch: ${fmtNum(d.pitch, 1, '°')} / Roll: ${fmtNum(d.roll, 1, '°')}\nG/S: ${fmtNum(d.gs, 1, 'm/s')} / TAS: ${fmtNum(d.csvTas, 1, 'm/s')}\nクリックでシーク`;
      },
      onClick: (mx) => {
        if (mx < padL || mx > padL + pw) return;
        seekTo(vStart + ((mx - padL) / pw) * (vEnd - vStart));
      }
    });
  }

  // ============================================================
  // タブ 3: 散布図・不安定さ相関 (plot_instability + plot_pitch_altitude 準拠)
  // ============================================================
  function renderScatterTab(body, ctx) {
    const { tStart, tEnd } = getActiveRange(ctx);
    const ck = `scatter_${tStart}_${tEnd}_${state.scatterWinSec}`;
    const res = state.cache[ck] || (state.cache[ck] = DSP.computeInstabilityAndScatter(ctx.flightData, tStart, tEnd, state.scatterWinSec, 7));

    body.innerHTML = `
      <div class="an-controls">
        <label>表示モード:
          <select id="an-sc-mode">
            <option value="instab" ${state.scatterMode === 'instab' ? 'selected' : ''}>不安定さ相関 (経過時間・平均ピッチ vs 揺れ RMS)</option>
            <option value="pitch_alt" ${state.scatterMode === 'pitch_alt' ? 'selected' : ''}>ピッチ vs 時刻・高度・TAS (色分け散布図)</option>
          </select>
        </label>
        ${state.scatterMode === 'instab' ? `
          <label>不安定さの窓長 [秒]:
            <input type="number" id="an-sc-win" min="15" max="120" step="5" value="${state.scatterWinSec}" style="width:65px;">
          </label>
        ` : `
          <label>横軸:
            <select id="an-sc-x">
              <option value="time" ${state.scatterX === 'time' ? 'selected' : ''}>経過時間 [分]</option>
              <option value="altitude" ${state.scatterX === 'altitude' ? 'selected' : ''}>高度 [m]</option>
              <option value="tas" ${state.scatterX === 'tas' ? 'selected' : ''}>TAS / G/S [m/s]</option>
            </select>
          </label>
          <label>色分け:
            <select id="an-sc-color">
              <option value="altitude" ${state.scatterColor === 'altitude' ? 'selected' : ''}>高度 [m]</option>
              <option value="tas" ${state.scatterColor === 'tas' ? 'selected' : ''}>TAS / G/S [m/s]</option>
              <option value="time" ${state.scatterColor === 'time' ? 'selected' : ''}>経過時間 [分]</option>
            </select>
          </label>
        `}
        <span style="color:#64748b;">散布図の点をクリックすると、その時刻へシークします</span>
      </div>
      <div id="an-sc-content"></div>
    `;

    document.getElementById('an-sc-mode').addEventListener('change', (e) => { state.scatterMode = e.target.value; renderActiveTab(); });
    const winEl = document.getElementById('an-sc-win');
    if (winEl) winEl.addEventListener('change', (e) => { state.scatterWinSec = Math.max(15, Math.min(120, parseInt(e.target.value, 10) || 45)); renderActiveTab(); });
    const xEl = document.getElementById('an-sc-x');
    if (xEl) xEl.addEventListener('change', (e) => { state.scatterX = e.target.value; renderActiveTab(); });
    const cEl = document.getElementById('an-sc-color');
    if (cEl) cEl.addEventListener('change', (e) => { state.scatterColor = e.target.value; renderActiveTab(); });

    const container = document.getElementById('an-sc-content');
    if (state.scatterMode === 'instab') {
      if (res.windows.length < 3) {
        container.innerHTML = '<div class="an-error">不安定さ相関を算出するための十分な窓データがありません（窓長を短くしてみてください）。</div>';
        return;
      }
      container.innerHTML = `
        <div class="an-cards" style="margin-bottom:10px;">
          <div class="an-card"><div class="an-card-label">経過時間 × ピッチ揺れ r</div><div class="an-card-val">${fmtNum(res.corr.time_pitchInstab, 2)}</div></div>
          <div class="an-card"><div class="an-card-label">経過時間 × ロール揺れ r</div><div class="an-card-val">${fmtNum(res.corr.time_rollInstab, 2)}</div></div>
          <div class="an-card"><div class="an-card-label">平均ピッチ × ピッチ揺れ r</div><div class="an-card-val">${fmtNum(res.corr.pitch_pitchInstab, 2)}</div></div>
          <div class="an-card"><div class="an-card-label">平均ピッチ × V/S揺れ r</div><div class="an-card-val">${fmtNum(res.corr.pitch_vsInstab, 2)}</div></div>
        </div>
        <div class="an-grid-2">
          <div class="an-chart-box"><div class="an-chart-head"><span class="an-chart-title">経過時間 [分] vs ピッチ・ロール不安定さ (RMS)</span></div><canvas id="an-cv-in1"></canvas></div>
          <div class="an-chart-box"><div class="an-chart-head"><span class="an-chart-title">平均ピッチ [deg] vs ピッチ・ロール不安定さ (RMS)</span></div><canvas id="an-cv-in2"></canvas></div>
        </div>
      `;

      drawDualScatter(document.getElementById('an-cv-in1'), res.windows,
        w => w.elapsedMin, '経過時間 [分]',
        [
          { name: 'ピッチ揺れ [deg]', color: '#7c3aed', fn: w => w.pitchInstab },
          { name: 'ロール揺れ [deg]', color: '#dc2626', fn: w => w.rollInstab }
        ], ctx);

      drawDualScatter(document.getElementById('an-cv-in2'), res.windows,
        w => w.pitchMean, '窓内平均ピッチ [deg]',
        [
          { name: 'ピッチ揺れ [deg]', color: '#7c3aed', fn: w => w.pitchInstab },
          { name: 'ロール揺れ [deg]', color: '#dc2626', fn: w => w.rollInstab }
        ], ctx);
    } else {
      if (res.points.length < 3) {
        container.innerHTML = '<div class="an-error">散布図の点が不足しています。</div>';
        return;
      }
      container.innerHTML = `
        <div class="an-chart-box">
          <div class="an-chart-head">
            <span class="an-chart-title">7秒平均ピッチ散布図（色分け: ${state.scatterColor}）</span>
            <span class="an-chart-hint">紫(低) → 緑 → 黄(高)</span>
          </div>
          <canvas id="an-cv-pa"></canvas>
        </div>
      `;
      const cv = document.getElementById('an-cv-pa');
      const { g, w, h } = setupCanvas(cv, 380);
      const pad = { l: 58, r: 24, t: 18, b: 36 };
      const xGet = state.scatterX === 'altitude' ? p => p.alt : (state.scatterX === 'tas' ? p => p.tas : p => p.elapsedMin);
      const xLabel = state.scatterX === 'altitude' ? '高度 [m]' : (state.scatterX === 'tas' ? 'TAS / G/S [m/s]' : '経過時間 [分]');
      const cGet = state.scatterColor === 'tas' ? p => p.tas : (state.scatterColor === 'time' ? p => p.elapsedMin : p => p.alt);

      const valid = res.points.filter(p => Number.isFinite(xGet(p)) && Number.isFinite(p.pitch));
      if (valid.length < 2) {
        container.innerHTML = '<div class="an-error">ピッチまたは横軸のデータが欠損しています。</div>';
        return;
      }
      let xMin = Infinity, xMax = -Infinity, yMin = Infinity, yMax = -Infinity, cMin = Infinity, cMax = -Infinity;
      for (const p of valid) {
        const xv = xGet(p), yv = p.pitch, cvVal = cGet(p);
        if (xv < xMin) xMin = xv; if (xv > xMax) xMax = xv;
        if (yv < yMin) yMin = yv; if (yv > yMax) yMax = yv;
        if (Number.isFinite(cvVal)) { if (cvVal < cMin) cMin = cvVal; if (cvVal > cMax) cMax = cvVal; }
      }
      const xPad = (xMax - xMin) * 0.06 || 1;
      const yPad = (yMax - yMin) * 0.1 || 1;
      const { xToPx, yToPx } = drawAxes(g, pad, w, h, xMin - xPad, xMax + xPad, yMin - yPad, yMax + yPad, {
        xLabel, yLabel: '平均ピッチ [deg]', yFmt: v => v.toFixed(1), xFmt: v => v.toFixed(1)
      });

      const ptsPx = valid.map(p => {
        const px = xToPx(xGet(p)), py = yToPx(p.pitch);
        const cvVal = cGet(p);
        const norm = (Number.isFinite(cvVal) && cMax > cMin) ? (cvVal - cMin) / (cMax - cMin) : 0.5;
        g.fillStyle = viridisColor(norm);
        g.beginPath(); g.arc(px, py, 4.5, 0, Math.PI * 2); g.fill();
        g.strokeStyle = 'rgba(15,23,42,0.25)'; g.lineWidth = 0.8; g.stroke();
        return { px, py, p };
      });

      attachInteractive(cv, {
        onHover: (mx, my) => {
          let best = null, bestD = 16;
          for (const item of ptsPx) {
            const d = Math.hypot(item.px - mx, item.py - my);
            if (d < bestD) { bestD = d; best = item.p; }
          }
          if (!best) return null;
          return `時刻: ${fmtSec(best.t, ctx)} (${best.elapsedMin.toFixed(1)}分)\nピッチ: ${fmtNum(best.pitch, 2, '°')}\n高度: ${fmtNum(best.alt, 1, 'm')} / TAS: ${fmtNum(best.tas, 1, 'm/s')}\nクリックでシーク`;
        },
        onClick: (mx, my) => {
          let best = null, bestD = 18;
          for (const item of ptsPx) {
            const d = Math.hypot(item.px - mx, item.py - my);
            if (d < bestD) { bestD = d; best = item.p; }
          }
          if (best) seekTo(best.t);
        }
      });
    }
  }

  function drawDualScatter(cv, items, xGet, xLabel, seriesList, ctx) {
    const { g, w, h } = setupCanvas(cv, 290);
    const pad = { l: 52, r: 16, t: 24, b: 36 };
    let xMin = Infinity, xMax = -Infinity, yMax = 0.5;
    for (const it of items) {
      const xv = xGet(it);
      if (!Number.isFinite(xv)) continue;
      if (xv < xMin) xMin = xv; if (xv > xMax) xMax = xv;
      for (const s of seriesList) {
        const yv = s.fn(it);
        if (Number.isFinite(yv) && yv > yMax) yMax = yv;
      }
    }
    if (!Number.isFinite(xMin)) return;
    const xp = (xMax - xMin) * 0.06 || 1;
    const { xToPx, yToPx } = drawAxes(g, pad, w, h, xMin - xp, xMax + xp, 0, yMax * 1.15, {
      xLabel, yLabel: '揺れ RMS [deg]', xFmt: v => v.toFixed(1), yFmt: v => v.toFixed(2)
    });

    const pts = [];
    seriesList.forEach((s, si) => {
      g.fillStyle = s.color;
      g.font = 'bold 11px sans-serif'; g.textAlign = 'left'; g.textBaseline = 'top';
      g.fillText(`● ${s.name}`, pad.l + 8 + si * 125, 6);
      for (const it of items) {
        const xv = xGet(it), yv = s.fn(it);
        if (!Number.isFinite(xv) || !Number.isFinite(yv)) continue;
        const px = xToPx(xv), py = yToPx(yv);
        g.beginPath(); g.arc(px, py, 4, 0, Math.PI * 2); g.fill();
        pts.push({ px, py, it });
      }
    });

    attachInteractive(cv, {
      onHover: (mx, my) => {
        let best = null, bestD = 15;
        for (const p of pts) { const d = Math.hypot(p.px - mx, p.py - my); if (d < bestD) { bestD = d; best = p.it; } }
        if (!best) return null;
        return `時刻: ${fmtSec(best.t, ctx)} (${best.elapsedMin.toFixed(1)}分)\n平均ピッチ: ${fmtNum(best.pitchMean, 2, '°')}\nピッチ揺れ: ${fmtNum(best.pitchInstab, 2, '°')}\nロール揺れ: ${fmtNum(best.rollInstab, 2, '°')}\nV/S揺れ: ${fmtNum(best.vsInstab, 2, 'm/s')}\nクリックでシーク`;
      },
      onClick: (mx, my) => {
        let best = null, bestD = 16;
        for (const p of pts) { const d = Math.hypot(p.px - mx, p.py - my); if (d < bestD) { bestD = d; best = p.it; } }
        if (best) seekTo(best.t);
      }
    });
  }

  // ============================================================
  // タブ 4: スペクトル & スペクトログラム
  // ★ 前半／後半（旋回前後）の比較は 2026 年大会に特化した分析なので入れない（2026-11-20）。
  // ============================================================
  function renderSpectrumTab(body, ctx) {
    const { tStart, tEnd } = getActiveRange(ctx);
    const ck = `spec_${tStart}_${tEnd}_${state.specChannel}`;
    const res = state.cache[ck] || (state.cache[ck] = DSP.computeSpectrumAndSpectrogram(ctx.flightData, tStart, tEnd, state.specChannel, 90));

    if (res.error) {
      body.innerHTML = `<div class="an-error">${res.error}</div>`;
      return;
    }

    body.innerHTML = `
      <div class="an-controls">
        <label>対象信号:
          <select id="an-sp-ch">
            <option value="pitch" ${state.specChannel === 'pitch' ? 'selected' : ''}>ピッチ [deg]</option>
            <option value="roll" ${state.specChannel === 'roll' ? 'selected' : ''}>ロール [deg]</option>
            <option value="vs" ${state.specChannel === 'vs' ? 'selected' : ''}>KF 昇降速度 V/S [m/s]</option>
            <option value="gs" ${state.specChannel === 'gs' ? 'selected' : ''}>対地速度 G/S [m/s]</option>
          </select>
        </label>
        <span style="color:#64748b;">区間の一部だけを見たいときは、上の「区間内の開始／終了」で絞ってください</span>
      </div>
      <div class="an-chart-box">
        <div class="an-chart-head">
          <span class="an-chart-title">Welch パワースペクトル (${res.label})</span>
          <span class="an-chart-hint">対象: ${fmtSec(res.t0, ctx)}〜${fmtSec(res.t1, ctx)} / 横軸: 周期 [秒] (対数)</span>
        </div>
        <canvas id="an-cv-psd"></canvas>
      </div>
      <div class="an-grid-2">
        <div class="an-chart-box">
          <div class="an-chart-head">
            <span class="an-chart-title">スペクトログラム (窓 ${res.spec.winSec}s / ステップ ${res.spec.stepSec}s)</span>
            <span class="an-chart-hint">クリックで該当時刻へシーク</span>
          </div>
          <canvas id="an-cv-stft"></canvas>
        </div>
        <div class="an-chart-box">
          <div class="an-chart-head">
            <span class="an-chart-title">周期帯別 揺れ RMS 推移 [${res.unit}]</span>
            <span class="an-chart-hint">クリックで該当時刻へシーク</span>
          </div>
          <canvas id="an-cv-bands"></canvas>
        </div>
      </div>
    `;

    document.getElementById('an-sp-ch').addEventListener('change', (e) => { state.specChannel = e.target.value; renderActiveTab(); });

    // 1. Welch PSD 描画 (横軸: 対数周期 2s..120s)
    {
      const cv = document.getElementById('an-cv-psd');
      const { g, w, h } = setupCanvas(cv, 220);
      const pad = { l: 58, r: 20, t: 14, b: 34 };
      const pMin = 2.0, pMax = 120.0;
      const logMin = Math.log10(pMin), logMax = Math.log10(pMax);
      const pw = w - pad.l - pad.r, ph = h - pad.t - pad.b;

      let maxPsd = 1e-6;
      const scanMax = (obj) => {
        if (!obj) return;
        for (let k = 1; k < obj.freqs.length; k++) {
          const per = 1.0 / obj.freqs[k];
          if (per >= pMin && per <= pMax && obj.psd[k] > maxPsd) maxPsd = obj.psd[k];
        }
      };
      scanMax(res.psdAll);

      const xToPx = (per) => pad.l + ((Math.log10(per) - logMin) / (logMax - logMin)) * pw;
      const yToPx = (v) => pad.t + ph - (v / (maxPsd * 1.12)) * ph;

      // グリッド
      g.strokeStyle = '#e2e8f0'; g.lineWidth = 1; g.font = '11px sans-serif'; g.fillStyle = '#64748b';
      for (let i = 0; i <= 4; i++) {
        const v = maxPsd * 1.12 * (i / 4);
        const py = yToPx(v);
        g.beginPath(); g.moveTo(pad.l, py); g.lineTo(pad.l + pw, py); g.stroke();
        g.textAlign = 'right'; g.textBaseline = 'middle';
        g.fillText(v.toFixed(2), pad.l - 6, py);
      }
      for (const per of [2, 3, 5, 10, 20, 30, 60, 120]) {
        const px = xToPx(per);
        g.beginPath(); g.moveTo(px, pad.t); g.lineTo(px, pad.t + ph); g.stroke();
        g.textAlign = 'center'; g.textBaseline = 'top';
        g.fillText(`${per}s`, px, pad.t + ph + 5);
      }
      g.strokeStyle = '#94a3b8'; g.strokeRect(pad.l, pad.t, pw, ph);

      const drawPsdLine = (obj, color) => {
        if (!obj) return;
        g.strokeStyle = color; g.lineWidth = 2; g.beginPath();
        let started = false;
        for (let k = obj.freqs.length - 1; k >= 1; k--) {
          const per = 1.0 / obj.freqs[k];
          if (per < pMin || per > pMax) continue;
          const px = xToPx(per), py = yToPx(obj.psd[k]);
          if (!started) { g.moveTo(px, py); started = true; } else g.lineTo(px, py);
        }
        g.stroke();
      };
      drawPsdLine(res.psdAll, '#0284c7');
    }

    // 2. スペクトログラム描画
    {
      const cv = document.getElementById('an-cv-stft');
      const { g, w, h } = setupCanvas(cv, 250);
      const pad = { l: 48, r: 14, t: 14, b: 32 };
      const pw = w - pad.l - pad.r, ph = h - pad.t - pad.b;
      const spec = res.spec;
      if (spec.times.length > 0) {
        const pMin = 2.0, pMax = Math.min(120.0, spec.winSec);
        const logMin = Math.log10(pMin), logMax = Math.log10(pMax);
        const freqs = spec.freqs;
        // 対数パワー範囲
        let minDb = Infinity, maxDb = -Infinity;
        for (const col of spec.cols) {
          for (let k = 1; k < freqs.length; k++) {
            const per = 1.0 / freqs[k];
            if (per < pMin || per > pMax) continue;
            const db = 10 * Math.log10(col[k] + 1e-8);
            if (db < minDb) minDb = db;
            if (db > maxDb) maxDb = db;
          }
        }
        const loDb = Math.max(minDb, maxDb - 30);
        const colW = pw / spec.times.length;

        for (let ci = 0; ci < spec.cols.length; ci++) {
          const x = pad.l + ci * colW;
          const col = spec.cols[ci];
          for (let k = 1; k < freqs.length; k++) {
            const per = 1.0 / freqs[k];
            if (per < pMin || per > pMax) continue;
            const perNext = k > 1 ? Math.min(pMax, 1.0 / freqs[k - 1]) : pMax;
            const yTop = pad.t + ph - ((Math.log10(perNext) - logMin) / (logMax - logMin)) * ph;
            const yBot = pad.t + ph - ((Math.log10(per) - logMin) / (logMax - logMin)) * ph;
            const db = 10 * Math.log10(col[k] + 1e-8);
            const norm = (db - loDb) / (maxDb - loDb || 1);
            g.fillStyle = viridisColor(norm);
            g.fillRect(x, yTop, Math.ceil(colW) + 0.5, Math.max(1.5, yBot - yTop + 0.5));
          }
        }
        g.strokeStyle = '#94a3b8'; g.strokeRect(pad.l, pad.t, pw, ph);
        g.font = '10.5px sans-serif'; g.fillStyle = '#475569';
        for (const per of [2, 5, 10, 20, 40, 80].filter(v => v <= pMax)) {
          const py = pad.t + ph - ((Math.log10(per) - logMin) / (logMax - logMin)) * ph;
          g.textAlign = 'right'; g.textBaseline = 'middle';
          g.fillText(`${per}s`, pad.l - 5, py);
        }
        for (let i = 0; i <= 4; i++) {
          const tVal = res.t0 + (res.t1 - res.t0) * (i / 4);
          g.textAlign = 'center'; g.textBaseline = 'top';
          g.fillText(fmtSec(tVal, ctx), pad.l + pw * (i / 4), pad.t + ph + 5);
        }
        attachInteractive(cv, {
          onHover: (mx) => {
            if (mx < pad.l || mx > pad.l + pw) return null;
            const tH = res.t0 + ((mx - pad.l) / pw) * (res.t1 - res.t0);
            return `時刻: ${fmtSec(tH, ctx)}\nクリックでシーク`;
          },
          onClick: (mx) => {
            if (mx < pad.l || mx > pad.l + pw) return;
            seekTo(res.t0 + ((mx - pad.l) / pw) * (res.t1 - res.t0));
          }
        });
      }
    }

    // 3. 周期帯別 RMS 推移
    {
      const cv = document.getElementById('an-cv-bands');
      const { g, w, h } = setupCanvas(cv, 250);
      const pad = { l: 48, r: 14, t: 24, b: 32 };
      const spec = res.spec;
      if (spec.times.length > 1) {
        let yMax = 0.2;
        for (const ser of spec.bandSeries) for (const v of ser) if (v > yMax) yMax = v;
        const { xToPx, yToPx, pw } = drawAxes(g, pad, w, h, res.t0, res.t1, 0, yMax * 1.15, {
          yLabel: `RMS [${res.unit}]`, xFmt: v => fmtSec(v, ctx), yFmt: v => v.toFixed(2), xTicks: 4
        });
        res.bands.forEach((b, bi) => {
          g.strokeStyle = b.color; g.lineWidth = 2; g.beginPath();
          spec.times.forEach((tv, ti) => {
            const px = xToPx(tv), py = yToPx(spec.bandSeries[bi][ti]);
            if (ti === 0) g.moveTo(px, py); else g.lineTo(px, py);
          });
          g.stroke();
          g.fillStyle = b.color; g.font = 'bold 10.5px sans-serif'; g.textAlign = 'left'; g.textBaseline = 'top';
          g.fillText(`━ ${b.label}`, pad.l + 4 + bi * 112, 6);
        });
        drawCurrentCursor(g, pad, h, xToPx, res.t0, res.t1, ctx.currentTimeSec);
        attachInteractive(cv, {
          onHover: (mx) => {
            if (mx < pad.l || mx > pad.l + pw) return null;
            const tH = res.t0 + ((mx - pad.l) / pw) * (res.t1 - res.t0);
            let best = 0;
            for (let i = 1; i < spec.times.length; i++) if (Math.abs(spec.times[i] - tH) < Math.abs(spec.times[best] - tH)) best = i;
            const lines = [`時刻: ${fmtSec(spec.times[best], ctx)}`];
            res.bands.forEach((b, bi) => lines.push(`${b.label}: ${spec.bandSeries[bi][best].toFixed(2)} ${res.unit}`));
            return lines.join('\n');
          },
          onClick: (mx) => {
            if (mx < pad.l || mx > pad.l + pw) return;
            seekTo(res.t0 + ((mx - pad.l) / pw) * (res.t1 - res.t0));
          }
        });
      }
    }
  }

  // ============================================================
  // タブ 5: フゴイド解析 (plot_phugoid + plot_phugoid_windows 準拠)
  // ============================================================
  function renderPhugoidTab(body, ctx) {
    const { tStart, tEnd } = getActiveRange(ctx);
    const ck = `phug_${tStart}_${tEnd}_${state.phugoidPMin}_${state.phugoidPMax}`;
    const res = state.cache[ck] || (state.cache[ck] = DSP.computePhugoid(ctx.flightData, tStart, tEnd, state.phugoidPMin, state.phugoidPMax));

    if (res.error) {
      body.innerHTML = `<div class="an-error">${res.error}</div>`;
      return;
    }

    if (state.phugoidSelectedWin >= res.windows.length) state.phugoidSelectedWin = 0;
    // 初回は最もフゴイドらしさが高い区間を自動選択
    if (!state.cache[ck + '_init'] && res.windows.length > 0) {
      state.cache[ck + '_init'] = true;
      let best = 0;
      for (let i = 1; i < res.windows.length; i++) if (res.windows[i].score > res.windows[best].score) best = i;
      state.phugoidSelectedWin = best;
    }

    const gp = res.globalPhase;
    const selWin = res.windows[state.phugoidSelectedWin] || null;

    body.innerHTML = `
      <div class="an-controls">
        <label>抽出帯域 周期 [秒]:
          <input type="number" id="an-ph-pmin" min="1.5" max="8" step="0.5" value="${state.phugoidPMin}" style="width:60px;"> 〜
          <input type="number" id="an-ph-pmax" min="4" max="25" step="1" value="${state.phugoidPMax}" style="width:60px;">
        </label>
        <span style="color:#64748b;">V/S ソース: <b>${res.altSource}</b>（KF V/S と IMU の人工相関を避けるため気圧高度微分を使用）</span>
      </div>
      <div class="an-cards">
        <div class="an-card">
          <div class="an-card-label">推定周期 T (${res.primaryName} ACF)</div>
          <div class="an-card-val" style="color:#0284c7;">${res.fit ? fmtNum(res.fit.T, 2, ' 秒') : '---'}</div>
          <div class="an-card-sub">古典理論値 π√2·V/g: ${fmtNum(res.classicalT, 2, ' 秒')} (平均 ${fmtNum(res.meanV, 1, 'm/s')})</div>
        </div>
        <div class="an-card">
          <div class="an-card-label">減衰比 ζ (フィット R²)</div>
          <div class="an-card-val">${res.fit ? fmtNum(res.fit.zeta, 3) : '---'} <span style="font-size:0.8rem; color:#64748b;">(R²=${res.fit ? res.fit.r2.toFixed(2) : '---'})</span></div>
          <div class="an-card-sub">小さいほど振動が尾を引く</div>
        </div>
        <div class="an-card">
          <div class="an-card-label">条件1: ピッチ vs V/S 位相差</div>
          <div class="an-card-val" style="color:${Math.abs(gp.phase1Deg) < 45 ? '#16a34a' : '#64748b'};">${fmtNum(gp.phase1Deg, 1, '°')}</div>
          <div class="an-card-sub">フゴイドなら 0° 付近 (s1=${fmtNum(gp.s1, 2)})</div>
        </div>
        <div class="an-card">
          <div class="an-card-label">条件2: 高度 vs G/S 位相差</div>
          <div class="an-card-val" style="color:${Math.abs(gp.phase2Deg) > 135 ? '#16a34a' : '#64748b'};">${fmtNum(gp.phase2Deg, 1, '°')}</div>
          <div class="an-card-sub">フゴイドなら ±180° 付近 (s2=${fmtNum(gp.s2, 2)})</div>
        </div>
      </div>
      <div class="an-grid-2">
        <div class="an-chart-box">
          <div class="an-chart-head">
            <span class="an-chart-title">自己相関関数 (ACF) & 2次系減衰振動フィット</span>
            <span class="an-chart-hint">青: 実測 ACF / 赤破線: 2次系モデル</span>
          </div>
          <canvas id="an-cv-ph-acf"></canvas>
        </div>
        <div class="an-chart-box">
          <div class="an-chart-head">
            <span class="an-chart-title">60秒区間ごとの「フゴイドらしさ」推移</span>
            <span class="an-chart-hint">バーをクリックすると下の詳細波形を切替＆シーク</span>
          </div>
          <canvas id="an-cv-ph-win"></canvas>
        </div>
      </div>
      <div class="an-chart-box">
        <div class="an-chart-head">
          <span class="an-chart-title">選択区間の正規化波形 (位相確認): ${selWin ? `${fmtSec(selWin.tStart, ctx)} 〜 ${fmtSec(selWin.tEnd, ctx)} (らしさ=${selWin.score.toFixed(2)}, P-VS=${fmtNum(selWin.phase1Deg, 0, '°')}, Alt-GS=${fmtNum(selWin.phase2Deg, 0, '°')})` : ''}</span>
          <span class="an-chart-hint">紫: Pitch / 緑: 非KF V/S (同位相が目安) | 青: Alt / 橙破線: G/S (逆位相が目安)</span>
        </div>
        <canvas id="an-cv-ph-wave"></canvas>
      </div>
    `;

    document.getElementById('an-ph-pmin').addEventListener('change', (e) => { state.phugoidPMin = Math.max(1.5, parseFloat(e.target.value) || 2.5); renderActiveTab(); });
    document.getElementById('an-ph-pmax').addEventListener('change', (e) => { state.phugoidPMax = Math.max(state.phugoidPMin + 1, parseFloat(e.target.value) || 10); renderActiveTab(); });

    // 1. ACF 描画
    drawAcfChart(document.getElementById('an-cv-ph-acf'), res.acf, res.fit, res.fs, null, null);

    // 2. 区間別フゴイドらしさバー
    {
      const cv = document.getElementById('an-cv-ph-win');
      const { g, w, h } = setupCanvas(cv, 220);
      const pad = { l: 46, r: 14, t: 14, b: 32 };
      const wins = res.windows;
      if (wins.length > 0) {
        let maxScore = 0.1;
        for (const wi of wins) if (wi.score > maxScore) maxScore = wi.score;
        const { xToPx, yToPx, pw, ph } = drawAxes(g, pad, w, h, 0, wins.length, 0, maxScore * 1.15, {
          yLabel: 'フゴイドらしさ', xTicks: Math.min(5, wins.length),
          xFmt: v => {
            const idx = Math.min(wins.length - 1, Math.max(0, Math.floor(v)));
            return fmtSec(wins[idx].tCenter, ctx);
          },
          yFmt: v => v.toFixed(2)
        });
        const barW = Math.max(4, (pw / wins.length) * 0.78);
        wins.forEach((wi, idx) => {
          const cx = xToPx(idx + 0.5);
          const yTop = yToPx(wi.score);
          g.fillStyle = (idx === state.phugoidSelectedWin) ? '#dc2626' : '#0284c7';
          g.fillRect(cx - barW * 0.5, yTop, barW, pad.t + ph - yTop);
        });
        attachInteractive(cv, {
          onHover: (mx) => {
            if (mx < pad.l || mx > pad.l + pw) return null;
            const idx = Math.min(wins.length - 1, Math.max(0, Math.floor(((mx - pad.l) / pw) * wins.length)));
            const wi = wins[idx];
            return `区間: ${fmtSec(wi.tStart, ctx)}〜${fmtSec(wi.tEnd, ctx)}\nフゴイドらしさ: ${wi.score.toFixed(3)}\nP-VS 位相差: ${fmtNum(wi.phase1Deg, 1, '°')} (s1=${fmtNum(wi.s1, 2)})\nAlt-GS 位相差: ${fmtNum(wi.phase2Deg, 1, '°')} (s2=${fmtNum(wi.s2, 2)})\nクリックで波形表示＆シーク`;
          },
          onClick: (mx) => {
            if (mx < pad.l || mx > pad.l + pw) return;
            const idx = Math.min(wins.length - 1, Math.max(0, Math.floor(((mx - pad.l) / pw) * wins.length)));
            state.phugoidSelectedWin = idx;
            seekTo(wins[idx].tCenter);
          }
        });
      }
    }

    // 3. 選択区間の 4 本波形
    if (selWin) {
      const cv = document.getElementById('an-cv-ph-wave');
      const { g, w, h } = setupCanvas(cv, 210);
      const pad = { l: 48, r: 16, t: 14, b: 30 };
      const { xToPx, yToPx, pw } = drawAxes(g, pad, w, h, selWin.tStart, selWin.tEnd, -3, 3, {
        yLabel: '標準化振幅 [σ]', xFmt: v => fmtSec(v, ctx), yFmt: v => v.toFixed(0)
      });
      const drawNorm = (arr, color, dash) => {
        if (!arr) return;
        const sub = arr.subarray(selWin.i0, selWin.i1);
        const s = DSP.std(sub) || 1;
        g.strokeStyle = color; g.lineWidth = 1.7;
        g.setLineDash(dash || []);
        g.beginPath();
        for (let i = 0; i < sub.length; i++) {
          const px = xToPx(res.t[selWin.i0 + i]);
          const py = yToPx(Math.max(-3, Math.min(3, sub[i] / s)));
          if (i === 0) g.moveTo(px, py); else g.lineTo(px, py);
        }
        g.stroke();
        g.setLineDash([]);
      };
      drawNorm(res.pitchBp, '#7c3aed', []);
      drawNorm(res.vsBp, '#16a34a', []);
      drawNorm(res.altBp, '#0284c7', []);
      drawNorm(res.gsBp, '#ea580c', [5, 3]);
      drawCurrentCursor(g, pad, h, xToPx, selWin.tStart, selWin.tEnd, ctx.currentTimeSec);

      attachInteractive(cv, {
        onHover: (mx) => {
          if (mx < pad.l || mx > pad.l + pw) return null;
          const tH = selWin.tStart + ((mx - pad.l) / pw) * (selWin.tEnd - selWin.tStart);
          return `時刻: ${fmtSec(tH, ctx)}\nクリックでシーク`;
        },
        onClick: (mx) => {
          if (mx < pad.l || mx > pad.l + pw) return;
          seekTo(selWin.tStart + ((mx - pad.l) / pw) * (selWin.tEnd - selWin.tStart));
        }
      });
    }
  }

  // ============================================================
  // タブ 6: 横・方向モード (ダッチロール / スパイラル)
  // ============================================================
  function renderLateralTab(body, ctx) {
    const { tStart, tEnd } = getActiveRange(ctx);
    const ck = `lat_${tStart}_${tEnd}_${state.lateralThresh}`;
    const res = state.cache[ck] || (state.cache[ck] = DSP.computeLateralModes(ctx.flightData, tStart, tEnd, state.lateralThresh));

    if (res.error) {
      body.innerHTML = `<div class="an-error">${res.error}</div>`;
      return;
    }

    let cmpRes = null;
    if (state.compareData) {
      const cck = `lat_cmp_${state.compareData.name}_${state.lateralThresh}`;
      cmpRes = state.cache[cck] || (state.cache[cck] = DSP.computeLateralModes(state.compareData.records, state.compareData.takeoff, state.compareData.land, state.lateralThresh));
    }

    body.innerHTML = `
      <div class="an-controls">
        <label>旋回除外しきい値 |r| [deg/s]:
          <input type="number" id="an-lat-th" min="0.5" max="5" step="0.25" value="${state.lateralThresh}" style="width:65px;">
        </label>
        <input type="file" id="an-lat-cmp-input" accept=".csv" style="display:none;">
        <button id="an-lat-cmp-btn">📂 比較用 CSV を追加 (2機比較)</button>
        ${state.compareData ? `<span style="color:#ea580c; font-weight:bold;">比較中: ${escHtml(state.compareData.name)}${state.compareData.segLabel ? ` の最長の${state.compareData.segLabel}` : ''}</span> <button id="an-lat-cmp-clear">✕ 解除</button>` : ''}
      </div>
      <div class="an-cards">
        <div class="an-card">
          <div class="an-card-label">ダッチロール周期 T (r / Roll)</div>
          <div class="an-card-val" style="color:#0284c7;">${res.fitR ? fmtNum(res.fitR.T, 2, 's') : '---'} / ${res.fitRoll ? fmtNum(res.fitRoll.T, 2, 's') : '---'}</div>
          <div class="an-card-sub">${cmpRes && cmpRes.fitR ? `比較機: ${cmpRes.fitR.T.toFixed(2)}s / ${cmpRes.fitRoll ? cmpRes.fitRoll.T.toFixed(2) + 's' : '---'}` : `クリーン直進区間: ${(res.cleanRatio * 100).toFixed(0)}%`}</div>
        </div>
        <div class="an-card">
          <div class="an-card-label">ダッチロール減衰比 ζ (r / Roll)</div>
          <div class="an-card-val">${res.fitR ? fmtNum(res.fitR.zeta, 3) : '---'} / ${res.fitRoll ? fmtNum(res.fitRoll.zeta, 3) : '---'}</div>
          <div class="an-card-sub">${cmpRes && cmpRes.fitR ? `比較機: ${cmpRes.fitR.zeta.toFixed(3)} / ${cmpRes.fitRoll ? cmpRes.fitRoll.zeta.toFixed(3) : '---'}` : '小さいほど横揺れが収まりにくい'}</div>
        </div>
        <div class="an-card">
          <div class="an-card-label">スパイラル傾向 (低周波 r RMS)</div>
          <div class="an-card-val">${fmtNum(res.rSlowRms, 3, ' °/s')}</div>
          <div class="an-card-sub">${cmpRes ? `比較機: ${fmtNum(cmpRes.rSlowRms, 3, ' °/s')}` : `ゼロクロス間隔: ${fmtNum(res.meanZeroCrossSec, 1, '秒')}`}</div>
        </div>
        <div class="an-card">
          <div class="an-card-label">クリーン区間 ロール揺れ RMS</div>
          <div class="an-card-val">${fmtNum(res.rollRmsClean, 2, '°')}</div>
          <div class="an-card-sub">${cmpRes ? `比較機: ${fmtNum(cmpRes.rollRmsClean, 2, '°')}` : `高周波 r RMS: ${fmtNum(res.rFastRms, 2, ' °/s')}`}</div>
        </div>
      </div>
      <div class="an-chart-box">
        <div class="an-chart-head">
          <span class="an-chart-title">旋回角速度 r = d(TrueTrack)/dt [deg/s] と旋回除外区間</span>
          <span class="an-chart-hint">薄赤帯: 意図した旋回として除外した区間 / クリックでシーク</span>
        </div>
        <canvas id="an-cv-lat-r"></canvas>
      </div>
      <div class="an-grid-2">
        <div class="an-chart-box">
          <div class="an-chart-head">
            <span class="an-chart-title">クリーン区間 旋回角速度 r の自己相関 (ACF)</span>
            <span class="an-chart-hint">青: 自機 / 赤破線: フィット${cmpRes ? ' / 橙: 比較機' : ''}</span>
          </div>
          <canvas id="an-cv-lat-acfr"></canvas>
        </div>
        <div class="an-chart-box">
          <div class="an-chart-head">
            <span class="an-chart-title">クリーン区間 ロール角の自己相関 (ACF)</span>
            <span class="an-chart-hint">GPS に依らない IMU 裏取り</span>
          </div>
          <canvas id="an-cv-lat-acfroll"></canvas>
        </div>
      </div>
    `;

    document.getElementById('an-lat-th').addEventListener('change', (e) => {
      state.lateralThresh = Math.max(0.5, Math.min(5, parseFloat(e.target.value) || 1.5));
      renderActiveTab();
    });
    document.getElementById('an-lat-cmp-btn').addEventListener('click', () => document.getElementById('an-lat-cmp-input').click());
    document.getElementById('an-lat-cmp-input').addEventListener('change', (e) => {
      const f = e.target.files && e.target.files[0];
      if (!f) return;
      const reader = new FileReader();
      reader.onload = () => {
        const parsed = DSP.parseComparisonCSV(String(reader.result || ''));
        if (!parsed) { alert('TrueTrack 列を含む有効な CSV ではありませんでした。'); return; }
        // 比較機の CSV も、自機と同じ分け方の「動いている区間」の最長の 1 つで解析する（停止中は入れない）
        const cmpSegs = global.computeMovingSegments ? global.computeMovingSegments(parsed.records).segments : [];
        let best = null;
        for (const s of cmpSegs) if (!best || s.end - s.start > best.end - best.start) best = s;
        if (best) { parsed.takeoff = best.start; parsed.land = best.end; }
        state.compareData = {
          name: f.name, ...parsed,
          segLabel: best ? `区間 #${best.no}/${cmpSegs.length} (${fmtDur(best.end - best.start)})` : ''
        };
        renderActiveTab();
      };
      reader.readAsText(f);
    });
    const clr = document.getElementById('an-lat-cmp-clear');
    if (clr) clr.addEventListener('click', () => { state.compareData = null; renderActiveTab(); });

    // 1. r 時系列描画
    {
      const cv = document.getElementById('an-cv-lat-r');
      const { g, w, h } = setupCanvas(cv, 210);
      const pad = { l: 52, r: 16, t: 14, b: 30 };
      let maxAbs = 3.0;
      for (let i = 0; i < res.rRate.length; i++) if (Math.abs(res.rRate[i]) > maxAbs) maxAbs = Math.abs(res.rRate[i]);
      maxAbs = Math.min(25, maxAbs * 1.1);
      const t0 = res.t[0], t1 = res.t[res.t.length - 1];
      const { xToPx, yToPx, pw, ph } = drawAxes(g, pad, w, h, t0, t1, -maxAbs, maxAbs, {
        yLabel: 'r [deg/s]', xFmt: v => fmtSec(v, ctx), yFmt: v => v.toFixed(1)
      });

      // 旋回除外帯
      g.fillStyle = 'rgba(239, 68, 68, 0.14)';
      let runStart = -1;
      for (let i = 0; i <= res.isTurn.length; i++) {
        const turn = (i < res.isTurn.length && res.isTurn[i] === 1);
        if (turn && runStart < 0) runStart = i;
        if (!turn && runStart >= 0) {
          const xA = xToPx(res.t[runStart]), xB = xToPx(res.t[i - 1]);
          g.fillRect(xA, pad.t, Math.max(1, xB - xA), ph);
          runStart = -1;
        }
      }

      // r 波形
      g.strokeStyle = '#0284c7'; g.lineWidth = 1.2; g.beginPath();
      for (let i = 0; i < res.t.length; i++) {
        const px = xToPx(res.t[i]), py = yToPx(Math.max(-maxAbs, Math.min(maxAbs, res.rRate[i])));
        if (i === 0) g.moveTo(px, py); else g.lineTo(px, py);
      }
      g.stroke();
      drawCurrentCursor(g, pad, h, xToPx, t0, t1, ctx.currentTimeSec);

      attachInteractive(cv, {
        onHover: (mx) => {
          if (mx < pad.l || mx > pad.l + pw) return null;
          const tH = t0 + ((mx - pad.l) / pw) * (t1 - t0);
          const idx = Math.min(res.t.length - 1, Math.max(0, Math.round((tH - t0) * res.fs)));
          return `時刻: ${fmtSec(res.t[idx], ctx)}\n旋回角速度 r: ${res.rRate[idx].toFixed(2)} °/s\n判定: ${res.isTurn[idx] ? '旋回(除外)' : '直進(解析対象)'}\nクリックでシーク`;
        },
        onClick: (mx) => {
          if (mx < pad.l || mx > pad.l + pw) return;
          seekTo(t0 + ((mx - pad.l) / pw) * (t1 - t0));
        }
      });
    }

    // 2. ACF (r / Roll)
    drawAcfChart(document.getElementById('an-cv-lat-acfr'), res.acfR, res.fitR, res.fs, cmpRes ? cmpRes.acfR : null, cmpRes ? cmpRes.fitR : null);
    if (res.acfRoll) {
      drawAcfChart(document.getElementById('an-cv-lat-acfroll'), res.acfRoll, res.fitRoll, res.fs, cmpRes ? cmpRes.acfRoll : null, cmpRes ? cmpRes.fitRoll : null);
    } else {
      const cv = document.getElementById('an-cv-lat-acfroll');
      const { g, w, h } = setupCanvas(cv, 220);
      g.fillStyle = '#64748b'; g.font = '13px sans-serif'; g.textAlign = 'center'; g.textBaseline = 'middle';
      g.fillText('この CSV には Roll 列がありません', w * 0.5, h * 0.5);
    }
  }

  function drawAcfChart(cv, acf, fit, fs, cmpAcf, cmpFit) {
    const { g, w, h } = setupCanvas(cv, 220);
    const pad = { l: 46, r: 16, t: 14, b: 32 };
    const maxSec = (acf.length - 1) / fs;
    const { xToPx, yToPx, pw } = drawAxes(g, pad, w, h, 0, maxSec, -1, 1, {
      yLabel: 'ACF', xLabel: 'ラグ τ [秒]', yFmt: v => v.toFixed(1), xFmt: v => v.toFixed(0)
    });
    // 0 線
    g.strokeStyle = '#cbd5e1'; g.beginPath(); g.moveTo(pad.l, yToPx(0)); g.lineTo(pad.l + pw, yToPx(0)); g.stroke();

    const drawLine = (arr, color, width, dash) => {
      if (!arr) return;
      g.strokeStyle = color; g.lineWidth = width; g.setLineDash(dash || []);
      g.beginPath();
      for (let k = 0; k < arr.length; k++) {
        const px = xToPx(k / fs), py = yToPx(Math.max(-1, Math.min(1, arr[k])));
        if (k === 0) g.moveTo(px, py); else g.lineTo(px, py);
      }
      g.stroke();
      g.setLineDash([]);
    };

    if (cmpAcf) drawLine(cmpAcf, '#ea580c', 1.8, []);
    if (cmpFit) drawLine(cmpFit.fitCurve, '#f97316', 1.3, [4, 3]);
    drawLine(acf, '#0284c7', 2.0, []);
    if (fit) drawLine(fit.fitCurve, '#dc2626', 1.5, [5, 3]);
  }

  // ------------------------------------------------------------
  // エクスポート (KML / PNG)
  // ------------------------------------------------------------
  function exportKmlFromModal() {
    const ctx = getViewerContext();
    if (!ctx.flightData || ctx.flightData.length < 2) return;
    const { tStart, tEnd } = getActiveRange(ctx);
    const kml = DSP.buildKmlString(ctx.flightData, ctx.filename, { tStart, tEnd });
    const blob = new Blob([kml], { type: 'application/vnd.google-earth.kml+xml;charset=utf-8' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    const kseg = getSelectedSegment(ctx);
    a.download = `${(ctx.filename || 'pons_flight').replace(/\.csv$/i, '')}${kseg ? `_seg${kseg.no}` : ''}.kml`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  }

  function exportActiveCanvasesPng() {
    const canvases = Array.from(document.querySelectorAll('#an-body canvas'));
    if (canvases.length === 0) return;
    const gap = 16;
    const maxW = Math.max(...canvases.map(c => c.width));
    const totalH = canvases.reduce((s, c) => s + c.height + gap, gap);
    const merged = document.createElement('canvas');
    merged.width = maxW + gap * 2;
    merged.height = totalH;
    const g = merged.getContext('2d');
    g.fillStyle = '#ffffff';
    g.fillRect(0, 0, merged.width, merged.height);
    let y = gap;
    for (const c of canvases) {
      g.drawImage(c, gap, y);
      y += c.height + gap;
    }
    const url = merged.toDataURL('image/png');
    const a = document.createElement('a');
    a.href = url;
    const pseg = getSelectedSegment(getViewerContext());
    a.download = `pons_analysis_${state.activeTab}${pseg ? `_seg${pseg.no}` : ''}.png`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
  }

  // ============================================================
  // 生 IMU ログ (.bin) USB自動検出バナー & タブ 7 (ケイデンス / 高周波振動)
  // ============================================================
  function fmtBytesShort(b) {
    if (!b || b <= 0) return '0 B';
    if (b >= 1048576) return (b / 1048576).toFixed(1) + ' MB';
    if (b >= 1024) return (b / 1024).toFixed(0) + ' KB';
    return b + ' B';
  }

  // 生 IMU ログの読み込みで気になったこと（転送の欠落・区切りの探し直し・ゼロ埋め）。無ければ空
  function renderImuWarnings() {
    const list = [];
    if (state.imuTransferWarning) list.push(state.imuTransferWarning);
    if (state.imuDecoded && state.imuDecoded.warnings) list.push(...state.imuDecoded.warnings);
    if (list.length === 0) return '';
    return `<div class="an-note" style="border-left-color:#d97706; background:#fffbeb; color:#92400e;">
      ⚠️ 生 IMU ログ: ${list.map(escHtml).join(' / ')}。区切りを探し直した所の前後はデータが欠けています。
    </div>`;
  }

  function escHtml(s) {
    return String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');
  }

  function renderImuQuickBanner(ctx, targetTab) {
    const info = global.getPonsSerialImuMatch ? global.getPonsSerialImuMatch(ctx.filename) : { connected: false };
    let usbBtn = '';
    if (info.connected) {
      if (info.matched) {
        usbBtn = `<button id="an-imu-usb-fetch" class="btn-primary" style="font-size:0.78rem; padding:4px 10px;">⚡ 本体から同日生IMUを取得 (${escHtml(info.matched.path)}, ${fmtBytesShort(info.matched.size)})</button>`;
      } else if (info.listAllUnsupported) {
        usbBtn = `<span style="color:#b45309; font-size:0.76rem;">⚠️ この本体のファームは生IMU一覧取得 (PONS:LIST,ALL) に未対応です</span>`;
      } else if (!info.listIsAll) {
        usbBtn = `<button id="an-imu-usb-list" style="border-color:#0284c7; color:#0284c7; font-size:0.78rem; padding:4px 10px;">🔄 本体から生IMU一覧を取得</button>`;
      } else if (info.imuFiles && info.imuFiles.length > 0) {
        const opts = info.imuFiles.map(f => `<option value="${escHtml(f.path)}" data-size="${f.size}">${escHtml(f.path)} (${fmtBytesShort(f.size)})</option>`).join('');
        usbBtn = `<select id="an-imu-usb-select" style="font-size:0.78rem; padding:3px 6px;">${opts}</select>
                  <button id="an-imu-usb-fetch-sel" class="btn-primary" style="font-size:0.78rem; padding:4px 10px;">⚡ 本体から取得</button>`;
      } else {
        usbBtn = `<span style="color:#64748b; font-size:0.76rem;">本体 SD に imuraw/*.bin が見つかりませんでした</span>`;
      }
    } else {
      usbBtn = `<span style="color:#64748b; font-size:0.76rem;">PONS を USB 接続すると、同日の生 IMU ログを本体から直接取得できます</span>`;
    }
    return `
      <div class="an-controls" style="background:#f0fdf4; border-color:#bbf7d0;">
        <span style="font-weight:700; color:#166534;">📡 250Hz 生 IMU ログ (.bin):</span>
        ${usbBtn}
        <input type="file" id="an-imu-file-in" accept=".bin" style="display:none;">
        <button id="an-imu-file-btn" style="font-size:0.78rem; padding:4px 10px;">📂 ローカル .bin を開く</button>
      </div>
    `;
  }

  function bindImuQuickBannerEvents(ctx, targetTab) {
    const fBtn = document.getElementById('an-imu-file-btn');
    const fIn = document.getElementById('an-imu-file-in');
    if (fBtn && fIn) {
      fBtn.addEventListener('click', () => fIn.click());
      fIn.addEventListener('change', (e) => {
        const f = e.target.files && e.target.files[0];
        if (!f) return;
        const rd = new FileReader();
        rd.onload = () => loadImuBinBytes(rd.result, f.name, { openModal: true, tab: targetTab });
        rd.readAsArrayBuffer(f);
      });
    }
    const info = global.getPonsSerialImuMatch ? global.getPonsSerialImuMatch(ctx.filename) : { connected: false };
    const usbFetch = document.getElementById('an-imu-usb-fetch');
    if (usbFetch && info.matched && global.fetchPonsImuForAnalysis) {
      usbFetch.addEventListener('click', () => {
        closeAnalysisModal();
        global.fetchPonsImuForAnalysis(info.matched.path, info.matched.size, targetTab);
      });
    }
    const usbFetchSel = document.getElementById('an-imu-usb-fetch-sel');
    const usbSel = document.getElementById('an-imu-usb-select');
    if (usbFetchSel && usbSel && global.fetchPonsImuForAnalysis) {
      usbFetchSel.addEventListener('click', () => {
        const opt = usbSel.options[usbSel.selectedIndex];
        if (!opt) return;
        closeAnalysisModal();
        global.fetchPonsImuForAnalysis(opt.value, parseInt(opt.dataset.size || '0', 10), targetTab);
      });
    }
    const usbList = document.getElementById('an-imu-usb-list');
    if (usbList && global.requestAllPonsFilesForImu) {
      usbList.addEventListener('click', async () => {
        usbList.disabled = true;
        usbList.textContent = '🔄 一覧取得中...';
        await global.requestAllPonsFilesForImu();
      });
    }
  }

  function renderImuTab(body, ctx) {
    if (!state.imuDecoded || !state.imuMatch || !IMU()) {
      body.innerHTML = `
        ${renderImuQuickBanner(ctx, 'imu')}
        <div class="an-note">
          生 IMU ログ（<code>imuraw/YYYYMMDD.bin</code>）を読み込むと、以下の 250Hz 高帯域解析が利用できます：<br>
          • <b>ケイデンス推定 [RPM]</b>（前後加速度の踏み込み振動＋比のスペクトルで倍音・クランク1回転との取り違え検証）<br>
          • <b>高周波振動スペクトル（1〜125Hz）</b>（ペラ回転・機体共振・桁振動の周波数特定）<br>
          • <b>250Hz 直接計測 鉛直G</b>（「⚡ 鉛直G」タブに自動重ね表示）
        </div>
      `;
      bindImuQuickBannerEvents(ctx, 'imu');
      return;
    }

    const dec = state.imuDecoded;
    const match = state.imuMatch;
    const sess = match.session;
    // CSV と同期できていて、動いている区間があれば、その区間だけを解析する（停止中は解析しない）
    const hasCsv = ctx.flightData && ctx.flightData.length >= 5 && match.synced && !!getSelectedSegment(ctx);
    const { tStart, tEnd } = hasCsv ? getActiveRange(ctx) : { tStart: 0, tEnd: sess.durSec };

    const cck = `cad_${match.sessionIndex}_${tStart}_${tEnd}_${state.cadenceBandLow}_${state.cadenceBandHigh}_${state.cadenceMinSnr}_${state.cadenceAxis}`;
    const cad = state.cache[cck] || (state.cache[cck] = IMU().computeCadence(sess, match.toFlightSec, tStart, tEnd, {
      bandLow: state.cadenceBandLow,
      bandHigh: state.cadenceBandHigh,
      minSnrDb: state.cadenceMinSnr,
      axis: state.cadenceAxis
    }));

    const hfck = `hf_${match.sessionIndex}_${tStart}_${tEnd}_${state.hfChannel}`;
    const hf = state.cache[hfck] || (state.cache[hfck] = IMU().computeHighFreqVib(sess, match.toFlightSec, tStart, tEnd, state.hfChannel));

    const sessOpts = dec.sessions.map((s, idx) =>
      `<option value="${idx}" ${idx === match.sessionIndex ? 'selected' : ''}>#${idx} (${(s.durSec / 60).toFixed(1)}分, ${s.fs.toFixed(0)}Hz, ${s.nAcc.toLocaleString()}点)</option>`
    ).join('');

    const csvLoaded = ctx.flightData && ctx.flightData.length >= 5;
    const syncText = match.synced
      ? `🔗 飛行CSVと時刻同期済み (重複 ${(match.overlapSec / 60).toFixed(1)}分)`
      : (csvLoaded && match.hasCsvTime && sess.jstOffset !== null
        ? '⚠️ このセッションは飛行CSVと時刻が重なりません — 生IMU単独モード'
        : (csvLoaded && sess.jstOffset === null
          ? '⏱️ このセッションには GNSS の時刻が無く CSV と合わせられません — 生IMU単独モード'
          : '⏱️ 生IMU単独モード (CSVと未同期)'));

    body.innerHTML = `
      ${renderImuWarnings()}
      <div class="an-controls" style="background:#f0fdf4; border-color:#bbf7d0;">
        <span style="font-weight:700; color:#166534;">✅ ${escHtml(dec.filename)} (${fmtBytesShort(dec.byteLength)})</span>
        <label>セッション:
          <select id="an-imu-sess">${sessOpts}</select>
        </label>
        <span style="font-size:0.78rem; color:${match.synced ? '#15803d' : '#b45309'}; font-weight:600;">
          ${syncText}
        </span>
        <button id="an-imu-clear" style="margin-left:auto; font-size:0.76rem;">✕ 生IMU解除</button>
      </div>
      <div class="an-controls">
        <label>探索帯域プリセット:
          <select id="an-cad-preset">
            <option value="hpa" ${state.cadencePreset === 'hpa' ? 'selected' : ''}>人力機 (2.2–4.0Hz / 66–120 RPM)</option>
            <option value="bike" ${state.cadencePreset === 'bike' ? 'selected' : ''}>自転車テスト (1.0–2.6Hz / 30–78 RPM)</option>
            <option value="custom" ${state.cadencePreset === 'custom' ? 'selected' : ''}>カスタム</option>
          </select>
        </label>
        <label>帯域 [Hz]:
          <input type="number" id="an-cad-lo" min="0.5" max="8" step="0.1" value="${state.cadenceBandLow}" style="width:58px;"> 〜
          <input type="number" id="an-cad-hi" min="1.0" max="10" step="0.1" value="${state.cadenceBandHigh}" style="width:58px;">
        </label>
        <label>軸:
          <select id="an-cad-axis">
            <option value="fwd" ${state.cadenceAxis === 'fwd' ? 'selected' : ''}>前後 (機首 +Z)</option>
            <option value="up" ${state.cadenceAxis === 'up' ? 'selected' : ''}>上下 (比較用)</option>
          </select>
        </label>
        <label>SNR下限 [dB]:
          <input type="number" id="an-cad-snr" min="3" max="20" step="1" value="${state.cadenceMinSnr}" style="width:55px;">
        </label>
      </div>
      ${cad.error ? `<div class="an-error">${cad.error}</div>` : `
        <div class="an-cards">
          <div class="an-card">
            <div class="an-card-label">ケイデンス中央値 (クランク)</div>
            <div class="an-card-val" style="color:#dc2626;">${fmtNum(cad.medianRpm, 1, ' RPM')}</div>
            <div class="an-card-sub">踏み込み: ${fmtNum(cad.medianRpm * 2, 0, ' 回/分')} (${fmtNum(cad.medianRpm / 30, 2, ' Hz')})</div>
          </div>
          <div class="an-card">
            <div class="an-card-label">推定プロペラ回転数 (×1.5ギア比)</div>
            <div class="an-card-val" style="color:#0284c7;">${fmtNum(cad.medianRpm * 1.5, 1, ' RPM')}</div>
            <div class="an-card-sub">目安 135 RPM (クランク 90 RPM)</div>
          </div>
          <div class="an-card">
            <div class="an-card-label">検出率 (SNR ≥ ${cad.minSnrDb} dB)</div>
            <div class="an-card-val">${(cad.detectRate * 100).toFixed(0)}%</div>
            <div class="an-card-sub">${cad.detCount} / ${cad.totalWins} 窓</div>
          </div>
          <div class="an-card">
            <div class="an-card-label">高周波ピーク (${state.hfChannel.toUpperCase()}, >1Hz)</div>
            <div class="an-card-val">${hf.error ? '---' : fmtNum(hf.peakF, 1, ' Hz')}</div>
            <div class="an-card-sub">生IMU ${sess.fs.toFixed(0)}Hz (Nyquist ${(sess.fs * 0.5).toFixed(0)}Hz)</div>
          </div>
        </div>
        <div class="an-chart-box">
          <div class="an-chart-head">
            <span class="an-chart-title">ケイデンス [RPM] 時系列推移</span>
            <span class="an-chart-hint">赤点: SNR ≥ ${cad.minSnrDb}dB で検出 / クリックで該当時刻へシーク</span>
          </div>
          <canvas id="an-cv-cad-rpm"></canvas>
        </div>
        <div class="an-grid-2">
          <div class="an-chart-box">
            <div class="an-chart-head">
              <span class="an-chart-title">白色化スペクトログラム (0.5–8 Hz)</span>
              <span class="an-chart-hint">白点: 検出ピーク周波数 / クリックでシーク</span>
            </div>
            <canvas id="an-cv-cad-spec"></canvas>
          </div>
          <div class="an-chart-box">
            <div class="an-chart-head">
              <span class="an-chart-title">比のスペクトル (拾った線 = ×1.0 の妥当性確認)</span>
              <span class="an-chart-hint">×0.5 や ×2.0 が ×1.0 と同等なら帯域ズレの可能性</span>
            </div>
            <canvas id="an-cv-cad-ratio"></canvas>
          </div>
        </div>
      `}
      <div class="an-chart-box">
        <div class="an-chart-head">
          <span class="an-chart-title">高周波振動スペクトル (1–${Math.round(sess.fs * 0.5)} Hz, Welch PSD)</span>
          <label style="font-size:0.78rem;">軸:
            <select id="an-hf-ch">
              <option value="az" ${state.hfChannel === 'az' ? 'selected' : ''}>前後加速度 az (機首+Z)</option>
              <option value="ax" ${state.hfChannel === 'ax' ? 'selected' : ''}>上下加速度 ax (機体下+X)</option>
              <option value="ay" ${state.hfChannel === 'ay' ? 'selected' : ''}>左右加速度 ay (機体左+Y)</option>
              <option value="gx" ${state.hfChannel === 'gx' ? 'selected' : ''}>ジャイロ gx</option>
              <option value="gy" ${state.hfChannel === 'gy' ? 'selected' : ''}>ジャイロ gy</option>
              <option value="gz" ${state.hfChannel === 'gz' ? 'selected' : ''}>ジャイロ gz</option>
            </select>
          </label>
        </div>
        <canvas id="an-cv-hf-psd"></canvas>
      </div>
    `;

    // イベント結線
    document.getElementById('an-imu-sess').addEventListener('change', (e) => {
      const idx = parseInt(e.target.value, 10);
      if (!dec.sessions[idx]) return;
      // 選んだセッションで重なりを測り直す（重ならなければ単独モード。重複の分数もそのセッションのもの）
      state.imuMatch = IMU().matchSessionToFlight(dec, ctx.flightData, idx);
      state.cache = {};
      renderActiveTab();
    });
    document.getElementById('an-imu-clear').addEventListener('click', () => {
      state.imuDecoded = null; state.imuMatch = null; state.imuTransferWarning = null; state.cache = {};
      renderActiveTab();
    });
    document.getElementById('an-cad-preset').addEventListener('change', (e) => {
      state.cadencePreset = e.target.value;
      if (state.cadencePreset === 'hpa') { state.cadenceBandLow = 2.2; state.cadenceBandHigh = 4.0; }
      else if (state.cadencePreset === 'bike') { state.cadenceBandLow = 1.0; state.cadenceBandHigh = 2.6; }
      renderActiveTab();
    });
    document.getElementById('an-cad-lo').addEventListener('change', (e) => {
      state.cadencePreset = 'custom';
      state.cadenceBandLow = Math.max(0.5, parseFloat(e.target.value) || 2.2);
      renderActiveTab();
    });
    document.getElementById('an-cad-hi').addEventListener('change', (e) => {
      state.cadencePreset = 'custom';
      state.cadenceBandHigh = Math.max(state.cadenceBandLow + 0.3, parseFloat(e.target.value) || 4.0);
      renderActiveTab();
    });
    document.getElementById('an-cad-axis').addEventListener('change', (e) => { state.cadenceAxis = e.target.value; renderActiveTab(); });
    document.getElementById('an-cad-snr').addEventListener('change', (e) => { state.cadenceMinSnr = Math.max(2, parseFloat(e.target.value) || 8); renderActiveTab(); });
    document.getElementById('an-hf-ch').addEventListener('change', (e) => { state.hfChannel = e.target.value; renderActiveTab(); });

    if (!cad.error) {
      // 1. ケイデンス RPM 時系列
      {
        const cv = document.getElementById('an-cv-cad-rpm');
        const { g, w, h } = setupCanvas(cv, 210);
        const pad = { l: 54, r: 16, t: 14, b: 30 };
        const rpmMin = cad.bandLow * 30 - 5;
        const rpmMax = cad.bandHigh * 30 + 5;
        const { xToPx, yToPx, pw } = drawAxes(g, pad, w, h, tStart, tEnd, rpmMin, rpmMax, {
          yLabel: 'ケイデンス [RPM]', xFmt: v => hasCsv ? fmtSec(v, ctx) : `${(v / 60).toFixed(1)}m`, yFmt: v => v.toFixed(0)
        });
        g.strokeStyle = 'rgba(220, 38, 38, 0.35)'; g.lineWidth = 1.5; g.beginPath();
        let started = false;
        for (const p of cad.peaks) {
          if (!p.detected) { started = false; continue; }
          const px = xToPx(p.t), py = yToPx(p.rpm);
          if (!started) { g.moveTo(px, py); started = true; } else g.lineTo(px, py);
        }
        g.stroke();
        g.fillStyle = '#dc2626';
        for (const p of cad.peaks) {
          if (!p.detected) continue;
          g.beginPath(); g.arc(xToPx(p.t), yToPx(p.rpm), 3.2, 0, Math.PI * 2); g.fill();
        }
        if (hasCsv) drawCurrentCursor(g, pad, h, xToPx, tStart, tEnd, ctx.currentTimeSec);
        attachInteractive(cv, {
          onHover: (mx) => {
            if (mx < pad.l || mx > pad.l + pw || cad.peaks.length === 0) return null;
            const tH = tStart + ((mx - pad.l) / pw) * (tEnd - tStart);
            let best = cad.peaks[0];
            for (const p of cad.peaks) if (Math.abs(p.t - tH) < Math.abs(best.t - tH)) best = p;
            return `時刻: ${hasCsv ? fmtSec(best.t, ctx) : best.t.toFixed(1) + 's'}\nケイデンス: ${best.detected ? best.rpm.toFixed(1) + ' RPM' : '未検出'} (${best.fPeak.toFixed(2)} Hz)\nSNR: ${best.snrDb.toFixed(1)} dB${hasCsv ? '\nクリックでシーク' : ''}`;
          },
          onClick: (mx) => {
            if (!hasCsv || mx < pad.l || mx > pad.l + pw) return;
            seekTo(tStart + ((mx - pad.l) / pw) * (tEnd - tStart));
          }
        });
      }

      // 2. 白色化スペクトログラム
      {
        const cv = document.getElementById('an-cv-cad-spec');
        const { g, w, h } = setupCanvas(cv, 230);
        const pad = { l: 46, r: 12, t: 12, b: 30 };
        const pw = w - pad.l - pad.r, ph = h - pad.t - pad.b;
        const fMin = 0.5, fMax = 8.0;
        const colW = pw / Math.max(1, cad.cols.length);
        for (let ci = 0; ci < cad.cols.length; ci++) {
          const x = pad.l + ci * colW;
          const col = cad.cols[ci];
          for (let k = 1; k < cad.freqs.length; k++) {
            const f = cad.freqs[k];
            if (f < fMin || f > fMax) continue;
            const fPrev = cad.freqs[k - 1];
            const yTop = pad.t + ph - ((f - fMin) / (fMax - fMin)) * ph;
            const yBot = pad.t + ph - ((fPrev - fMin) / (fMax - fMin)) * ph;
            const norm = (col[k] + 2) / 16; // -2..+14 dB
            g.fillStyle = viridisColor(norm);
            g.fillRect(x, yTop, Math.ceil(colW) + 0.5, Math.max(1.5, yBot - yTop + 0.5));
          }
          if (cad.peaks[ci].detected) {
            const py = pad.t + ph - ((cad.peaks[ci].fPeak - fMin) / (fMax - fMin)) * ph;
            g.fillStyle = '#ffffff';
            g.fillRect(x + colW * 0.25, py - 1.5, Math.max(2, colW * 0.5), 3);
          }
        }
        // 探索帯域ライン
        g.strokeStyle = 'rgba(255,255,255,0.65)'; g.setLineDash([4, 3]); g.lineWidth = 1;
        for (const fb of [cad.bandLow, cad.bandHigh]) {
          const py = pad.t + ph - ((fb - fMin) / (fMax - fMin)) * ph;
          g.beginPath(); g.moveTo(pad.l, py); g.lineTo(pad.l + pw, py); g.stroke();
        }
        g.setLineDash([]);
        g.strokeStyle = '#94a3b8'; g.strokeRect(pad.l, pad.t, pw, ph);
        g.fillStyle = '#475569'; g.font = '10.5px sans-serif';
        for (const fv of [1, 2, 3, 4, 6, 8]) {
          const py = pad.t + ph - ((fv - fMin) / (fMax - fMin)) * ph;
          g.textAlign = 'right'; g.textBaseline = 'middle';
          g.fillText(`${fv}Hz`, pad.l - 5, py);
        }
        attachInteractive(cv, {
          onHover: (mx) => {
            if (mx < pad.l || mx > pad.l + pw) return null;
            const tH = tStart + ((mx - pad.l) / pw) * (tEnd - tStart);
            return `時刻: ${hasCsv ? fmtSec(tH, ctx) : tH.toFixed(1) + 's'}${hasCsv ? '\nクリックでシーク' : ''}`;
          },
          onClick: (mx) => {
            if (!hasCsv || mx < pad.l || mx > pad.l + pw) return;
            seekTo(tStart + ((mx - pad.l) / pw) * (tEnd - tStart));
          }
        });
      }

      // 3. 比のスペクトル
      {
        const cv = document.getElementById('an-cv-cad-ratio');
        const { g, w, h } = setupCanvas(cv, 230);
        const pad = { l: 46, r: 14, t: 18, b: 32 };
        let yMax = 10;
        for (let i = 0; i < cad.ratioSpec.length; i++) if (cad.ratioSpec[i] > yMax) yMax = cad.ratioSpec[i];
        const { xToPx, yToPx, ph } = drawAxes(g, pad, w, h, 0.3, 2.4, -2, yMax * 1.15, {
          yLabel: '相対強度 [dB]', xLabel: '周波数比 (拾った線 = 1.0)', xFmt: v => `×${v.toFixed(1)}`, yFmt: v => v.toFixed(0)
        });
        for (const ln of cad.lines) {
          const px = xToPx(ln.ratio);
          g.strokeStyle = ln.color; g.lineWidth = 1; g.setLineDash([3, 3]);
          g.beginPath(); g.moveTo(px, pad.t); g.lineTo(px, pad.t + ph); g.stroke();
          g.setLineDash([]);
          g.fillStyle = ln.color; g.font = 'bold 9.5px sans-serif'; g.textAlign = 'center'; g.textBaseline = 'top';
          g.fillText(`×${ln.ratio}`, px, pad.t + 2);
        }
        g.strokeStyle = '#0f172a'; g.lineWidth = 2; g.beginPath();
        for (let i = 0; i < cad.ratioAxis.length; i++) {
          const px = xToPx(cad.ratioAxis[i]), py = yToPx(Math.max(-2, cad.ratioSpec[i]));
          if (i === 0) g.moveTo(px, py); else g.lineTo(px, py);
        }
        g.stroke();
      }
    }

    // 4. 高周波振動 PSD (1..Nyquist Hz, 対数縦軸)
    if (!hf.error) {
      const cv = document.getElementById('an-cv-hf-psd');
      const { g, w, h } = setupCanvas(cv, 210);
      const pad = { l: 54, r: 18, t: 14, b: 32 };
      const fMax = Math.min(125, hf.fs * 0.5);
      const dbs = [];
      let minDb = Infinity, maxDb = -Infinity;
      for (let k = 1; k < hf.psd.freqs.length; k++) {
        const f = hf.psd.freqs[k];
        if (f < 1.0 || f > fMax) continue;
        const db = 10 * Math.log10(hf.psd.psd[k] + 1e-10);
        dbs.push({ f, db });
        if (db < minDb) minDb = db;
        if (db > maxDb) maxDb = db;
      }
      if (dbs.length > 2) {
        const yLo = Math.max(minDb - 3, maxDb - 55);
        const yHi = maxDb + 4;
        const { xToPx, yToPx } = drawAxes(g, pad, w, h, 1, fMax, yLo, yHi, {
          yLabel: `PSD [dB (${hf.unit})²/Hz]`, xLabel: '周波数 [Hz]', xFmt: v => v.toFixed(0), yFmt: v => v.toFixed(0)
        });
        g.strokeStyle = '#0284c7'; g.lineWidth = 1.4; g.beginPath();
        dbs.forEach((pt, i) => {
          const px = xToPx(pt.f), py = yToPx(Math.max(yLo, pt.db));
          if (i === 0) g.moveTo(px, py); else g.lineTo(px, py);
        });
        g.stroke();
      }
    }
  }

  global.openAnalysisModal = openAnalysisModal;
  global.closeAnalysisModal = closeAnalysisModal;
  global.isAnalysisModalOpen = () => state.open;
  global.resetAnalysisCache = resetAnalysisCache;
  global.loadImuBinBytes = loadImuBinBytes;
  global.onPonsFileListUpdated = function () {
    if (state.open) renderActiveTab();
  };
})(window);

