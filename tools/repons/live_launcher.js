// ============================================================
// File    : tools/repons/live_launcher.js
// Project : PONS v7.1 / v7.2 Re:PONS
// Role    : Re:PONS（index.html）の「📡 ライブ」。ライブ配信の窓（live.html）を別窓で開く（docs/pons_live_stream_plan.md）。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026-11-24
// ============================================================
//
// ★ ライブの窓と本体の窓のファイルの接続は、同じ USB のポートを同時に開けない（ブラウザが 1 つの窓にしか開かせない）ので、
//   取り合いの仕掛けは持たない。開けなかった側に「ほかの窓が使っている」と出る。
(function (global) {
  'use strict';
  let liveWin = null;
  function openLiveWindow() {
    if (liveWin && !liveWin.closed) { liveWin.focus(); return; }
    const v = (typeof VIEWER_VERSION !== 'undefined') ? VIEWER_VERSION : String(Date.now());
    // ★ まず名前だけで探す。URL を付けて開くと、本体の窓を読み直したあと（liveWin を忘れている）に押したとき、
    //   開いているライブの窓まで読み直してしまい、ポートが閉じて配信が切れる（つなぎ直しも手で押すまで止まる）。
    // 大きさは ×3 の画面＋帯＋パネル。配信のときはライブの窓の中で「配信用にする」を押して画面だけにする
    const w = global.open('', 'pons_live', 'popup,width=1120,height=1080');
    if (!w) { alert('別窓を開けませんでした。ブラウザのポップアップのブロックを外してから、もう一度押してください。'); return; }
    let blank;
    try { blank = (w.location.href === 'about:blank'); }
    catch (e) { blank = false; }   // 中を見られない（file:// で開いた live.html など）＝もう開いている
    if (blank) w.location.href = 'live.html?v=' + encodeURIComponent(v);
    try { w.focus(); } catch (e) {}
    liveWin = w;
  }
  global.openLiveWindow = openLiveWindow;
})(typeof window !== 'undefined' ? window : globalThis);
