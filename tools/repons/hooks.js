// ============================================================
// File    : tools/repons/hooks.js
// Project : PONS v7.1 / v7.2 Re:PONS
// Role    : ほかのファイル（analysis_ui.js・video_sync.js）へ渡す連携の口（window.getReponsAnalysisContext など）
// Author  : MasaoC (@masao_mobile)
// Updated : 2026-11-23
// ============================================================
//
// 2026-11-23 に index.html から分けた。読む順は index.html の一覧どおり。関数・変数はファイルをまたいで
// グローバルで見えるが、読み込みの最中に動くコードは先に読んだファイルのものしか使えない。

// 飛行解析モジュール（analysis_ui.js）向け連携フック
window.getReponsAnalysisContext = function () {
  return {
    flightData,
    takeoffSec,
    landSec,
    totalDurationSec,
    currentTimeSec,
    segments: movingSegments,   // 動いている区間（シークバーと同じ分け方。解析はこの 1 つずつを対象にする）
    speedThreshold: FLIGHT_SPEED_THRESHOLD,
    filename: currentCsvFilename || 'sample_flight.csv'
  };
};
// 動画の同期（video_sync.js）向け: 再生の状態と、CSV の 1 行目の時刻（JST の日内秒）
window.getReponsPlayback = function () {
  return {
    currentTimeSec, totalDurationSec, isPlaying, playbackSpeed,
    clockBaseSec: csvClockBaseSec,
    hasData: !!(flightData && flightData.length > 1),
    filename: currentCsvFilename
  };
};
// 動画側で再生・一時停止されたときにビューアを合わせる
window.setReponsPlaying = function (on) {
  if (!!on !== isPlaying && flightData && flightData.length > 1) togglePlay();
};
window.seekReponsTo = function (tSec, focusMap) {
  if (!flightData || flightData.length === 0) return;
  if (isPlaying) {
    isPlaying = false;
    const playBtn = document.getElementById('btn-play');
    if (playBtn) playBtn.textContent = '▶ 再生';
  }
  currentTimeSec = Math.max(0, Math.min(totalDurationSec, tSec));
  updateUI(currentTimeSec);
  if (focusMap && mapInitialized && map) {
    const d = interpolateAt(currentTimeSec);
    if (d && Number.isFinite(d.lat) && Number.isFinite(d.lon)) {
      map.invalidateSize(false);
      map.setView([d.lat, d.lon], Math.max(map.getZoom(), 14), { animate: false });
    }
  }
};
// Web Serial 経由での生 IMU ログ自動検出・取得フック
window.getPonsSerialImuMatch = function (csvFilename) {
  const connected = !!(serialWriter);
  const imuFiles = (ponsFileList || []).filter(f => fileTabOf(f.path) === 'imu');
  let matched = null;
  if (connected && imuFiles.length > 0 && window.PonsImuAnalysis) {
    const stamp = window.PonsImuAnalysis.extractDateStamp(csvFilename);
    if (stamp) {
      matched = imuFiles.find(f => f.path.includes(stamp)) || null;
    }
  }
  return { connected, listIsAll, listAllUnsupported, imuFiles, matched };
};
window.requestAllPonsFilesForImu = async function () {
  if (!serialWriter || listPending) return;
  await requestFileList(true);
};
window.fetchPonsImuForAnalysis = function (path, size, targetTab) {
  if (!serialWriter) { alert('PONS が USB 接続されていません。'); return; }
  window._pendingImuAnalysisTab = targetTab || 'imu';
  document.getElementById('serial-modal').classList.add('active');
  // 飛行 CSV が開かれていれば、そのフライト区間（前後 ±30 秒）だけを PONS:GETIMU でピンポイント取得する
  let imuRange = null;
  if (flightData && flightData.length >= 2 && flightData[0].timeStr) {
    const m = String(flightData[0].timeStr).match(/(\d+):(\d+):([\d.]+)/);
    if (m) {
      const baseJst = parseInt(m[1], 10) * 3600 + parseInt(m[2], 10) * 60 + parseFloat(m[3]);
      // 解析は動いている区間ごとなので、最初の区間の始まりから最後の区間の終わりまでを取る（停止中だけの前後は取らない）
      const segs = movingSegments || [];
      const relStart = segs.length > 0 ? segs[0].start : 0;
      const relEnd = segs.length > 0 ? segs[segs.length - 1].end : totalDurationSec;
      const jstStart = Math.max(0, Math.floor(baseJst + relStart - 30));
      const jstEnd = Math.min(86400, Math.ceil(baseJst + relEnd + 30));
      if (jstEnd > jstStart + 10) imuRange = { jstStart, jstEnd };
    }
  }
  downloadPonsFile(path, size || 0, 0, 'open', imuRange);
};
