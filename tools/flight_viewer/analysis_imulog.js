// ============================================================
// File    : tools/flight_viewer/analysis_imulog.js
// Project : PONS v7 / v7.1 Flight Viewer
// Role    : 生 IMU バイナリログ (imuraw/YYYYMMDD.bin) のデコーダ・
//           時刻同期・250Hz 直接鉛直G・ケイデンス推定 (cadence.py 準拠)・
//           高周波振動解析エンジン
// ============================================================

(function (global) {
  'use strict';

  const DSP = global.PonsAnalysisDSP;
  const G0 = 9.80665;
  const REC_SIZE = 28;
  const WRAP_U32 = 4294967296; // 2^32
  const GPS_LEAP_SEC = 18;
  const JST_OFFSET_SEC = 9 * 3600;

  const ID_BOOT = 0x00;
  const ID_GYRO = 0x01;
  const ID_ACCEL = 0x02;
  const ID_GNSSVEL = 0x10;
  const ID_BARO = 0x20;

  // ------------------------------------------------------------
  // 28 バイト固定長バイナリのデコード & セッション分割
  // REC_FMT = "<IIBBH4f" (t_us, s_us, id, acc, seq, v0, v1, v2, v3)
  // ------------------------------------------------------------
  // 生ログに出うる id（tools/imulog/decode_imulog.py の KINDS と同じ。欠番も過去ログのために残す）
  const KNOWN_IDS = new Uint8Array(256);
  [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x10, 0x20].forEach(id => { KNOWN_IDS[id] = 1; });
  const RESYNC_RUN = 16;        // 区切りを探し直すとき、この数だけ続けて正しく読めた位置を新しい区切りとする
  const RESYNC_SPAN = 4096;     // 探す範囲 [レコード]

  // 形として正しく読めるレコードか（decode_imulog.py の _rec_valid と同じ判定）
  function recValid(dv, off) {
    const id = dv.getUint8(off + 8);
    if (!KNOWN_IDS[id]) return false;
    if (id === ID_BOOT) {
      // BOOT は s_us に BUILDDATE（20xxxxxx）が入る。ゼロ埋め（v0=0）はここでは通し、あとで捨てる
      const s = dv.getUint32(off + 4, true);
      return (s >= 20200101 && s <= 20991231) || dv.getFloat32(off + 12, true) === 0;
    }
    for (let k = 0; k < 3; k++) {
      const v = dv.getFloat32(off + 12 + 4 * k, true);
      if (!Number.isFinite(v) || Math.abs(v) >= 1e5) return false;
    }
    return true;
  }

  // レコードの先頭位置の一覧を作る。途中で区切りがずれていたら探し直す（decode_imulog.py の _read_records と同じ）。
  // ★ ファイルの途中でバイトが抜けたり化けたりすると（USB 転送の欠落・電源断で書きかけのレコードへの追記）、
  //   そこから後ろが全部 1〜27 バイトずれる。28 バイトずつ読み続けると id も値もでたらめになり、
  //   2026-11-19 の usb_bad.bin では 48 分のセッションが 12 分で黙って切れていた。
  function readRecordOffsets(dv, len) {
    const offs = new Uint32Array(Math.floor(len / REC_SIZE));
    let n = 0, pos = 0, skipped = 0, nResync = 0, junk = 0;
    while (pos + REC_SIZE <= len) {
      if (recValid(dv, pos)) {
        // ゼロ埋め（id=0 かつ v0=0）は書きかけ・未書き込み領域。t_us=0 が後戻りに見えて偽のセッションを作るので捨てる
        if (dv.getUint8(pos + 8) === ID_BOOT && dv.getFloat32(pos + 12, true) === 0) junk++;
        else offs[n++] = pos;
        pos += REC_SIZE;
        continue;
      }
      // pos のレコードが壊れている。pos+1 バイトから先で、続けて正しく読める位置を探す
      let found = -1;
      const qMax = Math.min(len - REC_SIZE * RESYNC_RUN, pos + REC_SIZE * RESYNC_SPAN);
      for (let q = pos + 1; q <= qMax; q++) {
        let ok = true;
        for (let r = 0; r < RESYNC_RUN; r++) {
          if (!recValid(dv, q + r * REC_SIZE)) { ok = false; break; }
        }
        if (ok) { found = q; break; }
      }
      if (found < 0) { skipped += len - pos; break; }
      skipped += found - pos;
      nResync++;
      pos = found;
    }
    return { offs: offs.subarray(0, n), skipped, nResync, junk };
  }

  function decodeImuBin(bytesOrBuf, filename) {
    const u8 = (bytesOrBuf instanceof Uint8Array) ? bytesOrBuf : new Uint8Array(bytesOrBuf);
    if (u8.byteLength < REC_SIZE * 10) {
      return { error: '有効な生 IMU レコードが見つかりませんでした（ファイルが短すぎます）。' };
    }
    const dv = new DataView(u8.buffer, u8.byteOffset, u8.byteLength);
    const rd = readRecordOffsets(dv, u8.byteLength);
    const recOff = rd.offs;
    const totalRecs = recOff.length;
    if (totalRecs < 10) {
      return { error: '有効な生 IMU レコードが見つかりませんでした（形の正しいレコードがほとんどありません）。' };
    }
    const warnings = [];
    if (rd.nResync > 0 || rd.skipped > 0) {
      warnings.push(`レコードの区切りのずれを ${rd.nResync} か所で探し直した（読み飛ばし ${rd.skipped.toLocaleString()} バイト）`);
    }
    if (rd.junk > 0) warnings.push(`ゼロ埋めレコード ${rd.junk.toLocaleString()} 件を捨てた（書きかけ・未書き込み領域）`);

    // 1 パス目: セッション境界を検出
    const rawSessions = [];
    let curStart = 0;
    let prevT = -1;
    let curBootId = null;

    for (let i = 0; i < totalRecs; i++) {
      const off = recOff[i];
      const tUs = dv.getUint32(off, true);
      const id = dv.getUint8(off + 8);
      let isNew = false;

      if (prevT >= 0 && tUs < prevT) {
        const isWrap = (prevT > 0.9 * WRAP_U32 && tUs < 0.1 * WRAP_U32);
        if (!isWrap) isNew = true;
      }
      if (id === ID_BOOT) {
        const bid = Math.round(dv.getFloat32(off + 12, true));
        if (bid !== 0 && bid !== curBootId) {
          if (curBootId !== null || i > 0) isNew = true;
          curBootId = bid;
        }
      }
      if (isNew && i > curStart) {
        rawSessions.push([curStart, i]);
        curStart = i;
      }
      prevT = tUs;
    }
    if (totalRecs > curStart) rawSessions.push([curStart, totalRecs]);

    // 2 パス目: 各セッションの accel / gyro / gnssvel を抽出
    const sessions = [];
    for (let si = 0; si < rawSessions.length; si++) {
      const [a, b] = rawSessions[si];
      let nAcc = 0, nGyro = 0, nGnss = 0;
      for (let i = a; i < b; i++) {
        const id = dv.getUint8(recOff[i] + 8);
        if (id === ID_ACCEL) nAcc++;
        else if (id === ID_GYRO) nGyro++;
        else if (id === ID_GNSSVEL) nGnss++;
      }
      if (nAcc < 20) continue;

      const accT = new Float64Array(nAcc);
      const ax = new Float32Array(nAcc);
      const ay = new Float32Array(nAcc);
      const az = new Float32Array(nAcc);

      const gyroT = new Float64Array(nGyro);
      const gx = new Float32Array(nGyro);
      const gy = new Float32Array(nGyro);
      const gz = new Float32Array(nGyro);

      const gnssT = new Float64Array(nGnss);
      const gnssJst = new Float64Array(nGnss);
      const gnssGs = new Float32Array(nGnss);

      let ia = 0, ig = 0, inav = 0;
      let tWrap = 0, prevTU = -1;
      let sAccWrap = 0, prevSA = -1;
      let sGyrWrap = 0, prevSG = -1;
      let useSampleTimeAcc = true;

      // s_us がホスト時刻に近い（v7.1 SCH16T）か判定
      let sValidCnt = 0, sCheckCnt = 0;
      for (let i = a; i < Math.min(b, a + 400); i++) {
        const off = recOff[i];
        if (dv.getUint8(off + 8) === ID_ACCEL) {
          const tu = dv.getUint32(off, true);
          const su = dv.getUint32(off + 4, true);
          const diff = Math.abs(tu - su);
          if (diff < 2000000) sValidCnt++;
          sCheckCnt++;
        }
      }
      useSampleTimeAcc = (sCheckCnt > 0 && sValidCnt / sCheckCnt > 0.7);

      for (let i = a; i < b; i++) {
        const off = recOff[i];
        const tUs = dv.getUint32(off, true);
        const sUs = dv.getUint32(off + 4, true);
        const id = dv.getUint8(off + 8);

        if (prevTU >= 0 && tUs < prevTU - WRAP_U32 * 0.5) tWrap += WRAP_U32;
        prevTU = tUs;
        const tSec = (tUs + tWrap) * 1e-6;

        if (id === ID_ACCEL) {
          let ts = tSec;
          if (useSampleTimeAcc) {
            if (prevSA >= 0 && sUs < prevSA - WRAP_U32 * 0.5) sAccWrap += WRAP_U32;
            prevSA = sUs;
            ts = (sUs + sAccWrap) * 1e-6;
          }
          accT[ia] = ts;
          ax[ia] = dv.getFloat32(off + 12, true);
          ay[ia] = dv.getFloat32(off + 16, true);
          az[ia] = dv.getFloat32(off + 20, true);
          ia++;
        } else if (id === ID_GYRO) {
          let ts = tSec;
          if (useSampleTimeAcc) {
            if (prevSG >= 0 && sUs < prevSG - WRAP_U32 * 0.5) sGyrWrap += WRAP_U32;
            prevSG = sUs;
            ts = (sUs + sGyrWrap) * 1e-6;
          }
          gyroT[ig] = ts;
          gx[ig] = dv.getFloat32(off + 12, true);
          gy[ig] = dv.getFloat32(off + 16, true);
          gz[ig] = dv.getFloat32(off + 20, true);
          ig++;
        } else if (id === ID_GNSSVEL) {
          const iTOWms = sUs;
          const vN = dv.getFloat32(off + 12, true);
          const vE = dv.getFloat32(off + 16, true);
          const utcWeekSec = iTOWms * 1e-3 - GPS_LEAP_SEC;
          let jstDaySec = (utcWeekSec + JST_OFFSET_SEC) % 86400;
          if (jstDaySec < 0) jstDaySec += 86400;
          gnssT[inav] = tSec;
          gnssJst[inav] = jstDaySec;
          gnssGs[inav] = Math.hypot(vN, vE);
          inav++;
        }
      }

      // 実効サンプリング周波数
      const dts = [];
      for (let i = 1; i < Math.min(nAcc, 2000); i++) {
        const dt = accT[i] - accT[i - 1];
        if (dt > 0.0005 && dt < 0.2) dts.push(dt);
      }
      const medDt = DSP.median(dts) || 0.004;
      const fs = 1.0 / medDt;
      const durSec = accT[nAcc - 1] - accT[0];

      // GNSS iTOW から ホスト時刻 → JST 秒オフセットを中央値で推定（日跨ぎも考慮）
      let jstOffset = null;
      let jstStart = null, jstEnd = null;
      if (inav >= 3) {
        const diffs = [];
        const baseJst = gnssJst[0];
        for (let i = 0; i < inav; i++) {
          let j = gnssJst[i];
          if (j - baseJst < -43200) j += 86400;
          diffs.push(j - gnssT[i]);
        }
        jstOffset = DSP.median(diffs);
        jstStart = accT[0] + jstOffset;
        jstEnd = accT[nAcc - 1] + jstOffset;
      }

      sessions.push({
        index: sessions.length,
        nAcc, nGyro, nGnss, fs, durSec,
        t0: accT[0], t1: accT[nAcc - 1],
        accT, ax, ay, az,
        gyroT, gx, gy, gz,
        gnssT, gnssJst, gnssGs,
        jstOffset, jstStart, jstEnd
      });
    }

    if (sessions.length === 0) {
      return { error: '加速度データを含む有効なセッションがありませんでした。' };
    }
    return {
      filename: filename || 'imulog.bin', byteLength: u8.byteLength, sessions,
      warnings, resyncCount: rd.nResync, skippedBytes: rd.skipped, junkRecords: rd.junk
    };
  }

  // ------------------------------------------------------------
  // 飛行 CSV との時刻同期 (JST 秒経由でセッション自動選択)
  // ------------------------------------------------------------
  function parseJstSec(timeStr) {
    const m = String(timeStr || '').match(/(\d+):(\d+):([\d.]+)/);
    return m ? (parseInt(m[1], 10) * 3600 + parseInt(m[2], 10) * 60 + parseFloat(m[3])) : NaN;
  }

  // forceIdx を渡すと、そのセッションに決めて重なりを測る（解析画面でセッションを選び直したとき）。
  // ★ 以前は選び直しでは重なりを見ずに「同期済み」とし、重複の分数も前のセッションのままだった。
  function matchSessionToFlight(decoded, flightData, forceIdx) {
    if (!decoded || !decoded.sessions || decoded.sessions.length === 0) return null;
    let bestIdx = 0;
    let bestOverlap = -1;
    let csvJst0 = NaN, csvJst1 = NaN;

    if (flightData && flightData.length >= 2) {
      csvJst0 = parseJstSec(flightData[0].timeStr);
      csvJst1 = csvJst0 + flightData[flightData.length - 1].t;
    }
    const overlapOf = (s) => {
      if (!Number.isFinite(csvJst0) || s.jstStart === null) return -1;
      return Math.min(csvJst1, s.jstEnd) - Math.max(csvJst0, s.jstStart);
    };

    if (Number.isInteger(forceIdx) && forceIdx >= 0 && forceIdx < decoded.sessions.length) {
      bestIdx = forceIdx;
      bestOverlap = overlapOf(decoded.sessions[forceIdx]);
    } else {
      for (let i = 0; i < decoded.sessions.length; i++) {
        const ov = overlapOf(decoded.sessions[i]);
        if (ov > bestOverlap) { bestOverlap = ov; bestIdx = i; }
      }
      // 重複が見つからなければ最長セッションを選ぶ
      if (bestOverlap <= 0) {
        let maxDur = -1;
        for (let i = 0; i < decoded.sessions.length; i++) {
          if (decoded.sessions[i].durSec > maxDur) { maxDur = decoded.sessions[i].durSec; bestIdx = i; }
        }
      }
    }

    const sess = decoded.sessions[bestIdx];
    const synced = (bestOverlap > 5 && Number.isFinite(csvJst0) && sess.jstOffset !== null);
    const toFlightSec = (hostT) => {
      if (synced) return (hostT + sess.jstOffset) - csvJst0;
      return hostT - sess.t0;
    };

    return {
      sessionIndex: bestIdx, session: sess, synced, overlapSec: Math.max(0, bestOverlap), toFlightSec,
      hasCsvTime: Number.isFinite(csvJst0), forced: Number.isInteger(forceIdx)
    };
  }

  // ------------------------------------------------------------
  // 250Hz 直接計測 鉛直G (荷重倍数)
  // ------------------------------------------------------------
  function computeImuDirectG(session, toFlightSec, tStart, tEnd, smoothSec) {
    const sSec = Math.max(0.02, smoothSec || 0.25);
    const n = session.nAcc;
    const tList = [];
    const nNormList = [];
    const nZList = [];

    for (let i = 0; i < n; i++) {
      const ft = toFlightSec(session.accT[i]);
      if (ft < tStart || ft > tEnd) continue;
      const ax = session.ax[i], ay = session.ay[i], az = session.az[i];
      tList.push(ft);
      // 比力ノルム / g（姿勢に依らず主翼荷重倍数にほぼ一致）
      nNormList.push(Math.hypot(ax, ay, az) / G0);
      // 機体上向き（センサー -X）の比力 / g
      nZList.push(-ax / G0);
    }
    if (tList.length < 20) {
      return { error: '対象時間範囲に生 IMU 加速度データがありません。' };
    }

    const winSamples = Math.max(1, Math.round(sSec * session.fs));
    const normArr = Float64Array.from(nNormList);
    const zArr = Float64Array.from(nZList);
    const normSm = DSP.movingAverage(normArr, winSamples);
    const zSm = DSP.movingAverage(zArr, winSamples);

    let maxPosIdx = 0, minIdx = 0, maxPosSmIdx = 0, minSmIdx = 0;
    for (let i = 0; i < normArr.length; i++) {
      if (normArr[i] > normArr[maxPosIdx]) maxPosIdx = i;
      if (zArr[i] < zArr[minIdx]) minIdx = i;
      if (normSm[i] > normSm[maxPosSmIdx]) maxPosSmIdx = i;
      if (zSm[i] < zSm[minSmIdx]) minSmIdx = i;
    }

    // 描画用に最大 2500 点へダウンサンプル（ピークを落とさないようブロック最大・平均を保持）
    const maxPts = 2500;
    const step = Math.max(1, Math.floor(tList.length / maxPts));
    const plotT = [], plotNorm = [], plotSm = [];
    for (let i = 0; i < tList.length; i += step) {
      const end = Math.min(tList.length, i + step);
      let hi = normArr[i], smSum = 0;
      for (let j = i; j < end; j++) {
        if (Math.abs(normArr[j] - 1) > Math.abs(hi - 1)) hi = normArr[j];
        smSum += normSm[j];
      }
      plotT.push(tList[i]);
      plotNorm.push(hi);
      plotSm.push(smSum / (end - i));
    }

    return {
      fs: session.fs,
      smoothSec: sSec,
      count: tList.length,
      plotT: Float64Array.from(plotT),
      plotNorm: Float64Array.from(plotNorm),
      plotSm: Float64Array.from(plotSm),
      maxPosRaw: { t: tList[maxPosIdx], n: normArr[maxPosIdx] },
      maxPosSmooth: { t: tList[maxPosSmIdx], n: normSm[maxPosSmIdx] },
      maxNegRaw: { t: tList[minIdx], n: zArr[minIdx] },
      maxNegSmooth: { t: tList[minSmIdx], n: zSm[minSmIdx] }
    };
  }

  // ------------------------------------------------------------
  // ケイデンス推定 (tools/imulog/cadence.py --att lp 準拠)
  // ------------------------------------------------------------
  const CADENCE_LINES = [
    { ratio: 0.5,  label: '×0.5 クランク1回転', color: '#7c3aed' },
    { ratio: 0.75, label: '×0.75 プロペラ1回転', color: '#0284c7' },
    { ratio: 1.0,  label: '×1.0 踏み込み', color: '#dc2626' },
    { ratio: 1.5,  label: '×1.5 羽根通過(2枚)', color: '#ea580c' },
    { ratio: 2.0,  label: '×2.0 踏み込み2倍', color: '#16a34a' }
  ];

  function computeCadence(session, toFlightSec, tStart, tEnd, opts) {
    const bandLow = (opts && opts.bandLow) || 2.2;
    const bandHigh = (opts && opts.bandHigh) || 4.0;
    const minSnrDb = (opts && opts.minSnrDb !== undefined) ? opts.minSnrDb : 8.0;
    const axis = (opts && opts.axis) || 'fwd'; // 'fwd' | 'up'
    const winSec = 8.0;
    const stepSec = 2.0;
    const smoothSec = 4.0;
    const targetFs = Math.min(50.0, Math.max(20.0, Math.round(session.fs / Math.max(1, Math.round(session.fs / 50.0)))));

    // 対象区間を抽出して targetFs へ等間隔リサンプリング
    const srcT = [], srcAx = [], srcAy = [], srcAz = [];
    for (let i = 0; i < session.nAcc; i++) {
      const ft = toFlightSec(session.accT[i]);
      if (ft < tStart - 1 || ft > tEnd + 1) continue;
      srcT.push(ft);
      srcAx.push(session.ax[i]);
      srcAy.push(session.ay[i]);
      srcAz.push(session.az[i]);
    }
    if (srcT.length < 100) {
      return { error: 'ケイデンス解析に必要な生 IMU データが不足しています。' };
    }

    const t0 = Math.max(tStart, srcT[0]);
    const t1 = Math.min(tEnd, srcT[srcT.length - 1]);
    if (t1 - t0 < winSec + 2) {
      return { error: `ケイデンス解析には最低 ${winSec + 2} 秒以上のデータが必要です。` };
    }

    const n = Math.floor((t1 - t0) * targetFs) + 1;
    const tGrid = new Float64Array(n);
    const axG = new Float64Array(n);
    const ayG = new Float64Array(n);
    const azG = new Float64Array(n);
    let idx = 0;
    for (let k = 0; k < n; k++) {
      const tk = t0 + k / targetFs;
      tGrid[k] = tk;
      while (idx < srcT.length - 2 && srcT[idx + 1] < tk) idx++;
      const tA = srcT[idx], tB = srcT[idx + 1];
      const f = (tB > tA) ? Math.max(0, Math.min(1, (tk - tA) / (tB - tA))) : 0;
      axG[k] = srcAx[idx] + f * (srcAx[idx + 1] - srcAx[idx]);
      ayG[k] = srcAy[idx] + f * (srcAy[idx + 1] - srcAy[idx]);
      azG[k] = srcAz[idx] + f * (srcAz[idx + 1] - srcAz[idx]);
    }

    // 低域比力で「上」ベクトル u を推定し、重力を差し引く (cadence.py --att lp)
    const lpWin = Math.max(5, Math.round(targetFs * 2.0));
    const uxLp = DSP.movingAverage(axG, lpWin);
    const uyLp = DSP.movingAverage(ayG, lpWin);
    const uzLp = DSP.movingAverage(azG, lpWin);
    const sig = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      const norm = Math.hypot(uxLp[i], uyLp[i], uzLp[i]) || G0;
      const ux = uxLp[i] / norm, uy = uyLp[i] / norm, uz = uzLp[i] / norm;
      if (axis === 'up') {
        sig[i] = (axG[i] * ux + ayG[i] * uy + azG[i] * uz) - G0;
      } else {
        // 機首方向 e = (0, 0, 1) (センサー +Z)
        sig[i] = azG[i] - G0 * uz;
      }
    }

    // STFT (winSec=8s, stepSec=2s, Hann + 線形デトレンド)
    const nwin = Math.round(winSec * targetFs);
    const nstep = Math.round(stepSec * targetFs);
    let nfft = 1;
    while (nfft < nwin) nfft <<= 1;
    const half = (nfft >> 1) + 1;
    const df = targetFs / nfft;

    const win = new Float64Array(nwin);
    let winPow = 0;
    for (let i = 0; i < nwin; i++) {
      const w = 0.5 * (1 - Math.cos((2 * Math.PI * i) / (nwin - 1)));
      win[i] = w;
      winPow += w * w;
    }

    const times = [];
    const whiteCols = [];
    const maxFreqPlot = 10.0;
    const maxK = Math.min(half - 1, Math.ceil(maxFreqPlot / df));
    const medHalfBins = Math.max(2, Math.round(0.6 / df)); // ±0.6Hz で白色化

    // Radix-2 FFT を DSP 相当でローカル計算（welch と同等の高速 FFT）
    const freqs = new Float64Array(maxK + 1);
    for (let k = 0; k <= maxK; k++) freqs[k] = k * df;

    for (let start = 0; start + nwin <= n; start += nstep) {
      const seg = DSP.detrendLinear(sig.subarray(start, start + nwin));
      // DSP の welchPSD に単一窓で渡す代わりに直接 FFT
      const psdObj = DSP.welchPSD(seg, targetFs, winSec, 0);
      if (!psdObj) continue;
      const db = new Float64Array(maxK + 1);
      for (let k = 0; k <= maxK; k++) {
        db[k] = 10.0 * Math.log10((psdObj.psd[k] || 1e-12) + 1e-12);
      }
      // 周波数方向の移動中央値を引いて白色化
      const wCol = new Float64Array(maxK + 1);
      const buf = [];
      for (let k = 0; k <= maxK; k++) {
        buf.length = 0;
        const lo = Math.max(1, k - medHalfBins);
        const hi = Math.min(maxK, k + medHalfBins);
        for (let j = lo; j <= hi; j++) buf.push(db[j]);
        buf.sort((x, y) => x - y);
        const med = buf[buf.length >> 1] || 0;
        wCol[k] = db[k] - med;
      }
      times.push(tGrid[Math.min(n - 1, start + (nwin >> 1))]);
      whiteCols.push(wCol);
    }

    if (times.length < 2) {
      return { error: 'ケイデンスのスペクトル窓を作成できませんでした。' };
    }

    // 時間方向平滑 (±smoothSec/2)
    const smHalf = Math.max(0, Math.round((smoothSec * 0.5) / stepSec));
    const smoothCols = whiteCols.map((_, ci) => {
      const out = new Float64Array(maxK + 1);
      const a = Math.max(0, ci - smHalf);
      const b = Math.min(whiteCols.length - 1, ci + smHalf);
      const cnt = b - a + 1;
      for (let k = 0; k <= maxK; k++) {
        let s = 0;
        for (let j = a; j <= b; j++) s += whiteCols[j][k];
        out[k] = s / cnt;
      }
      return out;
    });

    // 帯域内ピーク検出 (放物線補間)
    const kLo = Math.max(1, Math.ceil(bandLow / df));
    const kHi = Math.min(maxK - 1, Math.floor(bandHigh / df));
    const peaks = [];
    const validRpms = [];

    for (let ci = 0; ci < times.length; ci++) {
      const col = smoothCols[ci];
      let bestK = kLo, bestVal = -Infinity;
      for (let k = kLo; k <= kHi; k++) {
        if (col[k] > bestVal) { bestVal = col[k]; bestK = k; }
      }
      // 放物線補間
      let fPeak = bestK * df;
      if (bestK > kLo && bestK < kHi) {
        const y0 = col[bestK - 1], y1 = col[bestK], y2 = col[bestK + 1];
        const denom = (y0 - 2 * y1 + y2);
        if (Math.abs(denom) > 1e-9) {
          const delta = Math.max(-0.5, Math.min(0.5, 0.5 * (y0 - y2) / denom));
          fPeak = (bestK + delta) * df;
        }
      }
      const detected = (bestVal >= minSnrDb && bestK > kLo && bestK < kHi);
      const rpm = fPeak * 30.0; // 1回転2回踏み込み → RPM = f * 60 / 2
      peaks.push({ t: times[ci], fPeak, rpm, snrDb: bestVal, detected });
      if (detected) validRpms.push(rpm);
    }

    // 比のスペクトル (0.30 .. 2.40)
    const ratioBins = 211;
    const ratioAxis = new Float64Array(ratioBins);
    const ratioSpec = new Float64Array(ratioBins);
    for (let i = 0; i < ratioBins; i++) ratioAxis[i] = 0.30 + i * 0.01;
    let detCount = 0;
    for (let ci = 0; ci < peaks.length; ci++) {
      if (!peaks[ci].detected) continue;
      detCount++;
      const f0 = peaks[ci].fPeak;
      const col = smoothCols[ci];
      for (let i = 0; i < ratioBins; i++) {
        const fTarget = ratioAxis[i] * f0;
        const kf = fTarget / df;
        const k0 = Math.floor(kf);
        if (k0 >= 1 && k0 + 1 <= maxK) {
          const frac = kf - k0;
          ratioSpec[i] += col[k0] + frac * (col[k0 + 1] - col[k0]);
        }
      }
    }
    if (detCount > 0) {
      for (let i = 0; i < ratioBins; i++) ratioSpec[i] /= detCount;
    }

    const medianRpm = validRpms.length > 0 ? DSP.median(validRpms) : NaN;
    const detectRate = peaks.length > 0 ? (detCount / peaks.length) : 0;

    return {
      times, freqs, cols: smoothCols, peaks,
      bandLow, bandHigh, minSnrDb, axis, targetFs,
      medianRpm, detectRate, detCount, totalWins: peaks.length,
      ratioAxis, ratioSpec, lines: CADENCE_LINES
    };
  }

  // ------------------------------------------------------------
  // 高周波振動スペクトル (1〜125Hz, 3軸加速度 / 3軸ジャイロ)
  // ------------------------------------------------------------
  function computeHighFreqVib(session, toFlightSec, tStart, tEnd, channel) {
    const isGyro = (channel === 'gx' || channel === 'gy' || channel === 'gz');
    const tSrc = isGyro ? session.gyroT : session.accT;
    const vSrc = channel === 'ax' ? session.ax :
                 channel === 'ay' ? session.ay :
                 channel === 'az' ? session.az :
                 channel === 'gx' ? session.gx :
                 channel === 'gy' ? session.gy : session.gz;
    const unit = isGyro ? 'rad/s' : 'm/s²';
    if (!tSrc || tSrc.length < 64) return { error: '該当チャンネルのサンプルが不足しています。' };

    const vals = [];
    for (let i = 0; i < tSrc.length; i++) {
      const ft = toFlightSec(tSrc[i]);
      if (ft >= tStart && ft <= tEnd) vals.push(vSrc[i]);
    }
    if (vals.length < 128) return { error: '対象区間の高周波データが不足しています。' };

    const fs = session.fs;
    const segSec = Math.min(4.0, Math.max(1.0, (vals.length / fs) * 0.25));
    const psd = DSP.welchPSD(Float64Array.from(vals), fs, segSec, 0.5);
    if (!psd) return { error: 'PSD の算出に失敗しました。' };

    // 最大ピーク周波数 (> 1Hz)
    let peakF = NaN, peakVal = -1;
    for (let k = 1; k < psd.freqs.length; k++) {
      if (psd.freqs[k] >= 1.0 && psd.psd[k] > peakVal) {
        peakVal = psd.psd[k];
        peakF = psd.freqs[k];
      }
    }
    return { channel, unit, fs, psd, peakF, peakVal, count: vals.length };
  }

  // ファイル名から YYYYMMDD を抽出 (例: 2026-11-20_0804.csv -> 20261120)
  function extractDateStamp(filename) {
    const s = String(filename || '');
    const m1 = s.match(/(20\d{2})[-_]?(\d{2})[-_]?(\d{2})/);
    return m1 ? `${m1[1]}${m1[2]}${m1[3]}` : null;
  }

  global.PonsImuAnalysis = {
    VERSION: '2026.11.20.15',
    decodeImuBin,
    matchSessionToFlight,
    computeImuDirectG,
    computeCadence,
    computeHighFreqVib,
    extractDateStamp,
    CADENCE_LINES
  };
})(window);
