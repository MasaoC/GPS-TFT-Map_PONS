// ============================================================
// File    : tools/repons/analysis_dsp.js
// Project : PONS v7 / v7.1 Re:PONS
// Role    : 飛行CSVログに対する信号処理・飛行力学解析エンジン（純JS）
//           tools/flightanalysis/*.py の定型分析をブラウザ内で完結させる。
// ============================================================

(function (global) {
  'use strict';

  const G0 = 9.80665;

  // ------------------------------------------------------------
  // 基本統計・配列ユーティリティ
  // ------------------------------------------------------------
  function mean(arr) {
    if (!arr || arr.length === 0) return NaN;
    let s = 0, c = 0;
    for (let i = 0; i < arr.length; i++) {
      const v = arr[i];
      if (Number.isFinite(v)) { s += v; c++; }
    }
    return c > 0 ? s / c : NaN;
  }

  function std(arr, ddof = 0) {
    const m = mean(arr);
    if (!Number.isFinite(m)) return NaN;
    let s = 0, c = 0;
    for (let i = 0; i < arr.length; i++) {
      const v = arr[i];
      if (Number.isFinite(v)) { const d = v - m; s += d * d; c++; }
    }
    const denom = c - ddof;
    return denom > 0 ? Math.sqrt(s / denom) : 0;
  }

  function rms(arr) {
    let s = 0, c = 0;
    for (let i = 0; i < arr.length; i++) {
      const v = arr[i];
      if (Number.isFinite(v)) { s += v * v; c++; }
    }
    return c > 0 ? Math.sqrt(s / c) : NaN;
  }

  function median(arr) {
    const clean = [];
    for (let i = 0; i < arr.length; i++) {
      if (Number.isFinite(arr[i])) clean.push(arr[i]);
    }
    if (clean.length === 0) return NaN;
    clean.sort((a, b) => a - b);
    const mid = clean.length >> 1;
    return (clean.length & 1) ? clean[mid] : 0.5 * (clean[mid - 1] + clean[mid]);
  }

  function pearsonR(xs, ys) {
    let n = 0, sx = 0, sy = 0;
    for (let i = 0; i < xs.length; i++) {
      if (Number.isFinite(xs[i]) && Number.isFinite(ys[i])) {
        sx += xs[i]; sy += ys[i]; n++;
      }
    }
    if (n < 3) return NaN;
    const mx = sx / n, my = sy / n;
    let sxx = 0, syy = 0, sxy = 0;
    for (let i = 0; i < xs.length; i++) {
      if (Number.isFinite(xs[i]) && Number.isFinite(ys[i])) {
        const dx = xs[i] - mx, dy = ys[i] - my;
        sxx += dx * dx; syy += dy * dy; sxy += dx * dy;
      }
    }
    return (sxx > 0 && syy > 0) ? (sxy / Math.sqrt(sxx * syy)) : NaN;
  }

  function detrendLinear(arr) {
    const n = arr.length;
    const out = new Float64Array(n);
    if (n < 2) {
      if (n === 1) out[0] = 0;
      return out;
    }
    const xm = 0.5 * (n - 1);
    let ym = 0;
    for (let i = 0; i < n; i++) ym += arr[i];
    ym /= n;
    let sxx = 0, sxy = 0;
    for (let i = 0; i < n; i++) {
      const dx = i - xm;
      sxx += dx * dx;
      sxy += dx * (arr[i] - ym);
    }
    const slope = sxx > 0 ? sxy / sxx : 0;
    const intercept = ym - slope * xm;
    for (let i = 0; i < n; i++) {
      out[i] = arr[i] - (slope * i + intercept);
    }
    return out;
  }

  function movingAverage(arr, win) {
    const n = arr.length;
    const out = new Float64Array(n);
    const w = Math.max(1, Math.round(win));
    if (w <= 1 || n === 0) {
      for (let i = 0; i < n; i++) out[i] = arr[i];
      return out;
    }
    const half = Math.floor(w / 2);
    const prefix = new Float64Array(n + 1);
    for (let i = 0; i < n; i++) prefix[i + 1] = prefix[i] + arr[i];
    for (let i = 0; i < n; i++) {
      const a = Math.max(0, i - half);
      const b = Math.min(n, i + half + 1);
      out[i] = (prefix[b] - prefix[a]) / (b - a);
    }
    return out;
  }

  function unwrapDeg(arr) {
    const n = arr.length;
    const out = new Float64Array(n);
    if (n === 0) return out;
    out[0] = arr[0];
    let offset = 0;
    for (let i = 1; i < n; i++) {
      let d = arr[i] - arr[i - 1];
      while (d > 180) { offset -= 360; d -= 360; }
      while (d < -180) { offset += 360; d += 360; }
      out[i] = arr[i] + offset;
    }
    return out;
  }

  function baroAltFromPressure(pHpa, p0Hpa) {
    if (!Number.isFinite(pHpa) || !Number.isFinite(p0Hpa) || pHpa <= 0 || p0Hpa <= 0) return NaN;
    return 44330.0 * (1.0 - Math.pow(pHpa / p0Hpa, 0.19029495));
  }

  // ------------------------------------------------------------
  // 等間隔リサンプリング
  // ------------------------------------------------------------
  function resampleUniform(records, getter, fs, tStart, tEnd, opts) {
    const isAngle = !!(opts && opts.unwrapDeg);
    const ts = [], ys = [];
    for (let i = 0; i < records.length; i++) {
      const r = records[i];
      if (r.t < tStart - 2 || r.t > tEnd + 2) continue;
      const v = getter(r);
      if (v !== null && v !== undefined && Number.isFinite(v)) {
        ts.push(r.t);
        ys.push(v);
      }
    }
    if (ts.length < 4) return null;

    const srcY = isAngle ? unwrapDeg(ys) : Float64Array.from(ys);
    const dt = 1.0 / fs;
    const t0 = Math.max(tStart, ts[0]);
    const t1 = Math.min(tEnd, ts[ts.length - 1]);
    if (t1 - t0 < dt * 4) return null;

    const n = Math.floor((t1 - t0) * fs) + 1;
    const tGrid = new Float64Array(n);
    const yGrid = new Float64Array(n);
    let idx = 0;
    for (let k = 0; k < n; k++) {
      const tk = t0 + k * dt;
      tGrid[k] = tk;
      while (idx < ts.length - 2 && ts[idx + 1] < tk) idx++;
      const tA = ts[idx], tB = ts[idx + 1];
      const frac = (tB > tA) ? Math.max(0, Math.min(1, (tk - tA) / (tB - tA))) : 0;
      yGrid[k] = srcY[idx] + frac * (srcY[idx + 1] - srcY[idx]);
    }
    return { t: tGrid, y: yGrid, fs, t0, t1, srcCount: ts.length };
  }

  // ------------------------------------------------------------
  // 2次 Butterworth ゼロ位相フィルタ (filtfilt 相当)
  // ------------------------------------------------------------
  function biquadCoeffsLP(fs, fc) {
    const nyq = fs * 0.5;
    const f = Math.max(1e-5, Math.min(fc, nyq * 0.95));
    const K = Math.tan(Math.PI * f / fs);
    const K2 = K * K;
    const sqrt2 = Math.SQRT2;
    const norm = 1.0 / (1.0 + sqrt2 * K + K2);
    return {
      b0: K2 * norm,
      b1: 2.0 * K2 * norm,
      b2: K2 * norm,
      a1: 2.0 * (K2 - 1.0) * norm,
      a2: (1.0 - sqrt2 * K + K2) * norm
    };
  }

  function biquadCoeffsHP(fs, fc) {
    const nyq = fs * 0.5;
    const f = Math.max(1e-5, Math.min(fc, nyq * 0.95));
    const K = Math.tan(Math.PI * f / fs);
    const K2 = K * K;
    const sqrt2 = Math.SQRT2;
    const norm = 1.0 / (1.0 + sqrt2 * K + K2);
    return {
      b0: 1.0 * norm,
      b1: -2.0 * norm,
      b2: 1.0 * norm,
      a1: 2.0 * (K2 - 1.0) * norm,
      a2: (1.0 - sqrt2 * K + K2) * norm
    };
  }

  function applyBiquad(x, c) {
    const n = x.length;
    const y = new Float64Array(n);
    let x1 = x[0], x2 = x[0];
    let y1 = x[0] * (c.b0 + c.b1 + c.b2) / (1 + c.a1 + c.a2);
    if (!Number.isFinite(y1)) y1 = 0;
    let y2 = y1;
    for (let i = 0; i < n; i++) {
      const x0 = x[i];
      const y0 = c.b0 * x0 + c.b1 * x1 + c.b2 * x2 - c.a1 * y1 - c.a2 * y2;
      y[i] = y0;
      x2 = x1; x1 = x0;
      y2 = y1; y1 = y0;
    }
    return y;
  }

  function filtfiltBiquad(arr, c) {
    const n = arr.length;
    if (n < 8) return Float64Array.from(arr);
    const pad = Math.min(n - 1, 24);
    const ext = new Float64Array(n + 2 * pad);
    for (let i = 0; i < pad; i++) ext[i] = 2 * arr[0] - arr[pad - i];
    for (let i = 0; i < n; i++) ext[pad + i] = arr[i];
    for (let i = 0; i < pad; i++) ext[pad + n + i] = 2 * arr[n - 1] - arr[n - 2 - i];

    const fwd = applyBiquad(ext, c);
    fwd.reverse();
    const bwd = applyBiquad(fwd, c);
    bwd.reverse();

    return bwd.subarray(pad, pad + n);
  }

  function bandpassZeroPhase(arr, fs, fLow, fHigh) {
    let x = Float64Array.from(arr);
    if (fHigh && fHigh < fs * 0.48) {
      x = filtfiltBiquad(x, biquadCoeffsLP(fs, fHigh));
    }
    if (fLow && fLow > 0) {
      x = filtfiltBiquad(x, biquadCoeffsHP(fs, fLow));
    }
    return x;
  }

  function lowpassZeroPhase(arr, fs, fCut) {
    if (!fCut || fCut >= fs * 0.48) return Float64Array.from(arr);
    return filtfiltBiquad(arr, biquadCoeffsLP(fs, fCut));
  }

  // ------------------------------------------------------------
  // Radix-2 FFT / IFFT
  // ------------------------------------------------------------
  function nextPow2(n) {
    let p = 1;
    while (p < n) p <<= 1;
    return p;
  }

  function fftRadix2(re, im, inverse) {
    const n = re.length;
    // Bit-reversal permutation
    let j = 0;
    for (let i = 1; i < n; i++) {
      let bit = n >> 1;
      for (; j & bit; bit >>= 1) j ^= bit;
      j ^= bit;
      if (i < j) {
        let tmp = re[i]; re[i] = re[j]; re[j] = tmp;
        tmp = im[i]; im[i] = im[j]; im[j] = tmp;
      }
    }
    const sign = inverse ? 1 : -1;
    for (let len = 2; len <= n; len <<= 1) {
      const ang = sign * 2.0 * Math.PI / len;
      const wlenRe = Math.cos(ang);
      const wlenIm = Math.sin(ang);
      const half = len >> 1;
      for (let i = 0; i < n; i += len) {
        let wRe = 1.0, wIm = 0.0;
        for (let k = 0; k < half; k++) {
          const uRe = re[i + k], uIm = im[i + k];
          const vRe = re[i + k + half] * wRe - im[i + k + half] * wIm;
          const vIm = re[i + k + half] * wIm + im[i + k + half] * wRe;
          re[i + k] = uRe + vRe;
          im[i + k] = uIm + vIm;
          re[i + k + half] = uRe - vRe;
          im[i + k + half] = uIm - vIm;
          const nwRe = wRe * wlenRe - wIm * wlenIm;
          wIm = wRe * wlenIm + wIm * wlenRe;
          wRe = nwRe;
        }
      }
    }
    if (inverse) {
      for (let i = 0; i < n; i++) { re[i] /= n; im[i] /= n; }
    }
  }

  // ------------------------------------------------------------
  // Welch PSD (Hann 窓 + 各セグメント線形デトレンド)
  // ------------------------------------------------------------
  function welchPSD(arr, fs, segSec, overlap) {
    const n = arr.length;
    const ov = (overlap === undefined) ? 0.5 : overlap;
    let nperseg = Math.min(n, Math.max(16, Math.round(segSec * fs)));
    const nfft = nextPow2(nperseg);
    const step = Math.max(1, Math.round(nperseg * (1 - ov)));

    const win = new Float64Array(nperseg);
    let winPow = 0;
    for (let i = 0; i < nperseg; i++) {
      const w = 0.5 * (1.0 - Math.cos((2.0 * Math.PI * i) / (nperseg - 1)));
      win[i] = w;
      winPow += w * w;
    }

    const half = (nfft >> 1) + 1;
    const psd = new Float64Array(half);
    let count = 0;
    const re = new Float64Array(nfft);
    const im = new Float64Array(nfft);

    for (let start = 0; start + nperseg <= n; start += step) {
      const seg = detrendLinear(arr.subarray(start, start + nperseg));
      re.fill(0);
      im.fill(0);
      for (let i = 0; i < nperseg; i++) re[i] = seg[i] * win[i];
      fftRadix2(re, im, false);
      const scale = 1.0 / (fs * winPow);
      for (let k = 0; k < half; k++) {
        const mag2 = re[k] * re[k] + im[k] * im[k];
        const factor = (k === 0 || k === half - 1) ? 1.0 : 2.0;
        psd[k] += mag2 * scale * factor;
      }
      count++;
    }

    if (count === 0) return null;
    const freqs = new Float64Array(half);
    for (let k = 0; k < half; k++) {
      psd[k] /= count;
      freqs[k] = (k * fs) / nfft;
    }
    return { freqs, psd, count, nperseg, nfft };
  }

  function integrateBandRms(freqs, psd, pMin, pMax) {
    const fMin = 1.0 / pMax;
    const fMax = 1.0 / pMin;
    let area = 0;
    for (let i = 1; i < freqs.length; i++) {
      const f0 = freqs[i - 1], f1 = freqs[i];
      if (f1 <= fMin || f0 >= fMax) continue;
      const a = Math.max(f0, fMin);
      const b = Math.min(f1, fMax);
      if (b <= a) continue;
      const fracA = (a - f0) / (f1 - f0);
      const fracB = (b - f0) / (f1 - f0);
      const pA = psd[i - 1] + fracA * (psd[i] - psd[i - 1]);
      const pB = psd[i - 1] + fracB * (psd[i] - psd[i - 1]);
      area += 0.5 * (pA + pB) * (b - a);
    }
    return Math.sqrt(Math.max(0, area));
  }

  // ------------------------------------------------------------
  // スペクトログラム (STFT)
  // ------------------------------------------------------------
  function computeSpectrogram(arr, tGrid, fs, winSec, stepSec, bands) {
    const n = arr.length;
    const nwin = Math.min(n, Math.max(32, Math.round(winSec * fs)));
    const nstep = Math.max(1, Math.round(stepSec * fs));
    const nfft = nextPow2(nwin);
    const half = (nfft >> 1) + 1;

    const win = new Float64Array(nwin);
    let winPow = 0;
    for (let i = 0; i < nwin; i++) {
      const w = 0.5 * (1.0 - Math.cos((2.0 * Math.PI * i) / (nwin - 1)));
      win[i] = w;
      winPow += w * w;
    }

    const freqs = new Float64Array(half);
    for (let k = 0; k < half; k++) freqs[k] = (k * fs) / nfft;

    const times = [];
    const cols = [];
    const bandSeries = bands.map(() => []);
    const re = new Float64Array(nfft);
    const im = new Float64Array(nfft);
    const scale = 1.0 / (fs * winPow);

    for (let start = 0; start + nwin <= n; start += nstep) {
      const seg = detrendLinear(arr.subarray(start, start + nwin));
      re.fill(0);
      im.fill(0);
      for (let i = 0; i < nwin; i++) re[i] = seg[i] * win[i];
      fftRadix2(re, im, false);
      const col = new Float64Array(half);
      for (let k = 0; k < half; k++) {
        const mag2 = re[k] * re[k] + im[k] * im[k];
        const factor = (k === 0 || k === half - 1) ? 1.0 : 2.0;
        col[k] = mag2 * scale * factor;
      }
      const centerIdx = Math.min(n - 1, start + (nwin >> 1));
      times.push(tGrid[centerIdx]);
      cols.push(col);
      for (let b = 0; b < bands.length; b++) {
        bandSeries[b].push(integrateBandRms(freqs, col, bands[b].pMin, bands[b].pMax));
      }
    }
    return { times, freqs, cols, bandSeries, winSec, stepSec };
  }

  // ------------------------------------------------------------
  // FFT ベース Hilbert 変換 (瞬時振幅 & 瞬時位相)
  // ------------------------------------------------------------
  function hilbertAnalytic(arr) {
    const n = arr.length;
    const nfft = nextPow2(n);
    const re = new Float64Array(nfft);
    const im = new Float64Array(nfft);
    for (let i = 0; i < n; i++) re[i] = arr[i];
    fftRadix2(re, im, false);

    // 解析信号の周波数マスク: DC と Nyquist はそのまま、正周波数は2倍、負周波数は0
    const half = nfft >> 1;
    for (let k = 1; k < half; k++) {
      re[k] *= 2.0;
      im[k] *= 2.0;
    }
    for (let k = half + 1; k < nfft; k++) {
      re[k] = 0;
      im[k] = 0;
    }
    fftRadix2(re, im, true);

    const amp = new Float64Array(n);
    const phase = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      amp[i] = Math.hypot(re[i], im[i]);
      phase[i] = Math.atan2(im[i], re[i]);
    }
    return { amp, phase };
  }

  // ------------------------------------------------------------
  // 自己相関関数 (ACF) & 2次系減衰振動モデルフィット
  // ACF(τ) = exp(-ζ ωn τ) * (cos(ωd τ) + (ζ / √(1-ζ²)) sin(ωd τ))
  // ------------------------------------------------------------
  function computeACF(arr, maxLag) {
    const n = arr.length;
    const m = mean(arr);
    const x = new Float64Array(n);
    let denom = 0;
    for (let i = 0; i < n; i++) {
      const v = arr[i] - m;
      x[i] = v;
      denom += v * v;
    }
    const lags = Math.min(maxLag, n - 2);
    const acf = new Float64Array(lags + 1);
    if (denom <= 1e-12) return acf;
    acf[0] = 1.0;
    for (let lag = 1; lag <= lags; lag++) {
      let s = 0;
      for (let i = 0; i < n - lag; i++) s += x[i] * x[i + lag];
      acf[lag] = s / denom;
    }
    return acf;
  }

  function dampedOscModel(tau, T, zeta) {
    const z = Math.max(0.005, Math.min(0.95, zeta));
    const wn = (2.0 * Math.PI) / T;
    const sq = Math.sqrt(1.0 - z * z);
    const wd = wn * sq;
    return Math.exp(-z * wn * tau) * (Math.cos(wd * tau) + (z / sq) * Math.sin(wd * tau));
  }

  function fitDampedOscACF(acf, fs, tMin, tMax) {
    const n = acf.length;
    if (n < 5) return null;
    const evalErr = (T, zeta) => {
      let sse = 0;
      for (let k = 1; k < n; k++) {
        const pred = dampedOscModel(k / fs, T, zeta);
        const diff = acf[k] - pred;
        sse += diff * diff;
      }
      return sse;
    };

    // 粗いグリッドサーチ
    let bestT = (tMin + tMax) * 0.5;
    let bestZ = 0.2;
    let bestErr = Infinity;
    const tSteps = 45;
    const zSteps = 22;
    for (let it = 0; it <= tSteps; it++) {
      const T = tMin + (tMax - tMin) * (it / tSteps);
      for (let iz = 0; iz <= zSteps; iz++) {
        const z = 0.02 + 0.78 * (iz / zSteps);
        const e = evalErr(T, z);
        if (e < bestErr) { bestErr = e; bestT = T; bestZ = z; }
      }
    }

    // 座標降下で精密化
    let stepT = (tMax - tMin) / tSteps;
    let stepZ = 0.04;
    for (let iter = 0; iter < 18; iter++) {
      let improved = false;
      for (const dT of [-stepT, stepT]) {
        const candT = Math.max(tMin, Math.min(tMax, bestT + dT));
        const e = evalErr(candT, bestZ);
        if (e < bestErr) { bestErr = e; bestT = candT; improved = true; }
      }
      for (const dZ of [-stepZ, stepZ]) {
        const candZ = Math.max(0.01, Math.min(0.92, bestZ + dZ));
        const e = evalErr(bestT, candZ);
        if (e < bestErr) { bestErr = e; bestZ = candZ; improved = true; }
      }
      if (!improved) { stepT *= 0.5; stepZ *= 0.5; }
    }

    // R² 計算
    let ym = 0;
    for (let k = 1; k < n; k++) ym += acf[k];
    ym /= (n - 1);
    let sst = 0;
    const fitCurve = new Float64Array(n);
    fitCurve[0] = 1.0;
    for (let k = 1; k < n; k++) {
      fitCurve[k] = dampedOscModel(k / fs, bestT, bestZ);
      const d = acf[k] - ym;
      sst += d * d;
    }
    const r2 = sst > 1e-12 ? (1.0 - bestErr / sst) : 0;
    return { T: bestT, zeta: bestZ, r2, fitCurve };
  }

  // ============================================================
  // 解析 1: 鉛直加速度 (G) & 荷重倍数 (plot_vertical_accel.py 準拠)
  // ============================================================
  function computeVerticalAccel(records, tStart, tEnd, smoothSec) {
    const sSec = Math.max(0.5, smoothSec || 1.5);
    // 地上静止ノイズフロア評価 (全レコードから gs < 1.5 m/s の連続区間を使う)
    const restAz = [];
    for (let i = 1; i < records.length; i++) {
      const a = records[i - 1], b = records[i];
      const dt = b.t - a.t;
      if (dt > 0.1 && dt < 1.2 && a.gs !== null && b.gs !== null && a.gs < 1.5 && b.gs < 1.5 &&
          a.kf_vs !== null && b.kf_vs !== null) {
        restAz.push((b.kf_vs - a.kf_vs) / dt);
      }
    }
    const noiseFloorRms = restAz.length >= 10 ? rms(restAz) : NaN;

    // 対象区間の差分加速度
    const tArr = [], azRaw = [], altArr = [], gsArr = [], rollArr = [], vsArr = [];
    for (let i = 1; i < records.length; i++) {
      const a = records[i - 1], b = records[i];
      if (b.t < tStart || a.t > tEnd) continue;
      const dt = b.t - a.t;
      if (dt <= 0.05 || dt > 1.5) continue;
      if (a.kf_vs === null || b.kf_vs === null) continue;
      const az = (b.kf_vs - a.kf_vs) / dt;
      if (!Number.isFinite(az)) continue;
      tArr.push(0.5 * (a.t + b.t));
      azRaw.push(az);
      altArr.push(b.kf_alt !== null ? b.kf_alt : NaN);
      gsArr.push(b.gs !== null ? b.gs : NaN);
      rollArr.push(b.roll !== null ? b.roll : NaN);
      vsArr.push(b.kf_vs);
    }

    if (tArr.length < 5) {
      return { error: 'KF_Vspeed データが不足しているため鉛直 G を算出できません。' };
    }

    const dts = [];
    for (let i = 1; i < tArr.length; i++) dts.push(tArr[i] - tArr[i - 1]);
    const medDt = median(dts) || 0.5;
    const winSamples = Math.max(1, Math.round(sSec / medDt));
    const azSmooth = movingAverage(Float64Array.from(azRaw), winSamples);

    const nRaw = new Float64Array(azRaw.length);
    const nSmooth = new Float64Array(azRaw.length);
    let maxPosIdx = 0, maxNegIdx = 0, maxPosSmIdx = 0, maxNegSmIdx = 0;
    for (let i = 0; i < azRaw.length; i++) {
      nRaw[i] = 1.0 + azRaw[i] / G0;
      nSmooth[i] = 1.0 + azSmooth[i] / G0;
      if (nRaw[i] > nRaw[maxPosIdx]) maxPosIdx = i;
      if (nRaw[i] < nRaw[maxNegIdx]) maxNegIdx = i;
      if (nSmooth[i] > nSmooth[maxPosSmIdx]) maxPosSmIdx = i;
      if (nSmooth[i] < nSmooth[maxNegSmIdx]) maxNegSmIdx = i;
    }

    // 上位ピークイベント抽出（最低 10 秒離す）
    const indices = Array.from({ length: azRaw.length }, (_, i) => i);
    indices.sort((i, j) => Math.abs(azRaw[j]) - Math.abs(azRaw[i]));
    const peaks = [];
    for (const idx of indices) {
      const ti = tArr[idx];
      if (peaks.some(p => Math.abs(p.t - ti) < 10.0)) continue;
      peaks.push({
        t: ti,
        nRaw: nRaw[idx],
        nSmooth: nSmooth[idx],
        azRaw: azRaw[idx],
        alt: altArr[idx],
        gs: gsArr[idx],
        roll: rollArr[idx]
      });
      if (peaks.length >= 8) break;
    }

    const flightAzRms = rms(azRaw);
    const snr = (Number.isFinite(noiseFloorRms) && noiseFloorRms > 0) ? (flightAzRms / noiseFloorRms) : NaN;

    return {
      t: Float64Array.from(tArr),
      azRaw: Float64Array.from(azRaw),
      azSmooth,
      nRaw,
      nSmooth,
      vs: Float64Array.from(vsArr),
      medDt,
      smoothSec: sSec,
      noiseFloorRms,
      noiseFloorG: Number.isFinite(noiseFloorRms) ? noiseFloorRms / G0 : NaN,
      flightAzRms,
      snr,
      maxPos: { t: tArr[maxPosIdx], n: nRaw[maxPosIdx], az: azRaw[maxPosIdx] },
      maxNeg: { t: tArr[maxNegIdx], n: nRaw[maxNegIdx], az: azRaw[maxNegIdx] },
      maxPosSmooth: { t: tArr[maxPosSmIdx], n: nSmooth[maxPosSmIdx], az: azSmooth[maxPosSmIdx] },
      maxNegSmooth: { t: tArr[maxNegSmIdx], n: nSmooth[maxNegSmIdx], az: azSmooth[maxNegSmIdx] },
      peaks
    };
  }

  // ============================================================
  // 解析 2: 散布図・不安定さ相関 (plot_pitch_altitude + plot_instability 準拠)
  // ============================================================
  function computeInstabilityAndScatter(records, tStart, tEnd, winSec, avgSec) {
    const wSec = Math.max(10, winSec || 45);
    const aSec = Math.max(2, avgSec || 7);

    // 窓ごとの不安定さ（デトレンド残差 RMS）
    const windows = [];
    const step = wSec * 0.5;
    for (let t0 = tStart; t0 + wSec <= tEnd + 1e-6; t0 += step) {
      const t1 = t0 + wSec;
      const pList = [], rList = [], vsList = [], altList = [], tasList = [], gsList = [];
      for (let i = 0; i < records.length; i++) {
        const d = records[i];
        if (d.t < t0) continue;
        if (d.t > t1) break;
        if (d.pitch !== null && Number.isFinite(d.pitch)) pList.push(d.pitch);
        if (d.roll !== null && Number.isFinite(d.roll)) rList.push(d.roll);
        if (d.kf_vs !== null && Number.isFinite(d.kf_vs)) vsList.push(d.kf_vs);
        if (d.kf_alt !== null && Number.isFinite(d.kf_alt)) altList.push(d.kf_alt);
        const tas = (d.csvTas !== null && Number.isFinite(d.csvTas)) ? d.csvTas : d.gs;
        if (tas !== null && Number.isFinite(tas)) tasList.push(tas);
        if (d.gs !== null && Number.isFinite(d.gs)) gsList.push(d.gs);
      }
      if (Math.max(pList.length, rList.length, vsList.length) < 10) continue;
      windows.push({
        t: t0 + 0.5 * wSec,
        elapsedMin: (t0 + 0.5 * wSec - tStart) / 60.0,
        pitchMean: pList.length >= 5 ? mean(pList) : NaN,
        pitchInstab: pList.length >= 10 ? std(detrendLinear(pList)) : NaN,
        rollInstab: rList.length >= 10 ? std(detrendLinear(rList)) : NaN,
        vsInstab: vsList.length >= 10 ? std(detrendLinear(vsList)) : NaN,
        altMean: altList.length >= 3 ? mean(altList) : NaN,
        tasMean: tasList.length >= 3 ? mean(tasList) : NaN,
        gsMean: gsList.length >= 3 ? mean(gsList) : NaN
      });
    }

    // 短窓平均ポイント (ピッチ vs 高度 / 時刻 / TAS)
    const points = [];
    for (let t0 = tStart; t0 + aSec <= tEnd + 1e-6; t0 += aSec) {
      const t1 = t0 + aSec;
      const pList = [], rList = [], altList = [], tasList = [], gsList = [], vsList = [];
      for (let i = 0; i < records.length; i++) {
        const d = records[i];
        if (d.t < t0) continue;
        if (d.t > t1) break;
        if (d.pitch !== null && Number.isFinite(d.pitch)) pList.push(d.pitch);
        if (d.roll !== null && Number.isFinite(d.roll)) rList.push(d.roll);
        if (d.kf_alt !== null && Number.isFinite(d.kf_alt)) altList.push(d.kf_alt);
        const tas = (d.csvTas !== null && Number.isFinite(d.csvTas)) ? d.csvTas : d.gs;
        if (tas !== null && Number.isFinite(tas)) tasList.push(tas);
        if (d.gs !== null && Number.isFinite(d.gs)) gsList.push(d.gs);
        if (d.kf_vs !== null && Number.isFinite(d.kf_vs)) vsList.push(d.kf_vs);
      }
      if (altList.length === 0 && pList.length === 0) continue;
      points.push({
        t: t0 + 0.5 * aSec,
        elapsedMin: (t0 + 0.5 * aSec - tStart) / 60.0,
        pitch: mean(pList),
        roll: mean(rList),
        alt: mean(altList),
        tas: mean(tasList),
        gs: mean(gsList),
        vs: mean(vsList)
      });
    }

    const corr = {
      time_pitchInstab: pearsonR(windows.map(w => w.elapsedMin), windows.map(w => w.pitchInstab)),
      time_rollInstab: pearsonR(windows.map(w => w.elapsedMin), windows.map(w => w.rollInstab)),
      time_vsInstab: pearsonR(windows.map(w => w.elapsedMin), windows.map(w => w.vsInstab)),
      pitch_pitchInstab: pearsonR(windows.map(w => w.pitchMean), windows.map(w => w.pitchInstab)),
      pitch_rollInstab: pearsonR(windows.map(w => w.pitchMean), windows.map(w => w.rollInstab)),
      pitch_vsInstab: pearsonR(windows.map(w => w.pitchMean), windows.map(w => w.vsInstab))
    };

    return { windows, points, corr, winSec: wSec, avgSec: aSec };
  }

  // ============================================================
  // 解析 3: スペクトル & スペクトログラム (plot_spectrum + plot_spectrogram 準拠)
  // ★ 前半／後半（旋回前後）の比較は 2026 年大会のコースに特化した分析なので入れない（2026-11-20）。
  //   区間の一部だけを見たいときは解析画面の「区間内の開始／終了」で絞る。
  // ============================================================
  const DEFAULT_BANDS = [
    { label: '短周期 (2–8s)', pMin: 2, pMax: 8, color: '#dc2626' },
    { label: '中周期 (8–20s)', pMin: 8, pMax: 20, color: '#ea580c' },
    { label: '長周期 (20–50s)', pMin: 20, pMax: 50, color: '#0284c7' },
    { label: '超長周期 (50–120s)', pMin: 50, pMax: 120, color: '#7c3aed' }
  ];

  function computeSpectrumAndSpectrogram(records, tStart, tEnd, channel, segSec) {
    const getters = {
      pitch: { fn: d => d.pitch, label: 'ピッチ', unit: 'deg', fs: 2.0 },
      roll:  { fn: d => d.roll,  label: 'ロール', unit: 'deg', fs: 2.0 },
      vs:    { fn: d => d.kf_vs, label: 'KF 昇降速度 (V/S)', unit: 'm/s', fs: 2.0 },
      gs:    { fn: d => d.gs,    label: '対地速度 (G/S)', unit: 'm/s', fs: 2.0 }
    };
    const ch = getters[channel] || getters.pitch;
    const res = resampleUniform(records, ch.fn, ch.fs, tStart, tEnd);
    if (!res || res.y.length < 32) {
      return { error: `${ch.label} のデータが不足しています。` };
    }

    const dur = res.t1 - res.t0;
    const sSec = Math.min(Math.max(16, segSec || 90), Math.max(16, dur * 0.45));
    const psdAll = welchPSD(res.y, res.fs, sSec, 0.5);

    const stftWin = Math.min(Math.max(30, Math.round(dur * 0.18)), 180);
    const stftStep = Math.max(5, Math.round(stftWin * 0.2));
    const spec = computeSpectrogram(res.y, res.t, res.fs, stftWin, stftStep, DEFAULT_BANDS);

    return {
      channel, label: ch.label, unit: ch.unit, fs: res.fs,
      t0: res.t0, t1: res.t1,
      psdAll,
      spec,
      bands: DEFAULT_BANDS
    };
  }

  // ============================================================
  // 解析 4: フゴイド解析 (plot_phugoid + plot_phugoid_windows 準拠)
  // ============================================================
  function computePhugoid(records, tStart, tEnd, pMin, pMax) {
    const fs = 2.0;
    const bpMin = Math.max(1.5, pMin || 2.5);
    const bpMax = Math.max(bpMin + 1.0, pMax || 10.0);
    const fLow = 1.0 / bpMax;
    const fHigh = 1.0 / bpMin;

    const rPitch = resampleUniform(records, d => d.pitch, fs, tStart, tEnd);
    const rGs = resampleUniform(records, d => d.gs, fs, tStart, tEnd);

    // 気圧があれば気圧高度から非KF V/S を算出（KF V/S は IMU 加速度計と人工相関するため）
    const pVals = records.filter(d => d.t >= tStart && d.t <= tEnd && d.press !== null && d.press > 500).map(d => d.press);
    let rAlt = null;
    let altSource = '気圧高度 (非KF)';
    if (pVals.length >= 20) {
      const p0 = pVals[pVals.length - 1];
      rAlt = resampleUniform(records, d => (d.press !== null && d.press > 500) ? baroAltFromPressure(d.press, p0) : null, fs, tStart, tEnd);
    }
    if (!rAlt) {
      altSource = 'KF_Altitude (気圧欠損のため代替)';
      rAlt = resampleUniform(records, d => d.kf_alt, fs, tStart, tEnd);
    }

    if (!rAlt || !rGs) {
      return { error: '高度または対地速度(G/S)のデータが不足しています。' };
    }

    // 共通時間軸に揃える
    const t0 = Math.max(rAlt.t0, rGs.t0, rPitch ? rPitch.t0 : rAlt.t0);
    const t1 = Math.min(rAlt.t1, rGs.t1, rPitch ? rPitch.t1 : rAlt.t1);
    const n = Math.floor((t1 - t0) * fs) + 1;
    if (n < 60) {
      return { error: 'フゴイド解析には最低 30 秒以上の飛行データが必要です。' };
    }

    const sliceToCommon = (res) => {
      const startIdx = Math.max(0, Math.round((t0 - res.t0) * fs));
      const out = new Float64Array(n);
      for (let i = 0; i < n; i++) out[i] = res.y[Math.min(res.y.length - 1, startIdx + i)];
      return out;
    };

    const tGrid = new Float64Array(n);
    for (let i = 0; i < n; i++) tGrid[i] = t0 + i / fs;

    const altRaw = sliceToCommon(rAlt);
    const gsRaw = sliceToCommon(rGs);
    const pitchRaw = rPitch ? sliceToCommon(rPitch) : null;

    // 気圧高度の平滑中心差分で非KF V/S を作成
    const altSmooth = lowpassZeroPhase(altRaw, fs, 0.45);
    const vsRaw = new Float64Array(n);
    for (let i = 1; i < n - 1; i++) vsRaw[i] = (altSmooth[i + 1] - altSmooth[i - 1]) * (0.5 * fs);
    vsRaw[0] = (altSmooth[1] - altSmooth[0]) * fs;
    vsRaw[n - 1] = (altSmooth[n - 1] - altSmooth[n - 2]) * fs;

    // フゴイド帯域バンドパス
    const altBp = bandpassZeroPhase(detrendLinear(altRaw), fs, fLow, fHigh);
    const vsBp = bandpassZeroPhase(detrendLinear(vsRaw), fs, fLow, fHigh);
    const gsBp = bandpassZeroPhase(detrendLinear(gsRaw), fs, fLow, fHigh);
    const pitchBp = pitchRaw ? bandpassZeroPhase(detrendLinear(pitchRaw), fs, fLow, fHigh) : null;

    // 自己相関 & 2次系モデルフィット（ピッチがあればピッチ、無ければ V/S）
    const primarySig = pitchBp || vsBp;
    const primaryName = pitchBp ? 'ピッチ' : 'V/S (気圧微分)';
    const maxLag = Math.min(Math.round(25 * fs), Math.floor(n * 0.25));
    const acf = computeACF(primarySig, maxLag);
    const fit = fitDampedOscACF(acf, fs, bpMin, bpMax);

    // 理論古典フゴイド周期 T = π√2 * V / g
    const meanV = mean(gsRaw);
    const classicalT = Number.isFinite(meanV) ? (Math.PI * Math.SQRT2 * meanV / G0) : NaN;

    // ヒルベルト変換で位相チェック
    const hAlt = hilbertAnalytic(altBp);
    const hVs = hilbertAnalytic(vsBp);
    const hGs = hilbertAnalytic(gsBp);
    const hPitch = pitchBp ? hilbertAnalytic(pitchBp) : null;

    const computePhaseStats = (aIdx, bIdx) => {
      let s1Num = 0, s1Den = 0, s2Num = 0, s2Den = 0;
      let d1Sin = 0, d1Cos = 0, d2Sin = 0, d2Cos = 0;
      let pSq = 0, vSq = 0, cnt = 0;
      for (let i = aIdx; i < bIdx; i++) {
        const w2 = hAlt.amp[i] * hGs.amp[i];
        const d2 = hAlt.phase[i] - hGs.phase[i];
        s2Num += w2 * Math.cos(d2 - Math.PI);
        s2Den += w2;
        d2Cos += w2 * Math.cos(d2);
        d2Sin += w2 * Math.sin(d2);

        if (hPitch) {
          const w1 = hPitch.amp[i] * hVs.amp[i];
          const d1 = hPitch.phase[i] - hVs.phase[i];
          s1Num += w1 * Math.cos(d1);
          s1Den += w1;
          d1Cos += w1 * Math.cos(d1);
          d1Sin += w1 * Math.sin(d1);
          pSq += pitchBp[i] * pitchBp[i];
        }
        vSq += vsBp[i] * vsBp[i];
        cnt++;
      }
      const s1 = s1Den > 0 ? s1Num / s1Den : NaN;
      const s2 = s2Den > 0 ? s2Num / s2Den : NaN;
      const phase1Deg = s1Den > 0 ? Math.atan2(d1Sin, d1Cos) * 180 / Math.PI : NaN;
      const phase2Deg = s2Den > 0 ? Math.atan2(d2Sin, d2Cos) * 180 / Math.PI : NaN;
      const ampRms = cnt > 0 ? Math.sqrt((hPitch ? pSq : vSq) / cnt) : 0;
      const score = ampRms * (Number.isFinite(s1) ? Math.max(0, s1) : 1.0) * Math.max(0, s2);
      return { s1, s2, phase1Deg, phase2Deg, ampRms, score };
    };

    const globalPhase = computePhaseStats(0, n);

    // 60秒区間（30秒ステップ）ごとのフゴイドらしさ
    const winLen = Math.min(Math.round(60 * fs), Math.max(20, Math.floor(n * 0.5)));
    const winStep = Math.max(10, Math.round(winLen * 0.5));
    const windows = [];
    for (let i0 = 0; i0 + winLen <= n; i0 += winStep) {
      const i1 = i0 + winLen;
      const st = computePhaseStats(i0, i1);
      windows.push({
        i0, i1,
        tStart: tGrid[i0],
        tEnd: tGrid[i1 - 1],
        tCenter: 0.5 * (tGrid[i0] + tGrid[i1 - 1]),
        ...st
      });
    }

    return {
      t: tGrid, fs, altSource, hasPitch: !!pitchBp, primaryName,
      bpMin, bpMax, classicalT, meanV,
      altBp, vsBp, gsBp, pitchBp,
      acf, fit, globalPhase, windows
    };
  }

  // ============================================================
  // 解析 5: 横・方向モード (ダッチロール / スパイラル) (plot_lateral_modes.py 準拠)
  // ============================================================
  function computeLateralModes(records, tStart, tEnd, turnThreshDegS) {
    const fs = 2.0;
    const thresh = (turnThreshDegS !== undefined) ? turnThreshDegS : 1.5;
    const rTT = resampleUniform(records, d => d.tt, fs, tStart, tEnd, { unwrapDeg: true });
    if (!rTT || rTT.y.length < 40) {
      return { error: 'TrueTrack データが不足しているため横モードを解析できません。' };
    }
    const n = rTT.y.length;
    const ttSmooth = lowpassZeroPhase(rTT.y, fs, 0.45);
    const rRate = new Float64Array(n);
    for (let i = 1; i < n - 1; i++) rRate[i] = (ttSmooth[i + 1] - ttSmooth[i - 1]) * (0.5 * fs);
    rRate[0] = (ttSmooth[1] - ttSmooth[0]) * fs;
    rRate[n - 1] = (ttSmooth[n - 1] - ttSmooth[n - 2]) * fs;

    const rRoll = resampleUniform(records, d => d.roll, fs, tStart, tEnd);
    let rollArr = null;
    if (rRoll && rRoll.y.length >= 40) {
      rollArr = new Float64Array(n);
      const offset = Math.round((rTT.t0 - rRoll.t0) * fs);
      for (let i = 0; i < n; i++) {
        const idx = Math.max(0, Math.min(rRoll.y.length - 1, i + offset));
        rollArr[i] = rRoll.y[idx];
      }
    }

    // 10 秒移動平均 |r| > thresh を旋回区間とし、前後 5 秒ガードして除外
    const rSlow = movingAverage(rRate, Math.round(10 * fs));
    const guard = Math.round(5 * fs);
    const isTurn = new Uint8Array(n);
    for (let i = 0; i < n; i++) {
      if (Math.abs(rSlow[i]) > thresh) {
        const a = Math.max(0, i - guard);
        const b = Math.min(n - 1, i + guard);
        for (let j = a; j <= b; j++) isTurn[j] = 1;
      }
    }

    // クリーンな直進セグメント（20秒以上）を抽出
    const cleanSegments = [];
    let segStart = -1;
    for (let i = 0; i <= n; i++) {
      const clean = (i < n && !isTurn[i]);
      if (clean && segStart < 0) segStart = i;
      if (!clean && segStart >= 0) {
        if (i - segStart >= Math.round(20 * fs)) cleanSegments.push([segStart, i]);
        segStart = -1;
      }
    }

    // クリーン区間が少なければ全区間をフォールバック
    const useSegs = cleanSegments.length > 0 ? cleanSegments : [[0, n]];
    const cleanRatio = cleanSegments.reduce((s, seg) => s + (seg[1] - seg[0]), 0) / n;

    // クリーン区間の重み付き平均 ACF を算出
    const maxLag = Math.min(Math.round(20 * fs), Math.floor(n * 0.25));
    const computePooledACF = (sig) => {
      const acc = new Float64Array(maxLag + 1);
      let wSum = 0;
      for (const [a, b] of useSegs) {
        const len = b - a;
        if (len <= maxLag + 4) continue;
        const sub = bandpassZeroPhase(detrendLinear(sig.subarray(a, b)), fs, 1.0 / 25.0, 0.45);
        const acfSeg = computeACF(sub, maxLag);
        for (let k = 0; k <= maxLag; k++) acc[k] += acfSeg[k] * len;
        wSum += len;
      }
      if (wSum === 0) return computeACF(detrendLinear(sig), maxLag);
      for (let k = 0; k <= maxLag; k++) acc[k] /= wSum;
      return acc;
    };

    const acfR = computePooledACF(rRate);
    const fitR = fitDampedOscACF(acfR, fs, 2.0, 18.0);

    let acfRoll = null, fitRoll = null, rollRmsClean = NaN;
    if (rollArr) {
      acfRoll = computePooledACF(rollArr);
      fitRoll = fitDampedOscACF(acfRoll, fs, 2.0, 18.0);
      const tmp = [];
      for (const [a, b] of useSegs) {
        const sub = detrendLinear(rollArr.subarray(a, b));
        for (let i = 0; i < sub.length; i++) tmp.push(sub[i]);
      }
      rollRmsClean = rms(tmp);
    }

    // スパイラル傾向指標: クリーン区間での低周波成分（周期 > 15s）の r RMS と ゼロクロス平均間隔
    let slowSq = 0, fastSq = 0, cleanCnt = 0, zeroCross = 0;
    const rSlowOnly = lowpassZeroPhase(rRate, fs, 1.0 / 15.0);
    for (const [a, b] of useSegs) {
      for (let i = a; i < b; i++) {
        slowSq += rSlowOnly[i] * rSlowOnly[i];
        const fast = rRate[i] - rSlowOnly[i];
        fastSq += fast * fast;
        cleanCnt++;
        if (i > a && ((rSlowOnly[i - 1] <= 0 && rSlowOnly[i] > 0) || (rSlowOnly[i - 1] >= 0 && rSlowOnly[i] < 0))) {
          zeroCross++;
        }
      }
    }
    const rSlowRms = cleanCnt > 0 ? Math.sqrt(slowSq / cleanCnt) : NaN;
    const rFastRms = cleanCnt > 0 ? Math.sqrt(fastSq / cleanCnt) : NaN;
    const meanZeroCrossSec = (zeroCross > 0 && cleanCnt > 0) ? (cleanCnt / fs) / zeroCross : NaN;

    return {
      t: rTT.t, fs, rRate, rSlow, rollArr, isTurn,
      cleanRatio, cleanSegments: useSegs, thresh,
      acfR, fitR, acfRoll, fitRoll,
      rSlowRms, rFastRms, rollRmsClean, meanZeroCrossSec
    };
  }

  // ============================================================
  // 解析 6: Google Earth 用 KML 生成 (logcsv2kml4earth.py 準拠)
  // ============================================================
  function buildKmlString(records, filename, opts) {
    const name = (filename || 'PONS_Flight').replace(/[<>&"]/g, '');
    const tStart = opts && opts.tStart !== undefined ? opts.tStart : 0;
    const tEnd = opts && opts.tEnd !== undefined ? opts.tEnd : Infinity;
    const coords = [];
    let firstRec = null, lastRec = null;
    for (let i = 0; i < records.length; i++) {
      const d = records[i];
      if (d.t < tStart || d.t > tEnd) continue;
      if (!Number.isFinite(d.lat) || !Number.isFinite(d.lon)) continue;
      const alt = (d.kf_alt !== null && Number.isFinite(d.kf_alt)) ? Math.max(0, d.kf_alt) : 0;
      coords.push(`${d.lon.toFixed(6)},${d.lat.toFixed(6)},${alt.toFixed(1)}`);
      if (!firstRec) firstRec = d;
      lastRec = d;
    }

    const esc = (s) => String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
    const pin = (label, rec) => rec ? `
    <Placemark>
      <name>${esc(label)} (${esc(rec.timeStr || '')})</name>
      <Point><coordinates>${rec.lon.toFixed(6)},${rec.lat.toFixed(6)},0</coordinates></Point>
    </Placemark>` : '';

    return `<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>${esc(name)}</name>
    <Style id="ponsTrack">
      <LineStyle>
        <color>ffeb6325</color>
        <width>3</width>
      </LineStyle>
    </Style>
    <Placemark>
      <name>${esc(name)} 航跡</name>
      <styleUrl>#ponsTrack</styleUrl>
      <LineString>
        <tessellate>1</tessellate>
        <altitudeMode>clampToGround</altitudeMode>
        <coordinates>
          ${coords.join(' ')}
        </coordinates>
      </LineString>
    </Placemark>${pin('開始', firstRec)}${pin('終了', lastRec)}
  </Document>
</kml>`;
  }

  // ------------------------------------------------------------
  // 簡易 CSV パーサ (横モード 2 機比較用の軽量版)
  // ------------------------------------------------------------
  function parseComparisonCSV(text) {
    const lines = text.split(/\r?\n/).filter(l => l.trim().length > 0);
    if (lines.length < 2) return null;
    const header = lines[0].split(',').map(h => h.trim().toLowerCase());
    const idx = {
      time: header.indexOf('time'),
      gs: header.indexOf('gs'),
      tt: header.indexOf('truetrack'),
      roll: header.indexOf('roll'),
      pitch: header.indexOf('pitch')
    };
    if (idx.tt < 0) return null;

    const parseSec = (s) => {
      const m = String(s || '').match(/(\d+):(\d+):([\d.]+)/);
      return m ? (parseInt(m[1], 10) * 3600 + parseInt(m[2], 10) * 60 + parseFloat(m[3])) : NaN;
    };

    const out = [];
    let t0 = null;
    for (let i = 1; i < lines.length; i++) {
      const p = lines[i].split(',').map(s => s.trim());
      const tt = parseFloat(p[idx.tt]);
      if (!Number.isFinite(tt)) continue;
      let sec = idx.time >= 0 ? parseSec(p[idx.time]) : i * 0.5;
      if (!Number.isFinite(sec)) sec = i * 0.5;
      if (t0 === null) t0 = sec;
      let rel = sec - t0;
      if (rel < -43200) rel += 86400;
      const gs = (idx.gs >= 0 && p[idx.gs] !== '') ? parseFloat(p[idx.gs]) : null;
      const roll = (idx.roll >= 0 && p[idx.roll] !== '') ? parseFloat(p[idx.roll]) : null;
      const pitch = (idx.pitch >= 0 && p[idx.pitch] !== '') ? parseFloat(p[idx.pitch]) : null;
      out.push({ t: rel, gs, tt, roll, pitch });
    }
    if (out.length < 20) return null;
    out.sort((a, b) => a.t - b.t);
    const minT = out[0].t;
    for (const d of out) d.t -= minT;
    let firstFly = out.findIndex(d => d.gs !== null && d.gs > 3.0);
    let lastFly = -1;
    for (let k = out.length - 1; k >= 0; k--) { if (out[k].gs !== null && out[k].gs > 2.0) { lastFly = k; break; } }
    const takeoff = firstFly >= 0 ? out[firstFly].t : 0;
    const land = lastFly >= 0 ? out[lastFly].t : out[out.length - 1].t;
    return { records: out, takeoff, land };
  }

  global.PonsAnalysisDSP = {
    VERSION: '2026.11.22.2',
    G0,
    mean, std, rms, median, pearsonR,
    detrendLinear, movingAverage, unwrapDeg,
    baroAltFromPressure, resampleUniform,
    bandpassZeroPhase, lowpassZeroPhase,
    welchPSD, integrateBandRms, computeSpectrogram,
    hilbertAnalytic, computeACF, dampedOscModel, fitDampedOscACF,
    computeVerticalAccel,
    computeInstabilityAndScatter,
    computeSpectrumAndSpectrogram,
    computePhugoid,
    computeLateralModes,
    buildKmlString,
    parseComparisonCSV
  };
})(window);

