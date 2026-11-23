// ============================================================
// File    : tools/repons/gauges.js
// Project : PONS v7.1 / v7.2 Re:PONS
// Role    : 計器とグラフの描画。風の三角形・人工水平儀・スクロールグラフ・高度と昇降
// Author  : MasaoC (@masao_mobile)
// Updated : 2026-11-23
// ============================================================
//
// 2026-11-23 に index.html から分けた。読む順は index.html の一覧どおり。関数・変数はファイルをまたいで
// グローバルで見えるが、読み込みの最中に動くコードは先に読んだファイルのものしか使えない。

// ============================================================
// 計器描画: 風の三角形 (Vector Triangle) - 白ベース対応
// ============================================================
function drawVectorTriangle(d, tas) {
  const canvas = document.getElementById('canvas-vector');
  // 表示サイズと内部解像度を完全一致させ、楕円歪みを防止
  const rect = canvas.getBoundingClientRect();
  const sz = Math.round(Math.min(rect.width, rect.height) || canvas.clientHeight || canvas.clientWidth);
  if (sz > 0 && (canvas.width !== sz || canvas.height !== sz)) {
    canvas.width = sz;
    canvas.height = sz;
  }
  const ctx = canvas.getContext('2d');
  const w = canvas.width, h = canvas.height;
  ctx.clearRect(0, 0, w, h);

  const cx = w / 2;
  const cy = h / 2;
  const baseR = Math.min(w, h) * 0.38;
  const scale = baseR / 7.2; // 1 m/s あたりのピクセル数

  // 方位盤リング (濃いグレー)
  ctx.save();
  ctx.strokeStyle = "#94a3b8";
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.arc(cx, cy, baseR, 0, Math.PI * 2);
  ctx.stroke();

  // 十字クロス目盛
  ctx.strokeStyle = "#e2e8f0";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(cx - baseR * 1.05, cy); ctx.lineTo(cx + baseR * 1.05, cy);
  ctx.moveTo(cx, cy - baseR * 1.05); ctx.lineTo(cx, cy + baseR * 1.05);
  ctx.stroke();
  ctx.restore();

  // 機体アイコン (士 型) - 矢印と重なっても視認性を損なわないグレー
  ctx.save();
  ctx.translate(cx, cy);
  const yawAngle = d.yaw !== null ? d.yaw : 0;
  ctx.rotate((yawAngle * Math.PI) / 180);

  ctx.fillStyle = "#94a3b8";
  ctx.strokeStyle = "#64748b";
  ctx.lineWidth = 1.0;

  // 胴体
  ctx.fillRect(-2, -22, 4, 34);
  // 主翼
  ctx.fillRect(-24, -14, 48, 4);
  // 尾翼
  ctx.fillRect(-9, 8, 18, 3);
  ctx.restore();

  // ベクトルの計算 (北=上、東=右の直交座標系)
  // 原点 = 機首 (cx, cy)
  const beta = d.estBeta !== null ? d.estBeta : 0;
  const yawVal = d.yaw !== null ? d.yaw : 0;
  const yawRad = (yawVal * Math.PI) / 180;
  const airBearingDeg = (yawVal + beta + 360) % 360;
  const airBearingRad = (airBearingDeg * Math.PI) / 180;
  const tasVal = (tas !== null && !isNaN(tas)) ? tas : 0;
  const tasX = cx + Math.sin(airBearingRad) * (tasVal * scale);
  const tasY = cy - Math.cos(airBearingRad) * (tasVal * scale);

  // 1. 機首方位点線 (Yaw reference) : 濃いシアン (向き: yaw)
  if (d.yaw !== null) {
    const yawRefLen = Math.max(tasVal * scale * 1.25, baseR * 0.5);
    ctx.save();
    ctx.strokeStyle = "#0284c7";
    ctx.lineWidth = 1.8;
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + Math.sin(yawRad) * yawRefLen, cy - Math.cos(yawRad) * yawRefLen);
    ctx.stroke();
    ctx.restore();
  }

  // 横滑り角 (Est.Beta) の扇形アーク表示 (推定値があり |beta| >= 0.3° で表示)
  if (d.estBeta !== null && Math.abs(d.estBeta) >= 0.3 && tasVal > 0) {
    ctx.save();
    ctx.strokeStyle = "#d97706";
    ctx.fillStyle = "rgba(217, 119, 6, 0.18)";
    ctx.lineWidth = 1.5;
    const arcR = Math.min(baseR * 0.65, tasVal * scale * 0.85);
    // Canvasの角度系 (右=0, 下=π/2)
    // ★ 終わりは機首から β だけ回した角度にする（正規化した airBearingRad を使うと、0°/360° を跨いだときに
    //   逆回りの約 350° の扇になる）
    const a0 = -Math.PI / 2 + yawRad;
    const a1 = a0 + (beta * Math.PI) / 180;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.arc(cx, cy, arcR, Math.min(a0, a1), Math.max(a0, a1));
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.restore();
  }

  // 2. 対気ベクトル (TAS) : 濃いアンバー (向き: yaw + estBeta)
  if (tas !== null && tas > 0) {
    drawArrow(ctx, cx, cy, tasX, tasY, "#ea580c", 3.2);
  }

  // 3. 風ベクトル (WIND) : 濃いグリーン (TASの先端から吹いていく向き)
  //   ★ TAS と機首方位があるときだけ TAS の先端から。無いときは機体（中心）から伸ばす
  if (d.wspd !== null && d.wspd > 0 && d.wdir !== null) {
    const windFlowRad = (d.wdir * Math.PI) / 180;
    const fromTas = tas !== null && tas > 0 && d.yaw !== null;
    const wx0 = fromTas ? tasX : cx, wy0 = fromTas ? tasY : cy;
    drawArrow(ctx, wx0, wy0, wx0 + Math.sin(windFlowRad) * (d.wspd * scale),
              wy0 - Math.cos(windFlowRad) * (d.wspd * scale), "#16a34a", 3.2);
  }

  // 4. 対地移動ベクトル (TT) : 漆黒太線
  //   ★ いつも GNSS の実測（真方位 TT と対地速度 GS）を描く（2026-11-14、BUG-129）。以前は風があると TAS + 風 の先端を
  //     描いていて、HUD の TT の数字と矢印の向きが食い違った（推定の TAS・風・横滑りは瞬時の実測と一致しない）。
  //     三角形が閉じないぶんは、推定の誤差としてそのまま見える
  if (d.gs !== null && d.gs > 0 && d.tt !== null) {
    const ttRad = (d.tt * Math.PI) / 180;
    drawArrow(ctx, cx, cy, cx + Math.sin(ttRad) * (d.gs * scale), cy - Math.cos(ttRad) * (d.gs * scale), "#0f172a", 4.0);
  }

  // HUDテキスト更新 (下部ステータスバー: データ欠損時は --- を表示)
  const yawAccStr = (d.yaw_acc95 !== null && !isNaN(d.yaw_acc95)) ? ` (±${d.yaw_acc95.toFixed(0)}°)` : "";
  const yawStr = (d.yaw !== null && !isNaN(d.yaw)) ? `${d.yaw.toFixed(0)}°${yawAccStr}` : "---°";
  const tasStr = (tas !== null && !isNaN(tas)) ? `${tas.toFixed(1)}m/s` : "--.-m/s";
  const betaStr = (d.estBeta !== null && !isNaN(d.estBeta)) ? `${d.estBeta > 0 ? '+' : ''}${d.estBeta.toFixed(1)}°` : "--.-°";
  const wdirStr = (d.wdir !== null && !isNaN(d.wdir)) ? ` (${d.wdir.toFixed(0)}°)` : "";
  const wspdStr = (d.wspd !== null && !isNaN(d.wspd)) ? `${d.wspd.toFixed(1)}m/s${wdirStr}` : "--.-m/s";
  const ttStr = (d.tt !== null && !isNaN(d.tt)) ? `${d.tt.toFixed(0)}°` : "---°";
  const gsStr = (d.gs !== null && !isNaN(d.gs)) ? `${d.gs.toFixed(1)}m/s` : "--.-m/s";

  document.getElementById('hud-vector-text').innerHTML = `
        <span style="color: #0284c7;">Yaw: ${yawStr}</span>
        <span style="color: #ea580c;">TAS: ${tasStr}</span>
        <span style="color: #d97706; font-weight:bold;">Est.Beta: ${betaStr}</span>
        <span style="color: #16a34a;">Wind: ${wspdStr}</span>
        <span style="color: #0f172a;">TT: ${ttStr} (${gsStr})</span>
      `;
}

function drawArrow(ctx, x1, y1, x2, y2, color, width) {
  const headlen = 10;
  const angle = Math.atan2(y2 - y1, x2 - x1);
  ctx.save();
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = width;
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x2, y2);
  ctx.stroke();

  // 矢印の先端
  ctx.beginPath();
  ctx.moveTo(x2, y2);
  ctx.lineTo(x2 - headlen * Math.cos(angle - Math.PI / 6), y2 - headlen * Math.sin(angle - Math.PI / 6));
  ctx.lineTo(x2 - headlen * Math.cos(angle + Math.PI / 6), y2 - headlen * Math.sin(angle + Math.PI / 6));
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

// ============================================================
// 計器描画: 人工水平儀 (Artificial Horizon) - 白ベース対応
// ============================================================
function drawArtificialHorizon(d) {
  const canvas = document.getElementById('canvas-horizon');
  // 表示サイズと内部解像度を完全一致させ、楕円歪みを防止
  const rect = canvas.getBoundingClientRect();
  const sz = Math.round(Math.min(rect.width, rect.height) || canvas.clientHeight || canvas.clientWidth);
  if (sz > 0 && (canvas.width !== sz || canvas.height !== sz)) {
    canvas.width = sz;
    canvas.height = sz;
  }
  const ctx = canvas.getContext('2d');
  const w = canvas.width, h = canvas.height;
  ctx.clearRect(0, 0, w, h);

  const cx = w / 2, cy = h / 2;
  const radius = Math.min(w, h) * 0.37;
  const pitchPixelsPerDeg = radius / 18.0;

  ctx.save();
  // 丸型マスク
  ctx.beginPath();
  ctx.arc(cx, cy, radius, 0, Math.PI * 2);
  ctx.clip();

  const hasAttitude = (d.roll !== null && !isNaN(d.roll) && d.pitch !== null && !isNaN(d.pitch));

  if (hasAttitude) {
    const pitchOffset = d.pitch * pitchPixelsPerDeg;

    // ロール回転
    ctx.translate(cx, cy);
    ctx.rotate((-d.roll * Math.PI) / 180);

    // 空 (深みのあるブルー)
    ctx.fillStyle = "#0284c7";
    ctx.fillRect(-w, -h + pitchOffset, w * 2, h);

    // 地面 (ディープアースブラウン)
    ctx.fillStyle = "#78350f";
    ctx.fillRect(-w, pitchOffset, w * 2, h);

    // 水平線 (白実線 + 黒フチで白背景でもくっきり)
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 3.0;
    ctx.beginPath();
    ctx.moveTo(-w, pitchOffset);
    ctx.lineTo(w, pitchOffset);
    ctx.stroke();

    // ピッチ目盛 (±5°, ±10°)
    ctx.lineWidth = 1.8;
    ctx.strokeStyle = "#ffffff";
    for (let p = -20; p <= 20; p += 5) {
      if (p === 0) continue;
      const py = pitchOffset - p * pitchPixelsPerDeg;
      const lineLen = p % 10 === 0 ? radius * 0.45 : radius * 0.22;
      ctx.beginPath();
      ctx.moveTo(-lineLen, py);
      ctx.lineTo(lineLen, py);
      ctx.stroke();
    }
  } else {
    // 姿勢データ欠損時: 偽の水平を出さずグレーアウトして「NO ATTITUDE」表示
    ctx.fillStyle = "#94a3b8";
    ctx.fillRect(0, 0, w, h);
    ctx.fillStyle = "#ffffff";
    ctx.font = "bold 11px sans-serif";
    ctx.textAlign = "center";
    ctx.fillText("NO ATTITUDE DATA", cx, cy - 6);
    ctx.font = "9px monospace";
    ctx.fillText("(NO IMU / ESKF)", cx, cy + 10);
  }

  ctx.restore();

  // 固定機体シンボル (黄色のレティクル + 黒フチ)
  const reticleW = radius * 0.6;
  ctx.save();
  ctx.strokeStyle = "#000000";
  ctx.lineWidth = 4.5;
  ctx.beginPath();
  ctx.moveTo(cx - reticleW, cy); ctx.lineTo(cx - reticleW * 0.3, cy);
  ctx.moveTo(cx - 3, cy); ctx.lineTo(cx + 3, cy);
  ctx.moveTo(cx + reticleW * 0.3, cy); ctx.lineTo(cx + reticleW, cy);
  ctx.stroke();

  ctx.strokeStyle = "#eab308";
  ctx.lineWidth = 2.6;
  ctx.beginPath();
  ctx.moveTo(cx - reticleW, cy); ctx.lineTo(cx - reticleW * 0.3, cy);
  ctx.moveTo(cx - 3, cy); ctx.lineTo(cx + 3, cy);
  ctx.moveTo(cx + reticleW * 0.3, cy); ctx.lineTo(cx + reticleW, cy);
  ctx.stroke();
  ctx.restore();

  // ロールインジケーター外枠リング (濃いグレー)
  ctx.save();
  ctx.strokeStyle = "#64748b";
  ctx.lineWidth = 2.0;
  ctx.beginPath();
  ctx.arc(cx, cy, radius, 0, Math.PI * 2);
  ctx.stroke();
  ctx.restore();

  // スリップ・スキッド・ボール (Turn & Slip Indicator: 横滑り角 Est.Beta)
  const hasBeta = (d.estBeta !== null && !isNaN(d.estBeta));
  const beta = hasBeta ? d.estBeta : 0;
  const tubeW = radius * 0.88;
  const tubeH = 13;
  const tubeY = cy + radius * 0.68;
  const ballR = 4.8;

  ctx.save();
  // チューブ外枠・背景 (半透明の暗色)
  ctx.fillStyle = "rgba(15, 23, 42, 0.50)";
  ctx.strokeStyle = "#cbd5e1";
  ctx.lineWidth = 1.2;
  ctx.beginPath();
  if (ctx.roundRect) {
    ctx.roundRect(cx - tubeW / 2, tubeY - tubeH / 2, tubeW, tubeH, 6);
  } else {
    ctx.rect(cx - tubeW / 2, tubeY - tubeH / 2, tubeW, tubeH);
  }
  ctx.fill();
  ctx.stroke();

  // 中央基準線 (| |)
  ctx.strokeStyle = "#ffffff";
  ctx.lineWidth = 1.5;
  const centerGap = 6.0;
  ctx.beginPath();
  ctx.moveTo(cx - centerGap, tubeY - tubeH / 2 + 1);
  ctx.lineTo(cx - centerGap, tubeY + tubeH / 2 - 1);
  ctx.moveTo(cx + centerGap, tubeY - tubeH / 2 + 1);
  ctx.lineTo(cx + centerGap, tubeY + tubeH / 2 - 1);
  ctx.stroke();

  // ボールの描画 (データあり時のみ描画、欠損時は透明)
  if (hasBeta) {
    const maxDisplace = tubeW / 2 - ballR - 2;
    const ballOffset = Math.max(-maxDisplace, Math.min(maxDisplace, (beta / 6.0) * maxDisplace));
    const ballX = cx + ballOffset;

    ctx.beginPath();
    ctx.arc(ballX, tubeY, ballR, 0, Math.PI * 2);
    ctx.fillStyle = "#ffffff";
    ctx.shadowColor = "rgba(0,0,0,0.5)";
    ctx.shadowBlur = 3;
    ctx.fill();
    ctx.shadowBlur = 0;
    ctx.strokeStyle = "#0f172a";
    ctx.lineWidth = 1.2;
    ctx.stroke();
  }

  // 小さなインジケーターラベル
  ctx.fillStyle = "rgba(255, 255, 255, 0.9)";
  ctx.font = "bold 8px " + (getComputedStyle(document.body).getPropertyValue('--font-mono') || 'monospace');
  ctx.textAlign = "center";
  ctx.fillText(hasBeta ? "SLIP" : "NO SLIP DATA", cx, tubeY - tubeH / 2 - 2);
  ctx.restore();

  // HUDテキスト (下部ステータスバー: データ欠損時は --- を表示)
  const rollStr = (d.roll !== null && !isNaN(d.roll)) ? `${d.roll > 0 ? '+' : ''}${d.roll.toFixed(1)}°` : "---°";
  const pitchStr = (d.pitch !== null && !isNaN(d.pitch)) ? `${d.pitch > 0 ? '+' : ''}${d.pitch.toFixed(1)}°` : "---°";
  const betaStr = (d.estBeta !== null && !isNaN(d.estBeta)) ? `${d.estBeta > 0 ? '+' : ''}${d.estBeta.toFixed(1)}°` : "--.-°";
  const pavgStr = (d.pavg !== null && !isNaN(d.pavg)) ? `${d.pavg.toFixed(1)}°` : "---°";
  const rtrimStr = (d.rtrim !== null && !isNaN(d.rtrim)) ? `${d.rtrim > 0 ? '+' : ''}${d.rtrim.toFixed(1)}°` : "---°";

  document.getElementById('hud-attitude-text').innerHTML = `
        <span style="color: #dc2626;">Roll: ${rollStr}</span>
        <span style="color: #7c3aed;">Pitch: ${pitchStr}</span>
        <span style="color: #d97706; font-weight:bold;">Est.Beta: ${betaStr}</span>
        <span style="color: #64748b;">AvgP: ${pavgStr}</span>
        <span style="color: #475569;">RTrim: ${rtrimStr}</span>
      `;
}

// ============================================================
// スクロールグラフ (ストリップチャート) 描画 - 白ベース対応
// ============================================================
function drawSpeedChart(t) {
  const canvas = document.getElementById('canvas-chart-speed');
  const ctx = canvas.getContext('2d');
  const w = canvas.width, h = canvas.height;
  ctx.clearRect(0, 0, w, h);

  const winSec = 10.0;
  const yMin = 3.0, yMax = 10.0;

  drawChartGrid(ctx, w, h, winSec, yMin, yMax, 1.0, "m/s");

  ctx.save();

  // G/S (黒実線)
  ctx.strokeStyle = "#0f172a";
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  let started = false;
  for (let dt = -winSec; dt <= 0; dt += 0.2) {
    let sample = interpolateAt(t + dt);
    if (!sample || sample.gs === null || isNaN(sample.gs)) {
      started = false;
      continue;
    }
    let x = w + (dt / winSec) * w;
    let y = h - ((sample.gs - yMin) / (yMax - yMin)) * h;
    if (!started) { ctx.moveTo(x, y); started = true; }
    else ctx.lineTo(x, y);
  }
  ctx.stroke();

  // TAS (濃いオレンジ破線)
  ctx.strokeStyle = "#ea580c";
  ctx.lineWidth = 2.2;
  ctx.setLineDash([4, 3]);
  ctx.beginPath();
  started = false;
  for (let dt = -winSec; dt <= 0; dt += 0.2) {
    let sample = interpolateAt(t + dt);
    if (!sample) { started = false; continue; }
    let tasEst = null;
    if (sample.csvTas !== null && !isNaN(sample.csvTas)) {
      tasEst = sample.csvTas;
    } else if (sample.gs !== null && sample.yaw !== null && sample.wdir !== null && sample.wspd !== null) {
      tasEst = Math.max(0, sample.gs - (Math.cos((sample.yaw - sample.wdir) * Math.PI / 180) * sample.wspd));
    }
    if (tasEst === null || isNaN(tasEst)) {
      started = false;
      continue;
    }
    let x = w + (dt / winSec) * w;
    let y = h - ((tasEst - yMin) / (yMax - yMin)) * h;
    if (!started) { ctx.moveTo(x, y); started = true; }
    else ctx.lineTo(x, y);
  }
  ctx.stroke();
  ctx.restore();
}

function drawRollPitchChart(t) {
  const canvas = document.getElementById('canvas-chart-rp');
  const ctx = canvas.getContext('2d');
  const w = canvas.width, h = canvas.height;
  ctx.clearRect(0, 0, w, h);

  const winSec = 10.0;
  const yMin = -8.0, yMax = 8.0;

  drawChartGrid(ctx, w, h, winSec, yMin, yMax, 4.0, "deg");

  // 水平0°基準線 (見やすいグレー実線)
  const zeroY = h - ((0 - yMin) / (yMax - yMin)) * h;
  ctx.save();
  ctx.strokeStyle = "#64748b";
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(0, zeroY); ctx.lineTo(w, zeroY);
  ctx.stroke();

  // Roll (赤実線)
  ctx.strokeStyle = "#dc2626";
  ctx.lineWidth = 2.2;
  ctx.beginPath();
  let started = false;
  for (let dt = -winSec; dt <= 0; dt += 0.2) {
    let sample = interpolateAt(t + dt);
    if (!sample || sample.roll === null || isNaN(sample.roll)) {
      started = false;
      continue;
    }
    let x = w + (dt / winSec) * w;
    let y = h - ((sample.roll - yMin) / (yMax - yMin)) * h;
    if (!started) { ctx.moveTo(x, y); started = true; }
    else ctx.lineTo(x, y);
  }
  ctx.stroke();

  // Pitch (紫実線)
  ctx.strokeStyle = "#7c3aed";
  ctx.lineWidth = 2.2;
  ctx.beginPath();
  started = false;
  for (let dt = -winSec; dt <= 0; dt += 0.2) {
    let sample = interpolateAt(t + dt);
    if (!sample || sample.pitch === null || isNaN(sample.pitch)) {
      started = false;
      continue;
    }
    let x = w + (dt / winSec) * w;
    let y = h - ((sample.pitch - yMin) / (yMax - yMin)) * h;
    if (!started) { ctx.moveTo(x, y); started = true; }
    else ctx.lineTo(x, y);
  }
  ctx.stroke();

  // Est.Beta (アンバー破線: 横滑り角)
  ctx.strokeStyle = "#d97706";
  ctx.lineWidth = 2.0;
  ctx.setLineDash([4, 3]);
  ctx.beginPath();
  started = false;
  for (let dt = -winSec; dt <= 0; dt += 0.2) {
    let sample = interpolateAt(t + dt);
    if (!sample || sample.estBeta === null || isNaN(sample.estBeta)) {
      started = false;
      continue;
    }
    let betaVal = sample.estBeta;
    let x = w + (dt / winSec) * w;
    let y = h - ((betaVal - yMin) / (yMax - yMin)) * h;
    if (!started) { ctx.moveTo(x, y); started = true; }
    else ctx.lineTo(x, y);
  }
  ctx.stroke();
  ctx.restore();
}

// ============================================================
// 高度 Alt [m] ＆ 昇降速度 VSI [m/s] 描画 (オートスケール＆2軸対応)
// ============================================================
function drawAltChart(t) {
  const canvas = document.getElementById('canvas-chart-alt');
  const ctx = canvas.getContext('2d');
  const w = canvas.width, h = canvas.height;
  ctx.clearRect(0, 0, w, h);

  const winSec = 20.0;
  const altMin = flightAltMin;
  const altMax = flightAltMax;
  const vsiMin = -2.0, vsiMax = 2.0;       // 昇降速度スケール [-2 .. +2 m/s]

  const step = Math.max(1, Math.round((altMax - altMin) / 4));
  drawChartGrid(ctx, w, h, winSec, altMin, altMax, step, "m");

  // VSI 目盛り表示 (右端)
  ctx.save();
  ctx.fillStyle = "#16a34a";
  ctx.font = "bold 9px monospace";
  ctx.fillText("+2m/s", w - 42, 12);
  ctx.fillText(" 0m/s", w - 42, h / 2 - 2);
  ctx.fillText("-2m/s", w - 42, h - 4);

  // VSI 0 m/s 基準線 (中央・濃い破線)
  const zeroVsiY = h / 2;
  ctx.strokeStyle = "#64748b";
  ctx.lineWidth = 1.2;
  ctx.setLineDash([4, 3]);
  ctx.beginPath();
  ctx.moveTo(0, zeroVsiY); ctx.lineTo(w, zeroVsiY);
  ctx.stroke();

  // 1. 高度 Alt [m] (シアンブルー実線)
  ctx.setLineDash([]);
  ctx.strokeStyle = "#0284c7";
  ctx.lineWidth = 2.8;
  ctx.beginPath();
  let started = false;
  for (let dt = -winSec; dt <= 0; dt += 0.25) {
    let sample = interpolateAt(t + dt);
    if (!sample || sample.kf_alt === null || isNaN(sample.kf_alt)) {
      started = false;
      continue;
    }
    let x = w + (dt / winSec) * w;
    let y = h - ((sample.kf_alt - altMin) / (altMax - altMin)) * h;
    if (!started) { ctx.moveTo(x, y); started = true; }
    else ctx.lineTo(x, y);
  }
  ctx.stroke();

  // 2. 昇降速度 VSI [m/s] (濃い緑色破線)
  ctx.strokeStyle = "#16a34a";
  ctx.lineWidth = 2.2;
  ctx.setLineDash([4, 3]);
  ctx.beginPath();
  started = false;
  for (let dt = -winSec; dt <= 0; dt += 0.25) {
    let sample = interpolateAt(t + dt);
    if (!sample || sample.kf_vs === null || isNaN(sample.kf_vs)) {
      started = false;
      continue;
    }
    let x = w + (dt / winSec) * w;
    let clampedVs = Math.max(vsiMin, Math.min(vsiMax, sample.kf_vs));
    let y = h - ((clampedVs - vsiMin) / (vsiMax - vsiMin)) * h;
    if (!started) { ctx.moveTo(x, y); started = true; }
    else ctx.lineTo(x, y);
  }
  ctx.stroke();

  ctx.restore();
}

function drawChartGrid(ctx, w, h, winSec, yMin, yMax, yStep, unit) {
  ctx.save();
  // チャート背景を薄いグレーで塗って白パネルと境界をつける
  ctx.fillStyle = "#f8fafc";
  ctx.fillRect(0, 0, w, h);

  // グリッド線 (はっきり見えるスレートグレー)
  ctx.strokeStyle = "#cbd5e1";
  ctx.lineWidth = 1.0;
  ctx.fillStyle = "#334155";
  ctx.font = "bold 10px monospace";

  // 縦グリッド (時間目盛)
  for (let sec = 2; sec < winSec; sec += 2) {
    let x = w - (sec / winSec) * w;
    ctx.beginPath();
    ctx.moveTo(x, 0); ctx.lineTo(x, h);
    ctx.stroke();
    ctx.fillText(`-${sec}s`, x + 3, h - 4);
  }

  // 横グリッド (数値目盛)
  for (let val = yMin; val <= yMax; val += yStep) {
    let y = h - ((val - yMin) / (yMax - yMin)) * h;
    ctx.beginPath();
    ctx.moveTo(0, y); ctx.lineTo(w, y);
    ctx.stroke();
    ctx.fillText(`${val}`, 4, y - 2);
  }
  ctx.restore();
}
