// ============================================================
// File    : tools/repons/live_map.js
// Project : PONS v7.1 / v7.2 Re:PONS
// Role    : ライブ配信の画面の地図。本体と同じベクター地図（vectormap_hires.js）を読み、
//           src/vectormap.cpp と display_tft.cpp（多角形の塗り・線のクリップ・地図の線・緯度経度→画面）の
//           写しで backscreen（240x240 のスプライト）へ描く。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026-11-25
// ============================================================
//
// ★ 地図データは OpenStreetMap 由来（ODbL 1.0）。配信の帯に "© OpenStreetMap contributors" を出す（live.js）。
// ★ 地図ファイルの形は tools/vectormap/export_repons_map.py の頭。作り直すのは build_vectormap.py --variant hires。
// ★ 地図は <script> で読む（vectormap_hires.js。中身は gzip の base64）。file:// で開いても読めるように
//   （ブラウザは file:// のページから fetch でファイルを読ませない。2026-11-25 までは fetch で、file:// だと白いままだった）。
(function (global) {
  'use strict';
  const f32 = Math.fround;
  const PI = 3.1415926535897932384626433832795;
  const DEG_TO_RAD = 0.017453292519943295769236907684886;
  const BACKSCREEN_SIZE = 240;

  // ---- 色（vectormap.cpp）----
  const VM_LANDMASS = 0, VM_WATER = 1, VM_PARK = 2, VM_MOTORWAY = 3, VM_TRUNK = 4, VM_RAIL = 5, VM_BUILDING = 6, VM_STREET = 7;
  const VMCOL_LAND = 0xffff, VMCOL_WATER = 0xb71f, VMCOL_PARK = 0xd77a, VMCOL_BUILDING = 0xded9,
    VMCOL_STREET = 0xd69a, VMCOL_MOTORWAY = 0xfeae, VMCOL_TRUNK = 0xFFE0, VMCOL_RAIL = 0x9cd3;
  const VM_LINE_WIDTH = [0, 0, 0, 3, 2, 1, 0, 1];
  const VM_COLOR = [VMCOL_LAND, VMCOL_WATER, VMCOL_PARK, VMCOL_MOTORWAY, VMCOL_TRUNK, VMCOL_RAIL, VMCOL_BUILDING, VMCOL_STREET];
  const VM_DRAW_ORDER = [VM_LANDMASS, VM_PARK, VM_WATER, VM_BUILDING, VM_STREET, VM_RAIL, VM_TRUNK, VM_MOTORWAY];
  const isArea = (c) => c <= VM_PARK || c === VM_BUILDING;
  const VM_MAX_PTS = 2048, VM_MAX_RINGS = 64, VM_VIEW_RADIUS_PX = 216.0;

  // ---- 地図ファイルを読む ----
  let MAP = null;
  async function gunzipIfNeeded(buf) {
    const u = new Uint8Array(buf);
    if (u[0] === 0x1f && u[1] === 0x8b) {
      if (!('DecompressionStream' in global)) throw new Error('このブラウザは gzip を開けません（Chrome を使ってください）');
      const ds = new DecompressionStream('gzip');
      const stream = new Blob([u]).stream().pipeThrough(ds);
      return await new Response(stream).arrayBuffer();
    }
    return buf;
  }
  function parse(buf) {
    const dv = new DataView(buf);
    const magic = String.fromCharCode(dv.getUint8(0), dv.getUint8(1), dv.getUint8(2), dv.getUint8(3));
    if (magic !== 'PVM1') throw new Error('地図ファイルの形が違います（' + magic + '）');
    let p = 4;
    const L = dv.getUint16(p, true), nt = dv.getUint16(p + 2, true), nc = dv.getUint16(p + 4, true); p += 8;
    const m = { L, unit: [], tsize: [], minscale: [], seabg: [], cfirst: [], cn: [], covers: [], tiles: [] };
    for (let i = 0; i < L; i++) { m.unit.push(dv.getInt32(p, true)); p += 4; }
    for (let i = 0; i < L; i++) { m.tsize.push(dv.getInt32(p, true)); p += 4; }
    for (let i = 0; i < L; i++) { m.minscale.push(dv.getFloat32(p, true)); p += 4; }
    for (let i = 0; i < L; i++) { m.seabg.push(dv.getUint8(p) !== 0); p += 1; }
    while (p % 4) p++;
    for (let i = 0; i < L; i++) { m.cfirst.push(dv.getUint16(p, true)); p += 2; }
    for (let i = 0; i < L; i++) { m.cn.push(dv.getUint16(p, true)); p += 2; }
    while (p % 4) p++;
    for (let i = 0; i < nc; i++) { m.covers.push([dv.getInt32(p, true), dv.getInt32(p + 4, true), dv.getInt32(p + 8, true), dv.getInt32(p + 12, true)]); p += 16; }
    for (let i = 0; i < nt; i++) {
      m.tiles.push({ lat: dv.getInt32(p, true), lon: dv.getInt32(p + 4, true), off: dv.getUint32(p + 8, true), len: dv.getUint32(p + 12, true), lod: dv.getUint8(p + 16) });
      p += 20;
    }
    const bs = dv.getUint32(p, true); p += 4;
    m.blob = new DataView(buf, p, bs); p += bs;
    const rdS = () => { const n = dv.getUint16(p, true); p += 2; const s = new TextDecoder().decode(new Uint8Array(buf, p, n)); p += n; return s; };
    m.attribution = rdS();
    m.sourceDate = rdS();
    return m;
  }
  async function loadFromArrayBuffer(buf) {
    MAP = parse(await gunzipIfNeeded(buf));
    return MAP;
  }
  // src は vectormap_hires.js（?v= 付き）。読み終えたら base64 の文字列は捨てる（6MB あるので持ち続けない）
  function loadFromScript(src) {
    return new Promise((resolve, reject) => {
      const s = document.createElement('script');
      s.src = src;
      s.onload = () => {
        const b64 = global.LIVE_VECTORMAP_GZ_B64;
        global.LIVE_VECTORMAP_GZ_B64 = null;
        s.remove();
        if (typeof b64 !== 'string') { reject(new Error('地図ファイルの中身が違います')); return; }
        try {
          const bin = atob(b64);   // 改行は atob が飛ばす
          const u = new Uint8Array(bin.length);
          for (let i = 0; i < bin.length; i++) u[i] = bin.charCodeAt(i);
          loadFromArrayBuffer(u.buffer).then(resolve, reject);
        } catch (e) { reject(e); }
      };
      s.onerror = () => { s.remove(); reject(new Error(src.split('?')[0] + ' が見つかりません')); };
      document.head.appendChild(s);
    });
  }

  // ---- 線のクリップ（display_tft.cpp の clip_line_to_screen。double で計算して四捨五入）----
  const CLIP_MARGIN = 4;
  const lroundD = (x) => (x < 0 ? -Math.round(-x) : Math.round(x));
  function clipOutcode(x, y) {
    const lo = -CLIP_MARGIN, hi = BACKSCREEN_SIZE - 1 + CLIP_MARGIN;
    let c = 0;
    if (x < lo) c |= 1; else if (x > hi) c |= 2;
    if (y < lo) c |= 4; else if (y > hi) c |= 8;
    return c;
  }
  // [x0,y0,x1,y1] を返す。完全に外なら null
  function clipLine(x0, y0, x1, y1) {
    const lo = -CLIP_MARGIN, hi = BACKSCREEN_SIZE - 1 + CLIP_MARGIN;
    let c0 = clipOutcode(x0, y0), c1 = clipOutcode(x1, y1);
    for (let guard = 0; guard < 8; guard++) {
      if (!(c0 | c1)) break;
      if (c0 & c1) return null;
      const c = c0 ? c0 : c1;
      let x = 0, y = 0;
      if (c & 8) { x = x0 + (x1 - x0) * (hi - y0) / (y1 - y0); y = hi; }
      else if (c & 4) { x = x0 + (x1 - x0) * (lo - y0) / (y1 - y0); y = lo; }
      else if (c & 2) { y = y0 + (y1 - y0) * (hi - x0) / (x1 - x0); x = hi; }
      else { y = y0 + (y1 - y0) * (lo - x0) / (x1 - x0); x = lo; }
      if (c === c0) { x0 = x; y0 = y; c0 = clipOutcode(x0, y0); }
      else { x1 = x; y1 = y; c1 = clipOutcode(x1, y1); }
    }
    return [lroundD(x0), lroundD(y0), lroundD(x1), lroundD(y1)];
  }
  // 非アンチエイリアスの太線（display_tft.cpp の draw_map_line）
  function drawMapLine(bs, x0, y0, x1, y1, w, col) {
    const c = clipLine(x0, y0, x1, y1);
    if (!c) return;
    [x0, y0, x1, y1] = c;
    bs.drawLine(x0, y0, x1, y1, col);
    if (w <= 1) return;
    const steep = Math.abs(y1 - y0) > Math.abs(x1 - x0);
    for (let o = 1; o <= ((w - 1) / 2 | 0); o++) {
      if (steep) { bs.drawLine(x0 - o, y0, x1 - o, y1, col); bs.drawLine(x0 + o, y0, x1 + o, y1, col); }
      else { bs.drawLine(x0, y0 - o, x1, y1 - o, col); bs.drawLine(x0, y0 + o, x1, y1 + o, col); }
    }
    if (w % 2 === 0) {
      if (steep) bs.drawLine(x0 + (w / 2 | 0), y0, x1 + (w / 2 | 0), y1, col);
      else bs.drawLine(x0, y0 + (w / 2 | 0), x1, y1 + (w / 2 | 0), col);
    }
  }
  function drawClippedLine(bs, x0, y0, x1, y1, col) {
    const c = clipLine(x0, y0, x1, y1);
    if (c) bs.drawLine(c[0], c[1], c[2], c[3], col);
  }
  function drawClippedWideLine(bs, x0, y0, x1, y1, w, col) {
    const c = clipLine(x0, y0, x1, y1);
    if (c) bs.drawWideLine(c[0], c[1], c[2], c[3], w, col);
  }

  // ---- 多角形の even-odd 塗り（display_tft.cpp の fill_polygon_evenodd）----
  const VM_MAX_EDGES = 2048;
  const eX = new Float32Array(VM_MAX_EDGES), eD = new Float32Array(VM_MAX_EDGES);
  const eBot = new Int16Array(VM_MAX_EDGES), eNext = new Int16Array(VM_MAX_EDGES);
  const ET = new Int16Array(BACKSCREEN_SIZE);
  const AET = new Int16Array(VM_MAX_EDGES / 4);
  function fillPolygonEvenOdd(bs, xs, ys, ringStart, nrings, color) {
    const H = BACKSCREEN_SIZE, W = BACKSCREEN_SIZE;
    if (nrings === 0) return true;
    if (ringStart[nrings] > VM_MAX_EDGES) return false;
    ET.fill(-1);
    let nedges = 0, yMin = H, yMax = 0;
    for (let r = 0; r < nrings; r++) {
      const beg = ringStart[r], end = ringStart[r + 1], n = end - beg;
      if (n < 3) continue;
      for (let i = 0; i < n; i++) {
        const j = (i + 1 === n) ? 0 : i + 1;
        let x0 = xs[beg + i], y0 = ys[beg + i], x1 = xs[beg + j], y1 = ys[beg + j];
        if (y0 === y1) continue;
        if (y0 > y1) { let t = x0; x0 = x1; x1 = t; t = y0; y0 = y1; y1 = t; }
        if (y1 <= 0 || y0 >= H) continue;
        const dxdy = f32(f32(x1 - x0) / f32(y1 - y0));
        let xTop = f32(x0);
        let yTop = y0;
        if (yTop < 0) { xTop = f32(xTop + f32(dxdy * f32(0 - yTop))); yTop = 0; }
        const yBot = (y1 > H) ? H : y1;
        if (yTop >= yBot) continue;
        if (nedges >= VM_MAX_EDGES) return false;
        eX[nedges] = xTop; eD[nedges] = dxdy; eBot[nedges] = yBot; eNext[nedges] = ET[yTop];
        ET[yTop] = nedges;
        nedges++;
        if (yTop < yMin) yMin = yTop;
        if (yBot > yMax) yMax = yBot;
      }
    }
    if (nedges === 0) return true;
    let aetN = 0;
    for (let y = yMin; y < yMax; y++) {
      for (let e = ET[y]; e !== -1; e = eNext[e]) if (aetN < AET.length) AET[aetN++] = e;
      let w = 0;
      for (let i = 0; i < aetN; i++) if (eBot[AET[i]] > y) AET[w++] = AET[i];
      aetN = w;
      for (let i = 1; i < aetN; i++) {
        const key = AET[i], kx = eX[key];
        let j = i - 1;
        while (j >= 0 && eX[AET[j]] > kx) { AET[j + 1] = AET[j]; j--; }
        AET[j + 1] = key;
      }
      for (let i = 0; i + 1 < aetN; i += 2) {
        let xa = Math.ceil(eX[AET[i]]) | 0;
        let xb = (Math.ceil(eX[AET[i + 1]]) | 0) - 1;
        if (xb < 0 || xa >= W || xa > xb) continue;
        if (xa < 0) xa = 0;
        if (xb >= W) xb = W - 1;
        bs.drawFastHLine(xa, y, xb - xa + 1, color);
      }
      for (let i = 0; i < aetN; i++) eX[AET[i]] = f32(eX[AET[i]] + eD[AET[i]]);
    }
    return true;
  }

  // ---- ベクター地図（src/vectormap.cpp の写し）----
  function makeProj(cLat, cLon, scale, up, cy) {
    const c = cLat * DEG_TO_RAD;
    const sec = 1.0 / Math.cos(c);
    const tn = Math.tan(c);
    const a = f32(f32(up) * DEG_TO_RAD);
    return {
      cLat: f32(cLat), cLon: f32(cLon),
      kx: f32(111.321 * f32(scale)),
      ky: f32(6378.22347118 * DEG_TO_RAD * sec * f32(scale)),
      c2: f32(0.5 * tn * DEG_TO_RAD),
      c3: f32((tn * tn + sec * sec) / 6.0 * DEG_TO_RAD * DEG_TO_RAD),
      ca: f32(Math.cos(a)), sa: f32(Math.sin(a)), cy,
    };
  }
  function project(p, lat, lon, out, k) {
    const d = f32(lat - p.cLat);
    const x = f32(f32(lon - p.cLon) * p.kx);
    const y = f32(f32(p.ky * d) * f32(1.0 + f32(d * f32(p.c2 + f32(d * p.c3)))));
    const rx = f32(f32(x * p.ca) - f32(y * p.sa));
    const ry = f32(f32(x * p.sa) + f32(y * p.ca));
    let sx = (BACKSCREEN_SIZE / 2) + Math.trunc(rx);
    let sy = p.cy - Math.trunc(ry);
    if (sx < -20000) sx = -20000; else if (sx > 20000) sx = 20000;
    if (sy < -20000) sy = -20000; else if (sy > 20000) sy = 20000;
    out.x[k] = sx; out.y[k] = sy;
  }
  function selectLod(scale) {
    for (let lod = 0; lod < MAP.L; lod++) if (scale >= MAP.minscale[lod]) return lod;
    return MAP.L - 1;
  }
  function viewCovered(lod, latMin, latMax, lonMin, lonMax) {
    const first = MAP.cfirst[lod];
    for (let k = 0; k < MAP.cn[lod]; k++) {
      const r = MAP.covers[first + k];
      if (latMin >= r[0] && lonMin >= r[1] && latMax <= r[2] && lonMax <= r[3]) return true;
    }
    return false;
  }
  const PT = { x: new Int16Array(VM_MAX_PTS), y: new Int16Array(VM_MAX_PTS) };
  const RS = new Uint16Array(VM_MAX_RINGS + 1);
  const BB = { x: new Int16Array(4), y: new Int16Array(4) };
  function drawRecord(bs, p, t, lod, proj, clsFilter) {
    const B = MAP.blob;
    const cls = B.getUint8(p), nrings = B.getUint8(p + 1);
    let total = 0;
    for (let r = 0; r < nrings; r++) total += B.getUint16(p + 10 + 2 * r, true);
    const pts = p + 10 + 2 * nrings;
    const next = pts + total * 4;
    if (cls !== clsFilter) return next;
    if (nrings === 0 || nrings > VM_MAX_RINGS) return next;
    if (total === 0 || total > VM_MAX_PTS) return next;
    const unit = MAP.unit[lod];
    {
      const b0 = B.getInt16(p + 2, true), b1 = B.getInt16(p + 4, true), b2 = B.getInt16(p + 6, true), b3 = B.getInt16(p + 8, true);
      const la = [b0, b0, b2, b2], lo = [b1, b3, b1, b3];
      for (let i = 0; i < 4; i++) {
        const lat = f32((t.lat + Math.imul(la[i], unit)) * 1e-7);
        const lon = f32((t.lon + Math.imul(lo[i], unit)) * 1e-7);
        project(proj, lat, lon, BB, i);
      }
      let minx = BB.x[0], maxx = BB.x[0], miny = BB.y[0], maxy = BB.y[0];
      for (let i = 1; i < 4; i++) {
        if (BB.x[i] < minx) minx = BB.x[i]; if (BB.x[i] > maxx) maxx = BB.x[i];
        if (BB.y[i] < miny) miny = BB.y[i]; if (BB.y[i] > maxy) maxy = BB.y[i];
      }
      if (maxx < 0 || maxy < 0 || minx >= BACKSCREEN_SIZE || miny >= BACKSCREEN_SIZE) return next;
    }
    let k = 0;
    RS[0] = 0;
    for (let r = 0; r < nrings; r++) {
      const n = B.getUint16(p + 10 + 2 * r, true);
      for (let i = 0; i < n; i++) {
        const lat = f32((t.lat + Math.imul(B.getInt16(pts + k * 4, true), unit)) * 1e-7);
        const lon = f32((t.lon + Math.imul(B.getInt16(pts + k * 4 + 2, true), unit)) * 1e-7);
        project(proj, lat, lon, PT, k);
        k++;
      }
      RS[r + 1] = k;
    }
    if (isArea(cls)) {
      fillPolygonEvenOdd(bs, PT.x, PT.y, RS, nrings, VM_COLOR[cls]);
    } else {
      const w = VM_LINE_WIDTH[cls], col = VM_COLOR[cls];
      for (let r = 0; r < nrings; r++)
        for (let i = RS[r]; i + 1 < RS[r + 1]; i++) drawMapLine(bs, PT.x[i], PT.y[i], PT.x[i + 1], PT.y[i + 1], w, col);
    }
    return next;
  }
  // draw_vectormap(center_lat, center_lon, scale, up)。cy は自機の画面 Y（TRACKUP 180 / NORTHUP 120）
  function drawVectorMap(bs, centerLat, centerLon, scale, up, cy) {
    if (!MAP || MAP.tiles.length === 0) { bs.fillSprite(0xFFFF); return; }
    scale = f32(scale); up = f32(up);
    const proj = makeProj(centerLat, centerLon, scale, up, cy);
    const dlat = VM_VIEW_RADIUS_PX / proj.ky;
    const dlon = VM_VIEW_RADIUS_PX / proj.kx;
    const latMin = Math.trunc((centerLat - dlat) * 1e7), latMax = Math.trunc((centerLat + dlat) * 1e7);
    const lonMin = Math.trunc((centerLon - dlon) * 1e7), lonMax = Math.trunc((centerLon + dlon) * 1e7);
    let lod = selectLod(scale);
    while (lod < MAP.L - 1 && !viewCovered(lod, latMin, latMax, lonMin, lonMax)) lod++;
    bs.fillSprite(MAP.seabg[lod] ? VMCOL_WATER : 0xFFFF);
    const tsize = MAP.tsize[lod];
    for (let ci = 0; ci < VM_DRAW_ORDER.length; ci++) {
      const cls = VM_DRAW_ORDER[ci];
      for (const t of MAP.tiles) {
        if (t.lod !== lod) continue;
        if (t.lat > latMax || t.lat + tsize < latMin) continue;
        if (t.lon > lonMax || t.lon + tsize < lonMin) continue;
        let p = t.off;
        const end = t.off + t.len;
        while (p < end) p = drawRecord(bs, p, t, lod, proj, cls);
      }
    }
  }

  // ---- 緯度経度 → backscreen の座標（display_tft.cpp の latLonToXY。float の引数）----
  function latitudeToMercatorY(lat) { return Math.log(Math.tan(PI / 4.0 + (lat * PI / 180.0) / 2.0)); }
  function latLonToXY(lat, lon, cLat, cLon, scale, up, selfCy) {
    lat = f32(lat); lon = f32(lon); cLat = f32(cLat); cLon = f32(cLon); scale = f32(scale); up = f32(up);
    let xDist = f32(f32(lon - cLon) * 111.321);
    const yA = latitudeToMercatorY(cLat), yB = latitudeToMercatorY(lat);
    let yDist = (yB - yA) * 6378.22347118;
    xDist = f32(xDist * scale);
    yDist = yDist * scale;
    const a = f32(up * DEG_TO_RAD);
    const ca = f32(Math.cos(a)), sa = f32(Math.sin(a));
    const rx = f32(f32(xDist * ca) - yDist * sa);
    const ry = f32(f32(xDist * sa) + yDist * ca);
    return { x: (BACKSCREEN_SIZE / 2) + Math.trunc(rx), y: selfCy - Math.trunc(ry) };
  }
  const isOutside = (p) => p.x < 0 || p.x >= BACKSCREEN_SIZE || p.y < 0 || p.y >= BACKSCREEN_SIZE;

  // ---- 球面の計算（src/navdata.cpp・display_tft.cpp）----
  const deg2rad = (d) => d * PI / 180.0;
  const rad2deg = (r) => r * (180.0 / PI);
  function calculateDistanceKm(lat1, lon1, lat2, lon2) {
    lat1 = deg2rad(lat1); lon1 = deg2rad(lon1); lat2 = deg2rad(lat2); lon2 = deg2rad(lon2);
    const dlon = lon2 - lon1, dlat = lat2 - lat1;
    const a = Math.pow(Math.sin(dlat / 2), 2) + Math.cos(lat1) * Math.cos(lat2) * Math.pow(Math.sin(dlon / 2), 2);
    return 6371.0 * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  }
  function calculateTrueCourseRad(lat1, lon1, lat2, lon2) {
    const dl = lon2 - lon1;
    return Math.atan2(Math.sin(dl) * Math.cos(lat2), Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(dl));
  }
  function calculatePointC(lat1, lon1, lat2, lon2, distance) {
    const R = 6371.0;
    lat1 = deg2rad(lat1); lon1 = deg2rad(lon1); lat2 = deg2rad(lat2); lon2 = deg2rad(lon2);
    const d = distance / R;
    const bearing = calculateTrueCourseRad(lat2, lon2, lat1, lon1) + PI;
    let lat3 = Math.asin(Math.sin(lat2) * Math.cos(d) + Math.cos(lat2) * Math.sin(d) * Math.cos(bearing));
    let lon3 = lon2 + Math.atan2(Math.sin(bearing) * Math.sin(d) * Math.cos(lat2), Math.cos(d) - Math.sin(lat2) * Math.sin(lat3));
    return [rad2deg(lat3), rad2deg(lon3)];
  }
  function calculatePointD(lat1, lon1, lat2, lon2, distance) {
    const R = 6371.0;
    lat1 = deg2rad(lat1); lon1 = deg2rad(lon1); lat2 = deg2rad(lat2); lon2 = deg2rad(lon2);
    const d = distance / R;
    if (lat1 === lat2 && lon1 === lon2) return [rad2deg(lat1), rad2deg(lon1)];
    const bearing = calculateTrueCourseRad(lat1, lon1, lat2, lon2);
    const lat3 = Math.asin(Math.sin(lat1) * Math.cos(d) + Math.cos(lat1) * Math.sin(d) * Math.cos(bearing));
    const lon3 = lon1 + Math.atan2(Math.sin(bearing) * Math.sin(d) * Math.cos(lat1), Math.cos(d) - Math.sin(lat1) * Math.sin(lat3));
    return [rad2deg(lat3), rad2deg(lon3)];
  }
  function calculatePointFromBearing(lat1, lon1, bearingRad, distance) {
    const R = 6371.0;
    lat1 = deg2rad(lat1); lon1 = deg2rad(lon1);
    const d = distance / R;
    const lat2 = Math.asin(Math.sin(lat1) * Math.cos(d) + Math.cos(lat1) * Math.sin(d) * Math.cos(bearingRad));
    const lon2 = lon1 + Math.atan2(Math.sin(bearingRad) * Math.sin(d) * Math.cos(lat1), Math.cos(d) - Math.sin(lat1) * Math.sin(lat2));
    return [rad2deg(lat2), rad2deg(lon2)];
  }
  function centerlineBearingRad(pl) {
    const bn = calculateTrueCourseRad(deg2rad(pl.plaLat), deg2rad(pl.plaLon), deg2rad(pl.nLat), deg2rad(pl.nLon));
    const bw = calculateTrueCourseRad(deg2rad(pl.plaLat), deg2rad(pl.plaLon), deg2rad(pl.sLat), deg2rad(pl.sLon));
    return Math.atan2(Math.sin(bn) + Math.sin(bw), Math.cos(bn) + Math.cos(bw));
  }

  global.LiveMap = {
    loadFromScript, drawVectorMap, latLonToXY, isOutside, clipLine,
    drawMapLine, drawClippedLine, drawClippedWideLine, fillPolygonEvenOdd,
    calculateDistanceKm, calculateTrueCourseRad, calculatePointC, calculatePointD, calculatePointFromBearing,
    centerlineBearingRad, deg2rad, rad2deg, PI, DEG_TO_RAD,
  };
})(typeof window !== 'undefined' ? window : globalThis);
