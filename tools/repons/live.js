// ============================================================
// File    : tools/repons/live.js
// Project : PONS v7.1 / v7.2 Re:PONS
// Role    : ライブ配信の窓（live.html）。受信機と USB でつなぎ、PONS:LIVE,ON で流れてくる行を読み、
//           機体の画面（live_screen.js）を描いて、ボートの側の情報の帯を付ける。記録の保存と再生、数字の表も。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026-11-26
// ============================================================
//
// ★ 設計は docs/pons_live_stream_plan.md、行の形は docs/pons_webserial_protocol.md §4 (9)。
// ★ **PONS:CONNECT は送らない**（PC 接続の画面に入ると受信機の received/ の記録が止まる）。PONS:LIVE,ON と PONS:TIME だけ。
//   PONS:TIME は受信機から返事が来てから送る（起動中の受信機へ先に送ると、起動画面の間に時刻が古くなったまま GNSS へ入る）。
// ★ LinkTelem の並び（decodeLT）は lora_link/link_proto.h の LinkTelem と同じにする。並びを変えたら LINK_PROTO_VER が
//   上がるので、合わなければ描かない（PONS:LIVE,OK の版・大きさで止める）。tools/repons/test/ の照合を回すこと。
// ★ ボートの値を機体の値として出さない（設計書 §1.3）。機体の画面には LT・LN だけを使い、LS（受信機の状態）は帯にだけ出す。
// ★ 届いたフレームをすぐ描く（補間しない。設計書 §5.5）。
(function (global) {
  'use strict';
  const LIVE_FMT_VER = 1, LINK_PROTO_VER = 10, LINKTELEM_SIZE = 69;
  const RHAVE = { GS: 0x0001, TTRACK: 0x0002, GNSSALT: 0x0004, KFALT: 0x0008, KFVS: 0x0010, PRESS: 0x0020, DATE: 0x0040, NUMSAT: 0x0080,
    ATT: 0x0200, ATT_YAW: 0x0400, ATT_AVG: 0x0800, ATT_TRIM: 0x1000, ATT_WIND: 0x2000, TAS: 0x4000, ALPHA: 0x8000 };
  const W2S = { DATE: 0, WIND: 1, TRIM: 2, HACC: 3, ALTV: 4, WIND2: 5, PRESS: 6, NAV: 7 };
  const f32 = Math.fround;
  const NOSIG_LT_MS = 3000;     // LT がこれより来なければ（受信機が NO SIGNAL と言う前でも）灰色にする目安の下限
  const NOLINE_MS = 3000;       // 行が何も来なければ「PONS 未接続」
  const STRIP_H = 30;           // 帯の高さ（論理 px。画面と同じ倍率で拡大する）

  // ---- LinkTelem（lora_link/link_proto.h。69 バイト、詰めた並び、リトルエンディアン）----
  function hexToBytes(hex) {
    const n = hex.length >> 1, b = new Uint8Array(n);
    for (let i = 0; i < n; i++) b[i] = parseInt(hex.substr(i * 2, 2), 16);
    return b;
  }
  function decodeLT(bytes) {
    const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    return {
      ver: bytes[2], type: bytes[3], seq: dv.getUint16(4, true), group: bytes[6], scale_index: bytes[7], dest_id: dv.getUint16(8, true),
      year: dv.getUint16(10, true), month: bytes[12], day: bytes[13], hour: bytes[14], minute: bytes[15], second: bytes[16], centi: bytes[17],
      lat_1e7: dv.getInt32(18, true), lon_1e7: dv.getInt32(22, true), gs_cms: dv.getUint16(26, true), track_cdeg: dv.getUint16(28, true),
      gnss_alt_dm: dv.getInt16(30, true), kf_alt_dm: dv.getInt16(32, true), kf_vs_cms: dv.getInt16(34, true), press_dpa: dv.getUint16(36, true),
      roll_cdeg: dv.getInt16(38, true), pitch_cdeg: dv.getInt16(40, true), yaw_cdeg: dv.getUint16(42, true),
      pitch_avg_cdeg: dv.getInt16(44, true), roll_trim_cdeg: dv.getInt16(46, true), yaw_acc95_deg: bytes[48],
      wind_dmps: bytes[49], wind_dir_cdeg: dv.getUint16(50, true), tas_dmps: bytes[52], tas_flags: bytes[53],
      batt: bytes[54], numsat: bytes[55], hacc_dm: dv.getUint16(56, true), sacc_cms: dv.getUint16(58, true), fixflags: bytes[60],
      dest_index: dv.getInt8(61), nav_mode: bytes[62], have: dv.getUint16(63, true), status: dv.getUint16(65, true),
      alpha_avg_cdeg: bytes[2] >= 10 ? dv.getInt16(67, true) : 0,
    };
  }
  function decodeBatt(b) {
    if (b >= 1 && b <= 101) return { kind: 1, pct: b - 1 };
    if (b >= 102 && b <= 202) return { kind: 2, pct: b - 102 };
    if (b === 255) return { kind: 3, pct: 0 };
    return { kind: 0, pct: 0 };
  }
  // 画面が読む値（受信機の link_get_* と gnss.cpp のミラー中の分かれ道の写し。単位の換算だけ）
  function ltView(t, known) {
    const has = (bit) => (t.have & bit) !== 0;
    const slot = (s) => (known & (1 << s)) !== 0;
    let track = t.track_cdeg / 100.0;
    if (track < 0.0 || track > 360.0) track = 0.0;
    return {
      raw: t, known,
      lat: t.lat_1e7 / 1e7, lon: t.lon_1e7 / 1e7, gs: t.gs_cms / 100.0, track,
      fix: (t.fixflags & 1) !== 0,
      numsatShown: has(RHAVE.NUMSAT) ? t.numsat : 99,
      hacc_mm: slot(W2S.HACC) ? (t.hacc_dm * 100) >>> 0 : 0xFFFFFFFF,
      sacc: (slot(W2S.NAV) && t.sacc_cms !== 0xFFFF) ? { ok: true, mps: f32(t.sacc_cms * f32(0.01)) } : { ok: false, mps: 0 },
      att: has(RHAVE.ATT) ? { ok: true, roll: f32(t.roll_cdeg * f32(0.01)), pitch: f32(t.pitch_cdeg * f32(0.01)) } : { ok: false },
      yaw: has(RHAVE.ATT_YAW) ? { ok: true, yaw: f32(t.yaw_cdeg * f32(0.01)), acc95: t.yaw_acc95_deg } : { ok: false },
      pavg: has(RHAVE.ATT_AVG) ? { ok: true, v: f32(t.pitch_avg_cdeg * f32(0.01)) } : { ok: false, v: 0 },
      alpha: t.ver >= 10 && has(RHAVE.ALPHA) ? { ok: true, v: f32(t.alpha_avg_cdeg * f32(0.01)) } : { ok: false, v: 0 },
      tas: has(RHAVE.TAS) ? { ok: true, mps: f32(t.tas_dmps * f32(0.1)), clamped: (t.tas_flags & 1) !== 0 } : { ok: false, mps: 0 },
      trim: has(RHAVE.ATT_TRIM) ? { ok: true, v: f32(t.roll_trim_cdeg * f32(0.01)) } : { ok: false },
      wind: has(RHAVE.ATT_WIND) ? { ok: true, spd: f32(t.wind_dmps * f32(0.1)), dir: f32(t.wind_dir_cdeg * f32(0.01)) } : { ok: false },
      vs: has(RHAVE.KFVS) ? f32(t.kf_vs_cms * f32(0.01)) : 0,
      batt: slot(W2S.ALTV) ? decodeBatt(t.batt) : { kind: 0, pct: 0 },
      status: t.status,
      scaleIndex: slot(W2S.PRESS) ? t.scale_index : null,
      time: has(RHAVE.DATE) ? { h: t.hour, m: t.minute, s: t.second, cs: t.centi } : null,
    };
  }

  // ---- 状態 ----
  const st = {
    port: null, reader: null, writer: null, portInfo: null, lastPort: null, opening: false, timeSent: false,
    want: false,              // ライブにつなぎたい（切れたらつなぎ直す）
    live: false,              // PONS:LIVE,OK を受けた
    ok: null, okMismatch: '',
    onSentAt: 0, lastOnRetry: 0, message: '', messageKind: '',
    dests: [], pilon: null, maps: new Map(), dumpEnd: false,
    frame: null, lastLn: null, ls: null, lsAt: 0,
    lastLineAt: 0, lastLtAt: 0,
    lines: 0, ltCount: 0, lnCount: 0, badLines: 0,
    track: null, northUp: false,
    dirty: true,              // 描き直す（LT・LN が来た）
    rec: [], recT0: (typeof performance !== 'undefined') ? performance.now() : 0,
    player: null,
  };

  // ---- 行 ----
  function feedLine(line, now) {
    st.lastLineAt = now;
    st.lines++;
    if (!st.player) st.rec.push(Math.round(now - st.recT0) + '\t' + line);
    if (!line.startsWith('PONS:')) return;
    const p = line.split(',');
    const head = p[0];
    if (head === 'PONS:LT') return onLT(p, now);
    if (head === 'PONS:LN') return onLN(p);
    if (head === 'PONS:LS') return onLS(p, now);
    if (head === 'PONS:LIVE') return onLive(p, line);
    if (line.startsWith('PONS:ERROR,BUSY')) { setMessage('本体がファイルの転送中です。終わってからつなぎます。', 'warn'); st.lastOnRetry = 0; }
  }
  function onLive(p, line) {
    const kind = p[1];
    // ★ PC の時刻は、受信機が返事をした（コマンドを読んでいる）ことが分かってから 1 回だけ送る。ポートを開いてすぐ送ると、
    //   受信機の電源を入れ直したときの自動のつなぎ直しでは起動画面（約 8 秒）のあいだ読まれず、古い時刻が精度 2 秒のつもりで
    //   GNSS へ入る（BT1 が切れていたときの測位を遅らせる）。返事は見ない（Re:PONS の本体の窓と同じ。protocol §4 (7)）
    if (st.port && !st.player && !st.timeSent && (kind === 'OK' || kind === 'ERROR' || kind === 'OFF')) {
      st.timeSent = true;
      send('PONS:TIME,' + Date.now());
    }
    if (kind === 'OK') {
      // PONS:LIVE,OK,ON,<形の版>,<LINK_PROTO_VER>,<sizeof(LinkTelem)>,<group>,<ch>,<sf>,<送る間隔>,<版>
      const ok = { fmt: +p[3], proto: +p[4], size: +p[5], group: +p[6], ch: +p[7], sf: +p[8], period: +p[9], build: p[10] || '' };
      st.ok = ok;
      if (ok.fmt !== LIVE_FMT_VER || (ok.proto !== LINK_PROTO_VER && ok.proto !== 9) || ok.size !== LINKTELEM_SIZE) {
        const tooNew = ok.fmt > LIVE_FMT_VER || ok.proto > LINK_PROTO_VER;
        st.okMismatch = tooNew ? 'Re:PONS が古いです（ページを読み直してください）' : '本体のファームが古いです（焼き直してください）';
        setMessage(st.okMismatch + `　本体: 形 ${ok.fmt}・無線の版 ${ok.proto}・${ok.size} バイト／Re:PONS: 形 ${LIVE_FMT_VER}・無線の版 ${LINK_PROTO_VER}・${LINKTELEM_SIZE} バイト`, 'error');
        st.live = false;
        return;
      }
      st.okMismatch = '';
      st.live = true;
      st.dests = []; st.maps = new Map(); st.pilon = null; st.dumpEnd = false;
      if (!st.player) setMessage(`流れています（CH${ok.ch}・SF${ok.sf}・グループ ${'ABCD'[ok.group & 3]}・本体 ${ok.build}）`, 'ok');
    } else if (kind === 'DEST') {
      // ★ 読めない値（行の壊れ）は入れない。NaN の座標を描くと線が画面の左上 (0,0) へ伸びる
      // PONS:LIVE,DEST,<番号>,<緯度>,<経度>,<名前>
      const i = +p[2], lat = parseFloat(p[3]), lon = parseFloat(p[4]);
      if (Number.isInteger(i) && i >= 0 && i < 1000 && Number.isFinite(lat) && Number.isFinite(lon))
        st.dests[i] = { lat, lon, name: p.slice(5).join(',') };
    } else if (kind === 'PILON') {
      const v = p.slice(2).map(parseFloat);
      st.pilon = (v.length >= 10 && v.slice(0, 10).every(Number.isFinite))
        ? { plaLat: v[0], plaLon: v[1], nLat: v[2], nLon: v[3], sLat: v[4], sLon: v[5], k1Lat: v[6], k1Lon: v[7], tLat: v[8], tLon: v[9] }
        : null;
    } else if (kind === 'MAP') {
      st.maps.set(p[2] + p[3], { kind: p[2], name: p.slice(5).join(','), n: +p[4], pts: [] });
    } else if (kind === 'MP') {
      const m = st.maps.get(p[2] + p[3]);
      if (!m) return;
      const k0 = +p[4];
      for (let i = 5, k = k0; i + 1 < p.length; i += 2, k++) {
        const la = parseFloat(p[i]), lo = parseFloat(p[i + 1]);
        if (Number.isFinite(la) && Number.isFinite(lo) && k >= 0 && k < m.n) m.pts[k] = [la, lo];
      }
    } else if (kind === 'DUMP_END') {
      st.dumpEnd = true;
    } else if (kind === 'OFF') {
      st.live = false;
      const why = p[2] || '';
      const msg = { CMD: '止めました', CONNECT: 'ファイルの一覧の接続に替わったので止まりました', MODE: '受信機が受信モードでなくなったので止まりました', PORT: '止まりました' }[why] || ('止まりました（' + why + '）');
      setMessage(msg, why === 'CMD' ? 'info' : 'warn');
      st.lastOnRetry = performance.now();
    } else if (kind === 'ERROR') {
      st.live = false;
      if (p[2] === 'NOT_RX') setMessage('この PONS は受信モードではありません。WIRELESS 画面の MODE を RECEIVER にしてください（ライブは受信機だけの機能です）。', 'error');
      else setMessage('本体が断りました（' + p.slice(2).join(',') + '）', 'error');
      st.lastOnRetry = performance.now();
    }
  }
  function onLT(p, now) {
    // PONS:LT,<ms>,<回転枠>,<受けた回転枠 hex>,<LinkTelem hex>
    if (!st.live && !st.player) return;
    const hex = p[4] || '';
    if (hex.length !== LINKTELEM_SIZE * 2) { st.badLines++; return; }
    const t = decodeLT(hexToBytes(hex));
    if (t.ver !== LINK_PROTO_VER && t.ver !== 9) { st.badLines++; return; }
    const fr = { ms: +p[1] >>> 0, slot: +p[2], known: parseInt(p[3], 16), t, rxAt: now, ln: null };
    fr.view = ltView(t, fr.known);
    st.ltCount++;
    // 航跡（LIVE を始めてから受けた位置だけ。送信機が fix しているフレームだけ。gnss.cpp と同じ）
    if (fr.view.fix) st.track.feed(fr.ms, fr.view.lat, fr.view.lon, fr.view.gs);
    st.frame = fr;
    st.lastLtAt = now;
    st.dirty = true;
  }
  function onLN(p) {
    // PONS:LN,<seq>,<flags>,<hdr>,<strike>,<wait>,<egs>,<marker>,<ltt>,<steer>,<cwi>,<dps>,<db>,<dest>,<dmode>,<a10k>,
    //         <truec>,<dist>,<mg>,<mgh>,<mgm>,<mg5>,<mg5h>,<mg5m>
    if (p.length < 24) { st.badLines++; return; }
    const ln = {
      seq: +p[1], flags: parseInt(p[2], 16), hdr: +p[3], strike: +p[4], wait: +p[5], egs: +p[6], marker: +p[7],
      ltt: parseFloat(p[8]), steer: parseFloat(p[9]), cwi: parseFloat(p[10]), dps: parseFloat(p[11]), db: parseFloat(p[12]),
      dest: +p[13], dmode: +p[14], a10k: +p[15], truec: parseFloat(p[16]), dist: parseFloat(p[17]),
      mg: parseFloat(p[18]), mgh: +p[19], mgm: +p[20], mg5: parseFloat(p[21]), mg5h: +p[22], mg5m: +p[23],
    };
    st.lnCount++;
    st.lastLn = ln;
    // LN は受信機がそのフレームの画面を描いたあとに来る。来るまでは前の LN で描く
    if (st.frame && st.frame.t.seq === ln.seq) st.frame.ln = ln;
    st.dirty = true;
  }
  // 受信機の状態の印（live.cpp の push_ls）
  const LSF_RECEIVING = 0x01, LSF_DUP = 0x02, LSF_OFFLINE = 0x04, LSF_DEST_MISSING = 0x08;
  function onLS(p, now) {
    // PONS:LS,<flags>,<最後に受けてからの ms>,<rssi10>,<本数>,<受けた数>,<落ちた数>,<機体までの距離 m>,<送る間隔>,<lines>,<dropped>
    if (p.length < 11) { st.badLines++; return; }
    const ls = { flags: parseInt(p[1], 16), age: +p[2], rssi: +p[3], bars: +p[4], rx: +p[5], miss: +p[6],
      dist: +p[7], period: +p[8], lines: +p[9], dropped: +p[10], at: now };
    const prev = st.ls;
    ls.rate = (prev && ls.rx >= prev.rx && ls.miss >= prev.miss && (ls.rx - prev.rx + ls.miss - prev.miss) > 0)
      ? (ls.rx - prev.rx) / ((ls.rx - prev.rx) + (ls.miss - prev.miss)) : (prev ? prev.rate : null);
    st.ls = ls;
    st.lsAt = now;
  }

  const DEFAULT_LN = { seq: -1, flags: 0, hdr: 0, strike: 0, wait: 0, egs: 0, marker: 2, ltt: 0, steer: 0, cwi: 0, dps: 0, db: 0.2,
    dest: -1, dmode: 0, a10k: 0, truec: 0, dist: 0, mg: 0, mgh: 0, mgm: 0, mg5: 0, mg5h: 0, mg5m: 0 };

  // 1 コマの状態（live_screen.js の render へ渡す）
  function makeState(now) {
    const fr = st.frame;
    if (!fr) return null;
    const v = fr.view;
    const ln = fr.ln || st.lastLn || Object.assign({}, DEFAULT_LN, { marker: v.fix ? 2 : 0, ltt: v.track });
    const SL = global.LiveScreen.SCALELIST;
    const si = (v.scaleIndex !== null && v.scaleIndex >= 0 && v.scaleIndex < SL.length) ? v.scaleIndex : global.LiveScreen.SCALE_INDEX_DEFAULT;
    let time = { valid: false, sec: 0 };
    if (v.time) {
      // 本体の draw_map_footer() と同じく、受けてからの経過を足して時計を進める（記録の再生中は足さない）
      const el = st.player ? 0 : Math.max(0, now - fr.rxAt);
      time = { valid: true, sec: v.time.h * 3600 + v.time.m * 60 + v.time.s + Math.floor((v.time.cs * 10 + el) / 1000) };
    }
    return {
      lat: v.lat, lon: v.lon, trackDraw: v.track, yawDraw: v.yaw.ok ? v.yaw.yaw : 0,
      up: st.northUp ? 0 : v.track, trackup: !st.northUp, scale: SL[si],
      lt: v, ln, dests: st.dests, pilon: st.pilon, maps: [...st.maps.values()], track: st.track, time,
    };
  }

  // ---- 受信機の状態（帯・灰色）----
  function linkState(now) {
    if (st.player) {
      if (st.player.done) return { kind: 'replayEnd' };
    } else if (!st.port) return { kind: 'noport' };
    if (!st.player && now - st.lastLineAt > NOLINE_MS) return { kind: 'noport' };
    if (!st.live && !st.player) return { kind: 'notlive' };
    if (st.player && st.player.paused) return { kind: 'ok' };   // 一時停止の間は止めた画面のまま（灰色にしない）
    const ls = st.ls;
    const period = (ls && ls.period > 0) ? ls.period : 1000;
    const ltAge = now - st.lastLtAt;
    // ★ LS は 1 秒ごとなので、受信が戻った直後の LS はまだ「受けていない」と言う。そのあとに LT が届いていれば古い知らせとして見ない
    const lsSaysNo = ls && !(ls.flags & LSF_RECEIVING) && !(st.lastLtAt > ls.at);
    // 受信機がリプレイ・デモ中（LT が来なくなる）。NO SIGNAL より先に見る
    if (ls && (ls.flags & LSF_OFFLINE) && !(st.lastLtAt > ls.at)) return { kind: 'offline' };
    if (!st.frame || lsSaysNo || ltAge > Math.max(NOSIG_LT_MS, 3 * period)) {
      // 経過秒は受信機の数（最後に受けてからの ms）を使う。無ければ自分の時計で
      const sec = (ls && ls.age >= 0) ? Math.floor((ls.age + (now - st.lsAt)) / 1000) : Math.floor(ltAge / 1000);
      return { kind: 'nosignal', sec: st.frame ? sec : -1 };
    }
    return { kind: 'ok' };
  }

  // ---- 描く ----
  let canvas, ctx, off, offCtx, img, scaleN = 3, bg = '#000000';
  function setupCanvas() {
    canvas = document.getElementById('screen');
    ctx = canvas.getContext('2d');
    off = document.createElement('canvas');
    off.width = 240; off.height = 320;
    offCtx = off.getContext('2d');
    img = offCtx.createImageData(240, 320);
    resizeCanvas();
  }
  function resizeCanvas() {
    canvas.width = 240 * scaleN;
    canvas.height = (320 + STRIP_H) * scaleN;
    canvas.style.width = canvas.width / (window.devicePixelRatio || 1) + 'px';
    canvas.style.height = canvas.height / (window.devicePixelRatio || 1) + 'px';
    st.dirty = true;
  }
  let lastRenderAt = 0;
  // LT・LN が来たら次のコマで描く。来なくても 1 秒に 4 回（点滅・時計・NO SIGNAL の秒・帯）
  function frameLoop() {
    requestAnimationFrame(frameLoop);
    const now = performance.now();
    if (!st.dirty && now - lastRenderAt < 250) return;
    st.dirty = false;
    lastRenderAt = now;
    draw(now);
  }
  let drawErrShown = false;
  function draw(now) {
    // ★ 1 コマの失敗（壊れた行の値など）で描く輪が例外を出し続けないように、ここで止める
    try { drawFrame(now); }
    catch (e) {
      if (!drawErrShown) { drawErrShown = true; console.error(e); setMessage('画面を描けませんでした（' + (e && e.message ? e.message : e) + '）。数字の表とコンソールを見てください。', 'error'); }
    }
  }
  function drawFrame(now) {
    const ls = linkState(now);
    const S = makeState(now);
    ctx.imageSmoothingEnabled = false;
    ctx.fillStyle = bg;
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    if (S) {
      const scr = global.LiveScreen.render(S, now);
      global.LiveTFT.toImageData(scr, img);
      if (ls.kind !== 'ok') greyOut(img);   // 止まった画面（受信が切れた・未接続・受信機がリプレイ中）は灰色
      offCtx.putImageData(img, 0, 0);
      ctx.drawImage(off, 0, 0, 240 * scaleN, 320 * scaleN);
    } else {
      ctx.fillStyle = '#202428';
      ctx.fillRect(0, 0, 240 * scaleN, 320 * scaleN);
    }
    overlayText(ls, S);
    drawStrip(ls, now);
  }
  function greyOut(im) {
    const d = im.data;
    for (let i = 0; i < d.length; i += 4) {
      const y = (d[i] * 0.3 + d[i + 1] * 0.59 + d[i + 2] * 0.11) * 0.55 + 40;
      d[i] = d[i + 1] = d[i + 2] = y;
    }
  }
  function overlayText(ls, S) {
    let t1 = '', t2 = '';
    if (ls.kind === 'nosignal') { t1 = 'NO SIGNAL'; t2 = ls.sec >= 0 ? ls.sec + 's' : ''; }
    else if (ls.kind === 'noport') { t1 = 'PONS 未接続'; t2 = ''; }
    else if (ls.kind === 'notlive') { t1 = st.okMismatch ? '版が合いません' : '待っています'; }
    else if (ls.kind === 'replayEnd') { t1 = '再生が終わりました'; }
    else if (ls.kind === 'offline') { t1 = '受信機がリプレイ・デモ中'; }
    if (!t1) return;
    const W = 240 * scaleN, H = 320 * scaleN;
    ctx.save();
    ctx.fillStyle = 'rgba(0,0,0,0.55)';
    ctx.fillRect(0, H * 0.40, W, H * 0.20);
    ctx.fillStyle = (ls.kind === 'nosignal') ? '#ff5050' : '#ffffff';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.font = `bold ${Math.round(22 * scaleN)}px system-ui, -apple-system, "Segoe UI", sans-serif`;
    ctx.fillText(t1, W / 2, H * (t2 ? 0.47 : 0.50));
    if (t2) { ctx.font = `bold ${Math.round(16 * scaleN)}px system-ui, sans-serif`; ctx.fillText(t2, W / 2, H * 0.545); }
    ctx.restore();
  }
  // 画面の外の帯（ボートの側の情報。設計書 §5.8）。どれも受信機の LS の値（受信機が計算したもの）
  function drawStrip(ls, now) {
    const s = scaleN, W = 240 * s, y0 = 320 * s, H = STRIP_H * s;
    ctx.save();
    ctx.fillStyle = '#101418';
    ctx.fillRect(0, y0, W, H);
    ctx.textBaseline = 'middle';
    const small = `${Math.round(7 * s)}px system-ui, -apple-system, "Segoe UI", sans-serif`;
    const big = `bold ${Math.round(11 * s)}px system-ui, -apple-system, "Segoe UI", sans-serif`;
    const L = st.ls;
    const live = (ls.kind === 'ok' || ls.kind === 'offline');
    const row1 = y0 + H * 0.36, row2 = y0 + H * 0.80;
    // 受信強度（直近 10 秒の平均と本数。受信機の画面の数字と同じ）
    ctx.fillStyle = '#8fa3b0';
    ctx.font = small;
    ctx.textAlign = 'left';
    ctx.fillText('BOAT RSSI', 4 * s, row1);
    ctx.fillText('TO PLANE', 128 * s, row1);
    ctx.font = big;
    if (L && live && L.rssi !== 0) {
      ctx.fillStyle = L.bars >= 2 ? '#3ddc84' : L.bars === 1 ? '#ffb020' : '#ff5050';
      ctx.fillText(`${L.rssi} dBm`, 46 * s, row1);
      for (let i = 0; i < 3; i++) {
        ctx.fillStyle = i < L.bars ? (L.bars >= 2 ? '#3ddc84' : '#ffb020') : '#34404a';
        const bw = 3 * s, bh = (4 + i * 3) * s;
        ctx.fillRect((98 + i * 5) * s, row1 + 5 * s - bh, bw, bh);
      }
    } else {
      ctx.fillStyle = '#6b7780';
      ctx.fillText('--', 46 * s, row1);
    }
    // 機体までの距離（received/ の dist_m と同じ計算。ボートが fix していないときは --）
    ctx.fillStyle = '#e8eef2';
    let dtxt = '--';
    if (L && live && L.dist >= 0) dtxt = L.dist >= 1000 ? (L.dist / 1000).toFixed(2) + ' km' : L.dist + ' m';
    ctx.fillText(dtxt, 168 * s, row1);
    // 2 行目: 受信の状態・OSM の帰属表示
    ctx.font = small;
    let stTxt = '', stCol = '#8fa3b0';
    if (ls.kind === 'nosignal') { stTxt = 'NO SIGNAL' + (ls.sec >= 0 ? ' ' + ls.sec + 's' : ''); stCol = '#ff5050'; }
    else if (ls.kind === 'noport') { stTxt = 'PONS 未接続'; stCol = '#ffb020'; }
    else if (ls.kind === 'notlive') { stTxt = '待っています'; }
    else if (ls.kind === 'offline') { stTxt = '受信機がリプレイ・デモ中'; stCol = '#ffb020'; }
    else if (st.player) { stTxt = '記録の再生'; stCol = '#ffb020'; }
    else if (L) { stTxt = `LIVE  受信率 ${L.rate !== null && L.rate !== undefined ? Math.round(L.rate * 100) + '%' : '--'}`; stCol = '#3ddc84'; }
    // 送信機の目的地が受信機に無い: 配信の画面から目的地の線・TC・距離が消えている理由（パイロットの画面には出ている）。
    //   受信機の 3 台の目的地・パイロン座標をそろえれば消える。NO SIGNAL と DUP SENDER のほうが先
    if (L && ls.kind === 'ok' && (L.flags & LSF_DEST_MISSING)) { stTxt = 'PLANE DEST NOT IN BOAT PONS'; stCol = '#ffb020'; }
    if (L && live && (L.flags & LSF_DUP)) { stTxt = 'DUP SENDER !!'; stCol = '#ff5050'; }
    ctx.fillStyle = stCol;
    ctx.textAlign = 'left';
    ctx.fillText(stTxt, 4 * s, row2);
    ctx.fillStyle = '#6b7780';
    ctx.textAlign = 'right';
    ctx.fillText('© OpenStreetMap contributors', W - 4 * s, row2);
    ctx.restore();
  }

  // ---- つなぐ ----
  const enc = (typeof TextEncoder !== 'undefined') ? new TextEncoder() : null;
  async function send(line) {
    if (!st.writer) return;
    try { await st.writer.write(enc.encode(line + '\r\n')); } catch (e) { console.warn('send', e); }
  }
  async function connect() {
    if (st.player) stopPlayback();
    if (!('serial' in navigator)) {
      setMessage('このブラウザは USB の接続（Web Serial）に対応していません。Mac・Windows の Chrome を使ってください。', 'error');
      return;
    }
    // ★ 押したときは毎回選ばせる（前のポートを黙って使わない）。PONS はどれも同じ VID/PID なので、送信機と受信機を両方 USB に
    //   つないでいると見分けられず、違う方を開いたまま選び直せなくなる
    let port = null;
    try { port = await navigator.serial.requestPort(); } catch (e) { return; }   // 選ばずに閉じた
    st.want = true;
    await openPort(port);
  }
  function samePort(p) {
    if (!st.portInfo) return false;
    const i = p.getInfo();
    return i.usbVendorId === st.portInfo.usbVendorId && i.usbProductId === st.portInfo.usbProductId;
  }
  // 自動でつなぎ直す相手。前に開いたポートそのものを先に探し、無ければ VID/PID が同じものが 1 つだけのときに限る（推測しない）
  function autoPort(ports) {
    if (st.lastPort && ports.includes(st.lastPort)) return st.lastPort;
    const same = ports.filter(samePort);
    return (same.length === 1) ? same[0] : null;
  }
  function reopen() {
    navigator.serial.getPorts().then((ps) => {
      if (st.port || st.opening || !st.want || st.player) return;
      const p = autoPort(ps);
      if (p) openPort(p);
      else if (ps.filter(samePort).length > 1)
        setMessage('PONS が 2 台以上つながっていて、どれが受信機か分かりません。「🔌 接続」を押して受信機を選んでください。', 'warn');
    }).catch(() => {});
  }
  async function openPort(port) {
    // ★ 開いている途中にもう一度呼ばれない（USB の「つながった」の知らせと 3 秒ごとのつなぎ直しが重なると、同じポートを 2 回開こうとして失敗を出す）
    if (st.port || st.opening) return;
    st.opening = true;
    // 開けなくても覚えておく（ほかの窓・アプリが手放したあと、retryTick が同じポートを開き直せるように。
    //   USB のポートは 1 つの窓でしか開けないので、本体の窓のファイルの接続とはそれで取り合わない）
    try { st.portInfo = port.getInfo(); } catch (e) {}
    st.lastPort = port;
    try {
      await port.open({ baudRate: 115200, bufferSize: 64 * 1024 });
    } catch (e) {
      setMessage('ポートを開けませんでした。Re:PONS の本体の窓（ファイルの接続）や Arduino IDE のシリアルモニタが使っていると開けません。' +
        '閉じれば自動でつなぎ直します（「切断」を押すまで 3 秒ごとに試します）。（' + (e && e.message ? e.message : e) + '）', 'error');
      st.opening = false;
      return;
    }
    st.opening = false;
    st.port = port;
    st.writer = port.writable.getWriter();
    st.lastLineAt = performance.now();
    st.timeSent = false;
    readLoop(port);
    await sendOn();
    updateButtons();
  }
  async function sendOn() {
    st.onSentAt = performance.now();
    st.lastOnRetry = st.onSentAt;
    if (!st.live) setMessage('受信機に頼んでいます…', 'info');
    await send('PONS:LIVE,ON');
  }
  async function readLoop(port) {
    const dec = new TextDecoder();
    let buf = '';
    // ★ Web Serial の読み取りのエラーには続けられるもの（受け取りの詰まり BufferOverrunError・BreakError など）がある。そのときは
    //   port.readable が新しくなるので、読み直す。続けられないとき（ケーブルを抜いた）は readable が null になって抜ける。
    while (port.readable && st.port === port) {
      let reader;
      try { reader = port.readable.getReader(); st.reader = reader; } catch (e) { break; }
      let done = false;
      try {
        for (;;) {
          const r = await reader.read();
          if (r.done) { done = true; break; }
          buf += dec.decode(r.value, { stream: true });
          let i;
          while ((i = buf.indexOf('\n')) >= 0) {
            const line = buf.slice(0, i).replace(/\r$/, '');
            buf = buf.slice(i + 1);
            if (line) feedLine(line, performance.now());
          }
          if (buf.length > 4096) buf = '';
        }
      } catch (e) {
        console.warn('read', e);
        buf = '';   // 途中で切れた行は捨てる
      } finally {
        try { reader.releaseLock(); } catch (e) {}
        if (st.reader === reader) st.reader = null;
      }
      if (done) break;   // cancel() した（閉じるとき）
    }
    if (st.port === port) await closePort(false);
  }
  async function closePort(userAction) {
    const port = st.port;
    st.port = null; st.live = false;
    // ★ 読み取りのロックを外してから閉じる（ロックされたままだと port.close() が失敗して開いたまま残り、つなぎ直せなくなる）
    const reader = st.reader;
    st.reader = null;
    if (reader) { try { await reader.cancel(); } catch (e) {} try { reader.releaseLock(); } catch (e) {} }
    if (st.writer) { try { st.writer.releaseLock(); } catch (e) {} st.writer = null; }
    if (port) { try { await port.close(); } catch (e) { console.warn('close', e); } }
    if (userAction) { st.want = false; setMessage('切断しました', 'info'); }
    else if (st.want) setMessage('PONS との USB が切れました。つながったら自動でつなぎ直します。', 'warn');
    updateButtons();
  }
  async function disconnect() {
    if (st.port && st.writer) await send('PONS:LIVE,OFF');
    await closePort(true);
  }
  // 返事が無い・断られた・止まったときは 5 秒ごとに ON を送り直す（配信中に人が触らなくてよいように）
  function retryTick(now) {
    if (st.player || !st.want) return;
    if (!st.port) {
      if (now - st.lastOnRetry > 3000) {
        st.lastOnRetry = now;
        reopen();
      }
      return;
    }
    if (st.live) {
      // ★ 流れているはずなのに 5 秒、行が 1 つも来ない（LS は 1 秒ごとに来る）。受信機が LIVE を黙って止めた（OFF の行を
      //   書けなかった・USB の Serial が 1 秒 false だった など）。ON は何度送ってもよいので送り直す
      if (now - st.lastLineAt > 5000 && now - st.lastOnRetry > 5000) sendOn();
      return;
    }
    if (st.okMismatch) return;
    if (now - st.onSentAt > 4000 && now - st.onSentAt < 4500 && !st.message.startsWith('この PONS')) {
      setMessage('受信機から返事がありません。ファームが古い（LIVE を知らない）か、受信モードでない可能性があります。', 'warn');
    }
    if (now - st.lastOnRetry > 5000) sendOn();
  }
  if (typeof navigator !== 'undefined' && 'serial' in navigator) {
    navigator.serial.addEventListener('disconnect', (ev) => { if (ev.target === st.port) closePort(false); });
    navigator.serial.addEventListener('connect', () => { if (st.want && !st.port && !st.player) reopen(); });
  }

  // ---- 記録の保存・再生（設計書 §5.7）----
  function saveRecording() {
    const blob = new Blob([st.rec.join('\n') + '\n'], { type: 'text/plain' });
    const d = new Date();
    const pad = (n) => String(n).padStart(2, '0');
    const name = `pons_live_${d.getFullYear()}${pad(d.getMonth() + 1)}${pad(d.getDate())}_${pad(d.getHours())}${pad(d.getMinutes())}${pad(d.getSeconds())}.txt`;
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = name;
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 5000);
  }
  function resetStream() {
    st.frame = null; st.lastLn = null; st.ls = null; st.live = false;
    st.dests = []; st.maps = new Map(); st.pilon = null;
    st.track.reset();
  }
  async function startPlayback(file, startAtMs) {
    const text = await file.text();
    const rows = [];
    for (const raw of text.split('\n')) {
      const tab = raw.indexOf('\t');
      if (tab < 0) continue;
      rows.push({ t: +raw.slice(0, tab), line: raw.slice(tab + 1).replace(/\r$/, '') });
    }
    if (!rows.length) { setMessage('記録の行がありません（Re:PONS のライブで保存したファイルを選んでください）', 'error'); return; }
    if (st.port) await disconnect();
    resetStream();
    st.player = {
      rows, i: 0, t0: performance.now(), base: rows[0].t + (startAtMs || 0), done: false, paused: false, pausedAt: 0,
      step(now) {
        if (this.paused || this.done) return;
        const tt = this.base + (now - this.t0);
        while (this.i < this.rows.length && this.rows[this.i].t <= tt) {
          const r = this.rows[this.i++];
          feedLine(r.line, now);
          if (r.line.startsWith('PONS:LIVE,OK')) st.live = true;
        }
        if (this.i >= this.rows.length) this.done = true;
      },
    };
    st.live = true;
    setMessage(`記録を再生しています（${file.name}・${rows.length} 行）`, 'info');
    updateButtons();
  }
  function stopPlayback() {
    st.player = null;
    resetStream();
    setMessage('再生をやめました', 'info');
    updateButtons();
  }
  function togglePause() {
    const p = st.player;
    if (!p) return;
    if (p.paused) {
      const d = performance.now() - p.pausedAt;
      p.t0 += d;
      // ★ 止めていた時間を受信の途切れに数えない（続けた直後に「NO SIGNAL <止めていた秒>s」の灰色が出ていた。2026-11-25）。
      //   linkState() と時計が now との差で見る時刻を、止めていた分だけ後ろへずらす。0 は「まだ受けていない」なのでそのまま
      if (st.lastLtAt) st.lastLtAt += d;
      if (st.lsAt) st.lsAt += d;
      if (st.ls) st.ls.at += d;
      if (st.frame) st.frame.rxAt += d;
      p.paused = false;
    }
    else { p.paused = true; p.pausedAt = performance.now(); }
    updateButtons();
  }

  // ---- 地図 ----
  // ★ vectormap_hires.js を <script> で読む（file:// で開いても読める。live_map.js の loadFromScript）。
  //   2026-11-25 までは http のときだけ fetch で読み、file:// では「地図を選ぶ」で手で選ぶまで白いままだった。
  async function loadMap() {
    const el = document.getElementById('map-status');
    try {
      el.textContent = '地図: 読み込み中…';
      const v = new URLSearchParams(location.search).get('v') || '';
      const m = await global.LiveMap.loadFromScript('vectormap_hires.js' + (v ? '?v=' + encodeURIComponent(v) : ''));
      el.textContent = `地図: hires（${m.sourceDate}）`;
    } catch (e) {
      el.textContent = '地図: 読めませんでした（' + (e && e.message ? e.message : e) + '）。地図は白のままです';
      el.className = 'hint bad';
    }
  }

  // ---- 画面の操作 ----
  function setMessage(m, kind) {
    st.message = m; st.messageKind = kind;
    const el = (typeof document !== 'undefined') ? document.getElementById('message') : null;
    if (el) { el.textContent = m; el.className = 'msg ' + (kind || ''); }
  }
  function updateButtons() {
    const $ = (id) => document.getElementById(id);
    $('btn-connect').disabled = !!st.port || !!st.player;
    $('btn-disconnect').disabled = !st.port;
    $('btn-stop-play').disabled = !st.player;
    $('btn-pause').disabled = !st.player;
    $('btn-pause').textContent = st.player && st.player.paused ? '▶ 続き' : '⏸ 止める';
  }
  function saveSettings() {
    try { localStorage.setItem('pons_live_ui', JSON.stringify({ scaleN, bg, northUp: st.northUp })); } catch (e) {}
  }
  function loadSettings() {
    try {
      const s = JSON.parse(localStorage.getItem('pons_live_ui') || '{}');
      if (s.scaleN) scaleN = s.scaleN;
      if (s.bg) bg = s.bg;
      if (typeof s.northUp === 'boolean') st.northUp = s.northUp;
    } catch (e) {}
  }
  function togglePanel(force) {
    const hide = (force !== undefined) ? force : !document.body.classList.contains('bare');
    document.body.classList.toggle('bare', hide);
    if (hide) fitWindow();
  }
  function fitWindow() {
    // 配信用: 窓をキャンバスの大きさにする（スクリプトで開いた窓だけ変えられる）
    try {
      const w = canvas.width / (window.devicePixelRatio || 1), h = canvas.height / (window.devicePixelRatio || 1);
      window.resizeTo(Math.round(w + (window.outerWidth - window.innerWidth)), Math.round(h + (window.outerHeight - window.innerHeight)));
    } catch (e) {}
  }

  // ---- 数字の表（段階 1 の確かめ用）----
  function updateNumbers() {
    const el = document.getElementById('numbers');
    if (!el || el.hidden) return;
    const fr = st.frame;
    const v = fr && fr.view, t = fr && fr.t, ln = st.lastLn, L = st.ls;
    const fx = (x, n = 1) => (x === undefined || x === null || Number.isNaN(x)) ? '--' : Number(x).toFixed(n);
    const rows = [
      ['行', `${st.lines}（LT ${st.ltCount}・LN ${st.lnCount}・読めない ${st.badLines}）`],
      ['seq / 回転枠', t ? `${t.seq} / ${fr.slot}（受けた ${fr.known.toString(2).padStart(8, '0')}）` : '--'],
      ['位置', v ? `${v.lat.toFixed(7)}, ${v.lon.toFixed(7)}（fix ${v.fix ? 'あり' : 'なし'}）` : '--'],
      ['GS / 真方位', v ? `${fx(v.gs, 2)} m/s / ${fx(v.track, 1)}°` : '--'],
      ['高度 GNSS / KF', t ? `${t.have & RHAVE.GNSSALT ? fx(t.gnss_alt_dm / 10) : '--'} / ${t.have & RHAVE.KFALT ? fx(t.kf_alt_dm / 10) : '--'} m` : '--'],
      ['V/S', t && (t.have & RHAVE.KFVS) ? `${fx(v.vs, 2)} m/s` : '--'],
      ['R / P', v && v.att.ok ? `${fx(v.att.roll)} / ${fx(v.att.pitch)}` : '--'],
      ['ヨー ± 95%', v && v.yaw.ok ? `${fx(v.yaw.yaw)} ± ${v.yaw.acc95}` : '--'],
      ['平均ピッチ / TAS', v ? `${v.pavg.ok ? fx(v.pavg.v) : '--'} / ${v.tas.ok ? fx(v.tas.mps) : '--'}` : '--'],
      ['風', v && v.wind.ok ? `${fx(v.wind.spd)} m/s → ${fx(v.wind.dir, 0)}°` : '--'],
      ['衛星 / hAcc / sAcc', v ? `${v.numsatShown === 99 ? '--' : v.numsatShown} / ${v.hacc_mm === 0xFFFFFFFF ? '--' : (v.hacc_mm / 1000).toFixed(1) + 'm'} / ${v.sacc.ok ? fx(v.sacc.mps, 2) : '--'}` : '--'],
      ['電池', v ? ['分からない', '電池', '充電中', 'SW OFF'][v.batt.kind] + (v.batt.kind === 1 || v.batt.kind === 2 ? ' ' + v.batt.pct + '%' : '') : '--'],
      ['縮尺 / 目的地 ID', v ? `${v.scaleIndex === null ? '--' : v.scaleIndex} / ${t.dest_id}` : '--'],
      ['status / have', t ? `0x${t.status.toString(16).padStart(4, '0')} / 0x${t.have.toString(16).padStart(4, '0')}` : '--'],
      ['LN flags / hdr / 札', ln ? `0x${ln.flags.toString(16).padStart(4, '0')} / ${ln.hdr} / ${ln.strike}` : '--'],
      ['目的地 / モード', ln ? `${ln.dest}（${st.dests[ln.dest] ? st.dests[ln.dest].name : '?'}）/ ${ln.dmode} / ${ln.a10k}` : '--'],
      ['TC / 距離', ln ? `${fx(ln.truec)}° / ${fx(ln.dist, 3)} km` : '--'],
      ['ずれ / 警告 / deg/s', ln ? `${fx(ln.steer)} / ${fx(ln.cwi)} / ${fx(ln.dps, 2)}` : '--'],
      ['MaxGS', ln ? `${fx(ln.mg)}（${ln.mgh}:${String(ln.mgm).padStart(2, '0')}）/ 5分 ${fx(ln.mg5)}` : '--'],
      ['受信 RSSI / 本数', L ? `${L.rssi} dBm / ${L.bars}（受けた ${L.rx}・落ちた ${L.miss}）` : '--'],
      ['受信機の状態', L ? `flags 0x${L.flags.toString(16)}・最後から ${L.age} ms・間隔 ${L.period} ms` : '--'],
      ['機体まで', L ? (L.dist >= 0 ? L.dist + ' m' : '--（ボートの fix なし）') : '--'],
      ['受信機の LIVE', L ? `書いた ${L.lines}・捨てた ${L.dropped}` : '--'],
      ['目的地・地図の一覧', `${st.dests.length} 件・${st.maps.size} 枚・パイロン ${st.pilon ? 'あり' : 'なし'}${st.dumpEnd ? '（そろった）' : ''}`],
    ];
    el.innerHTML = '<table>' + rows.map(([k, x]) => `<tr><th>${k}</th><td>${escapeHtml(String(x))}</td></tr>`).join('') + '</table>';
  }
  function escapeHtml(s) { return s.replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c])); }

  // ---- 初期化 ----
  function init() {
    st.track = new global.LiveScreen.Track();
    loadSettings();
    setupCanvas();
    const $ = (id) => document.getElementById(id);
    $('sel-scale').value = String(scaleN);
    $('sel-bg').value = bg;
    $('sel-up').value = st.northUp ? 'north' : 'track';
    document.body.style.background = bg;
    $('btn-connect').onclick = connect;
    $('btn-disconnect').onclick = disconnect;
    $('sel-scale').onchange = (e) => { scaleN = +e.target.value; resizeCanvas(); saveSettings(); if (document.body.classList.contains('bare')) fitWindow(); };
    $('sel-bg').onchange = (e) => { bg = e.target.value; document.body.style.background = bg; saveSettings(); };
    $('sel-up').onchange = (e) => { st.northUp = e.target.value === 'north'; saveSettings(); };
    $('btn-save').onclick = saveRecording;
    $('btn-clear-track').onclick = () => st.track.reset();
    $('file-play').onchange = (e) => { if (e.target.files[0]) startPlayback(e.target.files[0]); e.target.value = ''; };
    $('btn-stop-play').onclick = stopPlayback;
    $('btn-pause').onclick = togglePause;
    $('btn-numbers').onclick = () => { $('numbers').hidden = !$('numbers').hidden; updateNumbers(); };
    $('btn-bare').onclick = () => togglePanel(true);
    document.addEventListener('keydown', (e) => {
      if (e.target && (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT')) return;
      if (e.key === 'h' || e.key === 'H') togglePanel();
      if (e.key === 'Escape') togglePanel(false);
    });
    canvas.addEventListener('dblclick', () => togglePanel());
    window.addEventListener('beforeunload', () => {
      if (st.port && st.writer) { try { st.writer.write(enc.encode('PONS:LIVE,OFF\r\n')); } catch (e) {} }
    });
    updateButtons();
    setMessage('「🔌 接続」を押して、USB でつないだ受信機（受信モードの PONS）を選んでください。', 'info');
    const q = new URLSearchParams(location.search);
    const mapReady = loadMap();
    // ?replay=<記録のファイル>（http で開いたとき。練習・開発用）・?bare=1（パネルを隠す）・?scale=4・?up=north
    if (q.get('scale')) { scaleN = +q.get('scale') || scaleN; $('sel-scale').value = String(scaleN); resizeCanvas(); }
    if (q.get('up')) { st.northUp = q.get('up') === 'north'; $('sel-up').value = st.northUp ? 'north' : 'track'; }
    if (q.get('bare') === '1') document.body.classList.add('bare');
    if (q.get('replay')) {
      const name = q.get('replay');
      mapReady.then(() => fetch(name)).then((r) => r.blob())
        .then((b) => startPlayback(new File([b], name.split('/').pop()), +(q.get('seek') || 0)))
        .catch((e) => setMessage('記録を読めませんでした（' + e + '）', 'error'));
    }
    setInterval(updateNumbers, 250);
    // 記録の再生と、つなぎ直しは描くのと別に回す（窓が隠れて requestAnimationFrame が止まっても行の処理は止めない）
    setInterval(() => { const now = performance.now(); if (st.player) st.player.step(now); retryTick(now); }, 50);
    requestAnimationFrame(frameLoop);
  }

  global.PonsLive = { init, decodeLT, ltView, feedLine, st, makeState, hexToBytes };
  if (typeof document !== 'undefined') {
    if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
    else init();
  }
})(typeof window !== 'undefined' ? window : globalThis);
