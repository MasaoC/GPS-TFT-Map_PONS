// ============================================================
// File    : tools/repons/live_screen.js
// Project : PONS v7.1 / v7.2 Re:PONS
// Role    : ライブ配信の画面。本体の地図画面（GPS_TFT_map.ino の draw_current_screen() の MODE_MAP と
//           display_tft.cpp の描く関数）の写し。live.js が作る 1 コマの状態 S から 240x320 の RGB565 を描く。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026-11-23
// ============================================================
//
// ★ **判断はしない**（docs/pons_live_stream_plan.md §1.4）。どの札・線・色を出すかは受信機が LN 行で送ってくる
//   （display_tft.cpp の live_collect_screen()・live.cpp）。ここで計算するのは描くための幾何
//   （緯度経度を画面の点にする・角度）と、描く関数の中にある色の境目だけ。
// ★ 描く順番・座標・色・文字の大きさは display_tft.cpp のまま。本体の描き方を変えたら、ここも同じに直す
//   （CLAUDE.md の「破ると静かに壊れる約束」）。関数名は本体と同じにしてある（draw_header → drawHeader など）。
// ★ 描かないもの（受信機の側だけのもの）: 受信枠・電波と船のアイコン・受信強度の数値・NO SIGNAL / DUP SENDER /
//   目的地の食い違いの箱・受信機の充電ポップアップ・自動減光の DIM MODE・ボートの位置。
//   機体の中だけのもの（電波に載っていない）: 充電ポップアップ・高温・SW OFF の注意・警報ミュートの箱。
(function (global) {
  'use strict';
  const T = global.LiveTFT, M = global.LiveMap;
  const { C, Sprite, sprintf, lround } = T;
  const f32 = Math.fround;

  // ---- 画面の大きさ（display_tft.h）----
  const SCREEN_WIDTH = 240, SCREEN_HEIGHT = 320, HEADERFOOTER_HEIGHT = 50, BACKSCREEN_SIZE = 240;
  const VSI_X = 235, VSI_W = 5;

  // ---- settings.h・display_tft.cpp の定数（本体と同じ値。tools/repons/test/run_live_tests.py が照合する）----
  const K = {
    NAV_MIN_GS_MPS: 2.0, HACC_THRESHOLD_M: 10.0, HDOP_MIN_CIRCLE_RADIUS: 4, HDOP_CENTER_DOT_RADIUS: 3,
    TRACK_BREAK_M: 300.0, MAX_TRACK_CORDS: 500,
    ESKF_DRIFT_LINE_DASH_PX: 5, ESKF_DRIFT_LINE_SKIP_PX: 7, ESKF_DRIFT_LINE_GAP_PX: 4, ESKF_DRIFT_LINE_LEN_PX: 80,
    WIND_ARROW_LEN_PX: 66, WIND_ARROW_HEAD_PX: 18, WIND_ARROW_WIDTH_PX: 4, WIND_STALE_CROSS_PX: 3,
    WIND_COL_LOW_MAX: 2.0, WIND_COL_MID_MAX: 4.0,
    ROLL_TRIM_ALERT_DEG: 3.0, ROLL_TRIM_WARN_DEG: 1.5, ATTITUDE_IMPLAUSIBLE_DEG: 10.0, BANK_WARN_DEG: 4.0,
    NOTICE_Y_PWR_FAULT: 190, FOOTER_SATS0_BLINK_MS: 1000, GNSS_NUMSAT_UNKNOWN: 99,
    BAT_FULL_VOLTAGE: 4.15, BAT_ZERO_VOLTAGE: 3.4, BAT_LOW_VOLTAGE: 3.5,
    SCALE_SMALL_GMAP: 0.81765545,
    POWER_FAULT_ACTION_TEXT: 'SW OFF + UNPLUG USB',
  };
  K.BAT_HALF_VOLTAGE = (K.BAT_FULL_VOLTAGE + K.BAT_ZERO_VOLTAGE) / 2.0;
  // 地図の縮尺（.ino の scalelist。送信機の scale_index の添字）
  const SCALELIST = [0.2044138625, 0.81765545, 3.2706218, 13.08248718, 52.32994872, 200.0];
  const SCALE_INDEX_DEFAULT = 3;   // 回転枠 6 を受けるまで（.ino の scaleindex の初期値。画面幅 約 18km）

  // ---- LN の印（display_tft.h の LIVESCR_*）----
  const LS = {
    ESKF_EN: 0x0001, YAW_REL: 0x0002, YAW_HC: 0x0004, NEEDS_APPLY: 0x0008, GROUND_ROLL: 0x0010,
    GROUND_PITCH: 0x0020, GST_INVALID: 0x0040, TRACKWARN: 0x0080, CWARN: 0x0100, VSI: 0x0200,
    VARIO_OK: 0x0400, SCANNING: 0x0800, WEAK: 0x1000, NAV_VALID: 0x2000, WIND: 0x4000, LEAVING: 0x8000,
  };
  const HDR_TT = 0, HDR_PAVG = 1, HDR_TAS = 2, HDR_TAS_CLAMP = 3, HDR_TAS_GROUND = 4, HDR_TAS_VS_LEGACY = 5, HDR_TAS_UNAVAILABLE = 6, HDR_ALPHA = 7;
  const isTasHdr = (h) => h === HDR_TAS || h === HDR_TAS_CLAMP || h === HDR_TAS_GROUND || h === HDR_TAS_VS_LEGACY || h === HDR_TAS_UNAVAILABLE;
  const tasBoxColor = (h, mps) => (h === HDR_TAS_CLAMP || h === HDR_TAS_UNAVAILABLE || Number.isNaN(mps)) ? C.GRAY : C.BLUE;
  const HSTRIKE_NONE = 0, HSTRIKE_GROUND = 1, HSTRIKE_YAW_ALIGN = 2, HSTRIKE_NO_GNSS = 3, HSTRIKE_CALIB = 4;
  const EGS_OK = 0, EGS_NO_GNSS = 1, EGS_YAW_ALIGN = 2;
  const DMODE_FLYINTO = 0, DMODE_FLYAWAY = 1, DMODE_AUTO10K = 2;
  const AUTO10K_AWAY = 0, AUTO10K_INTO = 1;
  const LINK_ST_SD_ERROR = 0x0001, LINK_ST_PWR_FAULT = 0x0100;
  const BATT_UNKNOWN = 0, BATT_BATTERY = 1, BATT_CHARGING = 2, BATT_SW_OFF = 3;

  // ---- スプライト（setup_tft() と同じ初期状態から、毎コマ同じ順に描く。文字の状態はコマをまたいで残る）----
  let F = null, BS = null, HF = null, VSI = null, SCR = null;
  function setup() {
    if (SCR) return;
    const fo = T.fonts();
    F = { SMALL: fo.AA_FONT_SMALL, MEDIUM: fo.NM_FONT_MEDIUM, LARGE: fo.NM_FONT_LARGE };
    SCR = new Sprite(SCREEN_WIDTH, SCREEN_HEIGHT);
    SCR.fillSprite(C.WHITE);
    BS = new Sprite(SCREEN_WIDTH, BACKSCREEN_SIZE);
    BS.loadFont(F.SMALL);
    HF = new Sprite(SCREEN_WIDTH, HEADERFOOTER_HEIGHT);
    HF.loadFont(F.LARGE);
    VSI = new Sprite(VSI_W, BACKSCREEN_SIZE);
  }

  // ---- 1 コマの状態（live.js の makeState が作る）----
  let S = null;
  let NOW = 0;                    // Re:PONS の時計 [ms]（点滅の位相。本体の millis() の代わり）
  const blink1s = () => Math.floor(NOW / 1000) % 2 === 0;
  const lnf = (bit) => (S.ln.flags & bit) !== 0;
  const trackup = () => S.trackup;
  const selfCy = () => trackup() ? (BACKSCREEN_SIZE * 3 / 4) : (BACKSCREEN_SIZE / 2);
  const needleLen = () => trackup() ? 180 : 120;
  const xy = (lat, lon) => M.latLonToXY(lat, lon, S.lat, S.lon, S.scale, S.up, selfCy());

  // ---- 描く関数が読む値（本体の accessor の写し。ミラー中の分かれ道だけ）----
  const gnssMps = () => S.lt.gs;                               // link_get_gs()
  const truetrackDisplayDeg = () => {                          // truetrack_display_deg()
    const t = lround(f32(S.lt.track));
    return ((t % 360) + 360) % 360;
  };
  const headerShowsPitch = () => S.ln.hdr !== HDR_TT;
  const attitudeImplausible = (deg) => Math.abs(deg) >= K.ATTITUDE_IMPLAUSIBLE_DEG;
  const groundWarnShown = (pitch) => lnf(pitch ? LS.GROUND_PITCH : LS.GROUND_ROLL);
  const eskfYawReliable = () => lnf(LS.YAW_REL);
  const eskfYawHighConfidence = () => lnf(LS.YAW_HC);
  const eskfDisplayEnabled = () => lnf(LS.ESKF_EN);
  const egs = () => S.ln.egs;
  const pitchAvg = () => S.lt.pavg;                             // tas_pitch_avg()（{ok, v}）
  const tasHeaderValue = () => S.lt.tas.ok ? S.lt.tas.mps : NaN;
  const numsat = () => S.lt.numsatShown;                       // get_gnss_numsat()（ミラー中）
  const roundf = (x) => (x < 0 ? -Math.round(-x) : Math.round(x));

  // ---- 通知の箱が使う段（display_tft.cpp の notice_reserve / notice_free）----
  let RES = [];
  const noticeReserve = (y, h) => { if (RES.length < 10) RES.push([y, y + h]); };
  const noticeFree = (y, h) => RES.every(([a, b]) => !(y < b && y + h > a));

  function strikeThrough(spr, x, yMid, w, col) {
    spr.drawFastHLine(x, yMid, w, col);
    spr.drawFastHLine(x, yMid + 1, w, col);
  }

  // =====================================================================
  //  ヘッダー（display_tft.cpp の draw_header と、その下の関数）
  // =====================================================================
  function drawHeaderPitch(x, deg, col) {
    const SIGN_W = 25, DIGIT_W = 37, DOT_W = 10, ytop = 3;
    if (Number.isNaN(deg)) {
      HF.fillRect(x + 35, ytop + 18, 24, 6, C.GRAY);
      HF.fillRect(x + 71, ytop + 18, 24, 6, C.GRAY);
      return;
    }
    const neg = deg < 0;
    let a = f32(Math.abs(deg));
    if (a > f32(99.9)) a = f32(99.9);
    const bar_w = 21, bar_t = 7, cx = x + (SIGN_W / 2 | 0), cy = ytop + 21;
    HF.fillRect(cx - (bar_w / 2 | 0), cy - (bar_t / 2 | 0), bar_w, bar_t, col);
    if (!neg) HF.fillRect(cx - (bar_t / 2 | 0), cy - (bar_w / 2 | 0), bar_t, bar_w, col);
    x += SIGN_W;
    HF.setCursor(x, ytop);
    const tenths = lround(f32(a * 10.0));
    if (tenths < 100) {
      HF.printf('%d', (tenths / 10) | 0);
      x += DIGIT_W;
      HF.fillRect(x + 2, ytop + 36, 6, 6, col);
      x += DOT_W;
      HF.setCursor(x, ytop);
      HF.printf('%d', tenths % 10);
    } else {
      HF.printf('%02d', lround(a));
    }
  }

  function drawHeaderTas(x, h, mps) {
    const LABEL_W = 25, DIGIT_W = 37, DOT_W = 10, ytop = 3;
    const box = tasBoxColor(h, mps);
    HF.fillRect(x + 1, 0, SCREEN_WIDTH - x - 1, HEADERFOOTER_HEIGHT - 1, box);
    {
      const x0 = f32(x + 4), x1 = f32(x + LABEL_W - 4);
      const cy = f32(ytop + 21), amp = f32(4.0);
      const SEG = 8;
      let px = x0, py = cy;
      for (let i = 1; i <= SEG; i++) {
        const t = f32(i / SEG);
        const nx = f32(x0 + f32(f32(x1 - x0) * t));
        const ny = f32(cy - f32(amp * f32(Math.sin(f32(f32(2.0 * f32(Math.PI)) * t)))));
        HF.drawWideLine(px, py, nx, ny, 4.5, C.WHITE);
        px = nx; py = ny;
      }
    }
    x += LABEL_W;
    if (h === HDR_TAS_UNAVAILABLE || Number.isNaN(mps)) {
      HF.fillRect(x + 10, ytop + 18, 24, 6, C.WHITE);
      HF.fillRect(x + 46, ytop + 18, 24, 6, C.WHITE);
    } else {
      HF.loadFont(F.LARGE);
      HF.setTextColor(C.WHITE, box);
      let tenths = lround(f32(mps * 10.0));
      if (tenths < 0) tenths = 0;
      HF.setCursor(x, ytop);
      if (tenths < 100) {
        HF.printf('%ld', (tenths / 10) | 0);
        HF.fillRect(x + DIGIT_W + 2, ytop + 36, 6, 6, C.WHITE);
        HF.setCursor(x + DIGIT_W + DOT_W, ytop);
        HF.printf('%ld', tenths % 10);
      } else {
        let whole = lround(mps);
        if (whole > 99) whole = 99;
        HF.printf('%02ld', whole);
      }
    }
    HF.setTextColor(C.BLACK, C.WHITE);
  }

  function headerStrikeTag(l, r, word, fill, ink, edge) {
    HF.loadFont(F.SMALL);
    const w = HF.textWidth(word) + 6, h = 16, y = 16;
    let x = ((l + r - w) / 2) | 0;
    if (x + w > r) x = r - w;
    if (x < l - 3) x = l - 3;
    if (edge !== fill) HF.fillRect(x - 2, y - 2, w + 4, h + 4, edge);
    HF.fillRect(x, y, w, h, fill);
    HF.setTextColor(ink, fill);
    HF.setCursor(x + 3, y + 1);
    HF.print(word);
    HF.loadFont(F.LARGE);
    HF.setTextColor(C.BLACK, C.WHITE);
  }

  function headerStrike(mtvx) {
    switch (S.ln.strike) {
      case HSTRIKE_CALIB:
        for (let i = 0; i < 4; i++) HF.drawFastHLine(mtvx + 2, 22 + i, 111 - 4, C.RED);
        headerStrikeTag(mtvx + 25, SCREEN_WIDTH, 'CALIBRATE', C.RED, C.WHITE, C.RED);
        break;
      case HSTRIKE_NO_GNSS:
        strikeThrough(HF, mtvx + 2, 23, 111 - 4, C.ORANGE);
        headerStrikeTag(mtvx + 25, SCREEN_WIDTH, 'NO GNSS', C.ORANGE, C.BLACK, C.ORANGE);
        break;
      case HSTRIKE_YAW_ALIGN:
        strikeThrough(HF, mtvx + 2, 23, 111 - 4, C.YELLOW);
        headerStrikeTag(mtvx + 25, SCREEN_WIDTH, 'YAW ALIGN', C.YELLOW, C.BLACK, C.BLACK);
        break;
      case HSTRIKE_GROUND: {
        const x = mtvx + 25, lw = SCREEN_WIDTH - x - 1;
        const box = tasBoxColor(S.ln.hdr, tasHeaderValue());
        HF.fillRect(x, 20, lw, 8, box);
        HF.fillRect(x, 22, lw, 4, C.WHITE);
        const wait = S.ln.wait;
        if (wait > 0) headerStrikeTag(mtvx + 25, SCREEN_WIDTH, sprintf('WAIT %ds', wait), C.WHITE, box, box);
        else if (!lnf(LS.LEAVING)) headerStrikeTag(mtvx + 25, SCREEN_WIDTH, 'GROUND', C.WHITE, box, box);
        break;
      }
      default: break;
    }
  }

  function drawHeader() {
    HF.fillSprite(C.WHITE);
    HF.setTextWrap(false);
    HF.loadFont(F.LARGE);
    const sel_col = gnssMps() < K.NAV_MIN_GS_MPS ? C.GRAY : C.BLACK;
    HF.setTextColor(sel_col, C.WHITE);
    HF.setCursor(-1, 3);
    const _mps = f32(gnssMps());
    HF.printf(_mps >= 100.0 ? '%4.0f' : '%4.1f', _mps);
    const mtvx = SCREEN_WIDTH - 111;
    HF.drawFastVLine(mtvx, 0, HEADERFOOTER_HEIGHT, C.GRAY);
    {
      HF.loadFont(F.SMALL);
      HF.setTextColor(C.GRAY, C.WHITE);
      const uh = HF.fontHeight() - 2;
      const kUnit = ['m', 'p', 's'];
      for (let i = 0; i < 3; i++) {
        const w = HF.textWidth(kUnit[i]);
        HF.setCursor(mtvx - 2 - w - i * 2, HEADERFOOTER_HEIGHT - 1 - uh * (3 - i));
        HF.print(kUnit[i]);
      }
      HF.loadFont(F.LARGE);
    }
    const hdr = S.ln.hdr;
    if (isTasHdr(hdr)) {
      drawHeaderTas(mtvx, hdr, tasHeaderValue());
      headerStrike(mtvx);
    } else if (hdr === HDR_ALPHA || hdr === HDR_PAVG) {
      const angle = hdr === HDR_ALPHA ? S.lt.alpha : pitchAvg();
      const pv = angle.ok ? angle.v : NaN;
      const pcol = attitudeImplausible(pv) ? C.GRAY : (hdr === HDR_PAVG && groundWarnShown(true) ? C.RED : C.BLACK);
      HF.setTextColor(pcol, C.WHITE);
      drawHeaderPitch(mtvx, pv, pcol);
      headerStrike(mtvx);
    } else {
      HF.setCursor(mtvx, 3);
      if (lnf(LS.TRACKWARN)) {
        if (blink1s()) {
          HF.setTextColor(C.RED, C.WHITE);
        } else {
          HF.fillRect(mtvx, 0, 111, 3, C.RED);
          HF.setTextColor(C.WHITE, C.RED, true);
        }
        HF.printf('%03d', truetrackDisplayDeg());
        HF.setTextColor(sel_col, C.WHITE);
      } else {
        HF.setTextColor(sel_col, C.WHITE);
        HF.printf('%03d', truetrackDisplayDeg());
      }
    }
    HF.drawFastHLine(0, 49, 240, C.BLACK);
    if (lnf(LS.GST_INVALID)) {
      const len = headerShowsPitch() ? (mtvx - 5 - 2) : (SCREEN_WIDTH - 10);
      for (let i = 0; i < 4; i++) HF.drawFastHLine(5, 22 + i, len, C.RED);
      headerStrikeTag(5, mtvx - 7, 'NO GNSS', C.RED, C.WHITE, C.RED);
    }
    // （受信機の受信枠はここで描かない）
    HF.pushTo(SCR, 0, 0);
  }

  // =====================================================================
  //  地図の上の線と印
  // =====================================================================
  function mapdataColor(name) {
    switch ((name || '')[0]) {
      case 'r': return C.RED;
      case 'o': return C.ORANGE;
      case 'g': return C.BRIGHTGRAY;
      case 'm': return C.MAGENTA;
      case 'c': return C.CYAN;
      case 'b': return C.BLUE;
      default: return C.GREEN;
    }
  }
  const checkWithin = (latdif, londif, lat1, lat2, lon1, lon2) => Math.abs(lat1 - lat2) < latdif && Math.abs(lon1 - lon2) < londif;
  function drawMap(mp, color) {
    if (mp.pts.length < 2) return;
    let prev = xy(mp.pts[0][0], mp.pts[0][1]);
    for (let i = 1; i < mp.pts.length; i++) {
      const cur = xy(mp.pts[i][0], mp.pts[i][1]);
      M.drawClippedLine(BS, prev.x, prev.y, cur.x, cur.y, color);
      prev = cur;
    }
  }
  // ★ 点がそろった地図だけ描く（受信機の一覧は少しずつ届く。途中・欠けのある地図を描くと未定義の点で落ちる）
  const mapComplete = (mp) => {
    if (mp.pts.length !== mp.n) return false;
    for (let i = 0; i < mp.pts.length; i++) if (!mp.pts[i]) return false;
    return true;
  };
  function drawPolyMaps(kind) {
    for (const mp of S.maps) {
      if (mp.kind !== kind || mp.pts.length <= 1 || !mapComplete(mp)) continue;
      if (!checkWithin(1, 1, mp.pts[0][0], S.lat, mp.pts[0][1], S.lon)) continue;
      drawMap(mp, mapdataColor(mp.name));
    }
  }

  function trackGap(lat0, lon0, lat1, lon1, mPerDegLon) {
    const dy = f32(f32(lat1 - lat0) * f32(111320.0));
    const dx = f32(f32(lon1 - lon0) * mPerDegLon);
    return f32(f32(dx * dx) + f32(dy * dy)) > K.TRACK_BREAK_M * K.TRACK_BREAK_M;
  }
  function drawTrack() {
    const trk = S.track;
    const n = trk.count();
    if (n <= 0) return;
    const mPerDegLon = f32(f32(111320.0) * f32(Math.cos(f32(M.deg2rad(S.lat)))));
    const c0 = trk.get(0);
    const p0 = xy(c0[0], c0[1]);
    if ((p0.x !== BACKSCREEN_SIZE / 2 || p0.y !== selfCy()) &&
        !trackGap(c0[0], c0[1], f32(S.lat), f32(S.lon), mPerDegLon)) {
      M.drawMapLine(BS, p0.x, p0.y, BACKSCREEN_SIZE / 2, selfCy(), 2, C.GREEN);
    }
    if (c0[0] === 0 && c0[1] === 0) return;
    let prev = p0, prevc = c0;
    for (let i = 0; i < n - 1; i++) {
      const c1 = trk.get(i + 1);
      if (c1[0] === 0 && c1[1] === 0) break;
      const cur = xy(c1[0], c1[1]);
      if ((prev.x !== cur.x || prev.y !== cur.y) && !trackGap(prevc[0], prevc[1], c1[0], c1[1], mPerDegLon))
        M.drawMapLine(BS, prev.x, prev.y, cur.x, cur.y, 2, C.GREEN);
      prev = cur; prevc = c1;
    }
  }

  function drawPilonLine() {
    const P = S.pilon;
    const pla = xy(P.plaLat, P.plaLon), n = xy(P.nLat, P.nLon), s = xy(P.sLat, P.sLon), tk = xy(P.tLat, P.tLon);
    M.drawClippedWideLine(BS, pla.x, pla.y, n.x, n.y, 3, C.BLUE);
    M.drawClippedLine(BS, pla.x, pla.y, tk.x, tk.y, C.GREEN);
    M.drawClippedWideLine(BS, pla.x, pla.y, s.x, s.y, 3, C.BLUE);
    const b = M.centerlineBearingRad(P);
    const inner = M.calculatePointFromBearing(P.plaLat, P.plaLon, b, 1.0);
    const outer = M.calculatePointFromBearing(P.plaLat, P.plaLon, b, 10.975);
    const ci = xy(inner[0], inner[1]), co = xy(outer[0], outer[1]);
    M.drawClippedWideLine(BS, ci.x, ci.y, co.x, co.y, 2, C.ORANGE);
    const cos3529 = Math.cos(35.29 * M.DEG_TO_RAD);
    BS.drawCircle(pla.x, pla.y, Math.trunc(f32(f32(S.scale) * f32(10.975)) / cos3529), C.GREEN);
    BS.drawCircle(pla.x, pla.y, Math.trunc(f32(f32(S.scale) * f32(1.0)) / cos3529), C.GREEN);
  }
  function drawPilonMarks() {
    const P = S.pilon;
    const pla = xy(P.plaLat, P.plaLon), n = xy(P.nLat, P.nLon), s = xy(P.sLat, P.sLon), k1 = xy(P.k1Lat, P.k1Lon);
    for (const p of [n, s, k1]) {
      if (M.isOutside(p)) continue;
      BS.fillTriangle(p.x - 3, p.y, p.x + 3, p.y, p.x, p.y - 9, C.ORANGE);
      BS.drawTriangle(p.x - 3, p.y, p.x + 3, p.y, p.x, p.y - 9, C.BLACK);
    }
    if (!M.isOutside(pla)) {
      BS.fillRect(pla.x - 3, pla.y - 2, 6, 8, C.BLUE);
      BS.fillTriangle(pla.x - 2, pla.y - 1, pla.x + 1, pla.y - 1, pla.x, pla.y + 3, C.YELLOW);
      BS.drawLine(pla.x - 2, pla.y - 2, pla.x + 2, pla.y - 2, C.RED);
      BS.drawLine(pla.x - 2, pla.y - 3, pla.x + 2, pla.y - 3, C.RED);
    }
  }

  function drawFlyinto(dlat, dlon, thickness) {
    let goal = xy(dlat, dlon);
    if (M.isOutside(goal)) {
      const d = f32(200 / f32(S.scale));
      const q = M.calculatePointD(S.lat, S.lon, dlat, dlon, d);
      goal = xy(q[0], q[1]);
    }
    M.drawClippedWideLine(BS, 120, selfCy(), goal.x, goal.y, thickness, C.MAGENTA);
  }
  function drawFlyinto2(dlat, dlon, thickness) {
    let goal = xy(dlat, dlon);
    const d = f32(200 / f32(S.scale));
    if (M.isOutside(goal)) {
      const q = M.calculatePointD(S.lat, S.lon, dlat, dlon, d);
      goal = xy(q[0], q[1]);
      M.drawClippedWideLine(BS, 120, selfCy(), goal.x, goal.y, thickness, C.MAGENTA);
    } else {
      M.drawClippedWideLine(BS, 120, selfCy(), goal.x, goal.y, thickness, C.MAGENTA);
      const q = M.calculatePointC(dlat, dlon, S.lat, S.lon, -d);
      const o = xy(q[0], q[1]);
      M.drawClippedWideLine(BS, goal.x, goal.y, o.x, o.y, 2, C.MAGENTA);
    }
  }
  function drawFlyawayfrom(dlat, dlon) {
    drawFlyinto(dlat, dlon, 1);
    const d = f32(200 / f32(S.scale));
    const q = M.calculatePointC(dlat, dlon, S.lat, S.lon, d);
    const t = xy(q[0], q[1]);
    M.drawClippedWideLine(BS, BACKSCREEN_SIZE / 2, selfCy(), t.x, t.y, 5, C.MAGENTA);
  }

  function drawDegpersec(dps) {
    if (Math.abs(dps) < 0.5) return;
    BS.loadFont(F.SMALL);
    let col = C.GRAY;
    if (gnssMps() > K.NAV_MIN_GS_MPS) {
      if (dps > 1.0) col = C.GREEN;
      if (dps < -1.0) col = C.BLUE;
      if (Math.abs(dps) > 3.0) {
        BS.setTextColor(C.WHITE, col);
        if (dps < 0) BS.fillRect(0, 15, 48, 35, col);
        else BS.fillRect(SCREEN_WIDTH - 48, 15, 48, 35, col);
        if (Math.abs(dps) > 4.0) {
          if (dps > 0) { BS.drawRect(SCREEN_WIDTH - 48, 15, 48, 35, C.RED); BS.drawRect(SCREEN_WIDTH - 47, 16, 46, 33, C.RED); }
          else { BS.drawRect(0, 15, 48, 35, C.RED); BS.drawRect(1, 16, 46, 33, C.RED); }
        }
      } else BS.setTextColor(col);
    } else BS.setTextColor(col);
    const posx = dps > 0 ? SCREEN_WIDTH - 45 : 5;
    BS.setTextWrap(false);
    BS.setCursor(posx, 17);
    BS.printf('%+.1f', dps);
    BS.setCursor(posx - 2, 31);
    BS.print('deg/s');
  }

  function tryDrawKmDistance(scale, km) {
    let latnow = S.lt.lat;
    if (latnow < -80 || latnow > 80) latnow = 35;
    const distance_px = Math.trunc(f32(f32(scale) * f32(km)) / Math.cos(latnow * M.DEG_TO_RAD));
    if (distance_px > 125) return false;
    const ypos = 240 - 2, xpos = 240 - 2 - 5;
    BS.drawFastVLine(xpos, ypos - distance_px, distance_px, C.BLACK);
    BS.drawFastHLine(xpos - 5, ypos, 5, C.BLACK);
    BS.drawFastHLine(xpos - 5, ypos - distance_px, 5, C.BLACK);
    BS.setTextSize(1);
    BS.setTextColor(C.BLACK);
    if (km >= 100) { BS.setCursor(xpos - 18, ypos - distance_px - 8); BS.printf('%d', Math.trunc(km)); }
    else if (km < 1.0) { BS.setCursor(xpos - 18, ypos - distance_px - 8); BS.printf('%.1f', f32(km)); }
    else { BS.setCursor(xpos - 11, ypos - distance_px - 8); BS.printf('%d', Math.trunc(km)); }
    BS.setCursor(xpos - 12, ypos - distance_px);
    BS.print('km');
    return true;
  }
  function drawKmDistances(scale) {
    const distances = [400, 200, 100, 50, 20, 10, 5, 2, 1, 0.4, 0.2];
    let n = 0;
    BS.unloadFont();
    for (const d of distances) {
      if (tryDrawKmDistance(scale, f32(d))) { n++; if (n >= 2) break; }
    }
    BS.loadFont(F.SMALL);
  }

  function drawNofixCross() {
    const cx = BACKSCREEN_SIZE / 2, cy = selfCy(), arm = 12;
    if (trackup()) {
      const sh = 15;
      BS.drawFastVLine(cx, sh, cy - sh - 5, C.BLACK);
      BS.drawFastVLine(cx + 1, sh, cy - sh - 5, C.BLACK);
    }
    BS.drawWideLine(cx - arm, cy - arm, cx + arm, cy + arm, 3, C.GRAY);
    BS.drawWideLine(cx + arm, cy - arm, cx - arm, cy + arm, 3, C.GRAY);
  }
  function drawHaccCircle(scale, hacc_mm) {
    const cx = BACKSCREEN_SIZE / 2, cy = selfCy();
    if (trackup()) {
      const sh = 15;
      BS.drawFastVLine(cx, sh, cy - sh - 5, C.BLACK);
      BS.drawFastVLine(cx + 1, sh, cy - sh - 5, C.BLACK);
    }
    const unknown = (hacc_mm === 0xFFFFFFFF);
    const accuracy_m = unknown ? f32(0) : f32(hacc_mm / f32(1000.0));
    const r = unknown ? 0 : roundf(f32(f32(accuracy_m / f32(1000.0)) * f32(scale)));
    BS.fillCircle(cx, cy, K.HDOP_CENTER_DOT_RADIUS, C.BLUE);
    const MAX_R = BACKSCREEN_SIZE / 2;
    if (!unknown && r >= K.HDOP_MIN_CIRCLE_RADIUS && r <= MAX_R) {
      BS.drawCircle(cx, cy, r, C.BLUE);
    } else {
      let buf;
      if (unknown) buf = 'Acc:--';
      else if (accuracy_m < 1000.0) buf = sprintf('Acc:%.0fm', accuracy_m);
      else buf = sprintf('Acc:%.1fkm', f32(accuracy_m / f32(1000.0)));
      BS.setTextSize(1);
      BS.setTextColor(C.BLUE, C.BLACK);
      BS.setCursor(cx + K.HDOP_CENTER_DOT_RADIUS + 2, cy - 4);
      BS.print(buf);
    }
  }

  // 自機アイコン（draw_plane_icon）
  function planeIconColor() {
    if (!S.lt.yaw.ok) return C.BLACK;
    return eskfYawReliable() ? C.BLACK : C.GRAY;
  }
  function drawPlaneIcon(cx, cy, screenDeg, col) {
    const a = f32(M.deg2rad(screenDeg));
    const ca = f32(Math.cos(a)), sa = f32(Math.sin(a));
    const HW = 6, SZ = 18;
    const lwx = Math.trunc(f32(cx - f32(HW * 2 * ca))), lwy = Math.trunc(f32(cy - f32(HW * 2 * sa)));
    const rwx = Math.trunc(f32(cx + f32(HW * 2 * ca))), rwy = Math.trunc(f32(cy + f32(HW * 2 * sa)));
    const ltx = Math.trunc(cx + f32((-HW / 2 | 0) * ca) - (0.5 * SZ) * sa);
    const lty = Math.trunc(cy + f32((-HW / 2 | 0) * sa) + (0.5 * SZ) * ca);
    const rtx = Math.trunc(cx + f32((HW / 2 | 0) * ca) - (0.5 * SZ) * sa);
    const rty = Math.trunc(cy + f32((HW / 2 | 0) * sa) + (0.5 * SZ) * ca);
    const btx = Math.trunc(f32(cx - f32((SZ / 2 | 0) * sa)));
    const bty = Math.trunc(f32(cy + f32((SZ / 2 | 0) * ca)));
    BS.drawWideLine(lwx, lwy, rwx, rwy, 3, col);
    BS.drawWideLine(ltx, lty, rtx, rty, 2, col);
    BS.drawWideLine(cx, cy, btx, bty, 2, col);
    if (eskfYawHighConfidence()) {
      const step = K.ESKF_DRIFT_LINE_DASH_PX + K.ESKF_DRIFT_LINE_SKIP_PX;
      for (let d = K.ESKF_DRIFT_LINE_GAP_PX; d < K.ESKF_DRIFT_LINE_GAP_PX + K.ESKF_DRIFT_LINE_LEN_PX; d += step) {
        const x0 = cx + lround(f32(sa * d)), y0 = cy - lround(f32(ca * d));
        const x1 = cx + lround(f32(sa * (d + K.ESKF_DRIFT_LINE_DASH_PX))), y1 = cy - lround(f32(ca * (d + K.ESKF_DRIFT_LINE_DASH_PX)));
        BS.drawWideLine(x0, y0, x1, y1, 2, col);
      }
    }
  }
  function planeIconScreenDeg(ttrack) {
    if (!eskfYawReliable() || !S.lt.yaw.ok) return trackup() ? 0 : ttrack;
    return trackup() ? (S.yawDraw - ttrack) : S.yawDraw;
  }

  // 旋回指示の赤い弧と三角（draw_steer_overlay。push_backscreen で描く）
  let steerOverlay = null;
  function drawSteerOverlay() {
    if (!steerOverlay) return;
    const { ttrack, steer } = steerOverlay;
    steerOverlay = null;
    if (Math.floor(NOW / 1000) % 3 === 0) return;
    const nlen = needleLen();
    const cwi = f32(S.ln.cwi);
    const arcFactor = f32(cwi / 900.0);
    const arcLen = Math.trunc(f32(arcFactor * steer));
    const tri = (cx, cy, len, ang) => {
      const st = [[0.1, 0.35], [0.7, 0.95], [1.3, 1.55]];
      const th = [15, 55, 100];
      for (let k = 0; k < 3; k++) {
        if (Math.abs(steer) <= th[k]) continue;
        const s0 = ang + (steer < 0 ? -st[k][0] : st[k][0]);
        const s1 = ang + (steer < 0 ? -st[k][1] : st[k][1]);
        const x1 = f32((len - 12) * Math.sin(s0) + cx), y1 = f32((len - 12) * -Math.cos(s0) + cy);
        const x2 = f32((len - 32) * Math.sin(s0) + cx), y2 = f32((len - 32) * -Math.cos(s0) + cy);
        const x3 = f32((len - 22) * Math.sin(s1) + cx), y3 = f32((len - 22) * -Math.cos(s1) + cy);
        BS.fillTriangle(x1, y1, x2, y2, x3, y3, C.RED);
      }
    };
    if (!trackup()) {
      const tt = Math.trunc(ttrack);
      const ttr = M.deg2rad(tt);
      if (steer > 0) BS.drawArc(120, 120, nlen - 21, nlen - 23, (tt + 180) % 360, (tt + 180 + arcLen) % 360, C.RED, C.WHITE);
      else BS.drawArc(120, 120, nlen - 21, nlen - 23, (tt + 180 + arcLen) % 360, (tt + 180) % 360, C.RED, C.WHITE);
      tri(120, 120, nlen, ttr);
    } else {
      const cy = BACKSCREEN_SIZE * 3 / 4;
      const slen = nlen - 24;
      if (steer > 0) BS.drawArc(120, cy, slen - 21, slen - 23, 180, (180 + arcLen) % 360, C.RED, C.WHITE);
      else BS.drawArc(120, cy, slen - 21, slen - 23, (360 + 180 + arcLen) % 360, 180, C.RED, C.WHITE);
      tri(120, cy, slen, 0);
    }
  }

  // draw_triangle(ttrack, steer_angle)
  function drawTriangle(ttrack, steer) {
    steerOverlay = { ttrack, steer };
    const nlen = needleLen();
    const ltt = f32(S.ln.ltt);
    const warn = lnf(LS.TRACKWARN);
    if (!trackup()) {
      const tr = M.deg2rad(ttrack);
      const L = f32(M.deg2rad(f32(ltt - 15))), R = f32(M.deg2rad(f32(ltt + 15))), Cc = f32(M.deg2rad(ltt));
      const x1 = f32((nlen - 6) * Math.sin(L) + 120), y1 = f32((nlen - 6) * -Math.cos(L) + 120);
      const x2 = f32((nlen - 6) * Math.sin(R) + 120), y2 = f32((nlen - 6) * -Math.cos(R) + 120);
      const x3 = f32(nlen * Math.sin(Cc) + 120), y3 = f32(nlen * -Math.cos(Cc) + 120);
      if (!warn) BS.drawTriangle(x1, y1, x2, y2, x3, y3, C.BLACK);
      else BS.fillTriangle(x1, y1, x2, y2, x3, y3, blink1s() ? C.RED : C.BLACK);
      drawPlaneIcon(120, 120, planeIconScreenDeg(ttrack), planeIconColor());
      BS.drawWideLine(120 + Math.sin(tr) * 9, 120 - Math.cos(tr) * 9, 120 + Math.sin(tr) * nlen, 120 - Math.cos(tr) * nlen, 5, C.BLACK);
    } else {
      const cy = BACKSCREEN_SIZE * 3 / 4;
      const sh = headerShowsPitch() ? 28 : 16;
      BS.drawFastVLine(120, sh, cy - sh - 5, C.BLACK);
      BS.drawFastVLine(121, sh, cy - sh - 5, C.BLACK);
      drawPlaneIcon(120, cy, planeIconScreenDeg(ttrack), planeIconColor());
      const tlen = nlen - 26;
      const st = f32(ltt - ttrack);
      const L = f32(M.deg2rad(st - 15)), R = f32(M.deg2rad(st + 15)), Cc = f32(M.deg2rad(st));
      const x1 = f32((tlen - 6) * Math.sin(L) + 120), y1 = f32((tlen - 6) * -Math.cos(L) + cy);
      const x2 = f32((tlen - 6) * Math.sin(R) + 120), y2 = f32((tlen - 6) * -Math.cos(R) + cy);
      const x3 = f32(tlen * Math.sin(Cc) + 120), y3 = f32(tlen * -Math.cos(Cc) + cy);
      if (!warn) BS.drawTriangle(x1, y1, x2, y2, x3, y3, C.BLACK);
      else BS.fillTriangle(x1, y1, x2, y2, x3, y3, blink1s() ? C.RED : C.BLACK);
    }
  }

  function drawCompass(up, col) {
    BS.setTextWrap(false);
    const dist = trackup() ? (((73 * 2 / 3) | 0) - 4) : (((100 * 2 / 3) | 0) - 4);
    const cx = BACKSCREEN_SIZE / 2, cy = selfCy();
    const rad = f32(M.deg2rad(up));
    BS.setTextColor(col, C.WHITE);
    BS.setTextSize(2);
    BS.loadFont(F.SMALL);
    const sn = f32(Math.sin(-rad)), cs = f32(Math.cos(rad)), sp = f32(Math.sin(rad)), cn = f32(Math.cos(-rad)), sm = f32(Math.sin(-rad));
    const n = { x: Math.trunc(f32(cx + f32(sn * dist))), y: Math.trunc(f32(cy - f32(cs * dist))) };
    const e = { x: Math.trunc(f32(cx + f32(cs * dist))), y: Math.trunc(f32(cy - f32(sp * dist))) };
    const s = { x: Math.trunc(f32(cx + f32(sp * dist))), y: Math.trunc(f32(cy + f32(cs * dist))) };
    const w = { x: Math.trunc(f32(cx - f32(cn * dist))), y: Math.trunc(f32(cy - f32(sm * dist))) };
    if (!M.isOutside(n)) { BS.setTextColor(C.RED, C.WHITE); BS.setCursor(n.x - 5, n.y - 5); BS.print('N'); BS.setTextColor(col, C.WHITE); }
    if (!M.isOutside(e)) { BS.setCursor(e.x - 5, e.y - 5); BS.print('E'); }
    if (!M.isOutside(s)) { BS.setCursor(s.x - 5, s.y - 5); BS.print('S'); }
    if (!M.isOutside(w)) { BS.setCursor(w.x - 5, w.y - 5); BS.print('W'); }
    BS.setTextWrap(true);
  }

  // =====================================================================
  //  姿勢・ヨー（draw_eskf_attitude / draw_eskf_yaw）
  // =====================================================================
  const ESKF_LABEL_X = 2, WIND_BLOCK_RIGHT_X = BACKSCREEN_SIZE - 2 - 5 - 18 - 3, WIND_TEXT_RIGHT_X = BACKSCREEN_SIZE - 2 - 5;
  const ESKF_VALUE_RIGHT_X = 50, ESKF_ROW_GAP = 1, ESKF_BOTTOM_GAP = 2;
  function rollTrimColor(trim) {
    const a = Math.abs(trim);
    if (a >= K.ROLL_TRIM_ALERT_DEG) return C.RED;
    if (a >= K.ROLL_TRIM_WARN_DEG) return C.ORANGE;
    return C.GRAY;
  }
  function windArrowColor(mps) {
    if (mps <= K.WIND_COL_LOW_MAX) return C.GREEN;
    if (mps <= K.WIND_COL_MID_MAX) return C.WIND_PURPLE;
    return C.RED;
  }
  function drawWindArrow(cx, cy, screenDeg, col, dirTo) {
    const a = f32(M.deg2rad(screenDeg));
    const ca = f32(Math.cos(a)), sa = f32(Math.sin(a));
    const h = (K.WIND_ARROW_LEN_PX / 2) | 0;
    const tipx = cx + lround(f32(sa * h)), tipy = cy - lround(f32(ca * h));
    const tailx = cx - lround(f32(sa * h)), taily = cy + lround(f32(ca * h));
    const buf = sprintf('%03d', ((lround(f32(dirTo + 180.0)) % 360) + 360) % 360);
    BS.loadFont(F.SMALL);
    const tw = BS.textWidth(buf), th = BS.fontHeight();
    const hw = K.WIND_ARROW_HEAD_PX;
    const bx = cx + lround(f32(sa * (h - hw))), by = cy - lround(f32(ca * (h - hw)));
    const gap = lround(f32(f32(Math.abs(sa) * tw * 0.5) + f32(Math.abs(ca) * th * 0.5))) + 2;
    const gx = lround(f32(sa * gap)), gy = lround(f32(ca * gap));
    if (gap < h) BS.drawWideLine(tailx, taily, cx - gx, cy + gy, K.WIND_ARROW_WIDTH_PX, col);
    if (h - hw - gap >= K.WIND_ARROW_WIDTH_PX) BS.drawWideLine(cx + gx, cy - gy, bx, by, K.WIND_ARROW_WIDTH_PX, col);
    const px = lround(f32(ca * hw * f32(0.55))), py = lround(f32(sa * hw * f32(0.55)));
    BS.fillTriangle(tipx, tipy, bx + px, by + py, bx - px, by - py, col);
    BS.setTextColor(C.BLACK);
    BS.setCursor(cx - (tw / 2 | 0), cy - (th / 2 | 0));
    BS.print(buf);
  }

  let eskfWarnY = -1;   // 較正のやり直しの警告の位置（draw_eskf_attitude が決め、push で描く）
  function drawEskfAttitude() {
    if (!eskfDisplayEnabled()) return;
    if (!S.lt.att.ok) return;
    const r = S.lt.att.roll, p = S.lt.att.pitch;
    const rbuf = sprintf('%+5.1f', r), pbuf = sprintf('%+5.1f', p);
    BS.setTextWrap(false);
    BS.loadFont(F.SMALL);
    const fh_s = BS.fontHeight();
    const yrtrim = BACKSCREEN_SIZE - 9 - 8 - ESKF_BOTTOM_GAP;
    const yroll = yrtrim - fh_s - ESKF_ROW_GAP;
    const ypit = yroll - fh_s - ESKF_ROW_GAP;
    const ypavg = ypit - fh_s - ESKF_ROW_GAP;
    const pa = pitchAvg();
    {
      const has_wind = lnf(LS.WIND) && S.lt.wind.ok && S.lt.tas.ok;
      const wspd = S.lt.wind.spd, wdir = S.lt.wind.dir;
      const yblock_top = pa.ok ? ypavg : ypit;
      if (has_wind) {
        const nbuf = sprintf('%.1f', wspd);
        const wlw = BS.textWidth('W');
        const uw = BS.textWidth('m/s');
        BS.loadFont(F.MEDIUM);
        const fh_m = BS.fontHeight();
        const nw = BS.textWidth(nbuf);
        const wbot = BACKSCREEN_SIZE - 9 - ESKF_BOTTOM_GAP;
        const wtext_w = wlw + 3 + nw + 2 + uw;
        const wtext_x = WIND_TEXT_RIGHT_X - wtext_w;
        const ywtxt = wbot - fh_m;
        const arrow_top = ywtxt - ESKF_ROW_GAP - K.WIND_ARROW_LEN_PX;
        const ylbl = ywtxt + (((fh_m - fh_s) / 2) | 0);
        BS.setTextColor(C.BLACK);
        BS.setCursor(wtext_x + wlw + 3, ywtxt);
        BS.print(nbuf);
        BS.loadFont(F.SMALL);
        BS.setCursor(wtext_x, ylbl);
        BS.print('W');
        BS.setCursor(wtext_x + wlw + 3 + nw + 2, ylbl);
        BS.print('m/s');
        const wstale = egs() === EGS_NO_GNSS;
        const wscreen = trackup() ? f32(wdir - f32(S.up)) : wdir;
        const acx = WIND_BLOCK_RIGHT_X - ((K.WIND_ARROW_LEN_PX / 2) | 0) - 2;
        const acy = arrow_top + ((K.WIND_ARROW_LEN_PX / 2) | 0);
        drawWindArrow(acx, acy, wscreen, wstale ? C.GRAY : windArrowColor(wspd), wdir);
        if (wstale) {
          const ax_l = acx - ((K.WIND_ARROW_LEN_PX / 2) | 0);
          const x0 = (ax_l < wtext_x) ? ax_l : wtext_x, x1 = WIND_TEXT_RIGHT_X;
          const y0 = arrow_top, y1 = ywtxt + fh_m;
          BS.drawWideLine(x0, y0, x1, y1, K.WIND_STALE_CROSS_PX, C.ORANGE);
          BS.drawWideLine(x1, y0, x0, y1, K.WIND_STALE_CROSS_PX, C.ORANGE);
        }
      }
      if (lnf(LS.NEEDS_APPLY) && Math.floor(NOW / 1000) % 3 === 0) {
        // ミラー中なので「ボートなら ESKF を切る」は添えない
        eskfWarnY = yblock_top - ESKF_ROW_GAP - fh_s;
      }
    }
    const gpitch = groundWarnShown(true);
    BS.setTextColor(C.GRAY);
    BS.setCursor(ESKF_LABEL_X, ypit);
    BS.print('P');
    BS.setTextColor(attitudeImplausible(p) ? C.GRAY : (gpitch ? C.RED : C.BLACK));
    BS.setCursor(ESKF_VALUE_RIGHT_X - BS.textWidth(pbuf), ypit);
    BS.print(pbuf);
    // （地上チェックの許容範囲は電波に載っていないのでミラー中は出さない。draw_ground_ok_range）
    const gst = egs();
    if (pa.ok) {
      const avgBuf = sprintf('%+.1f', pa.v);
      const valueX = ESKF_LABEL_X + drawAveragePitchLabel(ESKF_LABEL_X, ypavg, C.GRAY) + 4;
      BS.setTextColor(attitudeImplausible(pa.v) ? C.GRAY : (gpitch ? C.RED : C.BLACK));
      BS.setCursor(valueX, ypavg);
      BS.print(avgBuf);
      if (gst !== EGS_OK)
        strikeThrough(BS, ESKF_LABEL_X, ypavg + (fh_s / 2 | 0), valueX + BS.textWidth(avgBuf) - ESKF_LABEL_X,
          gst === EGS_YAW_ALIGN ? C.YELLOW : C.ORANGE);
    }
    if (gst !== EGS_OK)
      strikeThrough(BS, ESKF_LABEL_X, ypit + (fh_s / 2 | 0), ESKF_VALUE_RIGHT_X - ESKF_LABEL_X, gst === EGS_YAW_ALIGN ? C.YELLOW : C.ORANGE);
    BS.setTextColor(C.GRAY);
    BS.setCursor(ESKF_LABEL_X, yroll);
    BS.print('R');
    const groll = groundWarnShown(false);
    BS.setTextColor(attitudeImplausible(r) ? C.GRAY : ((Math.abs(r) > K.BANK_WARN_DEG || groll) ? C.RED : C.BLACK));
    BS.setCursor(ESKF_VALUE_RIGHT_X - BS.textWidth(rbuf), yroll);
    BS.print(rbuf);
    if (gst !== EGS_OK)
      strikeThrough(BS, ESKF_LABEL_X, yroll + (fh_s / 2 | 0), ESKF_VALUE_RIGHT_X - ESKF_LABEL_X, gst === EGS_YAW_ALIGN ? C.YELLOW : C.ORANGE);
    BS.setTextColor(C.BLACK);
    if (S.lt.trim.ok) {
      BS.unloadFont();
      BS.setTextSize(1);
      BS.setTextColor(rollTrimColor(S.lt.trim.v));
      BS.setCursor(ESKF_LABEL_X, yrtrim);
      BS.printf('Rtrim%+.2f', S.lt.trim.v);
      BS.setTextColor(C.BLACK);
    }
    BS.loadFont(F.SMALL);
  }

  function drawEskfYaw() {
    if (!eskfDisplayEnabled()) return;
    if (!S.lt.yaw.ok) return;
    const y = S.lt.yaw.yaw;
    const buf = sprintf('%03d', ((lround(f32(y)) % 360) + 360) % 360);
    BS.loadFont(F.SMALL);
    BS.setTextWrap(false);
    const yrow = selfCy() + 9 + 3;
    const fh = BS.fontHeight();
    const YAW_3DIGIT_W = 27, PM_GAP_L = 1, PM_W = 6, PM_GAP_R = 1, x0 = BACKSCREEN_SIZE / 2 - 22;
    BS.setTextColor(eskfYawReliable() ? C.BLACK : C.GRAY);
    BS.setCursor(x0, yrow);
    BS.print(buf);
    {
      const abuf = sprintf('%.0f', S.lt.yaw.acc95);
      const x_pm = x0 + YAW_3DIGIT_W + PM_GAP_L;
      const x_acc = x_pm + PM_W + PM_GAP_R;
      const aw = BS.textWidth(abuf);
      const aligning = egs() === EGS_YAW_ALIGN;
      const acol = aligning ? C.BLACK : C.GRAY;
      if (aligning) BS.fillRect(x_pm - 1, yrow - 1, (x_acc + aw) - x_pm + 2, fh + 1, C.YELLOW);
      BS.fillRect(x_pm, yrow + 4, PM_W, 2, acol);
      BS.fillRect(x_pm + 2, yrow + 2, 2, 6, acol);
      BS.fillRect(x_pm, yrow + 10, PM_W, 2, acol);
      BS.setTextColor(acol);
      BS.setCursor(x_acc, yrow);
      BS.print(abuf);
    }
    BS.setTextColor(C.BLACK);
  }

  // =====================================================================
  //  地図の上端の文字（draw_gs_track）・下端（draw_map_footer）
  // =====================================================================
  function drawAveragePitchLabel(x, y, color) {
    BS.setTextColor(color);
    const width = BS.drawString('P', x, y + 2);
    BS.fillRect(x + 1, y, width - 1, 2, color);
    return width;
  }

  const AVERAGE_ALPHA_LABEL_WIDTH = 12;
  const AVERAGE_ALPHA_ROWS = [0x3800, 0x6CC0, 0xC680, 0xC380, 0xC300, 0xC700, 0x6D80, 0x38C0];
  function drawAverageAlphaLabel(x, y, color) {
    BS.fillRect(x + 1, y + 1, 10, 2, color);
    for (let row = 0; row < AVERAGE_ALPHA_ROWS.length; row++) {
      for (let col = 0; col < 10; col++) {
        if (AVERAGE_ALPHA_ROWS[row] & (0x8000 >> col))
          BS.drawPixel(x + 1 + col, y + 4 + row, color);
      }
    }
  }

  function drawTasLabelRow(h, leftLimit) {
    const box = tasBoxColor(h, tasHeaderValue());
    const fh = BS.fontHeight();
    const tw = BS.textWidth('TAS');
    const bw = tw + 6, bx = SCREEN_WIDTH - bw;
    BS.fillRect(bx, 0, bw, fh + 2, box);
    BS.setTextColor(C.WHITE, box);
    BS.drawString('TAS', bx + 3, 3);
    BS.fillRect(bx + 3, 1, tw, 2, C.WHITE);
    const a = S.lt.alpha;
    if (a.ok) {
      const buf = Math.abs(a.v) < f32(9.95) ? sprintf('%+.1f', a.v) : sprintf('%+.0f', a.v);
      const w = AVERAGE_ALPHA_LABEL_WIDTH + BS.textWidth(buf), x = bx - 3 - w;
      if (x >= leftLimit + 3) {
        BS.setTextColor(C.BLACK, C.WHITE);
        drawAverageAlphaLabel(x, 2, C.BLACK);
        BS.drawString(buf, x + AVERAGE_ALPHA_LABEL_WIDTH, 3);
        const gst = egs();
        if (gst !== EGS_OK) strikeThrough(BS, x, 3 + (fh / 2 | 0), w,
          gst === EGS_YAW_ALIGN ? C.YELLOW : C.ORANGE);
      }
    }
    BS.setTextColor(C.BLACK, C.WHITE);
  }

  function drawGsTrack() {
    BS.setTextWrap(false);
    const hdr = S.ln.hdr;
    const hdrPitch = hdr !== HDR_TT, hdrTas = isTasHdr(hdr);
    if (hdr === HDR_ALPHA) {
      drawAverageAlphaLabel(SCREEN_WIDTH - 61, 1, C.BLACK);
    } else if (hdr === HDR_PAVG) {
      drawAveragePitchLabel(SCREEN_WIDTH - 60, 1, C.BLACK);
    } else if (!hdrTas) {
      const warn = lnf(LS.TRACKWARN) && blink1s();
      BS.setTextColor(warn ? C.RED : C.BLACK, C.WHITE);
      BS.drawString('TT', SCREEN_WIDTH - 40, 1);
    }
    BS.setTextColor(C.BLACK, C.WHITE);
    BS.drawString('GS', 1, 1);
    let ttRight = SCREEN_WIDTH / 2;
    if (hdrPitch) {
      BS.setTextColor(C.BLACK, C.WHITE);
      const lw = BS.textWidth('TT');
      const lh = BS.fontHeight();
      const buf = sprintf('%03d', truetrackDisplayDeg());
      BS.loadFont(F.MEDIUM);
      const nw = BS.textWidth(buf);
      const nh = BS.fontHeight();
      const x0 = ((SCREEN_WIDTH - (lw + 4 + nw)) / 2) | 0;
      ttRight = x0 + lw + 4 + nw;
      BS.setCursor(x0 + lw + 4, 1);
      BS.print(buf);
      BS.loadFont(F.SMALL);
      BS.setCursor(x0, 1 + (((nh - lh) / 2) | 0));
      BS.print('TT');
      if (lnf(LS.GST_INVALID)) {
        for (let i = 0; i < 4; i++) BS.drawFastHLine(x0, 1 + (nh / 2 | 0) - 2 + i, lw + 4 + nw, C.RED);
      }
    }
    if (hdrTas) drawTasLabelRow(hdr, ttRight);
    {
      const gsw = BS.textWidth('GS');
      const gsh = BS.fontHeight();
      const sacc = S.lt.sacc;   // {ok, mps}
      BS.unloadFont();
      BS.setTextSize(1);
      const sacc_col = !sacc.ok ? C.GRAY : (sacc.mps < 0.3) ? C.GREEN : (sacc.mps < 3.0) ? C.ORANGE : C.RED;
      const sbuf = !sacc.ok ? 'sAcc --' : sprintf('sAcc%.2f', sacc.mps);
      const sx = 1 + gsw + 4;
      const sy = 1 + (((gsh - 8) / 2) | 0);
      if (sacc.ok && sacc.mps >= 0.3) {
        const bx = sx - 2, by = (sy >= 2) ? (sy - 2) : 0;
        const bw = BS.textWidth(sbuf) + 3, bh = 11;
        BS.fillRect(bx, by, bw, bh, C.WHITE);
        BS.drawRect(bx, by, bw, bh, sacc_col);
      }
      BS.setTextColor(sacc_col);
      BS.setCursor(sx, sy);
      BS.print(sbuf);
      BS.setTextColor(C.BLACK, C.WHITE);
      BS.loadFont(F.SMALL);
    }
  }

  function drawMapFooter() {
    BS.unloadFont();
    BS.setTextSize(1);
    BS.setTextColor(C.BLACK);
    if (S.time.valid) {
      const sec = ((S.time.sec + 9 * 3600) % 86400 + 86400) % 86400;
      BS.setCursor(1, BACKSCREEN_SIZE - 9);
      BS.printf('%02lu:%02lu:%02luJST', Math.floor(sec / 3600), Math.floor(sec / 60) % 60, sec % 60);
    }
    BS.setCursor(75, BACKSCREEN_SIZE - 9);
    const g = S.ln;
    if (f32(g.mg5) === f32(g.mg)) BS.printf('MaxGS %.1f(%02d:%02d)', f32(g.mg), g.mgh, g.mgm);
    else BS.printf('MaxGS%.1f(%02d:%02d)%.1f(%02d:%02d)', f32(g.mg5), g.mg5h, g.mg5m, f32(g.mg), g.mgh, g.mgm);
    BS.loadFont(F.SMALL);
  }

  // =====================================================================
  //  VSI と push_backscreen の上の層
  // =====================================================================
  function drawVsi() {
    const vs = lnf(LS.VARIO_OK) ? S.lt.vs : 0;
    VSI.fillSprite(C.BLACK);
    VSI.drawFastHLine(0, 120, VSI_W, C.WHITE);
    if (!lnf(LS.VARIO_OK)) return;
    const MAXV = f32(1.5), BAR = 120;
    if (vs > 0) {
      const bar = Math.trunc(f32(f32(Math.min(vs, MAXV) / MAXV) * BAR));
      VSI.fillRect(0, 120 - bar, VSI_W, bar, C.TFT_GREEN);
    } else if (vs < 0) {
      const bar = Math.trunc(f32(f32(Math.min(-vs, MAXV) / MAXV) * BAR));
      VSI.fillRect(0, 121, VSI_W, bar, C.TFT_CYAN);
    }
    const db = f32(S.ln.db);
    const dbpx = Math.trunc(f32(f32(f32(db / MAXV) * BAR) + f32(0.5)));
    VSI.drawFastHLine(0, 120 - dbpx, VSI_W, C.TFT_DARKGREY);
    VSI.drawFastHLine(0, 120 + dbpx, VSI_W, C.TFT_DARKGREY);
    VSI.drawFastHLine(0, 80, VSI_W, C.WHITE);
    VSI.drawFastHLine(0, 160, VSI_W, C.WHITE);
    VSI.drawFastHLine(0, 40, VSI_W, C.WHITE);
    VSI.drawFastHLine(0, 200, VSI_W, C.WHITE);
  }

  let cwPending = false, cwAngle = 0;
  function drawCourseWarning(steer) {
    noticeReserve(trackup() ? (selfCy() + 12) : 175, 25);
    cwPending = true; cwAngle = steer;
  }
  function drawCourseWarningBox() {
    if (!cwPending) return;
    cwPending = false;
    if (blink1s()) {
      const by = trackup() ? (selfCy() + 12) : 175;
      BS.fillRect(5, by, SCREEN_WIDTH - 10, 25, C.WHITE);
      BS.drawRect(5, by, SCREEN_WIDTH - 10, 25, C.RED);
      BS.drawRect(6, by + 1, SCREEN_WIDTH - 12, 23, C.RED);
      BS.setTextColor(C.RED);
      if (cwAngle > 0) { BS.setCursor(23, by + 5); BS.print('Turn RIGHT! Turn RIGHT!'); }
      else { BS.setCursor(30, by + 5); BS.print('Turn LEFT! Turn LEFT!'); }
      BS.setTextColor(C.BLACK, C.WHITE);
    }
  }
  function beginNoticeBox(x, y, w, h, col) {
    noticeReserve(y, h);
    BS.fillRect(x, y, w, h, C.WHITE);
    BS.drawRect(x, y, w, h, col);
    BS.drawRect(x + 1, y + 1, w - 2, h - 2, col);
    BS.setTextColor(col, C.WHITE);
  }
  function drawEskfCalibWarning() {
    if (eskfWarnY < 0) return;
    const yw = eskfWarnY;
    eskfWarnY = -1;
    BS.loadFont(F.SMALL);
    const fh = BS.fontHeight();
    const l1 = 'ESKF CALIBRATION REQUIRED!';
    BS.fillRect(ESKF_LABEL_X - 1, yw - 1, BS.textWidth(l1) + 2, fh + 1, C.WHITE);
    BS.setTextColor(C.RED, C.WHITE);
    BS.setCursor(ESKF_LABEL_X, yw);
    BS.print(l1);
    BS.setTextColor(C.BLACK, C.WHITE);
  }
  // 電源経路の異常（送信機の status の LINK_ST_PWR_FAULT。機体の画面に出ている箱）
  function drawPowerFaultWarning() {
    if (!(S.lt.status & LINK_ST_PWR_FAULT)) return;
    const bx = 5, by = K.NOTICE_Y_PWR_FAULT, bw = SCREEN_WIDTH - 10, bh = 40;
    beginNoticeBox(bx, by, bw, bh, C.RED);
    BS.loadFont(F.SMALL);
    BS.setCursor(12, by + 5);
    BS.print('POWER FAULT !!');
    BS.setCursor(12, by + 21);
    BS.print(K.POWER_FAULT_ACTION_TEXT);
    BS.setTextColor(C.BLACK, C.WHITE);
  }
  function drawNomapdata() {
    const scanning = lnf(LS.SCANNING), weak = lnf(LS.WEAK);
    if (!scanning && !weak) return;
    const bx = 5, bw = SCREEN_WIDTH - 10, bh = 40;
    let by = trackup() ? (selfCy() + 12) : 145;
    if (!noticeFree(by, bh)) {
      for (const y of [192, 148, 104, 82, 40]) if (noticeFree(y, bh)) { by = y; break; }
    }
    const col = scanning ? C.ORANGE : C.RED;
    beginNoticeBox(bx, by, bw, bh, col);
    BS.setTextSize(1);
    BS.loadFont(F.SMALL);
    const dots = '..........'.slice(0, Math.floor(NOW / 900) % 10);
    if (scanning) { BS.setCursor(42, by + 5); BS.print('Scanning GNSS Signal'); BS.setCursor(45, by + 20); BS.print('Stand by for fix' + dots); }
    else { BS.setCursor(56, by + 5); BS.print('Weak GNSS Signal'); BS.setCursor(45, by + 20); BS.print('Scanning signal' + dots); }
    BS.setTextColor(C.BLACK, C.WHITE);
  }
  function pushBackscreen() {
    // （受信機の受信枠・アイコン・NO SIGNAL などの箱はここで描かない）
    drawSteerOverlay();
    drawCourseWarningBox();
    if (lnf(LS.VSI)) { drawVsi(); VSI.pushTo(BS, VSI_X, 0); }
    drawEskfCalibWarning();
    drawPowerFaultWarning();
    drawNomapdata();
    BS.pushTo(SCR, 0, 50);
    RES = [];
  }

  // =====================================================================
  //  フッター（draw_footer。電池の欄は機体の画面の分かれ道で描く）
  // =====================================================================
  const batteryPercent = (v) => Math.min(100, Math.max(0, Math.trunc(f32(f32(f32(v) - f32(K.BAT_ZERO_VOLTAGE)) / f32(K.BAT_FULL_VOLTAGE - K.BAT_ZERO_VOLTAGE) * 100.0))));
  const batteryLowPct = (pct) => pct <= batteryPercent(K.BAT_LOW_VOLTAGE);
  const batteryColorPct = (pct) => batteryLowPct(pct) ? C.RED : (pct < batteryPercent(K.BAT_HALF_VOLTAGE) ? C.MAGENTA : C.GREEN);
  function drawFooter() {
    HF.setTextColor(C.BLACK, C.WHITE);
    HF.fillSprite(C.WHITE);
    HF.loadFont(F.SMALL);
    HF.drawFastHLine(0, 0, 240, C.BLACK);
    const ln = S.ln;
    const dest = (ln.dest >= 0) ? S.dests[ln.dest] : null;
    if (dest) {
      const navValid = lnf(LS.NAV_VALID);
      HF.setCursor(1, 1);
      HF.setTextColor(C.MAGENTA);
      if (!navValid) HF.print('TC---');
      else HF.printf('TC%3d', lround(f32(ln.truec)) % 360);
      HF.setCursor(60, 1);
      HF.setTextColor(C.BLACK);
      const dd = f32(ln.dist);
      if (!navValid) HF.print('---km');
      else HF.printf(dd > 1000 ? '%.0fkm' : dd > 100 ? '%.1fkm' : '%.2fkm', dd);
      HF.setCursor(1, 17);
      if (ln.dmode === DMODE_FLYAWAY) { HF.setTextColor(C.DARKORANGE); HF.print('FLY AWAY'); }
      if (ln.dmode === DMODE_FLYINTO) { HF.setTextColor(C.BLUE); HF.print('FLY INTO'); }
      if (ln.dmode === DMODE_AUTO10K) {
        if (ln.a10k === AUTO10K_INTO) { HF.setTextColor(C.BLUE); HF.print('10K INTO'); }
        if (ln.a10k === AUTO10K_AWAY) { HF.setTextColor(C.DARKORANGE); HF.print('10K AWAY'); }
      }
      HF.setTextColor(C.MAGENTA);
      HF.setCursor(80, 17);
      HF.setTextWrap(false);
      HF.print(dest.name);
    }
    // ---- 電池（機体の画面の欄。無線の「電池の表示」は送信機のヘッダーの電池の欄そのもの）----
    HF.setCursor(SCREEN_WIDTH - 37, 1);
    HF.setTextColor(C.GREEN);
    const b = S.lt.batt;
    if (S.lt.status & LINK_ST_PWR_FAULT) {
      HF.setCursor(SCREEN_WIDTH - 45, 1);
      if (blink1s()) { HF.fillRect(SCREEN_WIDTH - 47, 0, 47, 15, C.RED); HF.setTextColor(C.WHITE, C.RED); }
      else HF.setTextColor(C.RED);
      HF.print('PWR!');
    } else if (b.kind === BATT_SW_OFF || b.kind === BATT_CHARGING) {
      let chg, col;
      // ★ 本体の "REPLUG"（USB のまま SW が入って挿し直し待ち。power_charge_replug_needed()）も無線では SW OFF と同じ番号で来るので、
      //   ここでは "SW OFF" になる（見分ける番号は無い。どちらも電池は充電されていない USB 給電）。2026-11-25〜
      if (b.kind === BATT_SW_OFF) { chg = 'SW OFF'; col = C.DARKORANGE; }
      else if (b.pct >= 100) { chg = 'FULL'; col = C.GREEN; }
      else { chg = sprintf('CHG %d%%', b.pct); col = C.BLUE; }
      let w = HF.textWidth(chg);
      const small = w > 48;
      if (small) { HF.unloadFont(); HF.setTextSize(1); w = 6 * chg.length; }
      HF.setCursor(SCREEN_WIDTH - 2 - w, small ? 4 : 1);
      HF.setTextColor(col);
      HF.print(chg);
      if (small) HF.loadFont(F.SMALL);
    } else if (b.kind === BATT_BATTERY) {
      HF.setCursor(SCREEN_WIDTH - 45, 1);
      if (batteryLowPct(b.pct)) {
        if (!blink1s()) HF.setTextColor(C.RED);
        else { HF.fillRect(SCREEN_WIDTH - 47, 0, 47, 15, C.RED); HF.setTextColor(C.WHITE, C.RED); }
        HF.printf('%d%%', b.pct);
      } else {
        HF.setTextColor(batteryColorPct(b.pct));
        HF.printf('%d%%', b.pct);
      }
    } else {
      // まだ分からない（回転枠 4 を受けるまで）
      HF.setCursor(SCREEN_WIDTH - 45, 1);
      HF.setTextColor(C.GRAY);
      HF.print('--%');
    }
    // ---- 衛星 ----
    {
      const ns = numsat();
      const sx = SCREEN_WIDTH - 101, sw = 44, sh = 15;
      const redBox = (ns === 0) ? (Math.floor(NOW / K.FOOTER_SATS0_BLINK_MS) % 2 === 0) : (ns < 5);
      let fg;
      if (ns === K.GNSS_NUMSAT_UNKNOWN) { fg = C.GRAY; HF.setTextColor(fg); }
      else if (redBox) { fg = C.WHITE; HF.fillRect(sx, 1, sw, sh, C.RED); HF.setTextColor(fg, C.RED); }
      else if (ns === 0) { fg = C.RED; HF.setTextColor(fg, C.WHITE); }
      else if (ns < 15) { fg = C.DARKORANGE; HF.setTextColor(fg); }
      else { fg = C.GREEN; HF.setTextColor(fg); }
      const satbuf = (ns === K.GNSS_NUMSAT_UNKNOWN) ? '--' : String(ns);
      const tw = HF.textWidth(satbuf);
      const ix = sx + (((sw - (13 + 2 + tw)) / 2) | 0);
      HF.drawMask(ix, 2, 13, 13, global.LIVE_ASSETS.ICON_SAT, fg);
      HF.setCursor(ix + 13 + 2, 1);
      HF.print(satbuf);
    }
    // ---- SD（送信機の status の LINK_ST_SD_ERROR）----
    if (!(S.lt.status & LINK_ST_SD_ERROR)) { HF.fillRect(SCREEN_WIDTH - 22, 15, 22, 15, C.GREEN); HF.setTextColor(C.WHITE, C.GREEN); }
    else { HF.fillRect(SCREEN_WIDTH - 22, 15, 22, 15, C.RED); HF.setTextColor(C.WHITE, C.RED); }
    HF.setCursor(SCREEN_WIDTH - 20, 16);
    HF.print('SD');
    HF.pushTo(SCR, 0, 290);
  }

  // =====================================================================
  //  1 コマ（.ino の draw_current_screen の MODE_MAP の描き直し）
  // =====================================================================
  function render(state, nowMs) {
    setup();
    S = state;
    NOW = nowMs;
    const ln = S.ln;
    drawHeader();
    M.drawVectorMap(BS, S.lat, S.lon, S.scale, S.up, selfCy());
    if (S.scale > K.SCALE_SMALL_GMAP) { drawPolyMaps('F'); drawPolyMaps('X'); }
    drawTrack();
    const P = S.pilon;
    const drawPilon = !!P && S.scale > K.SCALE_SMALL_GMAP && checkWithin(0.6, 0.6, S.lat, P.plaLat, S.lon, P.plaLon);
    if (drawPilon) drawPilonLine();
    const dest = (ln.dest >= 0) ? S.dests[ln.dest] : null;
    if (lnf(LS.NAV_VALID) && dest) {
      if (ln.dmode === DMODE_FLYINTO) drawFlyinto2(dest.lat, dest.lon, 5);
      else if (ln.dmode === DMODE_FLYAWAY) drawFlyawayfrom(dest.lat, dest.lon);
      else if (ln.dmode === DMODE_AUTO10K) {
        if (ln.a10k === AUTO10K_AWAY) drawFlyawayfrom(dest.lat, dest.lon);
        else if (ln.a10k === AUTO10K_INTO) drawFlyinto2(dest.lat, dest.lon, 5);
      }
    }
    if (drawPilon) drawPilonMarks();
    drawDegpersec(ln.dps);
    drawKmDistances(S.scale);
    if (ln.marker === 0) drawNofixCross();
    else if (ln.marker === 1) drawHaccCircle(S.scale, S.lt.hacc_mm);
    else drawTriangle(S.trackDraw, f32(ln.steer));
    drawCompass(S.up, C.BLACK);
    drawEskfAttitude();
    drawEskfYaw();
    if (lnf(LS.CWARN)) drawCourseWarning(ln.steer);
    drawGsTrack();
    drawMapFooter();
    pushBackscreen();
    drawFooter();
    return SCR;
  }

  // 航跡（src/navdata.cpp の LatLonManager と gnss.cpp の add_track_point の写し）
  class Track {
    constructor() { this.lat = new Float32Array(K.MAX_TRACK_CORDS); this.lon = new Float32Array(K.MAX_TRACK_CORDS); this.n = 0; this.head = 0; this.lastMs = 0; }
    reset() { this.n = 0; this.head = 0; this.lastMs = 0; }
    add(lat, lon) {
      this.lat[this.head] = lat; this.lon[this.head] = lon;
      this.head = (this.head + 1) % K.MAX_TRACK_CORDS;
      if (this.n < K.MAX_TRACK_CORDS) this.n++;
    }
    count() { return this.n; }
    get(i) { const k = (this.head - 1 - i + K.MAX_TRACK_CORDS * 2) % K.MAX_TRACK_CORDS; return [this.lat[k], this.lon[k]]; }
    // 受信機の時計 ms で、速さに合わせて約 50m おき（1〜15 秒）に足す
    feed(ms, lat, lon, mps) {
      const iv = Math.min(15000, Math.max(1000, Math.trunc(50000 / (1.0 + mps / 2.0))));
      if (((ms - this.lastMs) >>> 0) > iv) { this.add(f32(lat), f32(lon)); this.lastMs = ms; }
    }
  }

  global.LiveScreen = { render, Track, SCALELIST, SCALE_INDEX_DEFAULT, LS, K, HDR_TT, HDR_PAVG, isTasHdr };
})(typeof window !== 'undefined' ? window : globalThis);
