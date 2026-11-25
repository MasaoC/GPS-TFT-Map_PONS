// ============================================================
// File    : tools/repons/live_tft.js
// Project : PONS v7.1 / v7.2 Re:PONS
// Role    : ライブ配信の画面のための TFT_eSPI（2.5.43）の写し。本体と同じ RGB565 の画素を作る
//           （スプライト・線・円・三角・弧・アンチエイリアスの太線・内蔵 6x8 とスムーズフォントの文字）。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026-11-23
// ============================================================
//
// ★ 本体の display_tft.cpp の描き方を JS に写した live_screen.js が、これを TFT_eSprite と同じ形で呼ぶ。
//   **関数の中身は TFT_eSPI のソース（TFT_eSPI.cpp / Extensions/Sprite.cpp / Extensions/Smooth_font.cpp）の写し**で、
//   丸め（float の切り捨て・整数の割り算）と描く順番まで合わせてある。TFT_eSPI を上げたら見比べること。
// ★ C の float は Math.fround で、int への変換（切り捨て）は |0 でまねる。C の printf は sprintf()（丸めは偶数へ）。
(function (global) {
  'use strict';
  const f32 = Math.fround;

  // ---- 色（display_tft.h の COLOR_* と TFT_eSPI.h の TFT_*）----
  const C = {
    BLACK: 0x0000, WHITE: 0xFFFF, BLUE: 0x001F, RED: 0xF800, MAGENTA: 0xF81F, CYAN: 0x07FF,
    YELLOW: 0xFFE0, ORANGE: 0xFDA0, PINK: 0xFE19,
    GREEN: 0x03E0,          // COLOR_GREEN = TFT_DARKGREEN
    DARKORANGE: 0x7BE0,     // COLOR_DARKORANGE = TFT_OLIVE
    GRAY: 0x7bcf, BRIGHTGRAY: 0xc618, LIGHT_BLUE: 0x3dbf,
    WIND_PURPLE: 0xA81F,
    TFT_GREEN: 0x07E0, TFT_CYAN: 0x07FF, TFT_DARKGREY: 0x7BEF,
  };

  // ---- 色の混ぜ方（TFT_eSPI::alphaBlend と fastBlend は同じ式。uint32 のまま折り返す）----
  function alphaBlend(alpha, fgc, bgc) {
    alpha &= 0xFF;
    let rxb = bgc & 0xF81F;
    rxb = (rxb + (Math.imul((((fgc & 0xF81F) - rxb) >>> 0), alpha >> 2) >>> 6)) >>> 0;
    let xgx = bgc & 0x07E0;
    xgx = (xgx + (Math.imul((((fgc & 0x07E0) - xgx) >>> 0), alpha) >>> 8)) >>> 0;
    return ((rxb & 0xF81F) | (xgx & 0x07E0)) & 0xFFFF;
  }

  // ---- スムーズフォント（VLW。TFT_eSPI::loadFont / loadMetrics の写し）----
  function parseVlw(bytes) {
    let p = 0;
    const rd = () => { const v = ((bytes[p] << 24) | (bytes[p + 1] << 16) | (bytes[p + 2] << 8) | bytes[p + 3]) >>> 0; p += 4; return v; };
    const f = { data: bytes };
    f.gCount = rd() & 0xFFFF; rd();
    rd(); rd();
    f.ascent = rd() & 0xFFFF;
    f.descent = rd() & 0xFFFF;
    f.maxAscent = f.ascent;
    f.maxDescent = f.descent;
    f.yAdvance = (f.ascent + f.descent) & 0xFFFF;
    const n = f.gCount;
    f.unicode = new Uint16Array(n); f.height = new Uint8Array(n); f.width = new Uint8Array(n);
    f.xAdvance = new Uint8Array(n); f.dY = new Int16Array(n); f.dX = new Int8Array(n); f.bitmap = new Uint32Array(n);
    let bitmapPtr = 24 + n * 28;
    p = 24;
    for (let g = 0; g < n; g++) {
      f.unicode[g] = rd() & 0xFFFF;
      f.height[g] = rd() & 0xFF;
      f.width[g] = rd() & 0xFF;
      f.xAdvance[g] = rd() & 0xFF;
      f.dY[g] = (rd() << 16) >> 16;
      f.dX[g] = (rd() << 24) >> 24;
      rd();
      const u = f.unicode[g];
      if ((f.height[g] - f.dY[g]) > f.maxDescent) {
        if ((u > 0x20 && u < 0xA0 && u !== 0x7F) || u > 0xFF) f.maxDescent = f.height[g] - f.dY[g];
      }
      f.bitmap[g] = bitmapPtr;
      bitmapPtr += f.width[g] * f.height[g];
    }
    f.yAdvance = (f.maxAscent + f.maxDescent) & 0xFFFF;
    f.spaceWidth = (((f.ascent + f.descent) * 2) / 7) | 0;
    f.index = new Map();
    for (let g = 0; g < n; g++) if (!f.index.has(f.unicode[g])) f.index.set(f.unicode[g], g);
    return f;
  }

  let FONTS = null, GLCD = null;
  function fonts() {
    if (!FONTS) {
      const A = global.LIVE_ASSETS;
      if (!A) throw new Error('live_assets.js が読まれていません');
      FONTS = {
        AA_FONT_SMALL: parseVlw(A.NotoSansBold15),
        NM_FONT_MEDIUM: parseVlw(A.Arial_Black22),
        NM_FONT_LARGE: parseVlw(A.Arial_Black56),
      };
      GLCD = A.GLCD;
    }
    return FONTS;
  }

  // ---- C の printf（newlib と同じく、ちょうど半分は偶数へ丸める）----
  function incDecimal(s) {
    const a = s.split('');
    let i = a.length - 1;
    while (i >= 0) {
      if (a[i] === '9') { a[i] = '0'; i--; continue; }
      a[i] = String.fromCharCode(a[i].charCodeAt(0) + 1);
      return a.join('');
    }
    return '1' + a.join('');
  }
  function fmtFixed(x, prec) {
    if (Number.isNaN(x)) return 'nan';
    if (!Number.isFinite(x)) return x < 0 ? '-inf' : 'inf';
    const neg = x < 0 || (x === 0 && 1 / x < 0);
    const ax = Math.abs(x);
    const ext = ax.toFixed(Math.min(100, prec + 60));
    const dot = ext.indexOf('.');
    const ip = dot < 0 ? ext : ext.slice(0, dot);
    const fr = dot < 0 ? '' : ext.slice(dot + 1);
    let digits = ip + fr.slice(0, prec);
    const rest = fr.slice(prec);
    let up = false;
    if (rest.length) {
      if (rest[0] > '5') up = true;
      else if (rest[0] === '5') up = /[1-9]/.test(rest.slice(1)) || ((digits.charCodeAt(digits.length - 1) - 48) & 1) === 1;
    }
    if (up) digits = incDecimal(digits);
    const il = digits.length - prec;
    let out = prec > 0 ? digits.slice(0, il) + '.' + digits.slice(il) : digits;
    if (out.startsWith('.')) out = '0' + out;
    return (neg ? '-' : '') + out;
  }
  function sprintf(fmt, ...args) {
    let ai = 0, out = '';
    for (let i = 0; i < fmt.length; i++) {
      const ch = fmt[i];
      if (ch !== '%') { out += ch; continue; }
      i++;
      if (fmt[i] === '%') { out += '%'; continue; }
      let flags = '';
      while ('-+ 0#'.includes(fmt[i])) flags += fmt[i++];
      let width = '';
      if (fmt[i] === '*') { width = String(args[ai++]); i++; } else while (fmt[i] >= '0' && fmt[i] <= '9') width += fmt[i++];
      let prec = null;
      if (fmt[i] === '.') {
        i++; prec = '';
        if (fmt[i] === '*') { prec = String(args[ai++]); i++; } else while (fmt[i] >= '0' && fmt[i] <= '9') prec += fmt[i++];
        prec = prec === '' ? 0 : parseInt(prec, 10);
      }
      while ('hlLqjzt'.includes(fmt[i])) i++;
      const t = fmt[i];
      let s, numeric = true, negv = false;
      const v = args[ai++];
      if (t === 'd' || t === 'i') {
        let n = Math.trunc(Number(v)); negv = n < 0; s = String(Math.abs(n));
        if (prec !== null) s = s.padStart(prec, '0');
        s = (negv ? '-' : flags.includes('+') ? '+' : flags.includes(' ') ? ' ' : '') + s;
      } else if (t === 'u') {
        s = String(Math.trunc(Number(v)) >>> 0);
      } else if (t === 'x' || t === 'X') {
        s = (Math.trunc(Number(v)) >>> 0).toString(16); if (t === 'X') s = s.toUpperCase();
        if (prec !== null) s = s.padStart(prec, '0');
      } else if (t === 'f' || t === 'F') {
        s = fmtFixed(Number(v), prec === null ? 6 : prec);
        negv = s.startsWith('-');
        if (!negv) s = (flags.includes('+') ? '+' : flags.includes(' ') ? ' ' : '') + s;
      } else if (t === 'c') {
        s = typeof v === 'number' ? String.fromCharCode(v) : String(v)[0] || ''; numeric = false;
      } else if (t === 's') {
        s = String(v); if (prec !== null) s = s.slice(0, prec); numeric = false;
      } else { s = ''; numeric = false; }
      const w = width === '' ? 0 : parseInt(width, 10);
      if (s.length < w) {
        if (flags.includes('-')) s = s.padEnd(w, ' ');
        else if (flags.includes('0') && numeric && (prec === null || t === 'f')) {
          const sign = (s[0] === '-' || s[0] === '+' || s[0] === ' ') ? s[0] : '';
          s = sign + s.slice(sign.length).padStart(w - sign.length, '0');
        } else s = s.padStart(w, ' ');
      }
      out += s;
    }
    return out;
  }

  // C の lroundf（0.5 は 0 から遠い方へ）
  function lround(x) { return x < 0 ? -Math.round(-x) : Math.round(x); }

  // ---- スプライト（TFT_eSprite。16bit、ビューポートは全体）----
  class Sprite {
    constructor(w, h) {
      this.w = w; this.h = h;
      this.buf = new Uint16Array(w * h + 1);   // 最後の 1 画素は setWindow が範囲の外を指すとき用（callocSprite と同じ）
      this.textcolor = 0xFFFF; this.textbgcolor = 0x0000; this.fillbg = false;
      this.textsize = 1; this.wrapX = true; this.wrapY = false;
      this.cursorX = 0; this.cursorY = 0; this.lastCursorX = 0; this.bgCursorX = 0;
      this.font = null;
      this.decoderState = 0; this.decoderBuffer = 0;
      this._xs = 0; this._ys = 0; this._xe = 0; this._ye = 0; this._xptr = 0; this._yptr = 0;
    }
    width() { return this.w; }
    height() { return this.h; }

    // ---- 点・線・矩形（Sprite.cpp）----
    drawPixel(x, y, color) {
      x |= 0; y |= 0;
      if (x < 0 || y < 0 || x >= this.w || y >= this.h) return;
      this.buf[x + y * this.w] = color & 0xFFFF;
    }
    readPixel(x, y) {
      x |= 0; y |= 0;
      if (x < 0 || y < 0 || x >= this.w || y >= this.h) return 0xFFFF;
      return this.buf[x + y * this.w];
    }
    drawFastHLine(x, y, w, color) {
      x |= 0; y |= 0; w |= 0;
      if (y < 0 || x >= this.w || y >= this.h) return;
      if (x < 0) { w += x; x = 0; }
      if (x + w > this.w) w = this.w - x;
      if (w < 1) return;
      this.buf.fill(color & 0xFFFF, y * this.w + x, y * this.w + x + w);
    }
    drawFastVLine(x, y, h, color) {
      x |= 0; y |= 0; h |= 0;
      if (x < 0 || x >= this.w || y >= this.h) return;
      if (y < 0) { h += y; y = 0; }
      if (y + h > this.h) h = this.h - y;
      if (h < 1) return;
      let p = x + this.w * y;
      const c = color & 0xFFFF;
      while (h--) { this.buf[p] = c; p += this.w; }
    }
    fillRect(x, y, w, h, color) {
      x |= 0; y |= 0; w |= 0; h |= 0;
      if (x >= this.w || y >= this.h) return;
      if (x < 0) { w += x; x = 0; }
      if (y < 0) { h += y; y = 0; }
      if (x + w > this.w) w = this.w - x;
      if (y + h > this.h) h = this.h - y;
      if (w < 1 || h < 1) return;
      const c = color & 0xFFFF;
      for (let yy = y; yy < y + h; yy++) this.buf.fill(c, yy * this.w + x, yy * this.w + x + w);
    }
    fillSprite(color) { this.fillRect(0, 0, this.w, this.h, color); }
    drawRect(x, y, w, h, color) {
      x |= 0; y |= 0; w |= 0; h |= 0;
      this.drawFastHLine(x, y, w, color);
      this.drawFastHLine(x, y + h - 1, w, color);
      this.drawFastVLine(x, y + 1, h - 2, color);
      this.drawFastVLine(x + w - 1, y + 1, h - 2, color);
    }
    drawLine(x0, y0, x1, y1, color) {
      x0 |= 0; y0 |= 0; x1 |= 0; y1 |= 0;
      const steep = Math.abs(y1 - y0) > Math.abs(x1 - x0);
      let t;
      if (steep) { t = x0; x0 = y0; y0 = t; t = x1; x1 = y1; y1 = t; }
      if (x0 > x1) { t = x0; x0 = x1; x1 = t; t = y0; y0 = y1; y1 = t; }
      const dx = x1 - x0, dy = Math.abs(y1 - y0);
      let err = dx >> 1, ystep = -1, xs = x0, dlen = 0;
      if (y0 < y1) ystep = 1;
      if (steep) {
        for (; x0 <= x1; x0++) {
          dlen++;
          err -= dy;
          if (err < 0) {
            err += dx;
            if (dlen === 1) this.drawPixel(y0, xs, color);
            else this.drawFastVLine(y0, xs, dlen, color);
            dlen = 0; y0 += ystep; xs = x0 + 1;
          }
        }
        if (dlen) this.drawFastVLine(y0, xs, dlen, color);
      } else {
        for (; x0 <= x1; x0++) {
          dlen++;
          err -= dy;
          if (err < 0) {
            err += dx;
            if (dlen === 1) this.drawPixel(xs, y0, color);
            else this.drawFastHLine(xs, y0, dlen, color);
            dlen = 0; y0 += ystep; xs = x0 + 1;
          }
        }
        if (dlen) this.drawFastHLine(xs, y0, dlen, color);
      }
    }

    // ---- 円・三角（TFT_eSPI.cpp）----
    drawCircle(x0, y0, r, color) {
      x0 |= 0; y0 |= 0; r |= 0;
      if (r <= 0) return;
      let f = 1 - r, ddF_y = -2 * r, ddF_x = 1, xs = -1, xe = 0, len = 0, first = true;
      do {
        while (f < 0) { ++xe; f += (ddF_x += 2); }
        f += (ddF_y += 2);
        if (xe - xs > 1) {
          if (first) {
            len = 2 * (xe - xs) - 1;
            this.drawFastHLine(x0 - xe, y0 + r, len, color);
            this.drawFastHLine(x0 - xe, y0 - r, len, color);
            this.drawFastVLine(x0 + r, y0 - xe, len, color);
            this.drawFastVLine(x0 - r, y0 - xe, len, color);
            first = false;
          } else {
            len = xe - xs++;
            this.drawFastHLine(x0 - xe, y0 + r, len, color);
            this.drawFastHLine(x0 - xe, y0 - r, len, color);
            this.drawFastHLine(x0 + xs, y0 - r, len, color);
            this.drawFastHLine(x0 + xs, y0 + r, len, color);
            this.drawFastVLine(x0 + r, y0 + xs, len, color);
            this.drawFastVLine(x0 + r, y0 - xe, len, color);
            this.drawFastVLine(x0 - r, y0 - xe, len, color);
            this.drawFastVLine(x0 - r, y0 + xs, len, color);
          }
        } else {
          ++xs;
          this.drawPixel(x0 - xe, y0 + r, color);
          this.drawPixel(x0 - xe, y0 - r, color);
          this.drawPixel(x0 + xs, y0 - r, color);
          this.drawPixel(x0 + xs, y0 + r, color);
          this.drawPixel(x0 + r, y0 + xs, color);
          this.drawPixel(x0 + r, y0 - xe, color);
          this.drawPixel(x0 - r, y0 - xe, color);
          this.drawPixel(x0 - r, y0 + xs, color);
        }
        xs = xe;
      } while (xe < --r);
    }
    fillCircle(x0, y0, r, color) {
      x0 |= 0; y0 |= 0; r |= 0;
      let x = 0, dx = 1, dy = r + r, p = -(r >> 1);
      this.drawFastHLine(x0 - r, y0, dy + 1, color);
      while (x < r) {
        if (p >= 0) {
          this.drawFastHLine(x0 - x, y0 + r, dx, color);
          this.drawFastHLine(x0 - x, y0 - r, dx, color);
          dy -= 2; p -= dy; r--;
        }
        dx += 2; p += dx; x++;
        this.drawFastHLine(x0 - r, y0 + x, dy + 1, color);
        this.drawFastHLine(x0 - r, y0 - x, dy + 1, color);
      }
    }
    drawTriangle(x0, y0, x1, y1, x2, y2, color) {
      this.drawLine(x0, y0, x1, y1, color);
      this.drawLine(x1, y1, x2, y2, color);
      this.drawLine(x2, y2, x0, y0, color);
    }
    fillTriangle(x0, y0, x1, y1, x2, y2, color) {
      x0 |= 0; y0 |= 0; x1 |= 0; y1 |= 0; x2 |= 0; y2 |= 0;
      let a, b, y, last, t;
      if (y0 > y1) { t = y0; y0 = y1; y1 = t; t = x0; x0 = x1; x1 = t; }
      if (y1 > y2) { t = y2; y2 = y1; y1 = t; t = x2; x2 = x1; x1 = t; }
      if (y0 > y1) { t = y0; y0 = y1; y1 = t; t = x0; x0 = x1; x1 = t; }
      if (y0 === y2) {
        a = b = x0;
        if (x1 < a) a = x1; else if (x1 > b) b = x1;
        if (x2 < a) a = x2; else if (x2 > b) b = x2;
        this.drawFastHLine(a, y0, b - a + 1, color);
        return;
      }
      const dx01 = x1 - x0, dy01 = y1 - y0, dx02 = x2 - x0, dy02 = y2 - y0, dx12 = x2 - x1, dy12 = y2 - y1;
      let sa = 0, sb = 0;
      if (y1 === y2) last = y1; else last = y1 - 1;
      for (y = y0; y <= last; y++) {
        a = x0 + ((sa / dy01) | 0);
        b = x0 + ((sb / dy02) | 0);
        sa += dx01; sb += dx02;
        if (a > b) { t = a; a = b; b = t; }
        this.drawFastHLine(a, y, b - a + 1, color);
      }
      sa = dx12 * (y - y1);
      sb = dx02 * (y - y0);
      for (; y <= y2; y++) {
        a = x1 + ((sa / dy12) | 0);
        b = x0 + ((sb / dy02) | 0);
        sa += dx12; sb += dx02;
        if (a > b) { t = a; a = b; b = t; }
        this.drawFastHLine(a, y, b - a + 1, color);
      }
    }

    // ---- 窓と 1 画素ずつ（TFT_eSprite::setWindow / pushColor。drawWedgeLine が使う）----
    setWindow(x0, y0, x1, y1) {
      let t;
      if (x0 > x1) { t = x0; x0 = x1; x1 = t; }
      if (y0 > y1) { t = y0; y0 = y1; y1 = t; }
      const w = this.w, h = this.h;
      if (x0 >= w || x1 < 0 || y0 >= h || y1 < 0) {
        this._xs = 0; this._ys = h; this._xe = 0; this._ye = h;
      } else {
        if (x0 < 0) x0 = 0;
        if (x1 >= w) x1 = w - 1;
        if (y0 < 0) y0 = 0;
        if (y1 >= h) y1 = h - 1;
        this._xs = x0; this._ys = y0; this._xe = x1; this._ye = y1;
      }
      this._xptr = this._xs; this._yptr = this._ys;
    }
    pushColor(color) {
      this.buf[this._xptr + this._yptr * this.w] = color & 0xFFFF;
      this._xptr++;
      if (this._xptr > this._xe) {
        this._xptr = this._xs;
        this._yptr++;
        if (this._yptr > this._ye) this._yptr = this._ys;
      }
    }

    // ---- アンチエイリアスの太線（TFT_eSPI::drawWideLine / drawWedgeLine。float で計算する）----
    drawWideLine(ax, ay, bx, by, wd, fg, bg = 0x00FFFFFF) {
      const r = f32(f32(wd) / 2.0);
      this.drawWedgeLine(ax, ay, bx, by, r, r, fg, bg);
    }
    drawWedgeLine(ax, ay, bx, by, ar, br, fg, bg = 0x00FFFFFF) {
      ax = f32(ax); ay = f32(ay); bx = f32(bx); by = f32(by); ar = f32(ar); br = f32(br);
      if (ar < 0 || br < 0) return;
      if (Math.abs(f32(ax - bx)) < f32(0.01) && Math.abs(f32(ay - by)) < f32(0.01)) bx = f32(bx + f32(0.01));
      let x0 = Math.floor(Math.min(f32(ax - ar), f32(bx - br)));
      let x1 = Math.ceil(Math.max(f32(ax + ar), f32(bx + br)));
      let y0 = Math.floor(Math.min(f32(ay - ar), f32(by - br)));
      let y1 = Math.ceil(Math.max(f32(ay + ar), f32(by + br)));
      // clipWindow（xe == 幅ちょうどのときは切らない。TFT_eSPI のまま）
      if (x0 >= this.w || y0 >= this.h) return;
      if (x1 < 0 || y1 < 0) return;
      if (x0 < 0) x0 = 0;
      if (y0 < 0) y0 = 0;
      if (x1 > this.w) x1 = this.w - 1;
      if (y1 > this.h) y1 = this.h - 1;
      let ys = Math.trunc(ay);
      if (f32(ax - ar) > f32(bx - br)) ys = Math.trunc(by);
      const rdt = f32(ar - br);
      let alpha = f32(1.0);
      ar = f32(ar + 0.5);
      let bgc = bg;
      const bax = f32(bx - ax), bay = f32(by - ay);
      const LO = f32(1.0 / 32.0), HI = f32(1.0 - LO);
      const dist = (xpax, ypay) => {
        const h = Math.max(Math.min(f32(f32(f32(xpax * bax) + f32(ypay * bay)) / f32(f32(bax * bax) + f32(bay * bay))), 1.0), 0.0);
        const dx = f32(xpax - f32(bax * h)), dy = f32(ypay - f32(bay * h));
        return f32(f32(Math.sqrt(f32(f32(dx * dx) + f32(dy * dy)))) + f32(h * rdt));
      };
      const scan = (yp) => {
        let swin = true, endX = false;
        const ypay = f32(yp - ay);
        for (let xp = xs; xp <= x1; xp++) {
          if (endX) if (alpha <= LO) break;
          const xpax = f32(xp - ax);
          alpha = f32(ar - dist(xpax, ypay));
          if (alpha <= LO) continue;
          if (!endX) { endX = true; xs = xp; }
          if (alpha > HI) {
            if (swin) { this.setWindow(xp, yp, x1, yp); swin = false; }
            this.pushColor(fg);
            continue;
          }
          if (bg === 0x00FFFFFF) { bgc = this.readPixel(xp, yp); swin = true; }
          if (swin) { this.setWindow(xp, yp, x1, yp); swin = false; }
          this.pushColor(alphaBlend((f32(alpha * 255.0)) & 0xFF, fg, bgc));
        }
      };
      let xs = x0;
      for (let yp = ys; yp <= y1; yp++) scan(yp);
      xs = x0;
      for (let yp = ys - 1; yp >= y0; yp--) scan(yp);
    }

    // ---- 弧（TFT_eSPI::drawArc。6 時から時計回り）----
    drawArc(x, y, r, ir, startAngle, endAngle, fg, bg, smooth = true) {
      x |= 0; y |= 0; r |= 0; ir |= 0;
      startAngle = (startAngle | 0) >>> 0; endAngle = (endAngle | 0) >>> 0;   // uint32_t の引数
      if (endAngle > 360) endAngle = 360;
      if (startAngle > 360) startAngle = 360;
      if (startAngle === endAngle) return;
      if (r < ir) { const t = r; r = ir; ir = t; }
      if (r <= 0 || ir < 0) return;
      if (endAngle < startAngle) {
        if (startAngle < 360) this.drawArc(x, y, r, ir, startAngle, 360, fg, bg, smooth);
        if (endAngle === 0) return;
        startAngle = 0;
      }
      let xs = 0, alpha = 0;
      const r2 = (r * r) >>> 0;
      if (smooth) r++;
      const r1 = (r * r) >>> 0;
      const w = r - ir;
      const r3 = (ir * ir) >>> 0;
      if (smooth) ir--;
      const r4 = (ir * ir) >>> 0;
      const startSlope = [0, 0, 0xFFFFFFFF, 0];
      const endSlope = [0, 0xFFFFFFFF, 0, 0];
      const minDivisor = f32(1.0 / 0x8000);
      const D2R = f32(3.14159265359 / 180.0);
      let fabscos = Math.abs(f32(Math.cos(f32(startAngle * D2R))));
      let fabssin = Math.abs(f32(Math.sin(f32(startAngle * D2R))));
      let slope = Math.trunc(f32(f32(fabscos / f32(fabssin + minDivisor)) * 65536)) >>> 0;
      if (startAngle <= 90) startSlope[0] = slope;
      else if (startAngle <= 180) startSlope[1] = slope;
      else if (startAngle <= 270) { startSlope[1] = 0xFFFFFFFF; startSlope[2] = slope; }
      else { startSlope[1] = 0xFFFFFFFF; startSlope[2] = 0; startSlope[3] = slope; }
      fabscos = Math.abs(f32(Math.cos(f32(endAngle * D2R))));
      fabssin = Math.abs(f32(Math.sin(f32(endAngle * D2R))));
      slope = Math.trunc(f32(f32(fabscos / f32(fabssin + minDivisor)) * 65536)) >>> 0;
      if (endAngle <= 90) { endSlope[0] = slope; endSlope[1] = 0; startSlope[2] = 0; }
      else if (endAngle <= 180) { endSlope[1] = slope; startSlope[2] = 0; }
      else if (endAngle <= 270) { endSlope[2] = slope; }
      else { endSlope[3] = slope; }
      for (let cy = r - 1; cy > 0; cy--) {
        const len = [0, 0, 0, 0];
        const xst = [-1, -1, -1, -1];
        const dy2 = ((r - cy) * (r - cy)) >>> 0;
        while ((((r - xs) * (r - xs) + dy2) >>> 0) >= r1) xs++;
        for (let cx = xs; cx < r; cx++) {
          const hyp = (((r - cx) * (r - cx)) + dy2) >>> 0;
          if (hyp > r2) {
            alpha = (~sqrtFraction(hyp)) & 0xFF;
          } else if (hyp >= r3) {
            slope = Math.floor(((r - cy) * 65536) / (r - cx)) >>> 0;
            if (slope <= startSlope[0] && slope >= endSlope[0]) { xst[0] = cx; len[0]++; }
            if (slope >= startSlope[1] && slope <= endSlope[1]) { xst[1] = cx; len[1]++; }
            if (slope <= startSlope[2] && slope >= endSlope[2]) { xst[2] = cx; len[2]++; }
            if (slope <= endSlope[3] && slope >= startSlope[3]) { xst[3] = cx; len[3]++; }
            continue;
          } else {
            if (hyp <= r4) break;
            alpha = sqrtFraction(hyp);
          }
          if (alpha < 16) continue;
          const pcol = alphaBlend(alpha, fg, bg);
          slope = Math.floor(((r - cy) * 65536) / (r - cx)) >>> 0;
          if (slope <= startSlope[0] && slope >= endSlope[0]) this.drawPixel(x + cx - r, y - cy + r, pcol);
          if (slope >= startSlope[1] && slope <= endSlope[1]) this.drawPixel(x + cx - r, y + cy - r, pcol);
          if (slope <= startSlope[2] && slope >= endSlope[2]) this.drawPixel(x - cx + r, y + cy - r, pcol);
          if (slope <= endSlope[3] && slope >= startSlope[3]) this.drawPixel(x - cx + r, y - cy + r, pcol);
        }
        if (len[0]) this.drawFastHLine(x + xst[0] - len[0] + 1 - r, y - cy + r, len[0], fg);
        if (len[1]) this.drawFastHLine(x + xst[1] - len[1] + 1 - r, y + cy - r, len[1], fg);
        if (len[2]) this.drawFastHLine(x - xst[2] + r, y + cy - r, len[2], fg);
        if (len[3]) this.drawFastHLine(x - xst[3] + r, y - cy + r, len[3], fg);
      }
      if (startAngle === 0 || endAngle === 360) this.drawFastVLine(x, y + r - w, w, fg);
      if (startAngle <= 90 && endAngle >= 90) this.drawFastHLine(x - r + 1, y, w, fg);
      if (startAngle <= 180 && endAngle >= 180) this.drawFastVLine(x, y - r + 1, w, fg);
      if (startAngle <= 270 && endAngle >= 270) this.drawFastHLine(x + r - w, y, w, fg);
    }

    // ---- 文字の状態 ----
    setTextSize(s) { s |= 0; if (s > 7) s = 7; this.textsize = s > 0 ? s : 1; }
    setTextColor(c, b, bgfill) {
      if (b === undefined) { this.textcolor = this.textbgcolor = c & 0xFFFF; return; }
      this.textcolor = c & 0xFFFF; this.textbgcolor = b & 0xFFFF; this.fillbg = !!bgfill;
    }
    setTextWrap(x, y = false) { this.wrapX = !!x; this.wrapY = !!y; }
    setCursor(x, y) { this.cursorX = ((x | 0) << 16) >> 16; this.cursorY = ((y | 0) << 16) >> 16; }
    loadFont(f) { this.font = f; }
    unloadFont() { this.font = null; }
    fontHeight() { return this.font ? this.font.yAdvance : 8 * this.textsize; }

    decodeUTF8(c) {
      if ((c & 0x80) === 0) { this.decoderState = 0; return c; }
      if (this.decoderState === 0) {
        if ((c & 0xE0) === 0xC0) { this.decoderBuffer = (c & 0x1F) << 6; this.decoderState = 1; return 0; }
        if ((c & 0xF0) === 0xE0) { this.decoderBuffer = (c & 0x0F) << 12; this.decoderState = 2; return 0; }
      } else {
        if (this.decoderState === 2) { this.decoderBuffer |= (c & 0x3F) << 6; this.decoderState--; return 0; }
        this.decoderBuffer |= (c & 0x3F); this.decoderState = 0; return this.decoderBuffer;
      }
      this.decoderState = 0;
      return c;
    }
    textWidth(str) {
      const bytes = utf8(str);
      if (this.font) {
        const f = this.font;
        let wsum = 0;
        for (let i = 0; i < bytes.length; i++) {
          const u = this.decodeUTF8(bytes[i]);
          if (!u) continue;
          if (u === 0x20) { wsum += f.spaceWidth; continue; }
          const g = f.index.get(u);
          if (g !== undefined) {
            if (wsum === 0 && f.dX[g] < 0) wsum -= f.dX[g];
            if (i + 1 < bytes.length) wsum += f.xAdvance[g];
            else wsum += (f.dX[g] + f.width[g]);
          } else wsum += f.spaceWidth + 1;
        }
        return (wsum << 16) >> 16;
      }
      return ((6 * bytes.length * this.textsize) << 16) >> 16;
    }

    // 内蔵 6x8（Sprite.cpp の drawChar）
    drawCharGlcd(x, y, c, color, bg, size) {
      fonts();
      if (x >= this.w || y >= this.h) return;
      if ((x + 6 * size - 1) < 0 || (y + 8 * size - 1) < 0) return;
      if (c > 255) return;
      if (c > 175) c++;
      const fillbg = (bg !== color);
      if (size === 1 && fillbg) {
        const column = [GLCD[c * 5], GLCD[c * 5 + 1], GLCD[c * 5 + 2], GLCD[c * 5 + 3], GLCD[c * 5 + 4], 0];
        let mask = 1;
        for (let j = 0; j < 8; j++) {
          let k;
          for (k = 0; k < 5; k++) this.drawPixel(x + k, y + j, (column[k] & mask) ? color : bg);
          mask <<= 1;
          this.drawPixel(x + k, y + j, bg);
        }
      } else {
        for (let i = 0; i < 6; i++) {
          let line = (i === 5) ? 0 : GLCD[c * 5 + i];
          if (size === 1) {
            for (let j = 0; j < 8; j++) { if (line & 1) this.drawPixel(x + i, y + j, color); line >>= 1; }
          } else {
            for (let j = 0; j < 8; j++) {
              if (line & 1) this.fillRect(x + i * size, y + j * size, size, size, color);
              else if (fillbg) this.fillRect(x + i * size, y + j * size, size, size, bg);
              line >>= 1;
            }
          }
        }
      }
    }

    // スムーズフォントの 1 文字（Sprite.cpp の drawGlyph）
    drawGlyph(code) {
      const f = this.font;
      const fg = this.textcolor;
      let bg = this.textbgcolor;
      const getBG = (fg === bg);
      if (this.lastCursorX !== this.cursorX) { this.bgCursorX = this.cursorX; this.lastCursorX = this.cursorX; }
      if (code < 0x21) {
        if (code === 0x20) {
          if (this.fillbg) this.fillRect(this.bgCursorX, this.cursorY, (this.cursorX + f.spaceWidth) - this.bgCursorX, f.yAdvance, bg);
          this.cursorX += f.spaceWidth;
          this.bgCursorX = this.cursorX; this.lastCursorX = this.cursorX;
          return;
        }
        if (code === 10) {
          this.cursorX = 0; this.bgCursorX = 0; this.lastCursorX = 0;
          this.cursorY += f.yAdvance;
          if (this.wrapY && this.cursorY >= this.h) this.cursorY = 0;
          return;
        }
      }
      const g = f.index.get(code);
      if (g !== undefined) {
        const gw = f.width[g], gh = f.height[g], gdX = f.dX[g], gdY = f.dY[g], gxa = f.xAdvance[g];
        if (this.wrapX && ((this.cursorX + gw + gdX) > this.w)) {
          this.cursorY += f.yAdvance; this.cursorX = 0; this.bgCursorX = 0; this.lastCursorX = 0;
        }
        if (this.wrapY && ((this.cursorY + f.yAdvance) > this.h)) this.cursorY = 0;
        if (this.cursorX === 0) this.cursorX -= gdX;
        const cy = ((this.cursorY + f.maxAscent - gdY) << 16) >> 16;
        const cx = ((this.cursorX + gdX) << 16) >> 16;
        let fxs = cx, fl = 0, bxs = cx, bl = 0, bx = 0;
        let fillwidth = 0, fillheight = 0;
        if (this.fillbg) {
          fillwidth = (this.cursorX + gxa) - this.bgCursorX;
          if (fillwidth > 0) {
            fillheight = f.maxAscent - gdY;
            if (fillheight > 0) this.fillRect(this.bgCursorX, this.cursorY, fillwidth, fillheight, this.textbgcolor);
          } else fillwidth = 0;
          if (this.bgCursorX < cx) this.fillRect(this.bgCursorX, cy, cx - this.bgCursorX, gh, this.textbgcolor);
          if (this.bgCursorX > cx) bx = this.bgCursorX - cx;
          if (cx + gw < this.cursorX + gxa) this.fillRect(cx + gw, cy, (this.cursorX + gxa) - (cx + gw), gh, this.textbgcolor);
        }
        const base = f.bitmap[g];
        for (let y = 0; y < gh; y++) {
          for (let x = 0; x < gw; x++) {
            const pixel = f.data[base + x + gw * y];
            if (pixel) {
              if (bl) { this.drawFastHLine(bxs, y + cy, bl, bg); bl = 0; }
              if (pixel !== 0xFF) {
                if (fl) {
                  if (fl === 1) this.drawPixel(fxs, y + cy, fg);
                  else this.drawFastHLine(fxs, y + cy, fl, fg);
                  fl = 0;
                }
                if (getBG) bg = this.readPixel(x + cx, y + cy);
                this.drawPixel(x + cx, y + cy, alphaBlend(pixel, fg, bg));
              } else {
                if (fl === 0) fxs = x + cx;
                fl++;
              }
            } else {
              if (fl) { this.drawFastHLine(fxs, y + cy, fl, fg); fl = 0; }
              if (this.fillbg) {
                if (x >= bx) { if (bl === 0) bxs = x + cx; bl++; }
              }
            }
          }
          if (fl) { this.drawFastHLine(fxs, y + cy, fl, fg); fl = 0; }
          if (bl) { this.drawFastHLine(bxs, y + cy, bl, bg); bl = 0; }
        }
        if (fillwidth > 0) {
          fillheight = (this.cursorY + f.yAdvance) - (cy + gh);
          if (fillheight > 0) this.fillRect(this.bgCursorX, cy + gh, fillwidth, fillheight, this.textbgcolor);
        }
        this.cursorX += gxa;
      } else {
        this.drawRect(this.cursorX, this.cursorY + f.maxAscent - f.ascent, f.spaceWidth, f.ascent, fg);
        this.cursorX += f.spaceWidth + 1;
      }
      this.bgCursorX = this.cursorX;
      this.lastCursorX = this.cursorX;
    }

    // print の 1 バイト（TFT_eSPI::write）
    write(b) {
      const u = this.decodeUTF8(b);
      if (!u) return;
      if (b === 13) return;
      if (this.font) {
        if (u < 32 && b !== 10) return;
        this.drawGlyph(u);
        return;
      }
      let uc = u;
      if (uc === 10) uc += 22;
      const cwidth = 6, cheight = 8 * this.textsize;
      if (b === 10) { this.cursorY += cheight; this.cursorX = 0; return; }
      if (this.wrapX && (this.cursorX + cwidth * this.textsize > this.w)) { this.cursorY += cheight; this.cursorX = 0; }
      if (this.wrapY && this.cursorY >= this.h) this.cursorY = 0;
      this.drawCharGlcd(this.cursorX, this.cursorY, uc, this.textcolor, this.textbgcolor, this.textsize);
      this.cursorX += 6 * this.textsize;
    }
    print(s) { const b = utf8(String(s)); for (let i = 0; i < b.length; i++) this.write(b[i]); }
    printf(fmt, ...a) { this.print(sprintf(fmt, ...a)); }
    // TL_DATUM・余白なしの drawString（TFT_eSPI::drawString）
    drawString(str, x, y) {
      const cwidth = this.textWidth(str);
      const b = utf8(String(str));
      if (this.font) {
        this.setCursor(x, y);
        let n = 0;
        while (n < b.length) {
          let c = b[n++];
          if ((c & 0x80) && (c & 0xE0) === 0xC0 && n < b.length) c = ((c & 0x1F) << 6) | (b[n++] & 0x3F);
          else if ((c & 0x80) && (c & 0xF0) === 0xE0 && n + 1 < b.length) { c = ((c & 0x0F) << 12) | ((b[n++] & 0x3F) << 6); c |= (b[n++] & 0x3F); }
          this.drawGlyph(c);
        }
        return cwidth;
      }
      let sumX = 0;
      for (let i = 0; i < b.length; i++) {
        if (!b[i]) continue;
        this.drawCharGlcd(x + sumX, y, b[i], this.textcolor, this.textbgcolor, this.textsize);
        sumX += 6 * this.textsize;
      }
      return sumX;
    }

    // 1bit のマスク（display_tft.cpp の draw_mask。0 は透明）
    drawMask(x, y, w, h, data, color) {
      const rowbytes = (w + 7) >> 3;
      for (let j = 0; j < h; j++)
        for (let i = 0; i < w; i++)
          if (data[j * rowbytes + (i >> 3)] & (0x80 >> (i & 7))) this.drawPixel(x + i, y + j, color);
    }

    // 別のスプライト・画面へ写す（pushToSprite / pushSprite。透明色なし）
    pushTo(dst, x, y) {
      for (let j = 0; j < this.h; j++) {
        const ty = y + j;
        if (ty < 0 || ty >= dst.h) continue;
        for (let i = 0; i < this.w; i++) {
          const tx = x + i;
          if (tx < 0 || tx >= dst.w) continue;
          dst.buf[tx + ty * dst.w] = this.buf[i + j * this.w];
        }
      }
    }
  }

  // sqrt_fraction（TFT_eSPI::sqrt_fraction。整数の平方根の小数部）
  function sqrtFraction(num) {
    num >>>= 0;
    if (num > 0x40000000) return 0;
    let bsh = 0x00004000, fpr = 0, osh = 0;
    while (num > bsh) { bsh = (bsh * 4) >>> 0; osh++; }
    do {
      const bod = (bsh + fpr) >>> 0;
      if (num >= bod) { num = (num - bod) >>> 0; fpr = (bsh + bod) >>> 0; }
      num = (num * 2) >>> 0;
    } while ((bsh >>>= 1));
    return (fpr >>> osh) & 0xFF;
  }

  const ENC = (typeof TextEncoder !== 'undefined') ? new TextEncoder() : null;
  function utf8(s) {
    if (ENC) return ENC.encode(s);
    const out = [];   // TextEncoder の無い所（jsc の試験）。BMP の範囲だけ
    for (const ch of s) {
      const c = ch.codePointAt(0);
      if (c < 0x80) out.push(c);
      else if (c < 0x800) out.push(0xC0 | (c >> 6), 0x80 | (c & 63));
      else out.push(0xE0 | (c >> 12), 0x80 | ((c >> 6) & 63), 0x80 | (c & 63));
    }
    return Uint8Array.from(out);
  }

  // RGB565 → RGBA（キャンバス）。5/6 ビットを 8 ビットへ広げる（上位を下に繰り返す）
  let LUT = null;
  function rgbaLut() {
    if (LUT) return LUT;
    LUT = new Uint32Array(65536);
    const le = new Uint8Array(new Uint32Array([1]).buffer)[0] === 1;
    for (let c = 0; c < 65536; c++) {
      const r5 = c >> 11, g6 = (c >> 5) & 63, b5 = c & 31;
      const r = (r5 << 3) | (r5 >> 2), g = (g6 << 2) | (g6 >> 4), b = (b5 << 3) | (b5 >> 2);
      LUT[c] = le ? ((255 << 24) | (b << 16) | (g << 8) | r) >>> 0 : ((r << 24) | (g << 16) | (b << 8) | 255) >>> 0;
    }
    return LUT;
  }
  // 16bit の画面を ImageData へ
  function toImageData(spr, imageData) {
    const lut = rgbaLut();
    const out = new Uint32Array(imageData.data.buffer);
    const n = spr.w * spr.h;
    for (let i = 0; i < n; i++) out[i] = lut[spr.buf[i]];
  }
  // RGB565 → CSS の色
  function css565(c) {
    const r5 = c >> 11, g6 = (c >> 5) & 63, b5 = c & 31;
    return `rgb(${(r5 << 3) | (r5 >> 2)},${(g6 << 2) | (g6 >> 4)},${(b5 << 3) | (b5 >> 2)})`;
  }

  global.LiveTFT = { C, Sprite, alphaBlend, sprintf, fmtFixed, lround, fonts, toImageData, css565, f32 };
})(typeof window !== 'undefined' ? window : globalThis);
