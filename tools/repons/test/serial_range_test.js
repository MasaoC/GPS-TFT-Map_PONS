// ============================================================
// File    : tools/repons/test/serial_range_test.js
// Project : PONS v7.1 / v7.2 Re:PONS
// Role    : serial.js の塊ごとの取り込み（PONS:GETRANGE。2026-11-25〜）の試験。模擬の本体が 64 B のパケットの後ろを
//           決めた割合で捨て（2026-11-25 に生 IMU ログで見た抜け方のまね）、Re:PONS が合わない塊だけ取り直して元と 1 バイトも
//           違わない中身になるかを、仮の時計で確かめる。終わりの印・DATA_START の行が抜けたとき（止まりを見て中断→頼み直し）、
//           BUSY、GETRANGE を知らない古いファーム（PONS:GET に戻す）、一覧のあとに伸びたファイル、空のファイル、中断、
//           取り直しても合わない塊（取れた分で保存してアラート）、飛行の時刻の範囲（GETIMU）が合わなかったときの取り直しも。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026-11-25
// ============================================================
// serial.js の塊ごとの取り込み（PONS:GETRANGE）を、模擬の本体と仮の時計で試す。
// 本体: 64 B のパケットに分けて送り、決めた割合でパケットの後ろを捨てる（2026-11-25 に見た抜け方のまね）。
var __rx = [];          // { at, bytes } PC が受けるバイト（時刻順）
var __saved = null;     // saveBytesAsFile に渡った中身
var __lineLog = [];     // 本体が受けた行

function rng(seed) { var s = seed >>> 0; return function () { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; }; }

function crc32hex(u8) {
  // serial.js の computeCrc32 と同じものを使う
  return computeCrc32(u8);
}
function enc(s) { return new TextEncoder().encode(s); }
function concat(list) {
  var n = 0; list.forEach(function (a) { n += a.length; });
  var out = new Uint8Array(n), p = 0;
  list.forEach(function (a) { out.set(a, p); p += a.length; });
  return out;
}

function makeDevice(opt) {
  var d = {
    file: opt.file, path: opt.path, old: !!opt.old, rand: rng(opt.seed || 1),
    lossP: opt.lossP || 0, busyP: opt.busyP || 0,
    busyUntil: 0, sched: [],    // この本体が出した __rx の項目（CANCEL で後ろを消す）
    rangeCount: 0, forced: opt.forced || {},   // forced[n] = 'trailer' | 'start' : n 回目の GETRANGE で必ずそこを落とす
    kbps: opt.kbps || 1000, imuFrom: opt.imuFrom || 0, imuSize: opt.imuSize || 0,
  };
  d.emit = function (bytes, lossy, force) {
    // 64 B ごとのパケット。lossy なら確率でパケットの後ろ k バイト（と続く数パケット）を捨てる
    var t = Math.max(__now + 1, d.busyUntil);
    var pkts = [];
    for (var i = 0; i < bytes.length; i += 64) pkts.push(bytes.subarray(i, Math.min(bytes.length, i + 64)));
    var kept = [];
    var skip = 0;
    for (var j = 0; j < pkts.length; j++) {
      var pk = pkts[j];
      if (skip > 0) { skip--; continue; }
      if (lossy && d.lossP > 0 && d.rand() < d.lossP) {
        var k = 1 + Math.floor(d.rand() * pk.length);
        kept.push(pk.subarray(0, pk.length - k));
        if (d.rand() < 0.3) skip = Math.floor(d.rand() * 150);   // 数 KB ごと捨てることもある
        continue;
      }
      kept.push(pk);
    }
    var all = concat(kept);
    if (force === 'start') {
      var nl = all.indexOf(10);
      all = all.subarray(nl - 5);            // DATA_START の行の頭を落とす
    } else if (force === 'trailer') {
      all = all.subarray(0, all.length - 20); // 終わりの印をほとんど落とす
    }
    // 4KB ずつ、kbps の速さで届く
    for (var p = 0; p < all.length; p += 4096) {
      var piece = all.slice(p, Math.min(all.length, p + 4096));
      t += piece.length / d.kbps;   // ms
      var ev = { at: t, bytes: piece };
      __rx.push(ev); d.sched.push(ev);
    }
    d.busyUntil = t + 2;   // 送り終えて後始末するまで
  };
  d.onLine = function (line) {
    __lineLog.push(line);
    if (line.indexOf('PONS:GETRANGE,') === 0) {
      if (d.old) return;   // 知らない
      if (__now < d.busyUntil || (d.busyP > 0 && d.rand() < d.busyP)) {
        d.busyCount = (d.busyCount || 0) + 1;
        var at = Math.max(__now + 1, d.busyUntil);
        var ev = { at: at + 0.5, bytes: enc('PONS:ERROR,BUSY\r\n') };
        __rx.push(ev);
        return;
      }
      var m = line.substring(14).match(/^(\d+),(\d+),(.*)$/);
      var from = +m[1], len = +m[2];
      var fs = d.file.length;
      if (from > fs) { d.emit(enc('PONS:ERROR,INVALID_RANGE\r\n'), false); return; }
      var end = Math.min(fs, from + len);
      d.rangeCount++;   // 本当に送った回数（BUSY は数えない）
      var body = d.file.subarray(from, end);
      d.emit(enc('PONS:GETRANGE_ACCEPTED\r\n'), false);
      var stream = concat([enc('PONS:DATA_START,' + m[3] + ',' + (end - from) + ',' + from + ',' + fs + '\r\n'), body,
                           enc('\r\nPONS:DATA_END,' + crc32hex(body) + '\r\n')]);
      d.emit(stream, true, d.forced[d.rangeCount]);
    } else if (line.indexOf('PONS:GETIMU,') === 0) {
      // 範囲は固定（from = imuFrom, 大きさ imuSize）。1 回目は lossy で送る
      var mi = line.substring(12).match(/^(\d+),(\d+),(.*)$/);
      var b3 = d.file.subarray(d.imuFrom, d.imuFrom + d.imuSize);
      d.emit(enc('PONS:GETIMU_ACCEPTED\r\n'), false);
      d.emit(concat([enc('PONS:DATA_START,' + mi[3] + ',' + b3.length + ',' + d.imuFrom + '\r\n'), b3,
                     enc('\r\nPONS:DATA_END,' + crc32hex(b3) + '\r\n')]), true);
    } else if (line.indexOf('PONS:GET,') === 0) {
      var body2 = d.file;
      d.emit(concat([enc('PONS:DATA_START,' + line.substring(9) + ',' + body2.length + '\r\n'), body2,
                     enc('\r\nPONS:DATA_END,' + crc32hex(body2) + '\r\n')]), false);
    } else if (line === 'PONS:CANCEL') {
      if (__now < d.busyUntil) {
        // 送っている途中: まだ届いていない分を消して CANCEL_OK
        __rx = __rx.filter(function (ev) { return !(d.sched.indexOf(ev) >= 0 && ev.at > __now); });
        d.busyUntil = __now;
        d.emit(enc('\r\nPONS:CANCEL_OK\r\n'), false);
      } else {
        d.emit(enc('PONS:CANCEL_OK\r\n'), false);
      }
    }
  };
  return d;
}

function runUntilIdle(maxMs) {
  var guard = 0;
  while (guard++ < 2000000) {
    drainMicrotasks();
    __rx.sort(function (a, b) { return a.at - b.at; });
    __timers.sort(function (a, b) { return a.at - b.at; });
    var nr = __rx.length ? __rx[0].at : Infinity;
    var nt = __timers.length ? __timers[0].at : Infinity;
    var next = Math.min(nr, nt);
    if (next === Infinity || next > maxMs) break;
    __now = next;
    if (nr <= nt) {
      var ev = __rx.shift();
      lastSerialRxMs = __now;
      feedSerialBytes(ev.bytes);
    } else {
      var t = __timers.shift();
      if (t.repeat) __timers.push({ id: t.id, at: __now + t.ms, fn: t.fn, ms: t.ms, repeat: true });
      t.fn();
    }
    // 転送が終わってタイマーが残っていなければ抜ける
    if (!downloadBusy() && __rx.length === 0 && __timers.length === 0) break;
  }
  drainMicrotasks();
}

function startCase(dev, listSize, mode) {
  __rx = []; __timers = []; __saved = null; __alerts = []; __warns = []; __lineLog = []; __now = 0;
  rangeSupport = null; rangeDl = null; rangeStallTimer = null;
  endDownloadUi();
  serialWriter = {};
  sendSerialLine = function (s) { var dv = dev; var at = __now + 0.5;
    __timers.push({ id: __timerSeq++, at: at, fn: function () { dv.onLine(s); }, ms: 0, repeat: false });
    return Promise.resolve(); };
  saveBytesAsFile = function (bytes, path) { __saved = bytes.slice(); };
  downloadPonsFile(dev.path, listSize, 0, mode || 'save');
}

function sameBytes(a, b) {
  if (!a || a.length !== b.length) return false;
  for (var i = 0; i < a.length; i++) if (a[i] !== b[i]) return false;
  return true;
}

var fails = 0;
function check(name, cond, extra) {
  print((cond ? 'OK   ' : 'FAIL ') + name + (extra ? '  ' + extra : ''));
  if (!cond) fails++;
}

function randFile(n, seed) {
  var r = rng(seed), u = new Uint8Array(n);
  for (var i = 0; i < n; i++) u[i] = Math.floor(r() * 256);
  return u;
}

var FILE = randFile(5 * 1024 * 1024 + 12345, 7);   // 5.01 MB（最後の塊が半端）
var note = function () { return document.getElementById('serial-file-note').textContent; };

// 1. 抜けなし
var dev = makeDevice({ file: FILE, path: 'imuraw/20261125.bin' });
startCase(dev, FILE.length); runUntilIdle(1e9);
check('抜けなし: 中身が一致', sameBytes(__saved, FILE), note());
check('抜けなし: 塊の数', dev.rangeCount === 6, '送った塊 ' + dev.rangeCount + ' 回・BUSY ' + (dev.busyCount || 0) + ' 回');

// 2. ふつうの抜け（2026-11-25 より多め）。何回か種を変える
for (var seed = 1; seed <= 8; seed++) {
  dev = makeDevice({ file: FILE, path: 'imuraw/20261125.bin', lossP: 0.00003, seed: seed });
  startCase(dev, FILE.length); runUntilIdle(1e9);
  check('抜けあり seed ' + seed + ': 中身が一致', sameBytes(__saved, FILE), note() + ' / alerts ' + __alerts.length);
}

// 3. 終わりの印が抜ける（2 回目の塊）・DATA_START の行が抜ける（4 回目の頼み）→ 止まったのを見て頼み直す
dev = makeDevice({ file: FILE, path: 'imuraw/20261125.bin', forced: { 2: 'trailer', 4: 'start' } });
startCase(dev, FILE.length); runUntilIdle(1e9);
check('印が抜ける: 中身が一致', sameBytes(__saved, FILE), note());
check('印が抜ける: CANCEL で片付けた', __lineLog.filter(function (l) { return l === 'PONS:CANCEL'; }).length === 2,
      __lineLog.filter(function (l) { return l === 'PONS:CANCEL'; }).length + ' 回');

// 4. BUSY が返る
dev = makeDevice({ file: FILE, path: 'imuraw/20261125.bin', busyP: 0.3, seed: 3 });
startCase(dev, FILE.length); runUntilIdle(1e9);
check('BUSY: 中身が一致・アラートなし', sameBytes(__saved, FILE) && __alerts.length === 0, note());

// 5. 古いファーム（GETRANGE を知らない）→ 5 秒で PONS:GET
dev = makeDevice({ file: FILE, path: 'imuraw/20261125.bin', old: true });
startCase(dev, FILE.length); runUntilIdle(1e9);
check('古いファーム: GET に戻って中身が一致', sameBytes(__saved, FILE) && rangeSupport === false, note());

// 6. 一覧のあとでファイルが伸びた（一覧は 3MB と言っていた）
dev = makeDevice({ file: FILE, path: 'imuraw/20261125.bin' });
startCase(dev, 3 * 1024 * 1024); runUntilIdle(1e9);
check('一覧より伸びた: 本体の大きさで全部取る', sameBytes(__saved, FILE), note());

// 7. 空のファイル
var EMPTY = new Uint8Array(0);
dev = makeDevice({ file: EMPTY, path: 'settings.txt' });
startCase(dev, 0); runUntilIdle(1e9);
check('空のファイル', __saved !== null && __saved.length === 0, note());

// 8. 途中で中断
dev = makeDevice({ file: FILE, path: 'imuraw/20261125.bin' });
startCase(dev, FILE.length); runUntilIdle(2500);
cancelPonsDownload(); runUntilIdle(1e9);
check('中断: 保存しない・転送の状態が下りる', __saved === null && !downloadBusy() && rangeDl === null && rangeStallTimer === null);

// 9. ひどい抜け（取り直しても合わない塊が出る）→ 取れた分で保存してアラート
dev = makeDevice({ file: FILE, path: 'imuraw/20261125.bin', lossP: 0.002, seed: 5 });
startCase(dev, FILE.length); runUntilIdle(1e9);
check('ひどい抜け: 保存してアラート', __saved !== null && __alerts.length === 1 && __saved.length < FILE.length,
      (__saved ? __saved.length : -1) + ' B / ' + note());

// 10. 開く（📊 解析）も同じ道を通る
var __opened = null;
loadImuBinBytes = function (bytes, name, o) { __opened = { bytes: bytes.slice(), warn: o.transferWarning }; };
dev = makeDevice({ file: FILE, path: 'imuraw/20261125.bin', lossP: 0.00003, seed: 11 });
startCase(dev, FILE.length, 'open'); runUntilIdle(1e9);
check('解析で開く: 中身が一致・警告なし', __opened && sameBytes(__opened.bytes, FILE) && __opened.warn === null);


// 11. 飛行の時刻の範囲（GETIMU）。1 回目は抜けて合わない → その範囲を 1MB ずつ取り直す
function startImu(dev, mode) {
  __rx = []; __timers = []; __saved = null; __alerts = []; __warns = []; __lineLog = []; __now = 0; __opened = null;
  rangeSupport = null; rangeDl = null; rangeStallTimer = null;
  endDownloadUi();
  serialWriter = {};
  sendSerialLine = function (s) { var dv = dev; var at = __now + 0.5;
    __timers.push({ id: __timerSeq++, at: at, fn: function () { dv.onLine(s); }, ms: 0, repeat: false });
    return Promise.resolve(); };
  downloadPonsFile(dev.path, dev.file.length, 0, 'open', { jstStart: 1000, jstEnd: 2000 });
}
var IF = 1234567, IS = 2 * 1024 * 1024 + 999;
var want = FILE.slice(IF, IF + IS);
dev = makeDevice({ file: FILE, path: 'imuraw/20261125.bin', lossP: 0.0003, seed: 21, imuFrom: IF, imuSize: IS });
// 1 回目（GETIMU）だけ抜けさせ、取り直し（GETRANGE）は抜けないようにする
var origOnLine = dev.onLine;
dev.onLine = function (line) { if (line.indexOf('PONS:GETRANGE,') === 0) dev.lossP = 0; origOnLine(line); };
startImu(dev); runUntilIdle(1e9);
check('GETIMU が合わない → 範囲を取り直して一致・警告なし', __opened && sameBytes(__opened.bytes, want) && __opened.warn === null,
      (__opened ? __opened.bytes.length + ' B' : 'なし') + ' / GETRANGE ' + dev.rangeCount + ' 回');

// 12. GETIMU は知っているが GETRANGE を知らない（1.12 まで）→ 1 回目の分を警告つきで開く
dev = makeDevice({ file: FILE, path: 'imuraw/20261125.bin', lossP: 0.0003, seed: 22, imuFrom: IF, imuSize: IS, old: true });
startImu(dev); runUntilIdle(1e9);
check('GETIMU・古いファーム: 1 回目の分を警告つきで開く', __opened && __opened.bytes.length <= IS && __opened.warn !== null &&
      rangeSupport === false && !downloadBusy(), __opened ? __opened.warn : 'なし');

// 13. GETIMU が 1 回で合ったら取り直さない
dev = makeDevice({ file: FILE, path: 'imuraw/20261125.bin', imuFrom: IF, imuSize: IS });
startImu(dev); runUntilIdle(1e9);
check('GETIMU が合った: 取り直さない', __opened && sameBytes(__opened.bytes, want) && dev.rangeCount === 0);

print(fails === 0 ? 'ALL PASS' : (fails + ' FAILED'));
