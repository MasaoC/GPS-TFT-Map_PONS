// ============================================================
// File    : tools/repons/test/serial_stubs.js
// Project : PONS v7.1 / v7.2 Re:PONS
// Role    : serial.js を jsc で読むための最小のスタブ（DOM・仮の時計とタイマー・TextDecoder・ほかのファイルの関数）。
//           run_serial_tests.py が serial_stubs.js → serial.js → serial_range_test.js の順に読む。
// Author  : MasaoC (@masao_mobile)
// Updated : 2026-11-25
// ============================================================
// jsc で serial.js を読むための最小のスタブ（DOM・タイマー・時計・TextDecoder）
var globalThis_ = this;
var window = this;
var __now = 0;
var performance = { now: function () { return __now; } };
var __timers = [];
var __timerSeq = 1;
function __addTimer(fn, ms, repeat) {
  var id = __timerSeq++;
  __timers.push({ id: id, at: __now + Math.max(0, ms || 0), fn: fn, ms: ms, repeat: repeat });
  return id;
}
function setTimeout(fn, ms) { return __addTimer(fn, ms, false); }
function setInterval(fn, ms) { return __addTimer(fn, ms, true); }
function clearTimeout(id) { __timers = __timers.filter(function (t) { return t.id !== id; }); }
var clearInterval = clearTimeout;

var console = {
  log: function () {},
  warn: function () { __warns.push(Array.prototype.join.call(arguments, ' ')); },
  error: function () { print('ERR ' + Array.prototype.join.call(arguments, ' ')); },
};
var __warns = [];
var __alerts = [];
function alert(s) { __alerts.push(String(s)); }
function confirm() { return false; }

function __el() {
  return { style: {}, textContent: '', innerHTML: '', disabled: false,
           classList: { add: function () {}, remove: function () {}, contains: function () { return true; } } };
}
var __els = {};
var document = {
  getElementById: function (id) { if (!__els[id]) __els[id] = __el(); return __els[id]; },
  createElement: function () { return __el(); },
  body: { appendChild: function () {}, removeChild: function () {} },
};

function TextEncoder() {}
TextEncoder.prototype.encode = function (s) {
  s = unescape(encodeURIComponent(s));
  var u = new Uint8Array(s.length);
  for (var i = 0; i < s.length; i++) u[i] = s.charCodeAt(i);
  return u;
};
function TextDecoder() {}
TextDecoder.prototype.decode = function (u) {
  var s = '';
  for (var i = 0; i < u.length; i++) s += String.fromCharCode(u[i]);
  try { return decodeURIComponent(escape(s)); } catch (e) { return s; }
};

// 他のファイルにあるもの
function logBytes(n) { return n + ' B'; }
function resetSettingsState() {}
function abortSettingsPending() {}
function handleSettingsSerialLine() {}
function renderFileList() {}
function closeSerialModal() {}
