// ============================================================
// File    : tools/flight_viewer/olddata_open.js
// Project : PONS v7 / v7.1 Flight Viewer
// Role    : 暗号化した過去の大会ログ（tools/olddata/*.enc。PONSENC2）をパスワードで開いて再生する
// Author  : Claude (2026-11-20)
// Updated : 2026-11-20
// ============================================================
//
// 形式は tools/olddata_crypt.py と同じ: "PONSENC2" | KDF=1, 反復回数, 0 (uint32 LE) | salt 16B | nonce 12B | 暗号文+tag
// 鍵は PBKDF2-HMAC-SHA256、本体は AES-256-GCM（ヘッダー 48 バイトを AAD）。どちらも WebCrypto だけで復号する（自前の暗号コードは無い）。
// パスワードはこのページを開いている間だけメモリに置く（保存しない）。
// 公開ページ（GitHub Pages）では ../olddata/manifest.json から一覧を出す。file:// で開いたときは手元の .enc を選ぶ。

(function (global) {
  'use strict';

  const VERSION = '2026.11.20.16';
  const OLDDATA_BASE = '../olddata/';
  const KDF_PBKDF2_SHA256 = 1;
  let rememberedPassword = null;   // このページを開いている間だけ

  // ------------------------------------------------------------
  // 復号（WebCrypto）
  // ------------------------------------------------------------
  async function decryptPonsEnc(buf, password) {
    const u8 = new Uint8Array(buf);
    const magic = String.fromCharCode(...u8.subarray(0, 8));
    if (magic === 'PONSENC1') {
      throw new Error('古い形式（scrypt）です。python3 tools/olddata_crypt.py decrypt で戻してください。');
    }
    if (magic !== 'PONSENC2' || u8.length < 48 + 16) throw new Error('PONSENC2 の形式ではありません。');
    const dv = new DataView(u8.buffer, u8.byteOffset, u8.byteLength);
    const kdf = dv.getUint32(8, true), iters = dv.getUint32(12, true);
    if (kdf !== KDF_PBKDF2_SHA256 || iters < 100000 || iters > 10000000) throw new Error('ヘッダーの数がおかしい（壊れているか別物）。');
    const header = u8.slice(0, 48), salt = u8.slice(20, 36), nonce = u8.slice(36, 48);
    if (!global.crypto || !global.crypto.subtle) throw new Error('このブラウザ（またはこの開き方）では WebCrypto が使えません。https の公開ページか Chrome / Edge で開いてください。');
    const pwKey = await crypto.subtle.importKey('raw', new TextEncoder().encode(password), 'PBKDF2', false, ['deriveKey']);
    const aesKey = await crypto.subtle.deriveKey({ name: 'PBKDF2', hash: 'SHA-256', salt, iterations: iters },
      pwKey, { name: 'AES-GCM', length: 256 }, false, ['decrypt']);
    try {
      const plain = await crypto.subtle.decrypt({ name: 'AES-GCM', iv: nonce, additionalData: header, tagLength: 128 },
        aesKey, u8.slice(48));
      return new Uint8Array(plain);
    } catch (e) {
      const err = new Error('復元できません（パスワードが違うか、ファイルが壊れています）。');
      err.badPassword = true;
      throw err;
    }
  }

  // ------------------------------------------------------------
  // 画面（一覧・パスワード）
  // ------------------------------------------------------------
  function injectStyle() {
    if (document.getElementById('od-style')) return;
    const st = document.createElement('style');
    st.id = 'od-style';
    st.textContent = `
      .od-overlay { position: fixed; inset: 0; z-index: 10020; background: rgba(15,23,42,0.6); display: flex;
                    align-items: center; justify-content: center; padding: 16px; }
      .od-box { background: #fff; color: #0f172a; border-radius: 10px; box-shadow: 0 20px 50px rgba(0,0,0,0.35);
                width: min(520px, 100%); padding: 16px 18px; font-size: 0.86rem; }
      .od-title { font-weight: 700; font-size: 1rem; margin-bottom: 8px; }
      .od-note { color: #64748b; font-size: 0.78rem; line-height: 1.5; margin: 6px 0 10px 0; }
      .od-list { display: flex; flex-direction: column; gap: 6px; margin: 8px 0; }
      .od-item { display: flex; justify-content: space-between; align-items: center; gap: 8px; padding: 8px 10px;
                 border: 1px solid #e2e8f0; border-radius: 6px; cursor: pointer; background: #f8fafc; text-align: left; }
      .od-item:hover { background: #e0f2fe; border-color: #7dd3fc; }
      .od-item small { color: #64748b; font-family: var(--font-mono, monospace); }
      .od-row { display: flex; gap: 8px; justify-content: flex-end; margin-top: 12px; flex-wrap: wrap; }
      .od-box input[type=password] { width: 100%; padding: 7px 9px; font-size: 0.95rem; border: 1px solid #cbd5e1;
                 border-radius: 6px; box-sizing: border-box; }
      .od-err { color: #b91c1c; min-height: 1.2em; margin-top: 6px; font-weight: 600; }
    `;
    document.head.appendChild(st);
  }

  function openOverlay(html) {
    injectStyle();
    const ov = document.createElement('div');
    ov.className = 'od-overlay';
    ov.innerHTML = `<div class="od-box">${html}</div>`;
    ov.addEventListener('click', (e) => { if (e.target === ov) ov.remove(); });
    document.body.appendChild(ov);
    return ov;
  }

  function escHtml(s) {
    return String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');
  }

  function labelOf(file) {
    const m = String(file).match(/(20\d\d)-(\d\d)-(\d\d)/);
    return m ? `${m[1]} 年大会（${m[1]}-${m[2]}-${m[3]}）` : file;
  }

  // パスワードを聞く。正しければ復号したバイト列を返す（キャンセルなら null）
  function askPasswordAndDecrypt(buf, name) {
    return new Promise((resolve) => {
      const ov = openOverlay(`
        <div class="od-title">🔒 ${escHtml(labelOf(name))}</div>
        <div class="od-note">公開の許可が無いので暗号化してあるログです。パスワードはチームの担当者に聞いてください。</div>
        <input type="password" id="od-pw" autocomplete="current-password" placeholder="パスワード">
        <label style="display:flex; gap:6px; align-items:center; margin-top:8px; color:#475569; font-size:0.8rem;">
          <input type="checkbox" id="od-remember" checked> このページを開いている間は覚える（保存はしない）
        </label>
        <div class="od-err" id="od-err"></div>
        <div class="od-row">
          <button id="od-cancel">キャンセル</button>
          <button id="od-ok" class="btn-primary">開く</button>
        </div>`);
      const pw = ov.querySelector('#od-pw'), err = ov.querySelector('#od-err'), ok = ov.querySelector('#od-ok');
      const close = (v) => { ov.remove(); resolve(v); };
      ov.querySelector('#od-cancel').onclick = () => close(null);
      const tryOpen = async () => {
        if (!pw.value) { err.textContent = 'パスワードを入れてください。'; return; }
        ok.disabled = true; err.style.color = '#475569'; err.textContent = '復号中…（1 秒ほど）';
        try {
          const bytes = await decryptPonsEnc(buf, pw.value);
          if (ov.querySelector('#od-remember').checked) rememberedPassword = pw.value;
          close(bytes);
        } catch (e) {
          err.style.color = '#b91c1c'; err.textContent = e.message; ok.disabled = false;
          if (e.badPassword) { pw.select(); pw.focus(); }
        }
      };
      ok.onclick = tryOpen;
      pw.addEventListener('keydown', (e) => { if (e.key === 'Enter') tryOpen(); if (e.key === 'Escape') close(null); });
      setTimeout(() => pw.focus(), 0);
    });
  }

  // .enc のバイト列を開いて再生する（覚えたパスワードがあればまずそれで試す）
  async function openEncryptedBytes(buf, name) {
    let bytes = null;
    if (rememberedPassword) {
      try { bytes = await decryptPonsEnc(buf, rememberedPassword); } catch (e) { if (!e.badPassword) { alert(e.message); return; } }
    }
    if (!bytes) bytes = await askPasswordAndDecrypt(buf, name);
    if (!bytes) return;
    const plainName = String(name).replace(/^.*\//, '').replace(/\.enc$/i, '');
    const text = new TextDecoder('utf-8').decode(bytes);
    if (typeof global.parseCSVText === 'function') global.parseCSVText(text, plainName);
  }

  // 手元の .enc（ファイル選択・ドロップ）
  function openEncryptedFile(file) {
    const rd = new FileReader();
    rd.onload = () => openEncryptedBytes(rd.result, file.name);
    rd.readAsArrayBuffer(file);
  }

  function pickLocalEnc() {
    const inp = document.createElement('input');
    inp.type = 'file'; inp.accept = '.enc';
    inp.onchange = () => { if (inp.files && inp.files[0]) openEncryptedFile(inp.files[0]); };
    inp.click();
  }

  // 「🔒 大会ログ」: 公開ページなら一覧から、そうでなければ手元の .enc を選ぶ
  async function openOldDataMenu() {
    const ov = openOverlay(`
      <div class="od-title">🔒 過去の大会ログ（暗号化）</div>
      <div class="od-note">2024〜2026 年の大会のログです。開くにはパスワードが要ります（チームの担当者に聞いてください）。</div>
      <div class="od-list" id="od-list"><div class="od-note">一覧を読み込み中…</div></div>
      <div class="od-row">
        <button id="od-local">📁 手元の .enc を開く</button>
        <button id="od-close">閉じる</button>
      </div>`);
    ov.querySelector('#od-close').onclick = () => ov.remove();
    ov.querySelector('#od-local').onclick = () => { ov.remove(); pickLocalEnc(); };
    const list = ov.querySelector('#od-list');
    let files = null;
    try {
      if (location.protocol !== 'http:' && location.protocol !== 'https:') throw new Error('file');
      const r = await fetch(OLDDATA_BASE + 'manifest.json', { cache: 'no-store' });
      if (!r.ok) throw new Error(String(r.status));
      files = (await r.json()).files || [];
    } catch (e) {
      list.innerHTML = `<div class="od-note">このページからは一覧を取れません（手元のファイルとして開いているか、まだ公開されていません）。
        リポジトリの <code>tools/olddata/*.enc</code> を「手元の .enc を開く」で選んでください。</div>`;
      return;
    }
    if (files.length === 0) { list.innerHTML = '<div class="od-note">ログがありません。</div>'; return; }
    list.innerHTML = '';
    files.forEach((f) => {
      const b = document.createElement('button');
      b.className = 'od-item';
      b.innerHTML = `<span>${escHtml(labelOf(f.file))}</span><small>${escHtml(f.file)} · ${(f.bytes / 1024).toFixed(0)} KB</small>`;
      b.onclick = async () => {
        b.disabled = true;
        try {
          const r = await fetch(OLDDATA_BASE + encodeURIComponent(f.file), { cache: 'no-store' });
          if (!r.ok) throw new Error(`取得できませんでした（${r.status}）`);
          const buf = await r.arrayBuffer();
          ov.remove();
          openEncryptedBytes(buf, f.file);
        } catch (e) {
          b.disabled = false;
          alert(e.message);
        }
      };
      list.appendChild(b);
    });
  }

  global.PONS_OLDDATA_VERSION = VERSION;
  global.decryptPonsEnc = decryptPonsEnc;
  global.openEncryptedFile = openEncryptedFile;
  global.openEncryptedBytes = openEncryptedBytes;
  global.openOldDataMenu = openOldDataMenu;
})(window);
