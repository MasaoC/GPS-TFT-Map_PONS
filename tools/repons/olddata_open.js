// ============================================================
// File    : tools/repons/olddata_open.js
// Project : PONS v7 / v7.1 Re:PONS
// Role    : 暗号化した過去の大会ログ（tools/olddata/*.enc。PONSENC2）をパスワードで開いて再生する
// Author  : Claude (2026-11-20)
// Updated : 2026-11-21
// ============================================================
//
// 形式は tools/olddata_crypt.py と同じ: "PONSENC2" | KDF=1, 反復回数, 0 (uint32 LE) | salt 16B | nonce 12B | 暗号文+tag
// 鍵は PBKDF2-HMAC-SHA256、本体は AES-256-GCM（ヘッダー 48 バイトを AAD）。どちらも WebCrypto だけで復号する（自前の暗号コードは無い）。
// パスワードはこのページを開いている間だけメモリに置く（保存しない）。
// 公開ページ（GitHub Pages）では ../olddata/manifest.json から一覧を出す。file:// で開いたときは手元の .enc を選ぶ。
// 大会動画（YouTube 限定公開）の URL は videos.json.enc に同じパスワードで暗号化してある。ログを開くときに同じパスワードで
// 復号して video_sync.js へ渡す（手元で開いたときは videos.json.enc を選ぶ・ドロップする）。
// 大会ログは年ごとに記録した機器・センサー・後処理が違うので、開いたら「データの注意」（OLDDATA_NOTES）を出す
//   （index.html が CSV を読み終えたときの 'pons-csv-loaded' イベント。平文の _fused.csv を開いても同じ名前なら出す）。
//   中身は tools/olddata/README.md の「年ごとの違い」と同じ。変換（convert_old_contest_csv.py・fuse_attitude_csv.py）を変えたら両方直す。

(function (global) {
  'use strict';

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
      .od-box.wide { width: min(760px, 100%); max-height: calc(100vh - 32px); overflow-y: auto; }
      .od-warn { background: #fffbeb; border: 1px solid #fcd34d; color: #78350f; border-radius: 6px; padding: 8px 10px;
                 margin: 6px 0 10px 0; line-height: 1.55; }
      .od-warn ul { margin: 0; padding-left: 1.2em; }
      .od-tablewrap { overflow-x: auto; }
      .od-table { border-collapse: collapse; width: 100%; font-size: 0.8rem; line-height: 1.5; }
      .od-table th, .od-table td { border-top: 1px solid #e2e8f0; padding: 5px 6px; text-align: left; vertical-align: top; }
      .od-table { table-layout: fixed; min-width: 520px; }
      .od-table th { color: #334155; }
      .od-table col.c1 { width: 17%; } .od-table col.c2 { width: 47%; } .od-table col.c3 { width: 36%; }
      .od-table td.none { color: #94a3b8; }
      .od-check { display: flex; align-items: center; gap: 6px; color: #64748b; font-size: 0.78rem; margin-right: auto; }
      #od-notes-btn { margin-left: 8px; background: #fef3c7; border: 1px solid #f59e0b; color: #92400e; font-weight: 700;
                 border-radius: 999px; padding: 1px 9px; font-size: 0.74rem; cursor: pointer; vertical-align: middle; }
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
          close({ bytes, password: pw.value });
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

  // 動画の対応表（videos.json）を使えるようにする
  function applyVideoMap(bytes) {
    try {
      const map = JSON.parse(new TextDecoder('utf-8').decode(bytes));
      if (global.PonsVideo) global.PonsVideo.setMap(map);
      return Object.keys(map || {}).length;
    } catch (e) {
      console.warn('videos.json を読めません', e);
      return -1;
    }
  }

  // 公開ページなら ../olddata/videos.json.enc を、ログと同じパスワードで読む（無い・違うパスワードなら黙って何もしない）
  async function loadRemoteVideoMap(password) {
    if (!global.PonsVideo || global.PonsVideo.hasMap()) return;
    if (location.protocol !== 'http:' && location.protocol !== 'https:') return;
    try {
      const r = await fetch(OLDDATA_BASE + 'videos.json.enc', { cache: 'no-store' });
      if (!r.ok) return;
      applyVideoMap(await decryptPonsEnc(await r.arrayBuffer(), password));
    } catch (e) {
      console.warn('videos.json.enc を開けません', e.message);
    }
  }

  // .enc のバイト列を開く（覚えたパスワードがあればまずそれで試す）。ログなら再生、videos.json なら動画の対応表に
  async function openEncryptedBytes(buf, name) {
    let res = null;
    if (rememberedPassword) {
      try { res = { bytes: await decryptPonsEnc(buf, rememberedPassword), password: rememberedPassword }; }
      catch (e) { if (!e.badPassword) { alert(e.message); return; } }
    }
    if (!res) res = await askPasswordAndDecrypt(buf, name);
    if (!res) return;
    const plainName = String(name).replace(/^.*\//, '').replace(/\.enc$/i, '');
    if (/videos[^\/]*\.json$/i.test(plainName)) {
      const n = applyVideoMap(res.bytes);
      if (n < 0) { alert('動画の対応表（videos.json）の形が読めません。'); return; }
      if (global.PonsVideo) global.PonsVideo.attachCurrent();
      alert(`大会動画の対応表を読み込みました（${n} 件）。対応するログを開くと動画が出ます。`);
      return;
    }
    await loadRemoteVideoMap(res.password);   // 再生を始める前に（CSV を読み終えたときに動画を出すため）
    const text = new TextDecoder('utf-8').decode(res.bytes);
    if (typeof global.parseCSVText === 'function') global.parseCSVText(text, plainName);
  }

  // 手元の .enc（ファイル選択・ドロップ）
  function openEncryptedFile(file) {
    const rd = new FileReader();
    rd.onload = () => openEncryptedBytes(rd.result, file.name);
    rd.readAsArrayBuffer(file);
  }

  // 複数の .enc（ファイル選択・ドロップ）。動画の対応表（videos.json.enc）を先に開く（同じパスワードなら聞くのは 1 回）
  async function openEncryptedFiles(files) {
    const fs = files.slice().sort((a, b) => (/videos/i.test(b.name) ? 1 : 0) - (/videos/i.test(a.name) ? 1 : 0));
    for (const f of fs) await openEncryptedBytes(await f.arrayBuffer(), f.name);
  }

  function pickLocalEnc() {
    const inp = document.createElement('input');
    inp.type = 'file'; inp.accept = '.enc'; inp.multiple = true;
    inp.onchange = () => openEncryptedFiles(Array.from(inp.files || []));
    inp.click();
  }

  // 「🔒 大会ログ」: 公開ページなら一覧から、そうでなければ手元の .enc を選ぶ
  async function openOldDataMenu() {
    const ov = openOverlay(`
      <div class="od-title">🔒 過去の大会ログ（暗号化）</div>
      <div class="od-note">2024〜2026 年の大会のログです。開くにはパスワードが要ります（チームの担当者に聞いてください）。
        大会動画（YouTube）があるログは、開くと動画が小窓に出て再生に合わせて動きます。</div>
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
    files = files.filter(f => /\.csv\.enc$/i.test(f.file));   // videos.json.enc は一覧に出さない（ログを開くときに一緒に読む）
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

  // ------------------------------------------------------------
  // 年ごとのデータの注意（tools/olddata/README.md の「年ごとの違い」と同じ中身）
  //   rows: [項目, 中身・出どころ, 注意]。中身が null の項目は「無い」と出す
  // ------------------------------------------------------------
  const OLDDATA_NOTES = {
    '2024-07-27_0936': {
      title: '2024 年大会（2024-07-27）',
      source: '追走ボートに載せた PONS v3 の位置の記録（1 秒ごと）だけ。',
      warn: [
        '<b>機体ではなく追走ボートの位置・速さ</b>です（2024 年は PONS を機体に載せていない。README の使用実績）。ボートは機体を後ろから追うので、機体とは位置も時刻も少しずれます。',
        '<b>位置・速さ・航跡だけのログ</b>です。高度・V/S・姿勢・方位・対気速度・風は入っていません（グラフと飛行解析のその部分は空になります）。',
        'G/S は元の記録が<b>ノット</b>だったので m/s に直してあります。',
        'Garmin（高度・パワー）と対気速度センサーのログは別にありますが、時刻を合わせきれなかったので入れていません。'
      ],
      rows: [
        ['位置・G/S・航跡', '追走ボートの PONS v3 の GNSS（1 秒ごと）', '機体ではなくボート。G/S はノット → m/s に換算'],
        ['高度', null, ''],
        ['V/S', null, ''],
        ['ロール・ピッチ', null, ''],
        ['方位', null, ''],
        ['対気速度', null, ''],
        ['風', null, ''],
        ['電圧・衛星数', null, '']
      ]
    },
    '2025-07-26_1539': {
      title: '2025 年大会（2025-07-26）',
      source: '機体（白夜）に載せた PONS v5 の位置の記録＋Garmin（サイクルコンピューター）＋電装のログ（BNO085・対気速度センサー）を、JST で 1 秒ごとに突き合わせたもの。',
      warn: [
        '<b>別々の機器のログを時刻で突き合わせた</b>ものです。機器どうしで 1 秒ほどずれていることがあります。',
        '姿勢・高度・対気速度があるのは <b>15:42:16〜16:28:31</b> だけです。その前と後は位置だけです。',
        '<b>V/S は記録に無く、ビューアが高度の 1 秒ごとの差から出しています。</b>Garmin の高度は 0.2 m 刻みなので、V/S は 0 / ±0.2 / ±0.4 m/s の飛び飛びの値になり、跳ねる所もあります。細かい上り下りは読めません。',
        '「TAS」の欄は PONS の推定ではなく、<b>対気速度センサーの実測を主翼の影響の係数で補正した値</b>です（YouTube の動画と同じ）。',
        'ロール・ピッチは <b>BNO085 使用、加速度エラーあり</b>。加速・旋回で傾きが見かけ上ずれます。直したのは取り付けのずれだけです。',
        '<b>北パイロンの位置は 2026 年から変わっています。</b>地図の北パイロンは 2026 年以降の位置で、2025 年の北パイロン（35.368138, 136.174102）は'
          + 'そこから<b>南西へ約 1.8 km</b>（プラットホームからの方位 318°。2026 年以降は 328°。距離はどちらも約 11.0 km）。旋回の位置を地図の北パイロンと比べないでください。'
      ],
      rows: [
        ['位置・G/S・航跡', 'PONS v5 の GNSS（1〜2 秒ごと）を 1 秒ごとの行へ補間', '3 秒より空いた所は空欄'],
        ['高度', 'Garmin の高度（海抜。湖面はおよそ 83 m）', '0.2 m 刻み。PONS の高度ではない'],
        ['V/S', 'ビューアが高度の差から計算', '0.2 m 刻みの段で粗い。跳ねる所あり'],
        ['ロール・ピッチ', '電装の BNO085。ロール = 生 − 8°、ピッチ = 8° − 生（向きを反転し取り付けの 8° を足した。動画と同じ）', '<b>BNO085 使用、加速度エラーあり</b>。0 点は取り付けの仮定'],
        ['方位', null, 'BNO085 の方位は 45 分で航跡との差が 50° 以上流れるので入れていない'],
        ['Est.Beta', null, '方位が無いので出さない（航跡で代わりにすると偏流角の変化が β に入る）'],
        ['地図の北パイロン', '2026 年以降の位置', '2025 年の北パイロンは南西へ約 1.8 km（35.368138, 136.174102）。1km パイロンは約 100 m 北、プラットホームと南パイロンは 1 m 程度の差'],
        ['対気速度（TAS 欄）', '対気速度センサーの実測 × 主翼の影響の補正係数（7 m/s で約 1.12。動画と同じ）', '推定 TAS ではない。低速では係数が大きくなるので地上の値は当てにならない'],
        ['風', null, ''],
        ['電圧・衛星数', null, '']
      ]
    },
    '2026-07-26_0804': {
      title: '2026 年大会（2026-07-26）',
      source: '機体（陽還）に載せた PONS v6β の飛行 CSV（0.5 秒ごと）＋同じ機体の BNO085 の生ログを後から処理した姿勢・方位・TAS・風＋log.txt の電圧・衛星数。',
      warn: [
        '<b>姿勢・方位・TAS・風は飛んだあとに PC で推定した値</b>で（flight_preprocess.py）、当時の機体が画面に出していた値ではありません。',
        'ロール・ピッチは <b>BNO085 使用、加速度エラーあり</b>。加速・旋回の見かけの傾きを GNSS から見積もって引いていますが、消しきれていません。',
        '<b>TAS は対気速度センサーの実測ではありません</b>（動画には電装の対気速度センサー値含む）。対地速度から推定した風を引いたもので、当日の現場の値（平均 TAS 6.1〜7.1 m/s・風 3 m/s 以下）に合うように解いています。',
        '方位・TAS・風は<b>飛んでいる区間だけ</b>です。',
        '高度と V/S は当時の機体のバリオ（気圧の KF）がそのまま記録した値です。'
      ],
      rows: [
        ['位置・G/S・航跡', '機体の PONS v6β の GNSS（0.5 秒ごと）', ''],
        ['高度', '当時の機体の気圧の KF 高度（KF_Altitude）。GNSS 高度も別の列にある', ''],
        ['V/S', '当時の機体のバリオの KF（KF_Vspeed）をそのまま', '機体が画面と音に使った値'],
        ['ロール・ピッチ', 'BNO085 を後から補正（取り付けのずれ・加速と旋回による見かけの傾き）', '<b>BNO085 使用、加速度エラーあり</b>（補正で消しきれていない）。リアルタイムの値ではない'],
        ['方位', 'BNO085 の方位から機体の磁気のずれを推定して除いた真方位', '後処理の推定。飛行区間だけ'],
        ['TAS', '対地速度 − 推定した風', 'センサーの実測ではない（動画には電装の対気速度センサー値含む）。平均 6.1〜7.1 m/s に合わせて解いている'],
        ['Est.Beta', '動画と同じ β（beta_air ＝ 対気ベクトルの向き − 真方位。後処理）', '風の三角形がぴったり閉じる値なので、風の推定や GPS のばらつきも入る。飛行区間だけ'],
        ['風', '後処理の推定（ゆっくり変わるものとして解いた）。向きは「吹いていく向き」に直した', '飛行区間だけ。3 m/s 以下に制約'],
        ['電圧・衛星数', 'log.txt の電圧（60 秒ごとを補間）・衛星数（約 10 秒ごと）', '細かい変化は見えない']
      ]
    }
  };
  const LS_NOTES_QUIET = 'pons_olddata_notes_quiet';   // 「次から自動で出さない」

  function notesKey(filename) {
    const name = String(filename || '').split(/[\\/]/).pop();
    return Object.keys(OLDDATA_NOTES).find(k => name.startsWith(k)) || null;
  }

  function lsGet(k) { try { return localStorage.getItem(k); } catch (e) { return null; } }
  function lsSet(k, v) { try { if (v === null) localStorage.removeItem(k); else localStorage.setItem(k, v); } catch (e) { /* 覚えられないだけ */ } }

  function showDataNotes(key) {
    const n = OLDDATA_NOTES[key];
    if (!n) return;
    document.querySelectorAll('.od-overlay.od-notes').forEach(o => o.remove());
    const rows = n.rows.map(([item, what, caution]) =>
      `<tr><th>${item}</th>${what === null ? '<td class="none">無い</td>' : `<td>${what}</td>`}<td>${caution || ''}</td></tr>`).join('');
    const quiet = lsGet(LS_NOTES_QUIET) === '1';
    const ov = openOverlay(`
      <div class="od-title">⚠ このログの注意 — ${n.title}</div>
      <div class="od-note">${n.source} 3 年とも記録した機器・センサー・後処理が違います。<b>同じ列名でも年によって出どころが違う</b>ので、年をまたいで比べるときは気をつけてください。</div>
      <div class="od-warn"><ul>${n.warn.map(w => `<li>${w}</li>`).join('')}</ul></div>
      <div class="od-tablewrap"><table class="od-table"><colgroup><col class="c1"><col class="c2"><col class="c3"></colgroup>
        <thead><tr><th>項目</th><th>中身・出どころ</th><th>注意</th></tr></thead><tbody>${rows}</tbody></table></div>
      <div class="od-row">
        <label class="od-check"><input type="checkbox" id="od-notes-quiet" ${quiet ? 'checked' : ''}>次から自動で出さない（タイトルの横の「⚠ データの注意」でいつでも見られる）</label>
        <button id="od-notes-ok" class="btn-primary">わかった</button>
      </div>`);
    ov.classList.add('od-notes');
    ov.querySelector('.od-box').classList.add('wide');
    ov.querySelector('#od-notes-quiet').onchange = (e) => lsSet(LS_NOTES_QUIET, e.target.checked ? '1' : null);
    ov.querySelector('#od-notes-ok').onclick = () => ov.remove();
  }

  // CSV を読み終えたら（index.html の parseCSVText の最後）: 大会ログならタイトルの横にボタンを付け、注意を出す
  document.addEventListener('pons-csv-loaded', (ev) => {
    document.querySelectorAll('.od-overlay.od-notes').forEach(o => o.remove());
    const key = notesKey(ev.detail && ev.detail.filename);
    if (!key) return;
    injectStyle();
    const title = document.getElementById('flight-title');
    if (title && !title.querySelector('#od-notes-btn')) {
      const b = document.createElement('button');
      b.id = 'od-notes-btn';
      b.textContent = '⚠ データの注意';
      b.title = 'この大会ログの記録のしかた・センサー・後処理の違い';
      b.onclick = () => showDataNotes(key);
      title.appendChild(b);
    }
    if (lsGet(LS_NOTES_QUIET) !== '1') showDataNotes(key);
  });

  global.showOldDataNotes = (filename) => { const k = notesKey(filename); if (k) showDataNotes(k); return !!k; };
  global.decryptPonsEnc = decryptPonsEnc;
  global.openEncryptedFile = openEncryptedFile;
  global.openEncryptedFiles = openEncryptedFiles;
  global.openEncryptedBytes = openEncryptedBytes;
  global.openOldDataMenu = openOldDataMenu;
})(window);
