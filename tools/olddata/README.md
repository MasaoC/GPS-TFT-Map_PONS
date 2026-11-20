# 過去の大会ログ（暗号化）

2024〜2026 年の鳥人間コンテストの飛行ログを、いまの飛行 CSV と同じ列の並びにしたもの。
**公開の許可を得ていないので、暗号化した `.enc` だけを置いている。** パスワードはチームの担当者に聞くこと
（リポジトリ・コミットメッセージ・Issue には書かない）。

| ファイル | 中身 |
|---|---|
| `2024-07-27_0936_fused.csv.enc` | 2024 年。PONS の位置・G/S・航跡だけ（元の G/S はノットだったので m/s に直した） |
| `2025-07-26_1539_fused.csv.enc` | 2025 年。PONS の位置＋姿勢（ロール・ピッチ）・Garmin の高度・対気速度センサー |
| `2026-07-26_0804_fused.csv.enc` | 2026 年。PONS の位置・高度＋補正済みの姿勢・推定した真方位・TAS・風・電圧・衛星数 |

## 開き方

**Flight Viewer から直接**: 公開ページ（https://masaoc.github.io/GPS-TFT-Map_PONS/tools/flight_viewer/）の
「🔒 大会ログ」から選んでパスワードを入れる。手元で開いたビューアなら「📁 CSV / log.txt を開く」かドロップで `.enc` を渡す。
復号はブラウザの中だけで行い、パスワードはそのページを開いている間しか覚えない。

**ファイルに戻す**:

```sh
pip3 install cryptography                     # 初回だけ
python3 tools/olddata_crypt.py decrypt        # パスワードを聞かれる。同じフォルダに .csv ができる（git には入らない）
```

できた `.csv` は Flight Viewer（`tools/flight_viewer/index.html`）でそのまま開ける。

## 大会動画（YouTube 限定公開）

ログに対応する動画の URL も公開できないので、同じパスワードで `videos.json.enc` に暗号化して置く。
Flight Viewer でログを開くと一緒に復号し、動画を小窓に出してビューアの再生（再生・一時停止・シーク・速度）に合わせて動かす。
ビューアが止まっているときに動画を動かすと、ビューアがそちらへ合わせる。

平文の `videos.json` はリポジトリの外で作る（`tools/olddata/` に置いても `tools/.gitignore` で入らない）:

```json
{
  "2026-07-26_0804_fused.csv": { "url": "https://youtu.be/xxxxxxxxxxx", "sync": { "video_sec": 0, "jst": "08:20:40.0" } },
  "2025-07-26_1539_fused.csv": { "url": "https://youtu.be/xxxxxxxxxxx" },
  "2024-07-27_0936_fused.csv": { "url": "https://youtu.be/xxxxxxxxxxx" }
}
```

- `sync` は「動画の `video_sec` 秒目が JST の `jst`」。分からなければ書かずに、ビューアで動画とログを同じ場面にして
  「⇆ ここで同期」を押し、±0.5s / ±5s で詰める。合わせた値はそのブラウザに覚え（`videos.json` の `sync` より優先。「↺」で消して戻す）、小窓の下に `videos.json` に書き写す形で出る。
- 暗号化: `python3 tools/olddata_crypt.py encrypt <どこか>/videos.json`（ログと同じパスワードにする。一覧の manifest.json も作り直す）
- 埋め込みは YouTube Studio で「埋め込みを許可」になっている必要がある。手元の HTML（file://）として開いたビューアでは
  YouTube に断られることがあるので、公開ページか http で開く（だめなときは小窓の「YouTube」で別のタブに開ける）。

## 作り方・注意

- 元データからの変換: 2026 年は `tools/fuse_attitude_csv.py`、2024・2025 年は `tools/convert_old_contest_csv.py`
  （元データは `~/Documents/Arduino/GPS_TFT_map/tools/olddata/` にあり、リポジトリには入れていない）。
  列の決め方（2025 年のピッチの符号反転・ロールの取り付けのずれ、2026 年の風向を「吹いていく向き」に直したこと など）は各スクリプトの頭に書いてある。
- 暗号化: `python3 tools/olddata_crypt.py encrypt <csv>...`（PBKDF2-HMAC-SHA256 100 万回 + AES-256-GCM。どちらもブラウザの WebCrypto にあるので Flight Viewer が開ける。パスワードは 12 文字以上が必須）。暗号化すると一覧の `manifest.json` も作り直す。
  **このリポジトリは公開なので、守りはパスワードの長さだけ。** 一度 push したものは履歴から消せないので、
  弱いパスワードで暗号化したものを push しないこと。
