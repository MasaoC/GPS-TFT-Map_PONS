#!/usr/bin/env python3
# ============================================================
# File    : tools/vectormap/export_repons_map.py
# Project : PONS v7.1 / v7.2
# Role    : 本体に焼くベクター地図（src/flashdata/vectormap_data*.cpp）を、Re:PONS のライブ配信の画面
#           （tools/repons/live_map.js）が読む形に書き出す（下の形のバイナリを gzip し、base64 にして .js に入れる）。
#           **本体が焼いたものと同じ地図**にするため、build_vectormap.py の出力の .cpp をそのまま読む。
# Author  : MasaoC (@masao_mobile)
# Updated : 2026-11-25
# ============================================================
#
# 使い方（ふだんは build_vectormap.py が最後に呼ぶ）:
#   python3 export_repons_map.py                       # hires（src/flashdata/vectormap_data_hires.cpp）
#   python3 export_repons_map.py --variant light       # 軽量版
#   python3 export_repons_map.py --cpp <path> --out <path>
# 出力の既定: tools/repons/vectormap_<variant>.js（hires は git に入れる。2026-11-23 ユーザー）
#
# ★ .js にして <script> で読むのは、Re:PONS を file://（ファイルをダブルクリック）で開いても地図が出るように。
#   ブラウザは file:// のページから fetch でファイルを読ませない（2026-11-25 までは .bin.gz を fetch していて、
#   file:// では「地図を選ぶ」で手で選ばないと地図が白いままだった）。文字の live_assets.js と同じやり方。
#   base64 は 100 文字ごとに改行する（atob は空白・改行を飛ばす）。中身が同じなら同じバイト列になる。
#
# ★ 地図データは OpenStreetMap 由来で ODbL 1.0（Derivative Database）。帰属表示を中に持たせ、
#   Re:PONS は配信の帯に "© OpenStreetMap contributors" を出す。
#
# ---- 形（すべてリトルエンディアン）----
#   "PVM1"                      4 バイト
#   u16 lod_count, u16 tile_count, u16 cover_count, u16 0
#   i32 lod_unit_e7[L] / i32 lod_tilesize_e7[L] / f32 lod_min_scale[L]
#   u8  lod_sea_background[L]（4 バイト境界まで 0 を詰める）
#   u16 lod_cover_first[L] / u16 lod_cover_n[L]（4 バイト境界まで 0 を詰める）
#   i32 cover_e7[cover_count][4]
#   tile × tile_count: i32 org_lat_e7, i32 org_lon_e7, u32 blob_off, u32 blob_len, u8 lod, u8 0, u16 0
#   u32 blob_size, blob（vm_blob そのもの。レコードの形は src/vectormap.h）
#   u16 len + 帰属表示（UTF-8）、u16 len + 元データの日付
import argparse
import base64
import gzip
import re
import struct
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent


def _array(src, name):
    m = re.search(r'\b' + re.escape(name) + r'\b\s*\[[^\]]*\]\s*=\s*\{([^}]*)\}', src)
    if not m:
        raise SystemExit(f'{name} が見つかりません')
    return [x.strip() for x in m.group(1).split(',') if x.strip()]


def _scalar(src, name):
    m = re.search(r'\b' + re.escape(name) + r'\b\s*=\s*([^;]+);', src)
    if not m:
        raise SystemExit(f'{name} が見つかりません')
    return m.group(1).strip()


def _cstr(src, name):
    m = re.search(r'\b' + re.escape(name) + r'\[\]\s*=\s*"((?:[^"\\]|\\.)*)"', src)
    return m.group(1) if m else ''


def parse_cpp(path):
    src = Path(path).read_text(encoding='utf-8')
    tile_count = int(_scalar(src, 'vm_tile_count'))
    unit = [int(x) for x in _array(src, 'vm_lod_unit_e7')]
    tsize = [int(x) for x in _array(src, 'vm_lod_tilesize_e7')]
    minscale = [float(x.rstrip('f')) for x in _array(src, 'vm_lod_min_scale')]
    seabg = [x == 'true' for x in _array(src, 'vm_lod_sea_background')]
    L = len(unit)
    if not (len(tsize) == len(minscale) == len(seabg) == L):
        raise SystemExit('LOD の表の長さがそろっていません')
    cover_count = int(_scalar(src, 'vm_cover_count'))
    m = re.search(r'vm_cover_e7\[\]\[4\]\s*=\s*\{(.*?)\};', src, re.S)
    covers = [tuple(int(v) for v in row.split(','))
              for row in re.findall(r'\{\s*([-\d\s,]+?)\s*\}', m.group(1))] if m else []
    if len(covers) != cover_count:
        raise SystemExit(f'収録範囲の数が合いません（{len(covers)} / {cover_count}）')
    cfirst = [int(x) for x in _array(src, 'vm_lod_cover_first')]
    cn = [int(x) for x in _array(src, 'vm_lod_cover_n')]
    m = re.search(r'const vm_tile vm_tiles\[\]\s*=\s*\{(.*?)\n\};', src, re.S)
    tiles = [tuple(int(v) for v in row.split(','))
             for row in re.findall(r'\{\s*([-\d\s,]+?)\s*\}', m.group(1))] if m else []
    if len(tiles) != tile_count:
        raise SystemExit(f'タイルの数が合いません（{len(tiles)} / {tile_count}）')
    i = src.index('const uint8_t vm_blob[]')
    j = src.index('{', i)
    k = src.index('};', j)
    blob = bytes(int(x, 16) for x in re.findall(r'0x([0-9A-Fa-f]{2})', src[j + 1:k]))
    blob_size = int(_scalar(src, 'vm_blob_size'))
    if len(blob) != blob_size:
        raise SystemExit(f'vm_blob の大きさが合いません（{len(blob)} / {blob_size}）')
    for t in tiles:
        if t[2] + t[3] > len(blob):
            raise SystemExit('タイルが blob の外を指しています')
    return dict(unit=unit, tsize=tsize, minscale=minscale, seabg=seabg, covers=covers,
                cfirst=cfirst, cn=cn, tiles=tiles, blob=blob,
                attribution=_cstr(src, 'vm_data_attribution'),
                source_date=_cstr(src, 'vm_data_source_date'))


def pack(d):
    L = len(d['unit'])
    out = bytearray(b'PVM1')
    out += struct.pack('<HHHH', L, len(d['tiles']), len(d['covers']), 0)
    out += struct.pack(f'<{L}i', *d['unit'])
    out += struct.pack(f'<{L}i', *d['tsize'])
    out += struct.pack(f'<{L}f', *d['minscale'])
    out += bytes(1 if b else 0 for b in d['seabg'])
    while len(out) % 4: out += b'\0'
    out += struct.pack(f'<{L}H', *d['cfirst'])
    out += struct.pack(f'<{L}H', *d['cn'])
    while len(out) % 4: out += b'\0'
    for c in d['covers']:
        out += struct.pack('<4i', *c)
    for (lat, lon, off, ln, lod, _pad) in d['tiles']:
        out += struct.pack('<iiIIBBH', lat, lon, off, ln, lod, 0, 0)
    out += struct.pack('<I', len(d['blob']))
    out += d['blob']
    for s in (d['attribution'], d['source_date']):
        b = s.encode('utf-8')
        out += struct.pack('<H', len(b)) + b
    return bytes(out)


def to_js(gz, name, attribution, source_date):
    """gzip のバイト列を、<script> で読める .js にする（globalThis.LIVE_VECTORMAP_GZ_B64 に base64 の文字列）"""
    b64 = base64.b64encode(gz).decode('ascii')
    body = '\n'.join(b64[i:i + 100] for i in range(0, len(b64), 100))
    head = ['// ============================================================',
            f'// File    : tools/repons/{name}',
            '// Project : PONS v7.1 / v7.2 Re:PONS',
            '// Role    : ライブ配信の画面のベクター地図（tools/vectormap/export_repons_map.py の生成物。手で編集しない）。',
            '//           本体に焼く地図の .cpp と同じ中身を PVM1 の形にして gzip し、base64 にしたもの。live_map.js が読む。',
            '//           <script> で読むのは file:// で開いても読めるように（fetch は file:// を読めない）。',
            f'//           {attribution}（元データ {source_date}）',
            '// ============================================================']
    return '\n'.join(head) + '\nglobalThis.LIVE_VECTORMAP_GZ_B64 = `\n' + body + '\n`;\n'


def export(cpp, out):
    d = parse_cpp(cpp)
    raw = pack(d)
    # mtime=0: 中身が同じなら同じバイト列になる（git の差分を出さない）
    gz = gzip.compress(raw, compresslevel=9, mtime=0)
    text = to_js(gz, Path(out).name, d['attribution'], d['source_date'])
    Path(out).write_text(text, encoding='utf-8', newline='\n')
    print(f'  Re:PONS 用の地図: {out}（{len(raw) / 1e6:.2f} MB → gzip {len(gz) / 1e6:.2f} MB → .js {len(text) / 1e6:.2f} MB、'
          f'タイル {len(d["tiles"])}・LOD {len(d["unit"])}・元データ {d["source_date"]}）')
    return out


def main():
    ap = argparse.ArgumentParser(description='本体のベクター地図を Re:PONS のライブ配信用に書き出す')
    ap.add_argument('--variant', choices=['light', 'hires'], default='hires')
    ap.add_argument('--cpp', default=None, help='読む .cpp（既定は src/flashdata/vectormap_data[_hires].cpp）')
    ap.add_argument('--out', default=None, help='書き出し先（既定は tools/repons/vectormap_<variant>.js）')
    a = ap.parse_args()
    cpp = a.cpp or str(ROOT / 'src' / 'flashdata' /
                       ('vectormap_data_hires.cpp' if a.variant == 'hires' else 'vectormap_data.cpp'))
    out = a.out or str(ROOT / 'tools' / 'repons' / f'vectormap_{a.variant}.js')
    if not Path(cpp).exists():
        sys.exit(f'{cpp} がありません（hires は build_vectormap.py --variant hires で作る）')
    export(cpp, out)


if __name__ == '__main__':
    main()
