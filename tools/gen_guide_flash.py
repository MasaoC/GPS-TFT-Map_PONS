#!/usr/bin/env python3
# ============================================================
# File    : gen_guide_flash.py
# Project : PONS v7 (Pilot Oriented Navigation System for HPA)
# Role    : 案内画像（日本語の全画面画像）を、本体 FLASH に焼き込む src/flashdata/*_data.cpp にする。
#             imu_guide … IMU/ESKF 画面の入口の案内（tools/imu_guide/imu_guide_*.svg、3 枚）
#             tas_guide … 推定 TAS の案内（tools/tas_guide/tas_guide_*.svg、1 枚。docs/pons_tas.md §4.4）
#
#           案内は日本語。本体に日本語フォントは無いので、文字も図も画像にして焼く。
#           原本は SVG（文字を <text> で持つので、文言の差分が見える）。
#           SVG → PNG は Chrome で等倍（1px = 1px）に描く。縮小すると漢字が潰れるため。
#           PNG もリポジトリに入れる。Chrome とフォント（ヒラギノ）が無い PC でも、
#           PNG からの生成と --check はできるようにするため。
#
# 使い方:
#   python3 tools/gen_guide_flash.py --render  # SVG → PNG（Chrome）→ *_data.cpp
#   python3 tools/gen_guide_flash.py           # PNG → *_data.cpp だけ
#   python3 tools/gen_guide_flash.py --check   # 生成せず、*_data.cpp が PNG と
#                                              # 一致しているか検査する（違えば終了コード 1）
#   --only imu_guide / --only tas_guide で片方だけ。
#
# 画像を差し替える手順:
#   1. tools/imu_guide/imu_guide_N.svg（または tools/tas_guide/tas_guide_N.svg）を置き換える
#      （別の AI に作らせるときの指示文は tools/imu_guide/prompt.md）
#   2. python3 tools/gen_guide_flash.py --render
#   3. PNG を開いて、文字が崩れていないか目で見る
#   ★ 2 を忘れると**古い画像のまま黙って焼ける**。リリース前は --check。
#
# 色: RGB565 への丸めは tools/png2bmp.py と同じ切り捨て（R>>3, G>>2, B>>3、透明は白）。
#     丸め方を 2 通りにすると、どちらで作ったかで色が変わるため。
#
# 大きさと枚数の正本は src/flashdata/<名前>_data.h（<マクロ>_W / _H / _PAGES）。
#
# ★ 2026-11-07 までは tools/gen_imu_guide_flash.py（IMU の案内だけ）。TAS の案内を足すときに共用にした。
#
# Author  : MasaoC (@masao_mobile)
# Updated : 2026/11/07
# ============================================================
import argparse
import datetime
import os
import re
import subprocess
import sys
import tempfile

from PIL import Image

# リポジトリのルート（このスクリプトは tools/ にある）
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# 案内の組。name は原本のフォルダ名・ファイル名の頭・生成物の名前、macro は .h の定義の頭。
SETS = [
    {'name': 'imu_guide', 'macro': 'IMU_GUIDE', 'what': 'IMU/ESKF 画面の入口の案内画像'},
    {'name': 'tas_guide', 'macro': 'TAS_GUIDE', 'what': '推定 TAS の案内画像'},
]

CHROME_MAC = '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome'

PER_LINE = 16    # 1 行に並べる画素数


def die(msg):
    print('ERROR: ' + msg, file=sys.stderr)
    sys.exit(1)


def rel(path):
    return os.path.relpath(path, ROOT)


def src_dir(st):
    return os.path.join(ROOT, 'tools', st['name'])


def out_cpp(st):
    return os.path.join(ROOT, 'src', 'flashdata', st['name'] + '_data.cpp')


def h_file(st):
    return os.path.join(ROOT, 'src', 'flashdata', st['name'] + '_data.h')


def guide_size(st):
    """<名前>_data.h の <マクロ>_W / _H / _PAGES を読む。"""
    with open(h_file(st), encoding='utf-8') as f:
        s = f.read()
    vals = {}
    for k in ('W', 'H', 'PAGES'):
        m = re.search(r'#define\s+%s_%s\s+(\d+)' % (st['macro'], k), s)
        if not m:
            die('%s に %s_%s が見つからない' % (rel(h_file(st)), st['macro'], k))
        vals[k] = int(m.group(1))
    return vals['W'], vals['H'], vals['PAGES']


def page_paths(st, pages, ext):
    return [os.path.join(src_dir(st), '%s_%d.%s' % (st['name'], i + 1, ext)) for i in range(pages)]


def render_svgs(st, w, h, pages, chrome):
    """SVG を Chrome で等倍の PNG にする。"""
    if not os.path.exists(chrome):
        die('Chrome が見つからない（%s）。--chrome で場所を指定するか、'
            'PNG を用意して --render なしで実行する' % chrome)
    for svg, png in zip(page_paths(st, pages, 'svg'), page_paths(st, pages, 'png')):
        if not os.path.exists(svg):
            die('%s が無い' % rel(svg))
        with tempfile.TemporaryDirectory() as prof:
            # ★ --force-device-scale-factor=1: Retina の Mac でも 2 倍で描かせない。
            #   --user-data-dir: 普段使いの Chrome が開いていても別プロセスで動かすため。
            cmd = [chrome, '--headless=new', '--disable-gpu', '--hide-scrollbars',
                   '--force-device-scale-factor=1', '--window-size=%d,%d' % (w, h),
                   '--default-background-color=FFFFFFFF', '--user-data-dir=' + prof,
                   '--screenshot=' + png, 'file://' + svg]
            r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if r.returncode != 0 or not os.path.exists(png):
            die('%s を描けなかった（Chrome の終了コード %d）\n%s' % (rel(svg), r.returncode, r.stderr))
        print('%s → %s' % (rel(svg), rel(png)))


def read_png(path, w, h):
    """PNG を読み、上の行から順に並べた RGB565 値のリストを返す。"""
    if not os.path.exists(path):
        die('%s が無い（--render で SVG から作る）' % rel(path))
    im = Image.open(path).convert('RGBA')
    if im.size != (w, h):
        die('%s は %dx%d。案内画像は %dx%d でなければならない'
            '（変えるなら _data.h も直す）' % (rel(path), im.size[0], im.size[1], w, h))
    d = im.tobytes()   # RGBA を 1 画素 4 バイトで上の行から
    out = []
    for i in range(0, len(d), 4):
        r, g, b, a = d[i], d[i + 1], d[i + 2], d[i + 3]
        out.append(0xFFFF if a == 0 else ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3))
    return out


def render_cpp(st, pages_px, names):
    n, m = st['name'], st['macro']
    lines = [
        '// ============================================================',
        '// File    : %s_data.cpp' % n,
        '// Project : PONS v7',
        '// Role    : %sの実体。自動生成ファイル。手で編集しないこと。' % st['what'],
        '//           生成: tools/gen_guide_flash.py（原本は tools/%s/%s_*.svg / .png）。' % (n, n),
        '//           上の行から順に並べた素の RGB565 値（pushImage には setSwapBytes(true) で渡す）。',
        '// Author  : MasaoC (@masao_mobile)',
        '// Updated : %s' % datetime.date.today().strftime('%Y/%m/%d'),
        '// ============================================================',
        '#include "%s_data.h"' % n,
        '',
        'const uint16_t %s_DATA[%s_PAGES][%s_W * %s_H] PROGMEM = {' % (m, m, m, m),
    ]
    for px, name in zip(pages_px, names):
        lines.append('  { // %s' % name)
        for i in range(0, len(px), PER_LINE):
            lines.append('    ' + ', '.join('0x%04X' % v for v in px[i:i + PER_LINE]) + ',')
        lines.append('  },')
    lines.append('};')
    return '\n'.join(lines) + '\n'


def read_generated(st, path, pages):
    """既存の *_data.cpp から 1 枚ずつ画素値を拾う（--check 用）。

    ヘッダの日付は生成のたびに変わるので、文字列ではなく値で比べる。
    """
    with open(path, encoding='utf-8') as f:
        s = f.read()
    body = s.split('PROGMEM', 1)
    if len(body) != 2:
        die('%s に %s_DATA の定義が見つからない' % (rel(path), st['macro']))
    blocks = re.split(r'\n  \{ // [^\n]*\n', body[1])[1:]
    if len(blocks) != pages:
        die('%s の画像が %d 枚（%d 枚のはず）' % (rel(path), len(blocks), pages))
    return [[int(x, 16) for x in re.findall(r'0x([0-9A-Fa-f]{4})\b', b)] for b in blocks]


def run_set(st, args):
    w, h, pages = guide_size(st)
    if args.render:
        render_svgs(st, w, h, pages, args.chrome)

    pngs = page_paths(st, pages, 'png')
    pages_px = [read_png(p, w, h) for p in pngs]
    names = [os.path.basename(p) for p in pngs]
    cpp = out_cpp(st)

    if args.check:
        if not os.path.exists(cpp):
            die('%s が無い' % rel(cpp))
        cur = read_generated(st, cpp, pages)
        bad = []
        for name, a, b in zip(names, cur, pages_px):
            if a != b:
                bad.append('%s（%d 画素）' % (name, sum(1 for x, y in zip(a, b) if x != y) + abs(len(a) - len(b))))
        if bad:
            die('%s が PNG と違う: %s。python3 tools/gen_guide_flash.py で作り直すこと'
                % (rel(cpp), ', '.join(bad)))
        print('OK: %s は tools/%s/*.png と一致している（%dx%d × %d 枚）'
              % (rel(cpp), st['name'], w, h, pages))
        return

    with open(cpp, 'w', encoding='utf-8', newline='\n') as f:
        f.write(render_cpp(st, pages_px, names))
    print('%s を書いた（%dx%d × %d 枚 / %d B）' % (rel(cpp), w, h, pages, w * h * 2 * pages))


def main():
    ap = argparse.ArgumentParser(
        description='tools/<案内>/ の案内画像から src/flashdata/<案内>_data.cpp を生成する')
    ap.add_argument('--render', action='store_true',
                    help='先に SVG を Chrome で PNG にする')
    ap.add_argument('--check', action='store_true',
                    help='生成せず、既存の *_data.cpp が PNG と一致するか検査する')
    ap.add_argument('--only', choices=[s['name'] for s in SETS],
                    help='この案内だけ扱う（省略すると全部）')
    ap.add_argument('--chrome', default=CHROME_MAC, help='Chrome の実行ファイル（--render 用）')
    args = ap.parse_args()
    if args.render and args.check:
        die('--render と --check は同時に使えない')
    for st in SETS:
        if args.only and st['name'] != args.only:
            continue
        run_set(st, args)


if __name__ == '__main__':
    main()
