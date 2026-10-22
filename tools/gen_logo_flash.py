#!/usr/bin/env python3
# ============================================================
# File    : gen_logo_flash.py
# Project : PONS v7 (Pilot Oriented Navigation System for HPA)
# Role    : tools/logo.bmp から、起動スプラッシュのロゴを本体 FLASH に
#           焼き込むための src/flashdata/logo_data.cpp を生成する。
#
#           ロゴは 0.960 で SD から FLASH へ移した（SD が読めない機体でも
#           ロゴが出るように、起動時の 1.9 秒待ちと 24KB の RAM を無くすため）。
#           SD の logo.bmp はもう読まないので、tools/logo.bmp は
#           「焼き込む原本」であって配布するカードには入れない。
#
# 使い方:
#   python3 tools/gen_logo_flash.py           # 生成（src/flashdata/logo_data.cpp を書く）
#   python3 tools/gen_logo_flash.py --check   # 生成せず、既存の生成物が
#                                             # tools/logo.bmp と一致しているか検査する
#                                             # （一致しなければ終了コード 1）
#
# ロゴを差し替える手順:
#   1. 240x52 の PNG を用意する（透明な画素は白になる）
#   2. python3 tools/png2bmp.py で tools/logo.bmp として保存する
#   3. python3 tools/gen_logo_flash.py
#   ★ 3 を忘れると**古いロゴのまま黙って焼ける**。
#
# BMP の仕様（これ以外は受け付けずエラーで止める）:
#   16bpp / BI_BITFIELDS / マスク R=F800 G=07E0 B=001F（＝ png2bmp.py の出力）。
#   大きさは src/flashdata/logo_data.h の LOGO_W x LOGO_H（240x52）。
#   24bpp などで来たら、ここで変換せず png2bmp.py で作り直すこと
#   （色の丸め方が 2 箇所に分かれると、どちらで作ったかで色が変わる）。
#
# Author  : MasaoC (@masao_mobile)
# Updated : 2026/10/22
# ============================================================
import argparse
import datetime
import os
import re
import struct
import sys

# リポジトリのルート（このスクリプトは tools/ にある）
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

SRC_BMP = os.path.join(ROOT, 'tools', 'logo.bmp')
OUT_CPP = os.path.join(ROOT, 'src', 'flashdata', 'logo_data.cpp')
LOGO_H_FILE = os.path.join(ROOT, 'src', 'flashdata', 'logo_data.h')

BI_BITFIELDS = 3
REQ_MASKS = (0xF800, 0x07E0, 0x001F)   # RGB565

PER_LINE = 12    # 1 行に並べる画素数（既存の logo_data.cpp と同じ）


def die(msg):
    print('ERROR: ' + msg, file=sys.stderr)
    sys.exit(1)


def logo_size():
    """logo_data.h の LOGO_W / LOGO_H を読む。大きさの正本はあちら。"""
    with open(LOGO_H_FILE, encoding='utf-8') as f:
        s = f.read()
    mw = re.search(r'#define\s+LOGO_W\s+(\d+)', s)
    mh = re.search(r'#define\s+LOGO_H\s+(\d+)', s)
    if not (mw and mh):
        die('%s に LOGO_W / LOGO_H が見つからない' % os.path.relpath(LOGO_H_FILE, ROOT))
    return int(mw.group(1)), int(mh.group(1))


def read_bmp(path, want_w, want_h):
    """BMP を読み、上の行から順に並べた RGB565 値のリストを返す。

    ★ BMP は通常**下の行から**格納されている（高さが正の値のとき）。
      pushImage() は上から順に描くので、ここで並べ替えておく。
      並べ替えを忘れるとロゴが上下逆さまに出る。
    """
    rel = os.path.relpath(path, ROOT)
    if not os.path.exists(path):
        die('%s が無い' % rel)
    with open(path, 'rb') as f:
        d = f.read()
    if len(d) < 66 or d[:2] != b'BM':
        die('%s は BMP ではない' % rel)

    off = struct.unpack_from('<I', d, 10)[0]
    (hdr_size, w, h, planes, bpp, comp) = struct.unpack_from('<IiiHHI', d, 14)
    if hdr_size < 40 or planes != 1:
        die('%s のヘッダが想定外（size=%d planes=%d）' % (rel, hdr_size, planes))
    if bpp != 16 or comp != BI_BITFIELDS:
        die('%s は %dbpp / 圧縮 %d。16bpp の BI_BITFIELDS（RGB565）だけを受け付ける。'
            'tools/png2bmp.py で作り直すこと' % (rel, bpp, comp))
    # マスクは 40 バイトのヘッダの直後。V4/V5 ヘッダでも同じ位置にある。
    masks = struct.unpack_from('<III', d, 14 + 40)
    if masks != REQ_MASKS:
        die('%s のマスクが RGB565 ではない（%s）。tools/png2bmp.py で作り直すこと'
            % (rel, ' '.join('%04X' % m for m in masks)))

    # ★ 大きさは logo_data.h の定数だけでは決まらない。display_tft.cpp の
    #   起動画面が backscreen.pushSprite(0, 52) で「ロゴの下」に地図を戻して
    #   いるので、高さを変えるならそこも直す必要がある。
    if w != want_w or abs(h) != want_h:
        die('%s は %dx%d。ロゴは %dx%d でなければならない'
            '（変えるなら logo_data.h と display_tft.cpp の pushSprite(0, 52) も直す）'
            % (rel, w, abs(h), want_w, want_h))

    stride = (w * 2 + 3) & ~3        # 1 行は 4 バイト境界まで詰め物される
    if off + stride * abs(h) > len(d):
        die('%s の画素データが途中で切れている' % rel)
    rows = [struct.unpack_from('<%dH' % w, d, off + y * stride) for y in range(abs(h))]
    if h > 0:                        # 正の高さ = ボトムアップ
        rows.reverse()
    return [v for r in rows for v in r]


def render_cpp(pixels):
    lines = [
        '// ============================================================',
        '// File    : logo_data.cpp',
        '// Project : PONS v7',
        '// Role    : 起動スプラッシュのロゴ画像の実体。自動生成ファイル。手で編集しないこと。',
        '//           生成: tools/gen_logo_flash.py（原本は tools/logo.bmp。240x52 / 16bpp RGB565）。',
        '//           BMP はボトムアップだが、ここでは**上から順**に並べ替えてあるので',
        '//           pushImage にそのまま渡せる。',
        '// Author  : MasaoC (@masao_mobile)',
        '// Updated : %s' % datetime.date.today().strftime('%Y/%m/%d'),
        '// ============================================================',
        '#include "logo_data.h"',
        '',
        'const uint16_t LOGO_DATA[LOGO_W * LOGO_H] PROGMEM = {',
    ]
    for i in range(0, len(pixels), PER_LINE):
        lines.append('  ' + ', '.join('0x%04X' % v for v in pixels[i:i + PER_LINE]) + ',')
    lines.append('};')
    return '\n'.join(lines) + '\n'


def read_generated(path):
    """既存の logo_data.cpp から画素値を拾う（--check 用）。

    ヘッダの日付は生成のたびに変わるので、文字列ではなく値で比べる。
    """
    with open(path, encoding='utf-8') as f:
        s = f.read()
    body = s.split('PROGMEM', 1)
    if len(body) != 2:
        die('%s に LOGO_DATA の定義が見つからない' % os.path.relpath(path, ROOT))
    return [int(x, 16) for x in re.findall(r'0x([0-9A-Fa-f]{4})\b', body[1])]


def main():
    ap = argparse.ArgumentParser(
        description='tools/logo.bmp から src/flashdata/logo_data.cpp を生成する')
    ap.add_argument('--check', action='store_true',
                    help='生成せず、既存の logo_data.cpp が tools/logo.bmp と一致するか検査する')
    args = ap.parse_args()

    w, h = logo_size()
    pixels = read_bmp(SRC_BMP, w, h)

    if args.check:
        if not os.path.exists(OUT_CPP):
            die('%s が無い' % os.path.relpath(OUT_CPP, ROOT))
        cur = read_generated(OUT_CPP)
        if cur != pixels:
            diff = sum(1 for a, b in zip(cur, pixels) if a != b) + abs(len(cur) - len(pixels))
            die('logo_data.cpp が tools/logo.bmp と違う（%d 画素）。'
                'python3 tools/gen_logo_flash.py で作り直すこと' % diff)
        print('OK: logo_data.cpp は tools/logo.bmp と一致している（%dx%d）' % (w, h))
        return

    with open(OUT_CPP, 'w', encoding='utf-8', newline='\n') as f:
        f.write(render_cpp(pixels))
    print('%s を書いた（%dx%d / %d B）'
          % (os.path.relpath(OUT_CPP, ROOT), w, h, len(pixels) * 2))


if __name__ == '__main__':
    main()
