#!/usr/bin/env python3
# ============================================================
# File    : tools/repons/gen_live_assets.py
# Project : PONS v7.1 / v7.2 Re:PONS
# Role    : ライブ配信の画面（live_tft.js）が使う本体と同じ文字とアイコンを live_assets.js に書き出す。
#             ・スムーズフォント 3 書体（src/flashdata/font_data.h。VLW 形式のバイト列そのもの）
#             ・TFT_eSPI の内蔵 6x8 フォント（ライブラリの Fonts/glcdfont.c）
#             ・フッターの衛星のアイコン（src/flashdata/link_icons.cpp の ICON_SAT）
# Author  : MasaoC (@masao_mobile)
# Updated : 2026-11-23
# ============================================================
#
# 使い方: python3 tools/repons/gen_live_assets.py [--tft-espi <TFT_eSPI ライブラリのフォルダ>]
# ★ 生成物（live_assets.js）は git に入れる。フォントかアイコンを差し替えたら回し、Re:PONS の VIEWER_VERSION を上げる。
#   --check は今の live_assets.js が元と一致するかだけ見る（tools/check_release.py から呼ぶ）。
import argparse
import base64
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
OUT = HERE / 'live_assets.js'


def c_array(src, name):
    m = re.search(r'\b' + re.escape(name) + r'\s*\[\]\s*(?:PROGMEM)?\s*=\s*\{(.*?)\};', src, re.S)
    if not m:
        raise SystemExit(f'{name} が見つかりません')
    body = re.sub(r'//[^\n]*', '', m.group(1))
    return bytes(int(x, 16) for x in re.findall(r'0x([0-9A-Fa-f]{1,2})', body))


def default_tft_espi():
    for p in (ROOT.parent.parent / 'libraries' / 'TFT_eSPI', Path.home() / 'Documents' / 'Arduino' / 'libraries' / 'TFT_eSPI'):
        if (p / 'Fonts' / 'glcdfont.c').exists():
            return p
    return None


def build(tft_espi):
    fonts = (ROOT / 'src' / 'flashdata' / 'font_data.h').read_text(encoding='utf-8')
    icons = (ROOT / 'src' / 'flashdata' / 'link_icons.cpp').read_text(encoding='utf-8')
    glcd = c_array((Path(tft_espi) / 'Fonts' / 'glcdfont.c').read_text(encoding='utf-8'), 'font')
    if len(glcd) != 256 * 5:
        raise SystemExit(f'glcdfont の大きさが違います（{len(glcd)}）')
    # display_tft.h の AA_FONT_SMALL / NM_FONT_MEDIUM / NM_FONT_LARGE と、内蔵 6x8・衛星のアイコン
    items = {
        'NotoSansBold15': c_array(fonts, 'NotoSansBold15'),
        'Arial_Black22':  c_array(fonts, 'Arial_Black22'),
        'Arial_Black56':  c_array(fonts, 'Arial_Black56'),
        'GLCD':           glcd,
        'ICON_SAT':       c_array(icons, 'ICON_SAT'),
    }
    lines = [
        '// ============================================================',
        '// File    : tools/repons/live_assets.js',
        '// Project : PONS v7.1 / v7.2 Re:PONS',
        '// Role    : ライブ配信の画面が使う本体と同じ文字とアイコン（gen_live_assets.py の生成物。手で編集しない）。',
        '//           スムーズフォントは src/flashdata/font_data.h（VLW）、内蔵 6x8 は TFT_eSPI の glcdfont.c、',
        '//           衛星のアイコンは src/flashdata/link_icons.cpp の ICON_SAT（13x13、行ごとに 2 バイト）',
        '// ============================================================',
        '(function (global) {',
        "  'use strict';",
        '  function b64(s) { const b = atob(s); const u = new Uint8Array(b.length); for (let i = 0; i < b.length; i++) u[i] = b.charCodeAt(i); return u; }',
        '  global.LIVE_ASSETS = {',
    ]
    for k, v in items.items():
        lines.append(f"    {k}: b64('{base64.b64encode(v).decode('ascii')}'),")
    lines += ['  };', '})(typeof window !== \'undefined\' ? window : globalThis);', '']
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description='ライブ配信の画面の文字とアイコンを書き出す')
    ap.add_argument('--tft-espi', default=None, help='TFT_eSPI ライブラリのフォルダ（Fonts/glcdfont.c を読む）')
    ap.add_argument('--check', action='store_true', help='live_assets.js が元と一致するかだけ見る')
    a = ap.parse_args()
    lib = Path(a.tft_espi) if a.tft_espi else default_tft_espi()
    if not lib or not (lib / 'Fonts' / 'glcdfont.c').exists():
        if a.check:
            print('  （TFT_eSPI が見つからないので live_assets.js の照合は飛ばした）')
            return
        sys.exit('TFT_eSPI のフォルダが見つかりません（--tft-espi で指定）')
    text = build(lib)
    if a.check:
        if not OUT.exists() or OUT.read_text(encoding='utf-8') != text:
            sys.exit('live_assets.js が font_data.h / link_icons.cpp / glcdfont.c と一致しません。gen_live_assets.py を回してください')
        print('  live_assets.js: 一致')
        return
    OUT.write_text(text, encoding='utf-8')
    print(f'  {OUT}（{len(text) / 1024:.0f} KB）')


if __name__ == '__main__':
    main()
