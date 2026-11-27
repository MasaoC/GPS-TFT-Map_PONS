#!/usr/bin/env python3
# ============================================================
# File    : tools/repons/test/run_live_tests.py
# Project : PONS v7.1 / v7.2 Re:PONS
# Role    : ライブ配信（Re:PONS の live*.js）の試験。tools/check_release.py から呼ぶ。
#             1) LinkTelem の並び: 本体の lora_link/link_proto.h で詰めたバイト列（linktelem_vectors.cpp）を
#                live.js の decodeLT() で読み、同じ値になるか（jsc。設計書 §5.2）
#             2) live_screen.js の定数（K）が settings.h と同じか
#             3) live.js の版（LINK_PROTO_VER・大きさ）が link_proto.h と同じか
#             4) C の printf と同じ丸め（sprintf）・色の混ぜ方（alphaBlend）
#             5) live_assets.js（文字とアイコン）が元と一致するか（gen_live_assets.py --check）
#             6) 写しの見張り: Re:PONS に写した本体の関数（地図画面の描き方・地図・LN に載せる判断・行の組み立て）の中身の
#                指紋が live_fingerprints.txt と同じか。本体のその関数を変えると止まり、どれが変わったかを出す
# Author  : MasaoC (@masao_mobile)
# Updated : 2026-11-24
# ============================================================
# 使い方: python3 tools/repons/test/run_live_tests.py [--update]
#   --update: 6) の指紋を書き直す。**Re:PONS の写し（live_screen.js など）を本体と同じに直してから**使う。
# ★ jsc（macOS の JavaScriptCore）が無い PC では 1) と 4) を飛ばす（飛ばしたと書く）。
import hashlib
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPONS = HERE.parent
ROOT = REPONS.parent.parent
JSC = '/System/Library/Frameworks/JavaScriptCore.framework/Versions/Current/Helpers/jsc'


def defines(path):
    out = {}
    for line in Path(path).read_text(encoding='utf-8').splitlines():
        m = re.match(r'\s*#define\s+([A-Z0-9_]+)\s+(.+?)\s*(//.*)?$', line)
        if m:
            out.setdefault(m.group(1), m.group(2).strip())
    return out


def num(v):
    v = v.strip().rstrip('fFuUlL').strip('()')
    try:
        return float(v)
    except ValueError:
        return None


def check_consts():
    ok = True
    js = (REPONS / 'live_screen.js').read_text(encoding='utf-8')
    m = re.search(r'const K = \{(.*?)\};', js, re.S)
    pairs = dict(re.findall(r'([A-Z0-9_]+):\s*([-\d.]+|\'[^\']*\')', m.group(1)))
    d = {}
    for f in ('settings.h', 'display_tft.h', 'gnss.h', 'src/navdata.h'):
        for k, v in defines(ROOT / f).items():
            d.setdefault(k, v)
    for k, v in pairs.items():
        if k not in d:
            print(f'  [NG] live_screen.js の {k} が settings.h などに見つからない')
            ok = False
            continue
        if v.startswith("'"):
            want = d[k].strip().strip('"')
            if v.strip("'") != want:
                print(f'  [NG] {k}: live_screen.js "{v}" / 本体 "{want}"')
                ok = False
        else:
            a, b = float(v), num(d[k])
            if b is None or abs(a - b) > 1e-9:
                print(f'  [NG] {k}: live_screen.js {a} / 本体 {d[k]}')
                ok = False
    # 地図の縮尺（.ino の scalelist の値は navdata.h の SCALE_*）
    sl = re.search(r'const SCALELIST = \[(.*?)\]', js).group(1).split(',')
    want = [num(d['SCALE_EXSMALL_GMAP']), num(d['SCALE_SMALL_GMAP']), num(d['SCALE_MEDIUM_GMAP']),
            num(d['SCALE_LARGE_GMAP']), num(d['SCALE_EXLARGE_GMAP']), 200.0]
    if [float(x) for x in sl] != want:
        print(f'  [NG] SCALELIST が .ino の scalelist と違う: {sl} / {want}')
        ok = False
    # 版と大きさ（live.js と link_proto.h・live.h）
    lj = (REPONS / 'live.js').read_text(encoding='utf-8')
    proto = defines(ROOT / 'lora_link' / 'link_proto.h')['LINK_PROTO_VER']
    size = re.search(r'sizeof\(LinkTelem\)\s*==\s*(\d+)', (ROOT / 'lora_link' / 'link_proto.h').read_text(encoding='utf-8')).group(1)
    fmt = defines(ROOT / 'live.h')['LIVE_FMT_VER']
    m2 = re.search(r'const LIVE_FMT_VER = (\d+), LINK_PROTO_VER = (\d+), LINKTELEM_SIZE = (\d+);', lj)
    if not m2 or (m2.group(1), m2.group(2), m2.group(3)) != (fmt, proto, size):
        print(f'  [NG] live.js の版・大きさ {m2.groups() if m2 else None} が本体（形 {fmt}・LINK_PROTO_VER {proto}・{size} バイト）と違う')
        ok = False
    editor = (REPONS / 'settings_editor.js').read_text(encoding='utf-8')
    field = re.search(r"key:\s*'airspeed_weight_kg'[^}]+", editor)
    fields = dict(re.findall(r"\b(min|max|defaultVal):\s*'?([\d.]+)", field.group(0))) if field else {}
    for prop, define in (('defaultVal', 'AIRSPEED_DESIGN_WEIGHT_KG'),
                         ('min', 'AIRSPEED_WEIGHT_MIN_KG'), ('max', 'AIRSPEED_WEIGHT_MAX_KG')):
        if prop not in fields or float(fields[prop]) != num(d[define]):
            print(f'  [NG] Re:PONS の総重量 {prop}: {fields.get(prop)} / 本体 {d[define]}')
            ok = False
    header_default = re.search(r"key:\s*'header_tas'[^}]*?defaultVal:\s*'([01])'", editor)
    sd_settings = (ROOT / 'sd' / 'settings.txt').read_text(encoding='utf-8')
    sd_header = re.search(r"^header_tas:\s*([01])\s*$", sd_settings, re.M)
    for name, match in (('Re:PONS', header_default), ('SD の雛形', sd_header)):
        if not match or match.group(1) != d['TAS_HEADER_DEFAULT']:
            print(f"  [NG] {name} の header_tas の既定が本体（{d['TAS_HEADER_DEFAULT']}）と違う")
            ok = False
    print('  [OK] 定数・版' if ok else '  [NG] 定数・版')
    return ok


JS_TEST = r'''
var fail = 0;
function eq(name, a, b) { if (a !== b) { print('  [NG] ' + name + ': ' + a + ' / 期待 ' + b); fail++; } }
// 1) LinkTelem
eq('LINK_PROTO_VER', LT_VECTORS.proto, 8 * 0 + LT_VECTORS.proto);
for (var i = 0; i < LT_VECTORS.list.length; i++) {
  var vec = LT_VECTORS.list[i];
  var t = PonsLive.decodeLT(PonsLive.hexToBytes(vec.hex));
  eq('hex の長さ', vec.hex.length, LT_VECTORS.size * 2);
  for (var k in vec.v) eq('vector ' + i + ' の ' + k, t[k], vec.v[k]);
}
// 4) printf の丸め（newlib と同じく、ちょうど半分は偶数へ）
var sp = LiveTFT.sprintf;
var cases = [
  ['%.0f', [2.5], '2'], ['%.0f', [3.5], '4'], ['%.1f', [0.25], '0.2'], ['%.1f', [0.35], '0.3'], ['%.1f', [0.75], '0.8'],
  ['%+5.1f', [3.14], ' +3.1'], ['%+5.1f', [-12.34], '-12.3'], ['%03d', [7], '007'], ['%4.1f', [7.2], ' 7.2'],
  ['%4.0f', [123.5], ' 124'], ['%+.1f', [-0.05], '-0.1'], ['%+.2f', [0.125], '+0.12'], ['%.2fkm', [12.345], '12.35km'],
  ['%02lu:%02lu:%02luJST', [8, 5, 9], '08:05:09JST'], ['%d%%', [76], '76%'], ['TC%3d', [7], 'TC  7'], ['%ld', [3], '3'],
  ['MaxGS %.1f(%02d:%02d)', [7.25, 8, 30], 'MaxGS 7.2(08:30)'], ['%.1f', [Math.fround(0.4)], '0.4'], ['Acc:%.0fm', [12.5], 'Acc:12m'],
];
for (var j = 0; j < cases.length; j++) eq('sprintf ' + cases[j][0] + ' ' + cases[j][1], sp.apply(null, [cases[j][0]].concat(cases[j][1])), cases[j][2]);
// 色の混ぜ方（TFT_eSPI::alphaBlend。uint32 で折り返す式。期待値は C の式を手で計算したもの。
// 赤・青は alpha を 64 段で混ぜるので、255 でも赤 31 → 30 になる（0xF000）。TFT_eSPI のまま）
eq('alphaBlend 255', LiveTFT.alphaBlend(255, 0xF800, 0x001F), 0xF000);
eq('alphaBlend 0', LiveTFT.alphaBlend(0, 0xF800, 0x001F), 0x001F);
eq('alphaBlend 128 白黒', LiveTFT.alphaBlend(128, 0xFFFF, 0x0000), 0x7BEF);
eq('alphaBlend 128 黒白', LiveTFT.alphaBlend(128, 0x0000, 0xFFFF), 0x7BEF);
print(fail ? ('  [NG] live.js / live_tft.js ' + fail + ' 件') : '  [OK] LinkTelem の並び・printf の丸め・色の混ぜ方');
if (fail) throw new Error('fail');
'''


def run_js():
    if not os.path.exists(JSC):
        print('  （jsc が無いので LinkTelem・printf の照合は飛ばした）')
        return True
    cxx = shutil.which('c++') or shutil.which('g++') or shutil.which('clang++')
    if not cxx:
        print('  [NG] C++ のコンパイラが無い')
        return False
    with tempfile.TemporaryDirectory() as td:
        exe = os.path.join(td, 'vec')
        r = subprocess.run([cxx, '-std=c++17', '-O1', '-o', exe, str(HERE / 'linktelem_vectors.cpp')],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        if r.returncode != 0:
            print(r.stdout)
            print('  [NG] linktelem_vectors.cpp を作れない')
            return False
        vec = os.path.join(td, 'vectors.js')
        Path(vec).write_text(subprocess.run([exe], stdout=subprocess.PIPE, text=True, check=True).stdout, encoding='utf-8')
        test = os.path.join(td, 'test.js')
        Path(test).write_text(JS_TEST, encoding='utf-8')
        r = subprocess.run([JSC, str(REPONS / 'live_tft.js'), str(REPONS / 'live.js'), vec, test],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        print(r.stdout.rstrip())
        return r.returncode == 0 and '[NG]' not in r.stdout


# ---- 6) 写しの見張り（指紋）----
# Re:PONS の live*.js は本体の描き方などを手で写したもの（判断は LN 行で受けるが、描き方そのものは写し）。本体だけ直して
# 写しを忘れると、配信の画面が**黙って**本体と違ってくる。そこで写した元の中身の指紋（コメントと空白を除いた SHA-1）を
# live_fingerprints.txt に持ち、変わったら止める。止まったら、出てきた関数の変更を写し先に同じに入れてから --update。
# ★ 見張る数を増やすのは安いが、写していない関数を入れると関係ない変更でも止まる（「止まっても無視」の癖がつく）。
#   写した関数だけを入れること。判断の関数（ground_warn_shown など）は LN の値でそのまま届くので入れない。
# 書き方: 関数名 / 'd:名前'＝宣言（struct・enum・const。; まで）/ '#define:A|B'＝名前が A か B で始まる #define を 1 つずつ /
#        '*'＝ファイルまるごと / '関数@目印'＝その関数の中の、目印のあとの { } だけ
FP_FILE = HERE / 'live_fingerprints.txt'
FP_TARGETS = [
    ('display_tft.cpp', 'live_screen.js / live_map.js', [
        'latLonToXY', 'get_self_cy', 'get_needle_len', 'draw_clipped_line', 'draw_clipped_wideline', 'draw_map_line',
        'try_draw_km_distance', 'draw_compass', 'draw_km_distances', 'notice_reserve', 'notice_free',
        'draw_course_warning', 'draw_course_warning_box', 'draw_nofix_cross', 'draw_hacc_circle', 'strike_through',
        'gnss_sacc_shown', 'plane_icon_color', 'draw_plane_icon', 'plane_icon_screen_deg', 'draw_steer_overlay',
        'draw_triangle', 'calculatePointC', 'calculatePointD', 'calculatePointFromBearing', 'draw_flyawayfrom',
        'draw_flyinto2', 'draw_flyinto', 'track_gap', 'draw_track', 'mapdata_color', 'draw_FlashMaps', 'draw_ExtraMaps',
        'draw_vsi', 'roll_trim_color', 'attitude_implausible', 'wind_arrow_color', 'draw_wind_arrow',
        'draw_ground_ok_range', 'eskf_yaw_row_y', 'draw_eskf_attitude', 'draw_eskf_yaw', 'begin_notice_box',
        'draw_power_fault_warning', 'draw_eskf_calib_warning', 'push_backscreen', 'truetrack_display_deg',
        'draw_average_pitch_label', 'draw_average_alpha_label',
        'tas_box_color', 'draw_tas_label_row', 'draw_gs_track', 'draw_degpersec', 'header_shows_pitch', 'header_pitch_value',
        'draw_header_pitch', 'draw_header_tas', 'live_collect_screen', 'header_strike_tag', 'header_strike',
        'draw_header', 'draw_map_footer', 'draw_batt_part', 'draw_footer', 'draw_map', 'clip_outcode',
        'clip_line_to_screen', 'fill_polygon_evenodd', 'draw_pilon_takeshima_line', 'draw_pilon_takeshima_marks',
        'draw_nomapdata', 'battery_percent', 'battery_low_pct', 'battery_color_pct',
        'd:NEEDLE_LEN_NORTHUP', 'd:NEEDLE_LEN_TRACKUP', 'd:EskfGnssState', 'd:HdrStrike',
        '#define:TRIANGLE_|NOTICE_RES_|TRACKUP_|ESKF_|WIND_|CLIP_MARGIN|VM_MAX_EDGES|PILON_LINE_|CENTERLINE_']),
    ('display_tft.h', 'live_screen.js / live_tft.js', [
        'd:cord_tft', 'd:LiveScreen',
        '#define:SCREEN_|HEADERFOOTER_|BACKSCREEN_|VSI_|COLOR_|AA_FONT_|NM_FONT_|LIVESCR_']),
    ('GPS_TFT_map.ino', 'live_screen.js（render）', ['draw_current_screen@screen_mode==MODE_MAP)']),
    ('src/vectormap.cpp', 'live_map.js', ['*']),
    ('src/navdata.cpp', 'live_map.js', ['rad2deg', 'deg2rad', 'calculateDistanceKm', 'calculateTrueCourseRad',
                                        'pla_centerline_bearing_rad', 'latitudeToMercatorY', 'check_within_latlon']),
    ('gnss.cpp', 'live_screen.js', ['add_track_point', 'get_gnss_numsat']),
    ('tas.h', 'live_screen.js', ['d:TasHdr']),
    ('tas.cpp', 'live_screen.js', ['tas_pitch_avg', 'tas_header_alpha_avg', 'tas_header_value']),
    ('link.cpp', 'live.js（ltView）', [
        'link_get_lat', 'link_get_lon', 'link_get_gs', 'link_get_truetrack', 'link_get_numsat', 'link_get_hacc_mm',
        'link_get_fix_ok', 'link_get_sacc_mps', 'link_get_kf_vspeed', 'link_get_attitude', 'link_get_yaw',
        'link_get_pitch_avg', 'link_get_alpha_avg', 'link_get_tas', 'link_get_roll_trim', 'link_get_wind', 'link_get_sender_battery',
        'link_get_sender_scale_index']),
    ('lora_link/link_proto.h', 'live.js（decodeBatt）', ['link_batt_decode', 'd:LinkBattKind']),
    ('lora_link/link_wire2.h', 'live.js（W2S）', ['#define:W2S_']),
    ('live.cpp', 'live.js（行の読み取り）と LIVE_FMT_VER', ['send_ok', 'dump_step', 'live_on_rx_frame', 'live_after_screen', 'push_ls']),
]


def strip_c(src):
    """コメントを消す（改行は残す）。同じ長さの「骨」も返す（文字列・文字の中身を '_' にしたもの。括弧の対応と空白に使う）"""
    code, skel = [], []
    i, n = 0, len(src)
    while i < n:
        c = src[i]
        nx = src[i + 1] if i + 1 < n else ''
        if c == '/' and nx == '/':
            j = src.find('\n', i)
            i = n if j < 0 else j
            continue
        if c == '/' and nx == '*':
            j = src.find('*/', i + 2)
            j = n if j < 0 else j + 2
            gap = ' ' + '\n' * src.count('\n', i, j)
            code.append(gap)
            skel.append(gap)
            i = j
            continue
        if c in '"\'':
            j = i + 1
            while j < n and src[j] != c and src[j] != '\n':
                j += 2 if src[j] == '\\' else 1
            j = min(j + 1, n)
            lit = src[i:j]
            code.append(lit)
            skel.append(lit[0] + '_' * (len(lit) - 2) + lit[-1] if len(lit) >= 2 else lit)
            i = j
            continue
        code.append(c)
        skel.append(c)
        i += 1
    return ''.join(code), ''.join(skel)


def close_of(skel, k, op, cl):
    """skel[k] の op に対応する cl の次の位置（無ければ -1）"""
    depth = 0
    for j in range(k, len(skel)):
        if skel[j] == op:
            depth += 1
        elif skel[j] == cl:
            depth -= 1
            if depth == 0:
                return j + 1
    return -1


def func_spans(skel, name_re):
    """関数の定義（頭が行の先頭から始まるもの）の (名前, 始まり, 終わり)。宣言（; で終わるもの）は飛ばす"""
    out = []
    for m in re.finditer(r'(?m)^(?![ \t#])[^\n;{}()]*?\b(' + name_re + r')\s*\(', skel):
        p = close_of(skel, m.end() - 1, '(', ')')
        if p < 0:
            continue
        q = re.match(r'\s*(?:const\s*)?\{', skel[p:])
        if not q:
            continue
        b = p + q.end() - 1
        e = close_of(skel, b, '{', '}')
        if e > 0:
            out.append((m.group(1), m.start(), e))
    return out


def decl_span(skel, name):
    """宣言（struct・enum・const など。行の先頭から、括弧の外の ; まで）"""
    m = re.search(r'(?m)^[ \t]*(?:static\s+|const\s+|constexpr\s+|typedef\s+)*(?:struct|enum|int|float|double|bool|char|uint\w+|int\w+)\b[^;\n]*?\b'
                  + re.escape(name) + r'\b', skel)
    if not m:
        return None
    depth = 0
    for j in range(m.start(), len(skel)):
        if skel[j] in '{(':
            depth += 1
        elif skel[j] in '})':
            depth -= 1
        elif skel[j] == ';' and depth == 0:
            return (m.start(), j + 1)
    return None


def fp_items():
    """(見張るものの名前, 写し先, 指紋 or None＝見つからない) の並び"""
    out = []
    for rel, dst, specs in FP_TARGETS:
        code, skel = strip_c((ROOT / rel).read_text(encoding='utf-8', errors='replace'))

        def digest(a, b):
            body = ''.join(ch for ch, sk in zip(code[a:b], skel[a:b]) if not sk.isspace())
            return hashlib.sha1(body.encode('utf-8')).hexdigest()[:12]

        for spec in specs:
            if spec == '*':
                out.append((f'{rel} (まるごと)', dst, digest(0, len(code))))
            elif spec.startswith('#define:'):
                pre = '|'.join(map(re.escape, spec[8:].split('|')))
                found = False
                for m in re.finditer(r'(?m)^[ \t]*#[ \t]*define[ \t]+((?:' + pre + r')\w*)[^\n]*', skel):
                    out.append((f'{rel} #define {m.group(1)}', dst, digest(m.start(), m.end())))
                    found = True
                if not found:
                    out.append((f'{rel} {spec}', dst, None))
            elif spec.startswith('d:'):
                sp = decl_span(skel, spec[2:])
                out.append((f'{rel} {spec[2:]}', dst, digest(*sp) if sp else None))
            elif '@' in spec:
                fn, anchor = spec.split('@', 1)
                spans = func_spans(skel, re.escape(fn))
                h = None
                if spans:
                    _, a, b = spans[0]
                    m = re.search(r'\s*'.join(map(re.escape, anchor)), skel[a:b])
                    if m:
                        k = skel.find('{', a + m.end(), b)
                        e = close_of(skel, k, '{', '}') if k >= 0 else -1
                        h = digest(a + m.start(), e) if e > 0 else None
                out.append((f'{rel} {fn}() の {anchor}', dst, h))
            else:
                spans = func_spans(skel, re.escape(spec))
                # 同じ名前の定義が #if で 2 つあるときは両方つなげて見る
                h = None
                if spans:
                    h = hashlib.sha1(''.join(digest(a, b) for _, a, b in spans).encode()).hexdigest()[:12] if len(spans) > 1 \
                        else digest(spans[0][1], spans[0][2])
                out.append((f'{rel} {spec}()', dst, h))
    return out


def check_fingerprints(update):
    items = fp_items()
    missing = [k for k, _, h in items if h is None]
    if missing:
        for k in missing:
            print(f'  [NG] 見張る相手が見つからない（名前が変わった・消えた？ run_live_tests.py の FP_TARGETS を直す）: {k}')
        return False
    now = {k: h for k, _, h in items}
    dst = {k: d for k, d, _ in items}
    old = {}
    if FP_FILE.exists():
        for line in FP_FILE.read_text(encoding='utf-8').splitlines():
            if line and not line.startswith('#'):
                k, _, h = line.rpartition(' ')
                old[k] = h
    changed = [k for k in now if k in old and old[k] != now[k]]
    added = [k for k in now if k not in old]
    gone = [k for k in old if k not in now]
    if update:
        lines = ['# tools/repons/test/live_fingerprints.txt',
                 '# Re:PONS のライブ配信（tools/repons/live*.js）が写している本体の関数の指紋（コメントと空白を除いた SHA-1 の頭 12 桁）。',
                 '# run_live_tests.py --update が書く。手で直さない。見張る関数の一覧は run_live_tests.py の FP_TARGETS。']
        lines += [f'{k} {h}' for k, h in now.items()]
        FP_FILE.write_text('\n'.join(lines) + '\n', encoding='utf-8')
        print(f'  [更新] 指紋を書き直した（{len(now)} 項目。変わった {len(changed)}・増えた {len(added)}・消えた {len(gone)}）')
        for k in changed + added + gone:
            print(f'         {k}')
        return True
    if not (changed or added or gone):
        print(f'  [OK] 写しの見張り（本体の {len(now)} 項目）')
        return True
    print('  [NG] Re:PONS に写した本体の関数が変わった。配信の画面が本体と食い違わないよう、写し先も同じに直す:')
    for k in changed:
        print(f'         {k}  → {dst[k]}')
    for k in added:
        print(f'         {k}（新しく見張る）  → {dst[k]}')
    for k in gone:
        print(f'         {k}（見張りから外れた）')
    print('       写し先を直したら（または写しに関係ない変更と確かめたら）: python3 tools/repons/test/run_live_tests.py --update')
    print('       変わったところは git diff で見る（例: git diff -- display_tft.cpp）')
    return False


def main():
    update = '--update' in sys.argv[1:]
    ok = check_consts()
    ok = run_js() and ok
    r = subprocess.run([sys.executable, str(REPONS / 'gen_live_assets.py'), '--check'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    print(r.stdout.rstrip())
    ok = ok and r.returncode == 0
    ok = check_fingerprints(update) and ok
    print('結果: ' + ('OK' if ok else 'NG'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
