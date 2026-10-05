#!/usr/bin/env python3
# ============================================================
# File    : gen_wav_flash.py
# Project : PONS v7 (Pilot Oriented Navigation System for HPA)
# Role    : wav/*.wav から、ファームウェアに内蔵する
#           src/flashdata/wav_data.cpp / wav_data.h を生成する。
#
#           0.982 で音声を SD から本体 FLASH へ移した。SD カードが無い・
#           壊れているときでも警報の音声が鳴るようにするため。
#           SD からの WAV 再生は廃止したので、wav/ は
#           「焼き込む原本」であって配布するカードには入れない。
#
# 使い方:
#   python3 tools/gen_wav_flash.py           # 生成（src/flashdata/ へ書く）
#   python3 tools/gen_wav_flash.py --check   # 生成せず、既存の生成物が
#                                            # wav/ と一致しているか検査する
#                                            # （一致しなければ終了コード 1）
#
#   --check はリリース前チェック用。音声を差し替えたまま生成を忘れると、
#   古い音のファームウェアが黙って焼かれる。CRC32 で気づけるようにしてある。
#
# 音声の仕様（これ以外は受け付けずエラーで止める）:
#   8bit unsigned PCM / 16000Hz / モノラル。src/sound.cpp の
#   タイマー割り込みが 16kHz 固定で、8bit を 128 中心として読むため。
#   録り直すときは tools/clean_wav8.py を通してから生成すること。
#
# Author  : MasaoC (@masao_mobile)
# Updated : 2026/10/01
# ============================================================
import argparse
import os
import re
import struct
import sys
import zlib

# リポジトリのルート（このスクリプトは tools/ にある）
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

WAV_DIR = os.path.join(ROOT, 'wav')
OUT_CPP = os.path.join(ROOT, 'src', 'flashdata', 'wav_data.cpp')
OUT_H = os.path.join(ROOT, 'src', 'flashdata', 'wav_data.h')

# 受け付ける音声フォーマット
REQ_FORMAT = 1       # WAVE_FORMAT_PCM
REQ_CHANNELS = 1     # モノラル
REQ_RATE = 16000     # Hz
REQ_BITS = 8         # bit/sample（unsigned）

# 1 エントリの先頭を 4 バイト境界に置く（memcpy をワード転送にするため）。
# 詰め物は 128（8bit unsigned PCM の無音）にしておく。万一読まれても無音で済む。
ALIGN = 4
PAD_BYTE = 128

# 呼び出し側のソースから音声名を拾う正規表現。
# 例: enqueueTask(createPlayWavTask("wav/opening.wav", 3));
NAME_RE = re.compile(r'"(wav/[A-Za-z0-9_\-]+\.wav)"')

# 音声名を grep する対象。src/flashdata/ は生成物なので除く。
SCAN_EXTS = ('.cpp', '.h', '.ino')
SCAN_SKIP_DIRS = {'tools', 'docs', 'kicad', 'case_3Dmodel', 'sd', 'wav',
                  '.git', 'TFT_eSPI', 'production'}
SCAN_SKIP_FILES = {os.path.join('src', 'flashdata', 'wav_data.cpp'),
                   os.path.join('src', 'flashdata', 'wav_data.h')}


# ------------------------------------------------------------
# WAV の読み込み
# ------------------------------------------------------------
def parse_wav(path):
    """RIFF をパースして (音声データ bytes, サンプルレート) を返す。

    ★ ヘッダ 44 バイト固定で切り落とさないこと。
      Audacity や TTS の書き出し設定によっては fmt の後ろに LIST や fact
      チャンクが入り、data の位置が 44 とは違う。ファームウェア側は
      「先頭から len バイトが音声」という前提で読むので、ここで正しく
      切り出しておかないと**冒頭にヘッダの残骸がノイズとして鳴る**。
    """
    with open(path, 'rb') as f:
        d = f.read()

    if len(d) < 12 or d[0:4] != b'RIFF' or d[8:12] != b'WAVE':
        raise ValueError('RIFF/WAVE ではない')

    i = 12
    fmt = None
    data = None
    while i + 8 <= len(d):
        cid = d[i:i + 4]
        sz = struct.unpack('<I', d[i + 4:i + 8])[0]
        body = d[i + 8:i + 8 + sz]
        if cid == b'fmt ':
            if len(body) < 16:
                raise ValueError('fmt チャンクが短い')
            fmt = struct.unpack('<HHIIHH', body[:16])
        elif cid == b'data':
            data = body
        i += 8 + sz + (sz & 1)   # チャンクは偶数境界

    if fmt is None:
        raise ValueError('fmt チャンクが無い')
    if data is None:
        raise ValueError('data チャンクが無い')

    afmt, ch, rate, _bps, _align, bits = fmt
    if afmt != REQ_FORMAT or ch != REQ_CHANNELS or rate != REQ_RATE or bits != REQ_BITS:
        raise ValueError(
            '%dbit %dch %dHz fmt=%d — 要求は %dbit %dch %dHz PCM'
            % (bits, ch, rate, afmt, REQ_BITS, REQ_CHANNELS, REQ_RATE))
    if len(data) == 0:
        raise ValueError('音声データが 0 バイト')

    return data, rate


def trim_trailing_zeros(data):
    """末尾に続く 0x00 を 128（無音）に直す。

    8bit unsigned PCM では 0x00 は最大負圧であって無音ではない。data の
    サイズが奇数のときに RIFF の偶数パディングが 1 バイト混ざることがあり、
    そのまま鳴らすとプチッと出る。src/sound.cpp が同じ後始末をしていたが、
    FLASH 化すれば生成時に直せるので、実機の毎回の処理からは外す。
    """
    out = bytearray(data)
    i = len(out) - 1
    n = 0
    while i >= 0 and out[i] == 0x00:
        out[i] = PAD_BYTE
        i -= 1
        n += 1
    return bytes(out), n


# ------------------------------------------------------------
# 呼び出し側のソースを走査して、使われている音声名を集める
# ------------------------------------------------------------
def scan_referenced_names():
    """firmware のソースに出てくる "wav/*.wav" の一覧を {名前: [出現箇所]} で返す。"""
    found = {}
    for dirpath, dirnames, filenames in os.walk(ROOT):
        rel_dir = os.path.relpath(dirpath, ROOT)
        top = rel_dir.split(os.sep)[0]
        if top in SCAN_SKIP_DIRS:
            dirnames[:] = []
            continue
        for fn in filenames:
            if not fn.endswith(SCAN_EXTS):
                continue
            rel = os.path.relpath(os.path.join(dirpath, fn), ROOT)
            if rel in SCAN_SKIP_FILES:
                continue
            with open(os.path.join(dirpath, fn), encoding='utf-8', errors='replace') as f:
                for lineno, line in enumerate(f, 1):
                    for m in NAME_RE.finditer(line):
                        found.setdefault(m.group(1), []).append('%s:%d' % (rel, lineno))
    return found


# ------------------------------------------------------------
# 収集
# ------------------------------------------------------------
class Entry:
    def __init__(self, name, data, crc, fixed_tail):
        self.name = name              # "wav/opening.wav" — 呼び出し側の文字列と一致させる
        self.data = data
        self.crc = crc
        self.fixed_tail = fixed_tail  # 128 に直した末尾 0x00 の数
        self.off = 0                  # WAV_BLOB 内のオフセット（後で決める）


def collect(strict):
    """wav/ を読んでエントリ一覧を作る。strict=True なら参照の食い違いをエラーにする。"""
    if not os.path.isdir(WAV_DIR):
        die('音声のディレクトリが無い: %s' % WAV_DIR)

    files = sorted(f for f in os.listdir(WAV_DIR) if f.lower().endswith('.wav'))
    if not files:
        die('%s に .wav が無い' % WAV_DIR)

    entries = []
    errors = []
    for fn in files:
        path = os.path.join(WAV_DIR, fn)
        try:
            data, _rate = parse_wav(path)
        except ValueError as e:
            errors.append('  %s: %s' % (fn, e))
            continue
        data, fixed = trim_trailing_zeros(data)
        entries.append(Entry('wav/' + fn, data, zlib.crc32(data) & 0xFFFFFFFF, fixed))

    if errors:
        die('音声フォーマットが仕様と違う:\n' + '\n'.join(errors)
            + '\n  → tools/clean_wav8.py を通すか、8bit/16kHz/mono で書き出し直すこと')

    have = {e.name for e in entries}
    refs = scan_referenced_names()

    # ソースが参照しているのに wav/ に無い → ビルドを通してはいけない。
    # 実機では「ERR wav missing」が出るまで気づけないので、ここで止める。
    missing = sorted(set(refs) - have)
    if missing:
        msg = ['ソースが参照している音声が %s に無い:' % os.path.relpath(WAV_DIR, ROOT)]
        for n in missing:
            msg.append('  %s   ← %s' % (n, ', '.join(refs[n])))
        die('\n'.join(msg))

    # wav/ にあるがソースから呼ばれていない → 焼くが警告する（消し忘れに気づけるように）
    unused = sorted(have - set(refs))
    if unused:
        warn('ソースから参照されていない音声（FLASH には焼く）: ' + ', '.join(unused))

    # オフセットを決める（4 バイト境界）
    off = 0
    for e in entries:
        e.off = off
        off += len(e.data)
        pad = (-off) % ALIGN
        off += pad
    total = off

    if strict and total == 0:
        die('音声データの合計が 0 バイト')

    return entries, total


# ------------------------------------------------------------
# 出力
# ------------------------------------------------------------
def manifest_lines(entries, total):
    """生成物の先頭に埋めるマニフェスト。--check がこれを読んで照合する。"""
    src = sum(len(e.data) for e in entries)
    lines = ['// ---- MANIFEST (tools/gen_wav_flash.py --check が照合する) ----']
    for e in entries:
        lines.append('//   %-30s %8d B  crc32=%08x' % (e.name, len(e.data), e.crc))
    lines.append('//   %-30s %8d B  (%.1f 秒 / %d ファイル)'
                 % ('TOTAL (音声データ)', src, src / REQ_RATE, len(entries)))
    lines.append('//   %-30s %8d B  (整列の詰め物 %d B を含む)'
                 % ('WAV_BLOB', total, total - src))
    lines.append('// ---- END MANIFEST ----')
    return lines


MANIFEST_RE = re.compile(r'^//\s+(wav/[A-Za-z0-9_\-]+\.wav)\s+(\d+) B\s+crc32=([0-9a-f]{8})\s*$')


def read_manifest(path):
    """生成済み .cpp のマニフェストを {名前: (長さ, crc)} で返す。無ければ None。"""
    if not os.path.exists(path):
        return None
    out = {}
    seen = False
    with open(path, encoding='utf-8', errors='replace') as f:
        for line in f:
            if 'END MANIFEST' in line:
                break
            if 'MANIFEST' in line:
                seen = True
                continue
            m = MANIFEST_RE.match(line.rstrip('\n'))
            if m:
                out[m.group(1)] = (int(m.group(2)), int(m.group(3), 16))
    return out if seen else None


HEADER_TMPL = '''// ============================================================
// File    : wav_data.h
// Project : PONS v7 (Pilot Oriented Navigation System for HPA)
// Role    : 本体 FLASH に内蔵した音声（8bit unsigned PCM / 16kHz / mono）の**宣言**。
//           実体は wav_data.cpp（tools/gen_wav_flash.py の生成物）。
//
//           0.982 で音声を SD から FLASH へ移した。SD が無くても・壊れていても
//           警報の音声が鳴るようにするため。SD からの WAV 再生は廃止した。
//
// ■ なぜ .h に実体を置かないのか
//   名前空間スコープの const は内部リンケージなので、.h に実体を書くと
//   include した .cpp ごとに複製される（インクルードガードでは止まらないし、
//   リンクエラーにもならないので気づけない）。logo_data.h と同じ理由。
//
// Author  : MasaoC (@masao_mobile)
// Updated : %(date)s
// ============================================================
#ifndef WAV_DATA_H
#define WAV_DATA_H
#include <Arduino.h>

// 音声 1 本ぶんの索引。
// name は **呼び出し側が書いている文字列リテラルと同じ**（例 "wav/opening.wav"）。
// SD 時代のパスをそのまま鍵に使うことで、createPlayWavTask() の呼び出しを
// 一切書き換えずに供給元だけ FLASH へ移せる。
struct WavEntry {
  const char* name;  // 音声名（＝呼び出し側の文字列）
  uint32_t    off;   // WAV_BLOB 内のオフセット [byte]
  uint32_t    len;   // 音声データ長 [byte]（= サンプル数。16kHz なので /16000 で秒）
};

// 生成時のサンプルレート [Hz]。
// ★ マクロにしてあるのは、sound.cpp が static_assert で自分の sampleRate と
//   突き合わせられるようにするため。extern const では別の翻訳単位の値なので
//   コンパイル時に比べられず、**取り違えたまま黙って通る**。
#define WAV_SAMPLE_RATE %(rate)d

extern const uint8_t  WAV_BLOB[];      // 全音声を連結したもの
extern const uint32_t WAV_BLOB_SIZE;   // WAV_BLOB の総バイト数（詰め物を含む）
extern const WavEntry WAV_ENTRIES[];   // name の昇順に並んでいる
extern const uint16_t WAV_COUNT;       // WAV_ENTRIES の要素数

#endif // WAV_DATA_H
'''

CPP_HEAD_TMPL = '''// ============================================================
// wav_data.cpp — 自動生成ファイル。手で編集しないこと。
// 生成: tools/gen_wav_flash.py   （原本は wav/*.wav）
//
// 音声を差し替えたら必ず生成し直すこと。忘れると古い音のまま焼ける。
// リリース前は tools/gen_wav_flash.py --check で照合できる。
//
// 仕様: 8bit unsigned PCM / %(rate)dHz / モノラル。無音は 128。
// ============================================================
%(manifest)s
#include "wav_data.h"

const uint16_t WAV_COUNT = %(count)d;
const uint32_t WAV_BLOB_SIZE = %(total)d;

// 各エントリの先頭が 4 バイト境界に来るよう詰めてある（memcpy をワード転送にするため）。
// 詰め物は 128（無音）。
const uint8_t WAV_BLOB[] __attribute__((aligned(4))) = {
'''


def write_output(entries, total):
    date = '2026/10/01'

    with open(OUT_H, 'w', encoding='utf-8') as f:
        f.write(HEADER_TMPL % {'date': date, 'rate': REQ_RATE})

    with open(OUT_CPP, 'w', encoding='utf-8') as f:
        f.write(CPP_HEAD_TMPL % {
            'rate': REQ_RATE,
            'count': len(entries),
            'total': total,
            'manifest': '\n'.join(manifest_lines(entries, total)),
        })

        written = 0
        for e in entries:
            f.write('  // ---- %s : off=%d len=%d (%.2f 秒) ----\n'
                    % (e.name, e.off, len(e.data), len(e.data) / REQ_RATE))
            assert written == e.off, '内部矛盾: オフセットがずれた'
            buf = e.data
            for i in range(0, len(buf), 16):
                f.write('  ' + ','.join('0x%02x' % b for b in buf[i:i + 16]) + ',\n')
            written += len(buf)
            pad = (-written) % ALIGN
            if pad:
                f.write('  ' + ','.join('0x%02x' % PAD_BYTE for _ in range(pad))
                        + ',  // 整列用の詰め物\n')
                written += pad
        assert written == total, '内部矛盾: 総バイト数がずれた'
        f.write('};\n\n')

        f.write('const WavEntry WAV_ENTRIES[] = {\n')
        for e in entries:
            f.write('  { "%s", %d, %d },\n' % (e.name, e.off, len(e.data)))
        f.write('};\n')


# ------------------------------------------------------------
def warn(msg):
    print('WARN: ' + msg, file=sys.stderr)


def die(msg):
    print('ERROR: ' + msg, file=sys.stderr)
    sys.exit(1)


def do_check(entries):
    """生成済み .cpp のマニフェストと wav/ の中身を照合する。"""
    man = read_manifest(OUT_CPP)
    if man is None:
        die('生成物が無い（またはマニフェストが読めない）: %s\n'
            '  → python3 tools/gen_wav_flash.py を実行すること'
            % os.path.relpath(OUT_CPP, ROOT))

    cur = {e.name: (len(e.data), e.crc) for e in entries}
    problems = []
    for name in sorted(set(cur) | set(man)):
        a = cur.get(name)
        b = man.get(name)
        if b is None:
            problems.append('  %s: wav/ にあるが生成物に無い' % name)
        elif a is None:
            problems.append('  %s: 生成物にあるが wav/ に無い' % name)
        elif a != b:
            problems.append('  %s: 中身が違う（wav/ %d B crc32=%08x / 生成物 %d B crc32=%08x）'
                            % (name, a[0], a[1], b[0], b[1]))
    if problems:
        die('生成物が wav/ と一致していない:\n' + '\n'.join(problems)
            + '\n  → python3 tools/gen_wav_flash.py を実行して焼き直すこと')

    total = sum(len(e.data) for e in entries)
    print('OK: %d ファイル / %d B (%.1f 秒) が生成物と一致している'
          % (len(entries), total, total / REQ_RATE))


def main():
    ap = argparse.ArgumentParser(
        description='wav/*.wav から src/flashdata/wav_data.cpp を生成する')
    ap.add_argument('--check', action='store_true',
                    help='生成せず、既存の生成物が wav/ と一致するか検査する')
    args = ap.parse_args()

    entries, total = collect(strict=True)

    if args.check:
        do_check(entries)
        return

    write_output(entries, total)

    src = sum(len(e.data) for e in entries)
    print('%-28s %8d B' % ('音声データ', src))
    print('%-28s %8d B  (整列の詰め物 %d B)' % ('WAV_BLOB', total, total - src))
    print('%-28s %8.1f 秒 / %d ファイル' % ('再生時間', src / REQ_RATE, len(entries)))
    fixed = sum(e.fixed_tail for e in entries)
    if fixed:
        print('%-28s %8d B' % ('末尾 0x00 を 128 に直した', fixed))
    for p in (OUT_H, OUT_CPP):
        print('%-28s %8.2f MB' % (os.path.relpath(p, ROOT), os.path.getsize(p) / 1048576))


if __name__ == '__main__':
    main()
