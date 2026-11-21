#!/usr/bin/env python3
# ============================================================
# File    : tools/olddata_crypt.py
# Project : PONS v7 / v7.1
# Role    : 公開の許可を得ていない過去の大会ログを、暗号化してリポジトリ（tools/olddata/*.enc）に置き、
#           パスワードを知っている人だけが復元できるようにする
# Author  : Claude (2026-11-20)
# Updated : 2026-11-20
# ============================================================
#
# ★ このリポジトリは公開（PUBLIC）。暗号化したファイルは誰でも手に入れて、手元で何度でもパスワードを試せる。
#   守りはパスワードの長さだけなので、12 文字以上を必須にしている（4 桁の数字は 1 万通りで、数時間で総当たりされる）。
#   git の履歴に入れたものは消しても残るので、弱いパスワードで一度でも push したら取り返しがつかない。
#   パスワードは別の経路（口頭・チームの非公開の場所）で渡す。リポジトリ・コミットメッセージ・Issue に書かない。
#
# 方式（PONSENC2。2026-11-20〜）: PBKDF2-HMAC-SHA256（100 万回）で鍵を作り、AES-256-GCM で暗号化。
#   どちらもブラウザの WebCrypto にあるので、Re:PONS（tools/repons/olddata_open.js）が自前の暗号コード無しで開ける。
#   ★ 最初は scrypt（PONSENC1）にしたが、WebCrypto に scrypt が無くビューアで JS の自前実装が要るのでやめた。
#     PBKDF2 は scrypt と違ってメモリを使わず GPU の総当たりにやや弱いので、反復は OWASP の推奨（60 万回）より多めの 100 万回にし、
#     パスワードの長さ（12 文字以上）で守る。PONSENC1 も読める（復元だけ）。
#   GCM の改ざん検出があるので、パスワードが違えば「復元できない」と分かる（でたらめな中身を出さない）。
#   ファイル形式: "PONSENC2" | KDF 番号 1 = PBKDF2-SHA256, 反復回数, 0 (uint32 LE ×3) | salt 16B | nonce 12B | 暗号文 + tag 16B
#   （PONSENC1 は 3 つの数が scrypt の N, r, p）。ヘッダー（先頭 48 バイト）は AAD に入れてあるので、数の書き換えも検出される。
#   暗号化すると、同じフォルダの .enc の一覧 manifest.json を作り直す（ビューアが公開ページで一覧を出すのに使う）。
#
# 使い方（パスワードは実行時に聞く。画面にも履歴にも残らない）:
#   python3 tools/olddata_crypt.py encrypt FILE...        # FILE.enc を tools/olddata/ に作る
#   python3 tools/olddata_crypt.py decrypt                # tools/olddata/*.enc を全部戻す（同じフォルダに .csv。git には入らない）
#   python3 tools/olddata_crypt.py decrypt --out DIR X.enc
#   python3 tools/olddata_crypt.py verify FILE...         # 元のファイルと .enc の中身が一致するか
#
# 大会動画（YouTube 限定公開）の URL も同じパスワードで暗号化して置く（tools/olddata/videos.json.enc）。
#   平文の videos.json はリポジトリの外で作る（tools/olddata/ に置いても tools/.gitignore で入らない）。形は tools/olddata/README.md。
#   python3 tools/olddata_crypt.py encrypt ~/…/videos.json   → Re:PONS がログを開くときに一緒に復号して動画を出す
# 環境変数 PONS_OLDDATA_PASSWORD があればそれを使う（試験用。ふだんは使わないこと）

import argparse
import getpass
import glob
import hashlib
import json
import os
import struct
import sys

try:
    from cryptography.hazmat.primitives.ciphers.aead import AESGCM
    from cryptography.exceptions import InvalidTag
except ImportError:
    sys.exit('cryptography が要ります: pip3 install cryptography')

HERE = os.path.dirname(os.path.abspath(__file__))
OLDDATA_DIR = os.path.join(HERE, 'olddata')
MAGIC2 = b'PONSENC2'           # PBKDF2-HMAC-SHA256（いまの形式）
MAGIC1 = b'PONSENC1'           # scrypt（2026-11-20 に試しただけ。読めるようにだけしてある）
KDF_PBKDF2_SHA256 = 1
PBKDF2_ITER = 1_000_000
MIN_PASSWORD_LEN = 12
HEADER_LEN = 8 + 12 + 16 + 12
MANIFEST = 'manifest.json'


def derive_key_pbkdf2(password, salt, iters):
    return hashlib.pbkdf2_hmac('sha256', password.encode('utf-8'), salt, iters, dklen=32)


def derive_key_scrypt(password, salt, n, r, p):
    return hashlib.scrypt(password.encode('utf-8'), salt=salt, n=n, r=r, p=p,
                          maxmem=256 * 1024 * 1024, dklen=32)


def ask_password(confirm):
    pw = os.environ.get('PONS_OLDDATA_PASSWORD')
    if pw is None:
        pw = getpass.getpass('パスワード: ')
        if confirm and getpass.getpass('もう一度: ') != pw:
            sys.exit('2 回の入力が一致しません')
    if confirm and len(pw) < MIN_PASSWORD_LEN:
        sys.exit(f'パスワードは {MIN_PASSWORD_LEN} 文字以上にしてください（公開リポジトリなので短いと総当たりで開く）')
    return pw


def encrypt_bytes(data, password):
    salt, nonce = os.urandom(16), os.urandom(12)
    header = MAGIC2 + struct.pack('<III', KDF_PBKDF2_SHA256, PBKDF2_ITER, 0) + salt + nonce
    key = derive_key_pbkdf2(password, salt, PBKDF2_ITER)
    return header + AESGCM(key).encrypt(nonce, data, header)


def decrypt_bytes(blob, password):
    if len(blob) < HEADER_LEN + 16 or blob[:8] not in (MAGIC1, MAGIC2):
        raise ValueError('PONSENC の形式ではありません')
    a, b, c = struct.unpack_from('<III', blob, 8)
    salt, nonce = blob[20:36], blob[36:48]
    header = blob[:HEADER_LEN]
    if blob[:8] == MAGIC2:
        if a != KDF_PBKDF2_SHA256 or not (100_000 <= b <= 10_000_000):
            raise ValueError('ヘッダーの数がおかしい（壊れているか別物）')
        key = derive_key_pbkdf2(password, salt, b)
    else:
        if a > (1 << 20) or b > 16 or c > 4:
            raise ValueError('ヘッダーのパラメーターが大きすぎます（壊れているか別物）')
        key = derive_key_scrypt(password, salt, a, b, c)
    try:
        return AESGCM(key).decrypt(nonce, blob[HEADER_LEN:], header)
    except InvalidTag:
        raise ValueError('復元できません（パスワードが違うか、ファイルが壊れています）')


def cmd_encrypt(args):
    pw = ask_password(confirm=True)
    os.makedirs(args.out, exist_ok=True)
    for src in args.files:
        data = open(src, 'rb').read()
        dst = os.path.join(args.out, os.path.basename(src) + '.enc')
        blob = encrypt_bytes(data, pw)
        # 書いたものがその場で戻せることを確かめてから置く
        if decrypt_bytes(blob, pw) != data:
            sys.exit(f'{src}: 暗号化の確認に失敗しました')
        with open(dst, 'wb') as f:
            f.write(blob)
        print(f'暗号化: {src} → {dst}（{len(data):,} → {len(blob):,} B、sha256 {hashlib.sha256(data).hexdigest()[:16]}…）')
    write_manifest(args.out)


def write_manifest(folder):
    """folder の .enc の一覧を manifest.json に書く（Re:PONS の「大会ログ」の一覧。中身・パスワードは含まない）"""
    files = []
    for path in sorted(glob.glob(os.path.join(folder, '*.enc'))):
        blob = open(path, 'rb').read(8)
        name = os.path.basename(path)
        kind = 'log' if name.endswith('.csv.enc') else ('videos' if name.startswith('videos') else 'other')
        files.append({'file': name, 'kind': kind, 'bytes': os.path.getsize(path),
                      'format': blob.decode('ascii', 'replace')})
    with open(os.path.join(folder, MANIFEST), 'w', encoding='utf-8') as f:
        json.dump({'note': '暗号化した過去の大会ログ。tools/olddata/README.md', 'files': files}, f, ensure_ascii=False, indent=1)
        f.write('\n')
    print(f'一覧: {os.path.join(folder, MANIFEST)}（{len(files)} 件）')


def cmd_decrypt(args):
    files = args.files or sorted(glob.glob(os.path.join(OLDDATA_DIR, '*.enc')))
    if not files:
        sys.exit(f'{OLDDATA_DIR} に .enc がありません')
    pw = ask_password(confirm=False)
    os.makedirs(args.out, exist_ok=True)
    ng = 0
    for src in files:
        name = os.path.basename(src)
        dst = os.path.join(args.out, name[:-4] if name.endswith('.enc') else name + '.dec')
        try:
            data = decrypt_bytes(open(src, 'rb').read(), pw)
        except ValueError as e:
            print(f'失敗: {src}: {e}')
            ng += 1
            continue
        if os.path.exists(dst) and not args.force and open(dst, 'rb').read() != data:
            print(f'飛ばした: {dst} が既にあり中身が違う（上書きするなら --force）')
            ng += 1
            continue
        with open(dst, 'wb') as f:
            f.write(data)
        print(f'復元: {src} → {dst}（{len(data):,} B）')
    sys.exit(1 if ng else 0)


def cmd_verify(args):
    pw = ask_password(confirm=False)
    ng = 0
    for src in args.files:
        enc = os.path.join(args.dir, os.path.basename(src) + '.enc')
        try:
            ok = decrypt_bytes(open(enc, 'rb').read(), pw) == open(src, 'rb').read()
        except (OSError, ValueError) as e:
            print(f'NG  {src}: {e}')
            ng += 1
            continue
        print(f'{"OK " if ok else "NG "} {src} ↔ {enc}')
        ng += not ok
    sys.exit(1 if ng else 0)


def main():
    ap = argparse.ArgumentParser(description='過去の大会ログの暗号化・復元（PBKDF2-SHA256 + AES-256-GCM）')
    sub = ap.add_subparsers(dest='cmd', required=True)
    e = sub.add_parser('encrypt', help='ファイルを暗号化して .enc を作る')
    e.add_argument('files', nargs='+')
    e.add_argument('--out', default=OLDDATA_DIR, help=f'既定: {OLDDATA_DIR}')
    e.set_defaults(func=cmd_encrypt)
    d = sub.add_parser('decrypt', help='.enc を元に戻す（既定: tools/olddata/*.enc を同じフォルダへ）')
    d.add_argument('files', nargs='*')
    d.add_argument('--out', default=OLDDATA_DIR)
    d.add_argument('--force', action='store_true', help='中身の違う同名ファイルを上書きする')
    d.set_defaults(func=cmd_decrypt)
    v = sub.add_parser('verify', help='元のファイルと .enc の中身が一致するか確かめる')
    v.add_argument('files', nargs='+')
    v.add_argument('--dir', default=OLDDATA_DIR, help='.enc のあるフォルダ')
    v.set_defaults(func=cmd_verify)
    args = ap.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
