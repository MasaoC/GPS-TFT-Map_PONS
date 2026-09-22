# ============================================================
# File    : clean_wav8.py
# Project : PONS v6 (Pilot Oriented Navigation System for HPA)
# Role    : sd/wav/ の 8bit unsigned PCM 音声ファイルの一括クリーンアップ。
#           無音区間のディザ雑音（「スーッ」というヒス）除去、
#           先頭・末尾の無音トリム、DC オフセット除去、
#           ラウドネス統一（ファイル間の音量差解消）を行う。
#
#           元の 16bit マスターが無い状態での後処理なので、
#           「発話中」の量子化ノイズは除去できない（信号と混ざっているため）。
#           消せるのは無音区間のヒスとレベルのバラつきのみ。
#
# 使い方  : python3 tools/clean_wav8.py sd/wav/*.wav          # 上書き
#           python3 tools/clean_wav8.py -o out/ sd/wav/*.wav  # 別ディレクトリへ
#           python3 tools/clean_wav8.py -n sd/wav/*.wav       # 解析のみ（書き込まない）
# Author  : MasaoC (@masao_mobile)
# ============================================================
import wave
import numpy as np
import argparse
import os

# ---- 調整パラメータ -----------------------------------------
GATE_LSB     = 2.5    # ノイズゲートのしきい値 [LSB]。無音部の RMS は約 0.7 なので約 3.5 倍
GATE_WIN_MS  = 5.0    # 包絡線を取る窓長 [ms]
HOLD_MS      = 30.0   # ゲート開放のホールド [ms]。語尾や息継ぎを切らないため前後に広げる
FADE_MS      = 8.0    # ゲート開閉のフェード [ms]。急に切るとプチッと鳴るため
LEAD_MS      = 10.0   # トリム後に残す先頭の完全無音 [ms]。アンプ ON 直後のポップ回避
TAIL_MS      = 30.0   # トリム後に残す末尾の完全無音 [ms]
TARGET_RMS   = 22.0   # 発話部の目標 RMS [LSB]。ファイル間の音量を揃える基準
PEAK_CEIL    = 112.0  # ピーク上限 [LSB]。±127 のうち 15 を残してバリオ音との加算クリップを避ける
# ------------------------------------------------------------


def load_u8(path):
    """8bit unsigned PCM WAV を読み、中心 0 の float 配列とサンプルレートを返す。"""
    with wave.open(path, 'rb') as w:
        if w.getsampwidth() != 1 or w.getnchannels() != 1:
            raise ValueError(f'{path}: 8bit モノラルではありません')
        sr = w.getframerate()
        d = np.frombuffer(w.readframes(w.getnframes()), dtype=np.uint8)
    return d.astype(np.float64) - 128.0, sr


def save_u8(path, d, sr):
    """float 配列を 8bit unsigned PCM WAV として書き出す。

    ★ここでディザを加えない（単純な四捨五入）ことが本スクリプトの肝。
      Audacity のデフォルト書き出しはディザを加えるため、無音部が 128 固定にならず
      ±1〜2 LSB で揺れ続け、それが再生中ずっと「スーッ」と聞こえていた。
    """
    q = np.clip(np.round(d) + 128.0, 0, 255).astype(np.uint8)
    with wave.open(path, 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(1)
        w.setframerate(sr)
        w.writeframes(q.tobytes())


def soft_limit(d, ceil):
    """しきい値以上をなだらかに圧縮するソフトリミッタ。

    音声のピークは全サンプルの 1% 未満しかないため、そこだけを丸めれば
    波形をほとんど変えずに全体のレベルを上げられる。
    ハードクリップと違い折れ曲がりが無いので歪みが目立たない。
    """
    knee = ceil * 0.6                       # ここから下は素通し
    over = np.abs(d) > knee
    if not np.any(over):
        return d
    out = d.copy()
    x = (np.abs(d[over]) - knee) / (ceil - knee)
    out[over] = np.sign(d[over]) * (knee + (ceil - knee) * np.tanh(x))
    return out


def process(d, sr):
    """DC 除去 → ノイズゲート → トリム → ラウドネス統一 → ソフトリミット。"""
    # --- DC オフセット除去 ---
    # ファイルによって無音の中心が 127 だったり 128 だったりバラついている。
    # 中央値を引くことで無音が確実に 0（＝書き出し後 128）になる。
    d = d - np.median(d)

    # --- ノイズゲート ---
    # 5ms 窓の RMS で「音があるか」を判定し、無い区間は完全に 0 にする。
    win = max(1, int(sr * GATE_WIN_MS / 1000))
    env = np.sqrt(np.convolve(d * d, np.ones(win) / win, mode='same'))
    gate = env > GATE_LSB

    # ホールド: 判定を前後に広げる（子音の立ち上がりや語尾の減衰を切らないため）
    hold = int(sr * HOLD_MS / 1000)
    gate = np.convolve(gate.astype(np.float64), np.ones(2 * hold + 1), mode='same') > 0

    # フェード: 0/1 を移動平均でなまし、開閉時のプチノイズを防ぐ
    fade = max(1, int(sr * FADE_MS / 1000))
    g = np.clip(np.convolve(gate.astype(np.float64), np.ones(fade) / fade, mode='same'), 0.0, 1.0)
    d = d * g

    # --- 先頭・末尾のトリム ---
    # 元ファイルには先頭 0〜0.28s、末尾 0.04〜0.59s の無音が付いている。
    # アンプは WAV 再生の全区間 ON なので、この無音部でもヒスが鳴っていた。
    nz = np.where(g > 0.01)[0]
    if len(nz) == 0:
        return np.zeros(1)
    lead = int(sr * LEAD_MS / 1000)
    tail = int(sr * TAIL_MS / 1000)
    d = np.concatenate([np.zeros(lead), d[nz[0]:nz[-1] + 1], np.zeros(tail)])

    # --- ラウドネス統一 ---
    # 発話部（ゲートが開いている区間）の RMS を揃える。
    voiced = d[np.abs(d) > 0]
    if len(voiced) == 0:
        return d
    rms = np.sqrt((voiced ** 2).mean())
    if rms > 0:
        d = d * (TARGET_RMS / rms)

    # --- ソフトリミット後、ピーク上限に収める ---
    d = soft_limit(d, PEAK_CEIL)
    peak = np.abs(d).max()
    if peak > PEAK_CEIL:
        d = d * (PEAK_CEIL / peak)
    return d


def measure(path):
    """レポート用の統計値を返す。"""
    d, sr = load_u8(path)
    idx = np.where(np.abs(d) > 2)[0]
    if len(idx) == 0:
        return dict(dur=len(d) / sr, peak=0, rms=0, quiet=0, sil=0)
    voc = d[idx[0]:idx[-1] + 1]
    # 100ms 窓で最も静かな区間の RMS = 実質のノイズフロア
    n = int(sr * 0.1)
    fr = d[:len(d) // n * n].reshape(-1, n) if len(d) >= n else d.reshape(1, -1)
    quiet = np.sqrt((fr ** 2).mean(axis=1)).min()
    return dict(dur=len(d) / sr, peak=np.abs(d).max(),
                rms=np.sqrt((voc ** 2).mean()), quiet=quiet,
                sil=100.0 * np.mean(d == 0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('files', nargs='+')
    ap.add_argument('-o', '--outdir', default=None, help='出力先（省略時は上書き）')
    ap.add_argument('-n', '--dry-run', action='store_true', help='書き込まず解析のみ')
    args = ap.parse_args()

    if args.outdir:
        os.makedirs(args.outdir, exist_ok=True)

    hdr = f'{"file":30s} {"長さ":>12s} {"ピーク":>10s} {"発話RMS":>12s} {"無音RMS":>13s} {"完全無音率":>13s}'
    print(hdr)
    print('-' * len(hdr))
    for path in args.files:
        before = measure(path)
        d, sr = load_u8(path)
        out = process(d, sr)
        dst = os.path.join(args.outdir, os.path.basename(path)) if args.outdir else path
        if args.dry_run:
            tmp = '/tmp/_clean8_probe.wav'
            save_u8(tmp, out, sr)
            after = measure(tmp)
            os.remove(tmp)
        else:
            save_u8(dst, out, sr)
            after = measure(dst)
        print(f'{os.path.basename(path):30s} '
              f'{before["dur"]:5.2f}→{after["dur"]:5.2f}s '
              f'{before["peak"]:4.0f}→{after["peak"]:4.0f} '
              f'{before["rms"]:5.1f}→{after["rms"]:5.1f} '
              f'{before["quiet"]:5.2f}→{after["quiet"]:5.2f} '
              f'{before["sil"]:5.1f}%→{after["sil"]:5.1f}%')


if __name__ == '__main__':
    main()
