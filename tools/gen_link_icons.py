# ============================================================
# gen_link_icons.py — src/flashdata/link_icons.cpp の電波アイコンを生成する
#
#   python3 tools/gen_link_icons.py
#     → 標準出力に C の配列が出るので、link_icons.cpp の
#       「---- 電波アイコン ----」以降を置き換える。
#
#   受信用 ICON_RSSI0-3: 枠 3 本（長さ違い）を常に描き、中を 0〜3 本ぶん塗る。
#     塗りだけだと 0 本・1 本のときに何の絵か分からなくなるため。
#   送信用 ICON_TX0-3   : 右端の発生源から左へ広がる電波の弧。右隣に飛行機
#     アイコンが並ぶので、**飛行機から電波が出ている**ように見える。
#     受信強度とは無関係で、弧の数は「今送った」ことを示す脈動に使う。
#   ICON_RSSI_X         : ロスト・モジュール不応答の強調用。両モードで共用。
#   形式は 1 行 (W+7)/8 バイト、MSB が左端、1=描く / 0=透明。
# ============================================================
import math

W, H = 19, 15
BAR_W = 5
XS  = [0, 7, 14]
HS  = [7, 11, 15]

def blank(): return [[0]*W for _ in range(H)]

def rect(g, x, y, w, h, fill):
    for j in range(y, y+h):
        for i in range(x, x+w):
            if fill or i in (x, x+w-1) or j in (y, y+h-1):
                g[j][i] = 1

def bars(n):
    g = blank()
    for k in range(3):
        rect(g, XS[k], H-HS[k], BAR_W, HS[k], k < n)
    return g

def cross():
    g = blank()
    for j in range(H):
        for i in range(W):
            u = i/(W-1); v = j/(H-1)
            if abs(u-v) < 0.09 or abs(u+v-1) < 0.09:
                g[j][i] = 1
    return g

def emit(name, g):
    rb = (W+7)//8
    out = [f"const uint8_t {name}[] PROGMEM = {{"]
    for row in g:
        bs = [0]*rb
        for i, p in enumerate(row):
            if p: bs[i >> 3] |= 0x80 >> (i & 7)
        art = "".join('#' if p else '.' for p in row)
        out.append("  " + ", ".join(f"0x{b:02X}" for b in bs) + f",  // {art}")
    out.append("};")
    return "\n".join(out)

# ---- 送信中を示す電波の弧 ----
# 発生源は**右端**。右隣に飛行機アイコンが来るので、そこから出ているように見える。
TX_CX, TX_CY = 17.0, 7.0
TX_RADII = [5.5, 10.0, 14.5]
TX_SPREAD = 58                 # 左へ広がる角度 [deg]

def tx_source(g):
    for j in range(int(TX_CY)-1, int(TX_CY)+2):
        for i in range(int(TX_CX)-1, int(TX_CX)+2):
            if 0 <= i < W and 0 <= j < H: g[j][i] = 1

def tx_arcs(n):
    g = blank(); tx_source(g)
    for k in range(n):
        r = TX_RADII[k]
        for j in range(H):
            for i in range(W):
                dx, dy = i-TX_CX, j-TX_CY
                if dx > 0: continue
                if not (r-1.05 <= math.hypot(dx, dy) <= r+0.55): continue
                if math.degrees(math.atan2(abs(dy), -dx)) > TX_SPREAD: continue
                g[j][i] = 1
    return g

parts  = [emit(f"ICON_RSSI{n}", bars(n))    for n in range(4)]
parts += [emit(f"ICON_TX{n}",   tx_arcs(n)) for n in range(4)]
parts.append(emit("ICON_RSSI_X", cross()))
print("\n\n".join(parts))
