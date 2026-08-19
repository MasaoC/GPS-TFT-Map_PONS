#!/usr/bin/env python3
# ============================================================
# File    : build_vectormap.py
# Project : PONS v6 (Pilot Oriented Navigation System for HPA)
# Role    : OpenStreetMap のベクタデータから、ファームウェアに内蔵する
#           vectormap_data.cpp を生成する。
#
#           SD カード上の BMP タイル方式（tools/mapimage/create_mapimages.py）を
#           置き換えるもので、Google Maps API キーは不要。
#
# 出力データのライセンス:
#   Map data (c) OpenStreetMap contributors, ODbL 1.0
#   https://www.openstreetmap.org/copyright
#   生成される vectormap_data.cpp は ODbL の Derivative Database に該当するため、
#   ODbL で配布すること（ファームウェアのコード本体は MIT のままでよい。
#   ODbL 4.5(a) Collective Database 免除）。
#
# 使い方:
#   推奨: Geofabrik の日付付き pbf をローカル処理する（API 負荷なし・再現性あり）
#     curl -O https://download.geofabrik.de/asia/japan-260816.osm.pbf
#     curl -O https://osmdata.openstreetmap.de/download/simplified-land-polygons-complete-3857.zip
#     python3 build_vectormap.py --lods 0,1,2,3 \
#         --pbf japan-260816.osm.pbf \
#         --land-shp simplified-land-polygons-complete-3857/simplified_land_polygons.shp \
#         --source-date 2026-08-16 --out ../../vectormap_data.cpp
#
#   簡易: Overpass API（追加インストール不要だが、広い LOD では時間がかかる）
#     python3 build_vectormap.py --lods 0 --out ../../vectormap_data.cpp
#
# 依存: shapely（--pbf は osmium、--land-shp は fiona/pyproj、Overpass は requests）
# ============================================================
import argparse
import hashlib
import json
import math
import os
import struct
import sys
import time
from pathlib import Path

try:
    from shapely.geometry import Polygon, MultiPolygon, LineString, MultiLineString, box, shape
    from shapely.ops import unary_union, linemerge
except ImportError:
    sys.exit("shapely が必要です:  pip install shapely")

# ===== vectormap.h の enum vm_class と一致させること =====
# VM_URBAN は形式上の予約枠。市街地は収録しない（フラッシュを食う割に
# 「幹線と湖面が分かればよい」という用途に寄与しないため）。
VM_LANDMASS, VM_WATER, VM_URBAN, VM_MOTORWAY, VM_TRUNK, VM_RAIL = range(6)
CLASS_NAME = ["LANDMASS", "WATER", "URBAN", "MOTORWAY", "TRUNK", "RAIL"]
IS_AREA = lambda c: c <= VM_URBAN

# ===== 飛行エリア =====
# 高ズーム(LOD0/1)はここに挙げたエリアだけを収録する。全国を高精細で持つのは
# フラッシュに入らないため、実際に飛ぶ場所を列挙する方式にしている。
# (lat_min, lon_min, lat_max, lon_max)
SITES = {
    "biwako":    (34.95, 135.80, 35.60, 136.40),   # 琵琶湖（HPA 大会）
    "kasaoka":   (34.35, 133.30, 34.65, 133.70),   # 笠岡ふれあい空港
    "shirahama": (33.50, 135.15, 33.80, 135.55),   # 南紀白浜
    "tokyobay":  (35.45, 139.70, 35.80, 140.10),   # 東京湾岸（新浦安ほか）
    "fujikawa":  (35.00, 138.45, 35.30, 138.80),   # 富士川滑空場
    "osaka":     (34.65, 135.15, 34.95, 135.70),   # 阪大・大阪平野（内蔵ポリゴンの範囲を包含）
    "karasu":    (34.50, 136.40, 34.80, 136.70),   # 烏（伊勢湾西岸）
}

# ===== LOD 定義 =====
# unit_e7  : 座標 1 単位あたりの 1e-7 度。int16 に収めるためタイルの一辺 / 32767 より大きくする
# tile_e7  : タイル一辺（1e-7 度）
# tol_scale : 簡略化の許容誤差を決める基準 scale [px/km]（1px 相当を許容誤差にする）。
#             1 つの LOD が複数のズーム段を兼ねる場合は「最も細かい側」に合わせること。
#             粗い側に合わせると、細かい側で海岸線がガタガタに見える。
# cull_scale: 面積・長さの足切りを決める基準 scale [px/km]（省略時は tol_scale）。
#             こちらは「最も粗い側」に合わせる。細かい側に合わせると、広域表示では
#             見えない小島まで大量に抱え込んでしまう。
# bboxes   : 収録範囲のリスト。飛行エリアごとに分けることで、間の何もない領域を持たずに済む
# land_hires  : True なら --land-shp-hires（land-polygons-split-4326, 1.3GB）を使う。
#               高ズームでは簡易版(z8 相当)では海岸線が粗すぎて使えないため。
#               低ズームは簡易版で十分で、全国分を高精細で読むと非常に遅い。
# min_area_px : 画面上でこの大きさ未満の面（池など）を捨てる
# min_len_px  : 画面上でこの長さ未満の線を捨てる。これは「1px 未満で見えない断片の除去」
#               であって、長さで幹線を選別するものではない。
#               道路網は交差点ごとに way が分かれ、linemerge もインターチェンジ等の
#               分岐で切れるため、結合後でも断片の長さは短い（高速道路で中央値 212m）。
#               ここを 1px より大きくすると道路が虫食いになる。実測では 3px にしていた
#               ときに LOD2 で高速道路の総延長の 37% が消えていた。
#               0.5px なら本数は 43% 減らせて総延長の損失は 3.6% に収まる。
LODS = [
    dict(lod=0, min_scale=26.0, unit_e7=100, tile_e7=1_250_000, tol_scale=52.32994872,
         bboxes=list(SITES.values()), sea_bg=True, land_hires=True,
         # 海面は OSM では「陸地の不在」で表されるため、海岸線（VM_LANDMASS）が
         # 無いと沿岸サイトで海と陸の区別がつかない。高ズームでは簡易版の
         # land polygons では粗すぎるので land-polygons-split-4326 を使うこと。
         classes=[VM_LANDMASS, VM_WATER, VM_MOTORWAY, VM_TRUNK, VM_RAIL],
         min_area_px=8.0, min_len_px=1.0,
         desc="飛行エリア・高精細"),
    dict(lod=1, min_scale=6.5, unit_e7=100, tile_e7=2_500_000, tol_scale=13.08248718,
         # 周辺マージン。LOD1 は 18km 表示で、画面に映るのは中心から半径 9km
         # (=0.08度) 程度。飛行エリア枠 + 0.15 度あれば十分で、以前の 0.35/0.45 度
         # (約 39/41km) は過大だった（収録面積が 2.7 倍＝そのままフラッシュ消費）。
         bboxes=[(a - 0.15, b - 0.18, c + 0.15, d + 0.18) for (a, b, c, d) in SITES.values()],
         sea_bg=True, land_hires=True,
         classes=[VM_LANDMASS, VM_WATER, VM_MOTORWAY, VM_TRUNK, VM_RAIL],
         min_area_px=8.0, min_len_px=1.0,
         desc="飛行エリア周辺・中精細"),
    dict(lod=2, min_scale=1.6, unit_e7=500, tile_e7=5_000_000, tol_scale=3.2706218,
         bboxes=[(30.00, 128.00, 42.00, 143.00)], sea_bg=True, land_hires=False,
         classes=[VM_LANDMASS, VM_WATER, VM_MOTORWAY, VM_RAIL],
         min_area_px=6.0, min_len_px=1.0,
         desc="本州・四国・九州北部"),
    # scalelist の 0.2044 と 0.8177 の 2 段を兼ねるため、幾何は 0.8177 基準で細かく、
    # 島の足切りは 0.2044 基準で粗く、と別々に指定する。
    dict(lod=3, min_scale=0.0, unit_e7=2000, tile_e7=20_000_000,
         tol_scale=0.81765545, cull_scale=0.2044138625,
         # 水面を入れないと、全国ズームで琵琶湖が消えてしまう（本機の用途上ここは必須）。
         bboxes=[(24.00, 122.00, 46.50, 146.50)], sea_bg=True, land_hires=False,
         classes=[VM_LANDMASS, VM_WATER],
         min_area_px=1.5, min_len_px=1.0,
         desc="日本全国・粗"),
]

# Overpass のタグ条件（クラス → クエリ断片）
OVERPASS_FILTERS = {
    VM_WATER:    ['way["natural"="water"]', 'relation["natural"="water"]'],
    VM_MOTORWAY: ['way["highway"~"^(motorway|motorway_link)$"]'],
    VM_TRUNK:    ['way["highway"~"^(trunk|primary)$"]'],
    VM_RAIL:     ['way["railway"="rail"]["service"!~"."]'],
}

# 公開 Overpass は混雑すると 504/429 を返す。複数ミラーを順に試す。
OVERPASS_URLS = [
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://overpass.osm.ch/api/interpreter",
    "https://overpass.private.coffee/api/interpreter",
]
# Overpass は requests の既定 User-Agent（python-requests/...）を 406 で弾くため、
# 用途と連絡先が分かる UA を必ず送る（Overpass の利用マナーでもある）。
USER_AGENT = "PONSv6-vectormap-builder/1.0 (https://github.com/MasaoC/GPS_TFT_map)"
KM_PER_DEG = 111.321


# ------------------------------------------------------------
# Overpass からの取得
# ------------------------------------------------------------
def overpass_query(q, cache_dir, tag, allow_giveup):
    """Overpass にクエリを投げる。結果はキャッシュしてリトライ時の再取得を避ける。

    戻り値: 成功なら JSON。allow_giveup=True で「応答が大きすぎる」系の失敗が
            続いた場合は None を返す（呼び出し側が bbox を分割して再帰する）。

    429（レート制限）はサイズの問題ではないので分割せず、待って同じクエリを再試行する。
    504/500 は応答過大かサーバ側の処理打ち切りなので、分割した方が確実に速い。
    """
    import requests
    cache_dir.mkdir(parents=True, exist_ok=True)
    key = hashlib.sha1(q.encode()).hexdigest()[:16]
    cache = cache_dir / f"{tag}_{key}.json"
    if cache.exists():
        return json.loads(cache.read_text())

    last = None
    for attempt in range(3):
        too_big = False
        rate_limited = False
        for url in OVERPASS_URLS:
            host = url.split("/")[2]
            try:
                r = requests.post(url, data={"data": q},
                                  headers={"User-Agent": USER_AGENT}, timeout=900)
            except Exception as e:
                last = type(e).__name__
                too_big = True   # タイムアウトも「重すぎる」と見なして分割対象にする
                continue
            if r.status_code == 200:
                cache.write_text(r.text)
                return r.json()
            last = f"HTTP {r.status_code}"
            if r.status_code == 429:
                rate_limited = True
            else:
                too_big = True
        if too_big and allow_giveup and not rate_limited:
            return None          # 呼び出し側で 4 分割して再帰する
        wait = 30 * (attempt + 1)
        print(f"      全ミラー失敗({last})。{wait}s 待機して再試行")
        time.sleep(wait)
    if allow_giveup:
        return None
    raise RuntimeError(f"Overpass への問い合わせに失敗しました ({last})。"
                       f"時間をおいて再実行してください（キャッシュ済み分は再取得されません）。")


def stitch_rings(ways):
    """端点を共有する way を繋いで閉じたリングにする。

    OSM のマルチポリゴン relation は外周が複数の way に分割されているため、
    そのままでは面にならない。端点をたどって環に組み立てる。
    """
    segs = [list(w) for w in ways if len(w) >= 2]
    rings = []
    while segs:
        cur = segs.pop(0)
        changed = True
        while changed and cur[0] != cur[-1]:
            changed = False
            for i, s in enumerate(segs):
                if s[0] == cur[-1]:
                    cur += s[1:]; segs.pop(i); changed = True; break
                if s[-1] == cur[-1]:
                    cur += s[-2::-1]; segs.pop(i); changed = True; break
                if s[-1] == cur[0]:
                    cur = s[:-1] + cur; segs.pop(i); changed = True; break
                if s[0] == cur[0]:
                    cur = s[::-1][:-1] + cur; segs.pop(i); changed = True; break
        if cur[0] == cur[-1] and len(cur) >= 4:
            rings.append(cur)
    return rings


def geoms_from_overpass(data, want_area):
    """Overpass の JSON から shapely ジオメトリを組み立てる。"""
    geoms = []
    for el in data.get("elements", []):
        if el["type"] == "way" and "geometry" in el:
            pts = [(p["lon"], p["lat"]) for p in el["geometry"]]
            if len(pts) < 2:
                continue
            if want_area:
                if pts[0] != pts[-1]:
                    pts.append(pts[0])
                if len(pts) >= 4:
                    p = Polygon(pts)
                    if p.is_valid or (p := p.buffer(0)).is_valid:
                        geoms.append(p)
            else:
                geoms.append(LineString(pts))
        elif el["type"] == "relation" and want_area:
            outer, inner = [], []
            for m in el.get("members", []):
                if "geometry" not in m:
                    continue
                pts = [(p["lon"], p["lat"]) for p in m["geometry"]]
                (inner if m.get("role") == "inner" else outer).append(pts)
            for ring in stitch_rings(outer):
                if len(ring) < 4:
                    continue
                holes = [h for h in stitch_rings(inner) if len(h) >= 4]
                try:
                    p = Polygon(ring, holes)
                    if not p.is_valid:
                        p = p.buffer(0)
                    if p.is_valid and not p.is_empty:
                        geoms.append(p)
                except Exception:
                    pass
    return geoms


# bbox 分割の最大深さ。0.5 段階で 4 分割していくので 5 なら 32x32 相当まで細かくなる。
MAX_SPLIT_DEPTH = 5


def fetch_class_overpass(cls, bbox, cache_dir, depth=0, _stat=None):
    """1 クラス分のジオメトリを Overpass から取得する。

    まず bbox 全体で問い合わせ、応答が大きすぎて 504 になった場合だけ 4 分割して
    再帰する。分割数を固定にすると、海上や山間部のような何も無い区画にまで
    無駄なクエリを投げることになり（LOD2 で 286 区画 x 3 クラス = 858 クエリ）、
    公開 Overpass への負荷も所要時間も現実的でなくなる。
    """
    top = _stat is None
    if top:
        _stat = {"n": 0, "split": 0}
    lat0, lon0, lat1, lon1 = bbox
    filters = OVERPASS_FILTERS[cls]
    body = "".join(f"{f}({lat0:.4f},{lon0:.4f},{lat1:.4f},{lon1:.4f});" for f in filters)
    q = f"[out:json][timeout:600];({body});out geom;"
    tag = f"c{cls}_{lat0:.3f}_{lon0:.3f}_{lat1:.3f}_{lon1:.3f}"

    data = overpass_query(q, cache_dir, tag, allow_giveup=(depth < MAX_SPLIT_DEPTH))
    if data is not None:
        _stat["n"] += 1
        geoms = geoms_from_overpass(data, IS_AREA(cls))
        if top:
            print(f"    問い合わせ {_stat['n']} 回 / 分割 {_stat['split']} 回")
        return geoms

    # 応答過大。4 分割して再帰する（キャッシュは分割後の bbox 単位で効く）
    _stat["split"] += 1
    mlat = (lat0 + lat1) / 2.0
    mlon = (lon0 + lon1) / 2.0
    quads = [(lat0, lon0, mlat, mlon), (lat0, mlon, mlat, lon1),
             (mlat, lon0, lat1, mlon), (mlat, mlon, lat1, lon1)]
    geoms = []
    for qb in quads:
        geoms += fetch_class_overpass(cls, qb, cache_dir, depth + 1, _stat)
    if top:
        print(f"    問い合わせ {_stat['n']} 回 / 分割 {_stat['split']} 回")
    return geoms


# ------------------------------------------------------------
# .osm.pbf からの取得（推奨）
# ------------------------------------------------------------
# Geofabrik の日付付きスナップショットを 1 回ダウンロードしてローカル処理する。
# Overpass と違い API 負荷ゼロ・レート制限なしで、日付を固定すればビルドが再現する。
# 2.3GB の走査に数分かかるが、全 LOD 分を 1 回の走査でまかなえる。
PBF_TAGS = {
    VM_WATER:    ("area", "natural", {"water"}),
    VM_MOTORWAY: ("way", "highway", {"motorway", "motorway_link"}),
    VM_TRUNK:    ("way", "highway", {"trunk", "primary"}),
    VM_RAIL:     ("way", "railway", {"rail"}),
}

# pbf 走査中に捨てる水面の下限 [平方度]。どの LOD の min_area_px よりも小さく
# しておくこと（ここで落とすと後段の LOD 別しきい値では戻せない）。
# 1e-4 平方度 ≒ 1 km^2 ではなく、ここでは 10,000 m^2 相当を目安にする。
PBF_MIN_WATER_M2 = 10000.0


def fetch_all_from_pbf(pbf_path, classes, cache_path=None):
    """pbf を 1 回走査して、必要な全クラスのジオメトリをまとめて取り出す。

    クラスごとに開き直すと 2.3GB を何度も読むことになるため、1 回で全部集める。
    """
    try:
        import osmium
        from shapely import wkb as shapely_wkb
    except ImportError:
        sys.exit("--pbf には pyosmium が必要です:  pip install osmium")

    # 走査は 2.5GB で 10 分以上かかるため、結果を WKB でキャッシュする。
    # LOD の閾値だけ変えて作り直したいときに再走査しなくて済む。
    if cache_path and Path(cache_path).exists():
        import pickle
        from shapely import wkb as shapely_wkb
        print(f"    走査キャッシュを使用: {cache_path}")
        raw = pickle.loads(Path(cache_path).read_bytes())
        if set(raw.keys()) >= set(classes):
            return {c: [shapely_wkb.loads(b) for b in raw[c]] for c in classes}
        print("    キャッシュのクラス構成が違うため再走査します")

    want_area = {c for c in classes if PBF_TAGS.get(c, ("",))[0] == "area"}
    want_way = {c for c in classes if PBF_TAGS.get(c, ("",))[0] == "way"}
    out = {c: [] for c in classes}
    if not want_area and not want_way:
        return out

    wkbfab = osmium.geom.WKBFactory()
    # 面の組み立て（マルチポリゴン relation の解決）は with_areas() が行う。
    # Overpass 版で自前実装した stitch_rings 相当を osmium がやってくれる。
    fp = osmium.FileProcessor(pbf_path).with_areas().with_locations()

    # 緯度経度 1 度あたりの面積は緯度で変わるので、日本の中心付近で概算する
    m2_per_deg2 = (111_321.0 ** 2) * math.cos(math.radians(36.0))
    min_area_deg2 = PBF_MIN_WATER_M2 / m2_per_deg2

    n_seen = 0
    for obj in fp:
        n_seen += 1
        if n_seen % 20_000_000 == 0:
            print(f"    ... {n_seen // 1_000_000}M オブジェクト走査", flush=True)
        tags = obj.tags
        if obj.is_area():
            for cls in want_area:
                _, key, vals = PBF_TAGS[cls]
                if tags.get(key) in vals:
                    try:
                        g = shapely_wkb.loads(wkbfab.create_multipolygon(obj), hex=True)
                    except Exception:
                        break
                    if not g.is_valid:
                        g = g.buffer(0)
                    if g.is_valid and not g.is_empty and g.area >= min_area_deg2:
                        out[cls].append(g)
                    break
        if isinstance(obj, osmium.osm.Way):
            for cls in want_way:
                _, key, vals = PBF_TAGS[cls]
                if tags.get(key) in vals:
                    if cls == VM_RAIL and tags.get("service"):
                        break   # 側線・引込線は除く（Overpass 版の service!~"." と同じ）
                    try:
                        g = shapely_wkb.loads(wkbfab.create_linestring(obj), hex=True)
                    except Exception:
                        break
                    if not g.is_empty:
                        out[cls].append(g)
                    break

    print(f"    走査完了 {n_seen:,} オブジェクト")
    for c in classes:
        print(f"      {CLASS_NAME[c]}: {len(out[c])} 個")
    if cache_path:
        import pickle
        from shapely import wkb as shapely_wkb
        Path(cache_path).write_bytes(
            pickle.dumps({c: [shapely_wkb.dumps(g) for g in out[c]] for c in classes}))
        print(f"    走査結果を {cache_path} にキャッシュしました")
    return out


def fetch_landmass_shp(shp_path, bbox):
    """osmdata.openstreetmap.de の land polygons（Shapefile）から陸地を取得する。

    OSM の natural=coastline は way が細切れで自力で閉じるのが非常に面倒なため、
    結合・分割済みのこのデータセットを使う。ライセンスは同じく ODbL。

    低ズーム用途なら simplified-land-polygons-complete-3857（23MB）で十分。
    こちらは EPSG:3857（Web メルカトル）なので WGS84 に再投影する。
    """
    try:
        import fiona
    except ImportError:
        sys.exit("--land-shp には fiona が必要です:  pip install fiona")
    lat0, lon0, lat1, lon1 = bbox

    with fiona.open(shp_path) as src:
        epsg = None
        try:
            epsg = src.crs.to_epsg() if hasattr(src.crs, "to_epsg") else src.crs.get("init", "")
        except Exception:
            pass
        need_reproject = str(epsg).endswith("3857")

        if need_reproject:
            from pyproj import Transformer
            from shapely.ops import transform as shp_transform
            fwd = Transformer.from_crs("EPSG:4326", "EPSG:3857", always_xy=True)
            inv = Transformer.from_crs("EPSG:3857", "EPSG:4326", always_xy=True)
            x0, y0 = fwd.transform(lon0, lat0)
            x1, y1 = fwd.transform(lon1, lat1)
            flt = (x0, y0, x1, y1)
        else:
            flt = (lon0, lat0, lon1, lat1)

        geoms = []
        for feat in src.filter(bbox=flt):
            g = shape(feat["geometry"])
            if need_reproject:
                g = shp_transform(lambda xs, ys: inv.transform(xs, ys), g)
            if not g.is_valid:
                g = g.buffer(0)
            if g.is_valid and not g.is_empty:
                geoms.append(g)
    return geoms


# ------------------------------------------------------------
# 簡略化・タイル分割・量子化
# ------------------------------------------------------------
def merge_lines(geoms):
    """繋がっている線を 1 本にまとめる。

    OSM の道路・鉄道は交差点やタグの変化のたびに way が分割されているため、
    way 単位で長さを見ると「1 本 200m の高速道路」が大量にできてしまい、
    長さによる足切りが正しく効かない。先に連結してから足切りする。
    """
    lines = [g for g in geoms if isinstance(g, (LineString, MultiLineString)) and not g.is_empty]
    if not lines:
        return []
    merged = linemerge(unary_union(lines))
    return list(iter_lines(merged))


def drop_tiny(geoms, want_area, min_area_deg2, min_len_deg):
    """画面上で数ピクセルにしかならない地物を捨てる。

    数十メートルの池や枝道は、描いても 1〜2px にしかならないのに
    フラッシュと描画時間を食う。LOD の考え方として間引くのが正しい。
    """
    out = []
    for g in geoms:
        if want_area:
            if g.area >= min_area_deg2:
                out.append(g)
        else:
            if g.length >= min_len_deg:
                out.append(g)
    return out


def count_points(g):
    """診断表示用の点数。Polygon / MultiPolygon / LineString / MultiLineString に対応。

    pbf 版の面は MultiPolygon で返るため、Polygon だけを数えると 0 になって
    「簡略化が効いていない」ように見えてしまう。
    """
    if g.is_empty:
        return 0
    t = g.geom_type
    if t == "Polygon":
        return len(g.exterior.coords) + sum(len(h.coords) for h in g.interiors)
    if t == "LineString":
        return len(g.coords)
    if hasattr(g, "geoms"):
        return sum(count_points(x) for x in g.geoms)
    return 0


def simplify_geoms(geoms, tol_deg, want_area):
    """Douglas-Peucker で点を間引く。

    ズームアウト時は細かい点が 1px 未満に潰れて無駄なので、LOD ごとに
    その縮尺での 1px 相当を許容誤差として間引く。
    """
    out = []
    for g in geoms:
        s = g.simplify(tol_deg, preserve_topology=True)
        if s.is_empty:
            continue
        if want_area and not s.is_valid:
            s = s.buffer(0)
            if s.is_empty:
                continue
        out.append(s)
    return out


def iter_polygons(g):
    if g.is_empty:
        return
    if isinstance(g, Polygon):
        yield g
    elif isinstance(g, MultiPolygon):
        for p in g.geoms:
            yield p
    elif hasattr(g, "geoms"):
        for p in g.geoms:
            if isinstance(p, (Polygon, MultiPolygon)):
                yield from iter_polygons(p)


def iter_lines(g):
    if g.is_empty:
        return
    if isinstance(g, LineString):
        yield g
    elif isinstance(g, MultiLineString):
        for l in g.geoms:
            yield l
    elif hasattr(g, "geoms"):
        for l in g.geoms:
            if isinstance(l, (LineString, MultiLineString)):
                yield from iter_lines(l)


def quantize_ring(coords, org_lat_e7, org_lon_e7, unit_e7):
    """緯度経度列をタイル原点相対の int16 に量子化し、連続する重複点を除く。"""
    out = []
    for lon, lat in coords:
        dlat = int(round((lat * 1e7 - org_lat_e7) / unit_e7))
        dlon = int(round((lon * 1e7 - org_lon_e7) / unit_e7))
        dlat = max(-32768, min(32767, dlat))
        dlon = max(-32768, min(32767, dlon))
        if out and out[-1] == (dlat, dlon):
            continue
        out.append((dlat, dlon))
    return out


DROPPED = {"area": 0, "line": 0}   # 点数上限で捨てたレコード数（無言の欠落を出さないため記録する）

# vectormap.cpp の VM_MAX_RINGS と揃えること
MAX_RINGS_PER_REC = 64


def build_records(cls, geoms, org_lat_e7, org_lon_e7, unit_e7, max_pts):
    """1 タイル分のジオメトリを、firmware が読むレコードのバイト列に変換する。"""
    recs = []
    if IS_AREA(cls):
        for poly in geoms:
            # 外周と穴（島）を 1 レコードにまとめる。firmware 側は even-odd で塗るので
            # 穴のリングは自動的に抜ける（巻き方向を揃える必要はない）。
            rings = [quantize_ring(poly.exterior.coords, org_lat_e7, org_lon_e7, unit_e7)]
            for h in poly.interiors:
                rings.append(quantize_ring(h.coords, org_lat_e7, org_lon_e7, unit_e7))
            rings = [r for r in rings if len(r) >= 3]
            if not rings:
                continue
            if sum(len(r) for r in rings) > max_pts:
                # firmware の点バッファ（VM_MAX_PTS）を超えるものは描けないので捨てる。
                # 発生した場合は簡略化不足かタイルが大きすぎる。海岸線に穴が空くため要調整。
                DROPPED["area"] += 1
                continue
            recs.append((cls, rings))
    else:
        # 線は「1 レコードに複数の折れ線」としてまとめる。
        # レコードヘッダは 10 バイト + 2 バイト/リングあり、3 点程度の短い道路を
        # 1 本ずつレコードにすると本体より大きくなってしまうため、
        # 同一タイル・同一クラスの線をまとめてヘッダを償却する。
        batch = []
        batch_pts = 0
        for line in geoms:
            r = quantize_ring(line.coords, org_lat_e7, org_lon_e7, unit_e7)
            if len(r) > max_pts:
                DROPPED["line"] += 1
                continue
            if len(r) < 2:
                continue
            # firmware 側の VM_MAX_PTS / VM_MAX_RINGS を超えないように区切る
            if batch and (batch_pts + len(r) > max_pts or len(batch) >= MAX_RINGS_PER_REC):
                recs.append((cls, batch))
                batch, batch_pts = [], 0
            batch.append(r)
            batch_pts += len(r)
        if batch:
            recs.append((cls, batch))
    return recs


def pack_record(cls, rings):
    """レコードをバイト列にする。vectormap.h のレコード形式と一致させること。"""
    all_pts = [p for r in rings for p in r]
    lats = [p[0] for p in all_pts]
    lons = [p[1] for p in all_pts]
    b = struct.pack("<BBhhhh", cls, len(rings), min(lats), min(lons), max(lats), max(lons))
    b += struct.pack(f"<{len(rings)}H", *[len(r) for r in rings])
    for r in rings:
        for dlat, dlon in r:
            b += struct.pack("<hh", dlat, dlon)
    return b


# ------------------------------------------------------------
# メイン
# ------------------------------------------------------------
def build_lod(spec, land_shp, land_shp_hires, cache_dir, max_pts, stats, pbf_cache=None):
    """1 LOD 分のタイルを作る。戻り値: [(org_lat_e7, org_lon_e7, blob_bytes), ...]"""
    lod = spec["lod"]
    bboxes = spec["bboxes"]
    tol_km = 1.0 / spec["tol_scale"]        # その縮尺での 1px 相当 [km]
    tol_deg = tol_km / KM_PER_DEG
    # 足切り閾値を度・度^2 に換算する（経度は緯度で縮むので全 bbox の中心緯度で補正）
    mid_lat = sum((b[0] + b[2]) / 2.0 for b in bboxes) / len(bboxes)
    km2_per_deg2 = KM_PER_DEG * KM_PER_DEG * math.cos(math.radians(mid_lat))
    cull_scale = spec.get("cull_scale", spec["tol_scale"])
    min_len_deg = (spec["min_len_px"] / cull_scale) / KM_PER_DEG
    min_area_deg2 = ((spec["min_area_px"] / cull_scale) ** 2) / km2_per_deg2
    print(f"\n=== LOD{lod} ({spec['desc']}) ===")
    print(f"  エリア {len(bboxes)} 箇所 / 簡略化許容誤差 {tol_km * 1000:.0f} m")
    print(f"  足切り: 面 {spec['min_area_px']:.0f}px 角未満 / 線 {spec['min_len_px']:.0f}px 未満"
          f"（基準 scale {cull_scale:g}）")

    # ---- クラスごとにジオメトリを集める ----
    per_class = {}
    for cls in spec["classes"]:
        print(f"  [{CLASS_NAME[cls]}] 取得中")
        if cls == VM_LANDMASS:
            shp = land_shp_hires if spec.get("land_hires") else land_shp
            if not shp:
                which = "--land-shp-hires" if spec.get("land_hires") else "--land-shp"
                print(f"    {which} 未指定のためスキップ（海岸線なし＝海と陸が区別できません）")
                continue
            geoms = []
            for bb in bboxes:
                geoms += fetch_landmass_shp(shp, bb)
        elif pbf_cache is not None:
            # pbf は全国分を 1 回で読んであるので、この LOD の範囲で切り出すだけ
            areas = [box(b[1], b[0], b[3], b[2]) for b in bboxes]
            geoms = [g for g in pbf_cache.get(cls, [])
                     if any(g.intersects(a) for a in areas)]
        else:
            geoms = []
            for bb in bboxes:
                geoms += fetch_class_overpass(cls, bb, cache_dir)
        n_raw = len(geoms)
        n_before = sum(count_points(g) for g in geoms)
        if not IS_AREA(cls):
            geoms = merge_lines(geoms)   # 足切りの前に連結する（way 分割の影響を消す）
        n_merged = len(geoms)
        geoms = drop_tiny(geoms, IS_AREA(cls), min_area_deg2, min_len_deg)
        geoms = simplify_geoms(geoms, tol_deg, IS_AREA(cls))
        n_after = sum(count_points(g) for g in geoms)
        print(f"    {n_raw} 個 → 連結 {n_merged} 個 → 足切り後 {len(geoms)} 個 / 点 {n_before} → {n_after}")
        per_class[cls] = geoms
        stats.setdefault(CLASS_NAME[cls], 0)

    # ---- タイルに切って量子化 ----
    tsize = spec["tile_e7"]
    unit = spec["unit_e7"]
    tiles = []
    # 複数エリアが重なっても同じタイルを二度出さないよう、格子番号を集合にまとめる
    cells = set()
    for (a0, o0, a1, o1) in bboxes:
        for iy in range(math.floor(a0 * 1e7 / tsize), math.ceil(a1 * 1e7 / tsize)):
            for ix in range(math.floor(o0 * 1e7 / tsize), math.ceil(o1 * 1e7 / tsize)):
                cells.add((iy, ix))
    print(f"  タイル格子 {len(cells)} 区画を走査")

    for (iy, ix) in sorted(cells):
        if True:
            org_lat_e7 = iy * tsize
            org_lon_e7 = ix * tsize
            tb = box(org_lon_e7 / 1e7, org_lat_e7 / 1e7,
                     (org_lon_e7 + tsize) / 1e7, (org_lat_e7 + tsize) / 1e7)
            recs = []
            for cls in spec["classes"]:
                # このタイルに掛かる当該クラスの断片をすべて集めてから 1 回で
                # レコード化する。線はここでまとめられ、10 バイトのレコードヘッダが
                # 複数本で償却される（1 本ずつ渡すと償却が効かない）。
                parts = []
                for g in per_class.get(cls, []):
                    if not g.intersects(tb):
                        continue
                    clipped = g.intersection(tb)
                    if clipped.is_empty:
                        continue
                    parts += list(iter_polygons(clipped)) if IS_AREA(cls) else list(iter_lines(clipped))
                if parts:
                    recs += build_records(cls, parts, org_lat_e7, org_lon_e7, unit, max_pts)
            if not recs:
                continue
            # クラス順に並べておく（firmware 側はクラス別に 6 パス走査する）
            recs.sort(key=lambda r: r[0])
            blob = b"".join(pack_record(c, rings) for c, rings in recs)
            for c, rings in recs:
                stats[CLASS_NAME[c]] = stats.get(CLASS_NAME[c], 0) + sum(len(r) for r in rings)
            tiles.append((org_lat_e7, org_lon_e7, blob))
    print(f"  → データのあるタイル {len(tiles)} 個 / {sum(len(t[2]) for t in tiles) / 1024:.1f} KB")
    if DROPPED["area"] or DROPPED["line"]:
        print(f"  !! 点数上限({max_pts})超過で捨てたレコード: 面 {DROPPED['area']} / 線 {DROPPED['line']}")
        print(f"     地図に欠落が出ます。tol_scale を粗くするか tile_e7 を小さくしてください。")
    DROPPED["area"] = DROPPED["line"] = 0
    return tiles


def emit_header(out_path, all_tiles, source_date, args, stats):
    """vectormap_data.cpp を書き出す。"""
    blob = bytearray()
    entries = []
    for lod, tiles in sorted(all_tiles.items()):
        for org_lat_e7, org_lon_e7, b in tiles:
            assert len(blob) % 2 == 0, "レコードは 2 バイト境界に揃っている必要がある"
            entries.append((org_lat_e7, org_lon_e7, len(blob), len(b), lod))
            blob += b

    # LOD 別・クラス別の内訳（MAPLIST 画面で表示する）
    lod_tiles = [0] * len(LODS)
    lod_bytes = [0] * len(LODS)
    for lod, tiles in all_tiles.items():
        lod_tiles[lod] = len(tiles)
        lod_bytes[lod] = sum(len(t[2]) for t in tiles)

    lines = []
    w = lines.append
    w("// ============================================================")
    w("// vectormap_data.cpp — 自動生成ファイル。手で編集しないこと。")
    w("// 生成: tools/vectormap/build_vectormap.py")
    w("//")
    w("// ------------------------------------------------------------")
    w("// Map data (c) OpenStreetMap contributors")
    w("// Licensed under the Open Database License (ODbL) v1.0")
    w("// https://www.openstreetmap.org/copyright")
    w("// https://opendatacommons.org/licenses/odbl/1-0/")
    w("//")
    w("// このファイルは ODbL の Derivative Database に該当する。再配布時は")
    w("// ODbL のままとし、帰属表示を保持すること（ファームウェアのコード本体は MIT）。")
    w(f"// 元データのスナップショット: {source_date}")
    w("// ------------------------------------------------------------")
    w("// ============================================================")
    w('#include "vectormap.h"')
    w("")
    w(f"const uint16_t vm_tile_count = {len(entries)};")
    w("")
    w("const int32_t vm_lod_unit_e7[VM_LOD_COUNT] = { " +
      ", ".join(str(l["unit_e7"]) for l in LODS) + " };")
    w("const int32_t vm_lod_tilesize_e7[VM_LOD_COUNT] = { " +
      ", ".join(str(l["tile_e7"]) for l in LODS) + " };")
    # 必ず小数点を含めること（"26f" は C の不正なリテラル）
    w("const float vm_lod_min_scale[VM_LOD_COUNT] = { " +
      ", ".join(f'{float(l["min_scale"]):.6f}f' for l in LODS) + " };")
    w("const bool vm_lod_sea_background[VM_LOD_COUNT] = { " +
      ", ".join("true" if l["sea_bg"] else "false" for l in LODS) + " };")
    w("")
    w(f'const char vm_data_attribution[] = "Map data (c) OpenStreetMap contributors (ODbL)";')
    w(f'const char vm_data_source_date[] = "{source_date}";')
    w("")
    w(f"const uint32_t vm_blob_size = {len(blob)};")
    w("const uint16_t vm_lod_tiles[VM_LOD_COUNT] = { " +
      ", ".join(str(n) for n in lod_tiles) + " };")
    w("const uint32_t vm_lod_bytes[VM_LOD_COUNT] = { " +
      ", ".join(str(n) for n in lod_bytes) + " };")
    w("const uint32_t vm_class_points[VM_CLASS_COUNT] = { " +
      ", ".join(str(stats.get(n, 0)) for n in CLASS_NAME) + " };")
    w("const char* const vm_class_names[VM_CLASS_COUNT] = { " +
      ", ".join(f'"{n}"' for n in CLASS_NAME) + " };")
    w("")
    w("const vm_tile vm_tiles[] = {")
    for lat, lon, off, ln, lod in entries:
        w(f"  {{ {lat}, {lon}, {off}, {ln}, {lod}, 0 }},")
    w("};")
    w("")
    w("// レコード内の int16 を整列アクセスできるよう 4 バイト境界に置く。")
    w("const uint8_t vm_blob[] __attribute__((aligned(4))) = {")
    for i in range(0, len(blob), 16):
        w("  " + ",".join(f"0x{b:02x}" for b in blob[i:i + 16]) + ",")
    w("};")
    w("")

    Path(out_path).write_text("\n".join(lines))
    return len(entries), len(blob)


def main():
    ap = argparse.ArgumentParser(description="OSM から vectormap_data.cpp を生成する")
    ap.add_argument("--out", default="../../vectormap_data.cpp",
                    help="出力先。配列の「定義」なので .cpp にすること（.h だと\n"
                         "どこからも include されずリンクエラーになる）")
    ap.add_argument("--lods", default="0,1,2,3", help="生成する LOD（カンマ区切り）")
    ap.add_argument("--land-shp", default=None,
                    help="低ズーム用の海岸線 shp（simplified-land-polygons-complete-3857 で十分）")
    ap.add_argument("--land-shp-hires", default=None,
                    help="高ズーム用の海岸線 shp（land-polygons-split-4326）。"
                         "沿岸の飛行エリアで海と陸を区別するのに必須")
    ap.add_argument("--pbf", default=None,
                    help="Geofabrik の .osm.pbf。指定すると Overpass ではなくこちらを使う"
                         "（API 負荷なし・再現性あり。推奨）")
    ap.add_argument("--pbf-scan-cache", default="./pbf_scan.pickle",
                    help="pbf 走査結果のキャッシュ先。閾値だけ変えて作り直すとき再走査を省ける")
    ap.add_argument("--cache", default="./cache", help="Overpass 応答のキャッシュ先")
    ap.add_argument("--max-pts", type=int, default=2048,
                    help="1 レコードの最大点数（vectormap.cpp の VM_MAX_PTS と揃える）")
    ap.add_argument("--source-date", default=None,
                    help="元データの日付。再現性のため記録する（既定: 本日）")
    args = ap.parse_args()

    source_date = args.source_date or time.strftime("%Y-%m-%d")
    want = {int(x) for x in args.lods.split(",") if x.strip() != ""}
    cache_dir = Path(args.cache)
    stats = {}

    # pbf を使う場合は、全 LOD で必要なクラスをまとめて 1 回だけ走査する
    pbf_cache = None
    if args.pbf:
        need = sorted({c for spec in LODS if spec["lod"] in want
                       for c in spec["classes"] if c != VM_LANDMASS})
        print(f"=== {args.pbf} を走査（クラス: {', '.join(CLASS_NAME[c] for c in need)}）===")
        t0 = time.time()
        pbf_cache = fetch_all_from_pbf(args.pbf, need, args.pbf_scan_cache)
        print(f"    所要 {time.time() - t0:.0f} 秒")

    all_tiles = {}
    for spec in LODS:
        if spec["lod"] not in want:
            continue
        all_tiles[spec["lod"]] = build_lod(spec, args.land_shp, args.land_shp_hires,
                                           cache_dir, args.max_pts, stats, pbf_cache)

    n_tiles, n_bytes = emit_header(args.out, all_tiles, source_date, args, stats)

    print("\n================ 生成結果 ================")
    print(f"  出力      : {args.out}")
    print(f"  タイル数  : {n_tiles}")
    print(f"  blob      : {n_bytes / 1024:.1f} KB")
    print(f"  tiles 表  : {n_tiles * 16 / 1024:.1f} KB")
    print(f"  フラッシュ合計 : {(n_bytes + n_tiles * 16) / 1024:.1f} KB")
    print("  クラス別の点数:")
    for k, v in sorted(stats.items(), key=lambda kv: -kv[1]):
        print(f"    {k:10s} {v:8d} 点  ({v * 4 / 1024:7.1f} KB)")
    print("\n  ライセンス: Map data (c) OpenStreetMap contributors, ODbL 1.0")
    print("  vectormap_data.cpp は ODbL で配布すること。")


if __name__ == "__main__":
    main()
