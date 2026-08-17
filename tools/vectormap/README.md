# 内蔵ベクタ地図の生成

`vectormap_data.cpp`（ファームウェアに焼き込む地図データ）を OpenStreetMap から生成する。

以前の `tools/mapimage/`（Google Maps API で BMP タイルを作り SD カードに置く方式）を
置き換えたもので、**API キー不要・SD カード不要**になっている。

## できあがるもの

| | |
|---|---|
| 出力 | `../../vectormap_data.cpp`（約 1.4 MB のバイト配列） |
| 収録範囲 | 飛行エリア 7 箇所を高精細＋本州広域＋日本全国 |
| 地物 | 海岸線・水面・高速道路・幹線道路・鉄道 |
| ライセンス | ODbL（`../../LICENSE.ODbL` を参照） |

## 準備

```sh
pip install shapely osmium fiona pyproj requests
```

入力データを 2 つ落とす（合計 3GB 強、一度落とせば使い回せる）。

```sh
# 1. OSM の日本抽出（道路・鉄道・水面）。日付を固定すること（latest だと結果が変わる）
curl -O https://download.geofabrik.de/asia/japan-260816.osm.pbf

# 2. 海岸線。高ズーム用（1.3GB）と低ズーム用（23MB）で精度が違うので両方要る
curl -O https://osmdata.openstreetmap.de/download/land-polygons-split-4326.zip
curl -O https://osmdata.openstreetmap.de/download/simplified-land-polygons-complete-3857.zip
unzip land-polygons-split-4326.zip
unzip simplified-land-polygons-complete-3857.zip
```

## 生成

```sh
python3 build_vectormap.py --lods 0,1,2,3 \
  --pbf japan-260816.osm.pbf \
  --land-shp simplified-land-polygons-complete-3857/simplified_land_polygons.shp \
  --land-shp-hires land-polygons-split-4326/land_polygons.shp \
  --source-date 2026-08-16 \
  --out ../../vectormap_data.cpp
```

pbf の走査に 10〜15 分かかるが、結果は `pbf_scan.pickle` にキャッシュされるので、
しきい値や収録範囲だけ変えて作り直すときは数分で終わる。

`--pbf` を省くと Overpass API から取得する（追加インストールが少なくて済むが、
広い範囲では時間がかかり公開サーバへの負荷も大きい。小範囲の試作向け）。

## 飛行エリアを追加する

`build_vectormap.py` の `SITES` に 1 行足すだけ。

```python
SITES = {
    "biwako": (34.95, 135.80, 35.60, 136.40),   # (lat_min, lon_min, lat_max, lon_max)
    ...
}
```

高ズーム（LOD0/1）はここに挙げた範囲だけを収録する。全国を高精細で持つと
フラッシュに入らないため、実際に飛ぶ場所を列挙する方式にしている。

追加したら `gps.cpp` の `DEMO_SITES` にも足すと、実機のデモ飛行で表示を確認できる。

## 容量を減らしたいとき

効く順に:

1. **LOD1 の周辺マージンを詰める** — 全体の半分以上を占める。`bboxes` の `± 0.35 / 0.45` を小さくする
2. **`VM_TRUNK` を外す** — `trunk` に加え `primary` も含むため点数が最多
3. **LOD2 の収録範囲を狭める**

生成時にクラス別・LOD 別の内訳が表示されるので、それを見て判断する。
同じ内訳は実機の MAPLIST 画面（設定 → Map detail）でも確認できる。

## パラメータの考え方

`tol_scale` と `cull_scale` を混同しないこと。1 つの LOD が複数のズーム段を
兼ねるとき、この 2 つは逆方向の要求を持つ。

- **`tol_scale`（簡略化の許容誤差）** … その LOD が担当する**最も細かいズーム**に合わせる。
  粗い側に合わせると、細かい側で海岸線がガタガタに見える。
- **`cull_scale`（面積・長さの足切り）** … 担当する**最も粗いズーム**に合わせる。
  細かい側に合わせると、広域表示では見えない小島まで大量に抱え込む。

## ライセンス

生成される `vectormap_data.cpp` は ODbL の Derivative Database に該当する。
再配布時は ODbL のままとし、帰属表示を保持すること。
詳細は `../../LICENSE.ODbL` を参照。
