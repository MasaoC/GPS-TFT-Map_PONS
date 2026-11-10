import csv
from lxml import etree


# This file creates map data csv from Google earth exported KML file.

def parse_kml(kml_file):
    # Parse the KML file
    tree = etree.parse(kml_file)
    root = tree.getroot()

    # Namespaces for KML
    ns = {'kml': 'http://www.opengis.net/kml/2.2'}

    # Extract coordinates from the KML
    placemarks = root.findall('.//kml:Placemark', namespaces=ns)
    map_data = []

    for placemark in placemarks:
        name_el = placemark.find('.//kml:name', namespaces=ns)
        coords_el = placemark.find('.//kml:coordinates', namespaces=ns)
        if coords_el is None or not (coords_el.text or '').strip():
            continue   # 座標の無い Placemark（フォルダの説明など）は飛ばす
        name = (name_el.text or '').strip() if name_el is not None else 'noname'
        coords_pairs = coords_el.text.strip().split()

        coordinates = []
        for pair in coords_pairs:
            # KML の座標は "lon,lat" か "lon,lat,alt"。高さの無い 2 要素でも読む（3 要素決め打ちだと ValueError）
            parts = pair.split(',')
            lon, lat = float(parts[0]), float(parts[1])
            coordinates.append((lon, lat))

        map_data.append((name, coordinates))

    return map_data

def write_csv(map_data, output_file):
    with open(output_file, 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        for name, coords in map_data:
            row = [name, len(coords)] + [item for sublist in coords for item in sublist]
            csvwriter.writerow(row)

def kml_to_csv(kml_file, csv_file):
    map_data = parse_kml(kml_file)
    write_csv(map_data, csv_file)

if __name__ == "__main__":
    kml_file = input("Enter the path to your KML file: ")
    csv_file = 'mapdata.csv'
    kml_to_csv(kml_file, csv_file)
    print(f"Conversion complete. The data has been saved to {csv_file}.")
