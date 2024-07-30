import csv
from lxml import etree

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
        name = placemark.find('.//kml:name', namespaces=ns).text.strip()
        coords_text = placemark.find('.//kml:coordinates', namespaces=ns).text.strip()
        coords_pairs = coords_text.split()

        coordinates = []
        for pair in coords_pairs:
            lon, lat, _ = map(float, pair.split(','))
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
