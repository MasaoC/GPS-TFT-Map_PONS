from pykml.factory import KML_ElementMaker as KML
from pykml.factory import GX_ElementMaker as GX
from lxml import etree
import pandas as pd
import zipfile
import os

# Read the CSV file
# csv_file = "PONSv5_planelog_2025-07-26_1539.csv"
csv_file = "PONSv4_boatlog_2025-07-26_1530.csv"
df = pd.read_csv(csv_file)

# Create a KML document
kml = KML.kml(
    KML.Document(
        KML.name("FlightPathTour_Boat"),
        # Define a style for smaller, simpler placemark icons
        KML.Style(
            KML.IconStyle(
                KML.scale(0.3),  # Smaller icon size
                KML.Icon(
                    KML.href("http://maps.google.com/mapfiles/kml/pushpin/ylw-pushpin.png")  # Simple yellow pushpin
                )
            ),
            id="simple_icon"
        )
    )
)

# Create a gx:Tour element
tour = GX.Tour(
    KML.name("FlightPathTour"),
    GX.Playlist()
)
kml.Document.append(tour)

# Parameters for camera
camera_altitude = 50  # meters for camera
tilt = 70  # degrees
range = 50  # distance from the point in meters, adjust as needed

# Iterate through each row in the CSV
for index, row in df.iterrows():
    lat = row['latitude']
    lon = row['longitude']
    heading = row['TrueTrack']
    date = row['date']
    time = row['time']
    gs = row['gs']
    
    # Create a placemark for each point on the ground (altitude 0)
    placemark = KML.Placemark(
        KML.name(f"GS{gs}m/s {time}"),
        KML.styleUrl("#simple_icon"),  # Reference the defined style
        KML.Point(
            KML.coordinates(f"{lon},{lat},0"),
            KML.altitudeMode("relativeToGround")
        )
    )
    kml.Document.append(placemark)
    
    # Create a gx:FlyTo element for the tour with camera at 100m
    flyto = GX.FlyTo(
        GX.duration(1.0),  # Duration in seconds for each point, adjust as needed
        GX.flyToMode("smooth"),
        KML.LookAt(
            KML.longitude(lon),
            KML.latitude(lat),
            KML.altitude(camera_altitude),
            KML.heading(heading),
            KML.tilt(tilt),
            KML.range(range),
            KML.altitudeMode("relativeToGround"),
            GX.timeStamp(
                KML.when(f"{date}T{time}Z")
            )
        )
    )
    tour.Playlist.append(flyto)

# Save the KML file
kml_file = "tour_2.kml"
with open(kml_file, 'wb') as f:
    f.write(etree.tostring(kml, pretty_print=True))

# Create a KMZ file by zipping the KML
kmz_file = "tour_2.kmz"
with zipfile.ZipFile(kmz_file, 'w', compression=zipfile.ZIP_DEFLATED) as zf:
    zf.write(kml_file, os.path.basename(kml_file))

# Clean up the temporary KML file
os.remove(kml_file)

print(f"KMZ file '{kmz_file}' has been created successfully.")