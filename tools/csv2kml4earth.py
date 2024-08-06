import os
import sys
import pandas as pd
import simplekml
from datetime import datetime, timedelta
import random

def generate_random_color():
    r = lambda: random.randint(0, 255)
    return simplekml.Color.rgb(r(), r(), r())

def csv_to_kml(input_folder):
    kml = simplekml.Kml()
    # Iterate through all CSV files in the input folder
    for csv_file in os.listdir(input_folder):
        if csv_file.endswith(".csv"):
            # Read the CSV file
            file_path = os.path.join(input_folder, csv_file)
            df = pd.read_csv(file_path)
            
            # Create a new linestring for each file
            linestring = kml.newlinestring(name=csv_file)
            linestring.coords = list(zip(df['longitude'], df['latitude']))
            
            # Generate a random color for the path and label
            path_color = generate_random_color()
            linestring.style.linestyle.color = path_color
            linestring.style.linestyle.width = 2  # Adjust the width as needed
            
            # Add time-based placemarks for animation
            for i, row in df.iterrows():
                point = kml.newpoint(
                    coords=[(row['longitude'], row['latitude'])],
                    description=f"{row['date']} {row['time']}"
                )
                point.timestamp.when = f"{row['date']}T{row['time']}Z"
                point.style.iconstyle.color = path_color

            # Add a gx:Track element for the path
            track = kml.newgxtrack(name=csv_file)
            track.newwhen([f"{row['date']}T{row['time']}Z" for _, row in df.iterrows()])
            track.newgxcoord([(row['longitude'], row['latitude'], 0) for _, row in df.iterrows()])
            track.style.linestyle.color = path_color
            track.style.linestyle.width = 2

            # Label the first point with date and time
            first_point = (df.iloc[0]['longitude'], df.iloc[0]['latitude'])
            date_time = f"{df.iloc[0]['date']} {df.iloc[0]['time']}"
            point = kml.newpoint(coords=[first_point], description=date_time)
            point.style.labelstyle.scale = 1.5  # Adjust the label scale as needed
            point.style.labelstyle.color = path_color  # Match label color to path color

    # Generate the output filename with current date
    current_date = datetime.now().strftime("%Y-%m-%d")
    output_file = f"{current_date}_pons4earth.kml"

    # Save the KML file
    kml.save(output_file)
    print(f"KML file saved as: {output_file}")

# Check for input folder argument
if len(sys.argv) > 1:
    input_folder = sys.argv[1]
else:
    input_folder = './csv/'

# Convert CSV files to KML
csv_to_kml(input_folder)
