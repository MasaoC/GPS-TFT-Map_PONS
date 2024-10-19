import requests
from PIL import Image
import numpy as np
import sys
import os
import math
import shutil


# This file will calculate the needed map images to cover specific area.
# Map images are then downloaded using static googlemap api.
# And then they are saved to png and converts them to valid bmp files, so they are ready to copied into a SD card.


# biwako
#center_latitude = 35.295109085322686
#center_longitude = 136.25322693133202
# tokyo
#center_latitude = 35.63413070650897
#center_longitude = 139.922110708556
#distance_radius = 10  # in kilometers, Especially important for zoom level 11 and 13.

center_latitude = float(input("Input center latitude (ex.35.634): "))
center_longitude = float(input("Input center longitude (ex.139.922):"))
distance_radius = int(input("Input range(radius) in km (ex.10): "))
key = input("Enter Google Map API Key: ")
mapid = "9820e68e0789f0b1"



def convert_png_bmp(inputfilename,outputfilename):
    def convert_to_rgb565(image):
        """Convert an image to 16-bit RGB565 format."""
        image = image.convert('RGB')
        np_image = np.array(image)
        
        # Convert RGB888 to RGB565
        r = (np_image[:, :, 0] >> 3).astype(np.uint16)
        g = (np_image[:, :, 1] >> 2).astype(np.uint16)
        b = (np_image[:, :, 2] >> 3).astype(np.uint16)
        
        rgb565 = (r << 11) | (g << 5) | b
        return rgb565


    def convert_color(image, target_color, replacement_color, tolerance):
        """Replace colors within a certain tolerance range around the target color with the replacement color."""
        image = image.convert('RGB')
        np_image = np.array(image)
        
        # Convert tolerance to array ranges
        target_color = np.array(target_color)
        tolerance = np.array([tolerance, tolerance, tolerance])
        
        lower_bound = np.clip(target_color - tolerance, 0, 255)
        upper_bound = np.clip(target_color + tolerance, 0, 255)
        
        # Create a mask for the target color within tolerance
        mask = np.all((np_image >= lower_bound) & (np_image <= upper_bound), axis=-1)
        
        # Replace target colors with the replacement color
        np_image[mask] = replacement_color
        
        # Convert back to PIL image
        return Image.fromarray(np_image)

    def save_bmp_rgb565(image, output_filename):
        # Convert image to RGB565 format and flip it vertically
        rgb565_data = convert_to_rgb565(image.transpose(Image.FLIP_TOP_BOTTOM))
        
        height, width = rgb565_data.shape
        padded_row_size = (width * 2 + 3) & ~3  # Ensure row size is a multiple of 4 bytes
        padding = padded_row_size - (width * 2)

        pixel_data = bytearray()
        for row in rgb565_data:
            pixel_data.extend(row.tobytes())
            pixel_data.extend(b'\x00' * padding)  # Add padding to each row

        file_header = bytearray([
            0x42, 0x4D,                             # 'BM' header
            0, 0, 0, 0,                             # File size (will be filled later)
            0, 0, 0, 0,                             # Reserved
            54 + 12, 0, 0, 0                        # Offset to pixel data (54 bytes for header + 12 bytes for masks)
        ])

        info_header = bytearray([
            40, 0, 0, 0,            # Header size (40 bytes)
            width & 0xFF, (width >> 8) & 0xFF, (width >> 16) & 0xFF, (width >> 24) & 0xFF,  # Image width (4 bytes)
            height & 0xFF, (height >> 8) & 0xFF, (height >> 16) & 0xFF, (height >> 24) & 0xFF,  # Image height (4 bytes)
            1, 0,                   # Planes (2 bytes)
            16, 0,                  # Bits per pixel (16 bits = 2 bytes per pixel)
            3, 0, 0, 0,             # Compression (3 = BI_BITFIELDS, to specify bit masks)
            len(pixel_data) & 0xFF, (len(pixel_data) >> 8) & 0xFF, (len(pixel_data) >> 16) & 0xFF, (len(pixel_data) >> 24) & 0xFF,  # Image size (4 bytes)
            0, 0, 0, 0,             # X pixels per meter (unused, 4 bytes)
            0, 0, 0, 0,             # Y pixels per meter (unused, 4 bytes)
            0, 0, 0, 0,             # Colors used (0 for no palette)
            0, 0, 0, 0              # Important colors (0 for no palette)
        ])

        # Bit masks for RGB565 format (5 bits for Red, 6 bits for Green, 5 bits for Blue)
        red_mask = bytearray([0x00, 0xF8, 0x00, 0x00])
        green_mask = bytearray([0xE0, 0x07, 0x00, 0x00])
        blue_mask = bytearray([0x1F, 0x00, 0x00, 0x00])

        # Fill in file size
        file_size = len(file_header) + len(info_header) + len(red_mask) + len(green_mask) + len(blue_mask) + len(pixel_data)
        file_header[2:6] = [
            file_size & 0xFF, (file_size >> 8) & 0xFF,
            (file_size >> 16) & 0xFF, (file_size >> 24) & 0xFF
        ]

        # Write BMP file
        with open(output_filename, 'wb') as f:
            f.write(file_header)
            f.write(info_header)
            f.write(red_mask)
            f.write(green_mask)
            f.write(blue_mask)
            f.write(pixel_data)

    # Example usage
    image = Image.open(inputfilename)

    # Could not delete highway IC green labels using Google map API style (mapid) which only appeares at zoomlevel 13,
    # so here we are replacing related green color.
    if "z13" in inputfilename:
        # Convert the target color to the replacement color
        target_color = (101, 147, 67)
        replacement_color = (255, 255, 255)
        image = convert_color(image, target_color, replacement_color,30)
        target_color = (150, 183, 131)
        replacement_color = (190, 210, 190)
        image = convert_color(image, target_color, replacement_color,30)

    save_bmp_rgb565(image, outputfilename)


def get_google_map_image(zoomlevel, latitude, longitude, key, mapid):
    # Create the filename based on latitude, longitude, and zoom level
    filename = f"{round(latitude*100)}_{round(longitude*100)}_z{zoomlevel}.png"
    
    # Check if the file already exists
    if os.path.exists(filename):
        print(f"Skipping, file already exists: {filename}")
        return
    
    # Create the URL for the Google Maps Static API
    url = f"https://maps.googleapis.com/maps/api/staticmap?center={latitude},{longitude}&zoom={zoomlevel}&size=640x640&key={key}&map_id={mapid}"
    
    # Access the URL and save the resulting PNG
    response = requests.get(url)
    if response.status_code == 200:
        with open(filename, 'wb') as file:
            file.write(response.content)
        print(f"Image saved as {filename}")
    else:
        print(f"Failed to retrieve the image. Status code: {response.status_code}")


def generate_images(start_latitude, start_longitude, iterate_interval_lat, iterate_interval_lon, size, zoomlevel, key, mapid):
    # Generate the arrays of latitudes and longitudes
    latitudes = [start_latitude + i * iterate_interval_lat for i in range(size)]
    longitudes = [start_longitude + i * iterate_interval_lon for i in range(size)]
    
    # Iterate through latitudes and longitudes to get the images
    for latitude in latitudes:
        for longitude in longitudes:
            get_google_map_image(zoomlevel, latitude, longitude, key, mapid)



def degree_to_km(degree):
    # Approximate conversion from degrees to kilometers for latitude
    return degree * 111.321

def round_to_interval(value, interval):
    # Rounds down the value to the nearest multiple of the interval
    return math.floor(value / interval) * interval

def calculate_parameters(center_latitude, center_longitude, distance_radius):
    zoom_levels = [5, 7, 9, 11, 13]
    iterate_intervals = {5: 12.0, 7: 3.0, 9: 0.8, 11: 0.2, 13: 0.05}
    
    parameters_list = []
    
    for zoomlevel in zoom_levels:
        iterate_interval = iterate_intervals[zoomlevel]
        
        # Round the center latitude and longitude to the nearest multiple of the iterate interval
        start_latitude = round_to_interval(center_latitude, iterate_interval)
        start_longitude = round_to_interval(center_longitude, iterate_interval)
        
        # Calculate the size needed to cover the radius
        size = math.ceil((distance_radius / degree_to_km(iterate_interval)) * 2)+1
        
        parameters_list.append((start_latitude, start_longitude, iterate_interval, size, zoomlevel))
    
    return parameters_list


parameters_list = calculate_parameters(center_latitude, center_longitude, distance_radius)

for params in parameters_list:
    print(f"Start Latitude: {params[0]}, Start Longitude: {params[1]}, Iterate Interval: {params[2]}, Size: {params[3]}, Zoom Level: {params[4]}")
    inkey = input("Enter 'P' to proceed")
    if inkey == "P":
        generate_images(params[0], params[1], params[2], params[2], params[3], params[4], key, mapid)


# List all files in the given directory
for filename in os.listdir("./"):
    # Check if the file has a .png extension
    if filename.endswith(".png"):
        # Change the extension to .bmp and print the filename
        bmp_filename = filename.replace(".png", ".bmp")
        print(bmp_filename)
        convert_png_bmp(filename,bmp_filename)





def relocate_files_with(search_text,directory):
    # Define the target folder name
    target_folder = os.path.join(directory, search_text)
    
    # Create the target folder if it does not exist
    if not os.path.exists(target_folder):
        os.makedirs(target_folder)
        print(f"Created folder: {target_folder}")

    # Iterate over files in the directory
    for filename in os.listdir(directory):
        if search_text in filename and filename.lower().endswith('.bmp') and os.path.isfile(os.path.join(directory, filename)):
            # Construct full file path
            file_path = os.path.join(directory, filename)
            
            # Move the file to the target folder
            shutil.move(file_path, os.path.join(target_folder, filename))
            print(f"Moved file: {file_path} -> {os.path.join(target_folder, filename)}")


relocate_files_with("z5",".")
relocate_files_with("z7",".")
relocate_files_with("z9",".")
relocate_files_with("z11",".")
relocate_files_with("z13",".")