import math

# This file evaluates if the following lat/lon Interval for each zoom level is small enough. (To cover all area needed)
# This file is here only for app development purpose.

zoom_interval = {5:12,
7:3,#5
9:0.8,#1.1
11:0.2,
13:0.05}



def pixels_per_degree(zoom):
    """
    Google Maps API approximation: pixels per degree at the given zoom level.
    Zoom level 5 is used as the base reference in this example.
    """
    return 256 * math.pow(2, zoom) / 360.0

def pixels_per_degree_lat(zoom, latitude):
    """
    Calculate pixels per degree for latitude.
    This adjusts the pixels per degree value based on the latitude.
    """
    return pixels_per_degree(zoom) / math.cos(math.radians(latitude))


def round_to_nearest_x_degrees(x, value):
    """
    Function to round a value to the nearest x degrees.
    """
    return round(value / x) * x

def generate_filename(zoom_level, map_lat, map_lon):
    """
    Generate a filename for the bitmap file based on the zoom level, map latitude, and map longitude.
    """
    map_lat4 = round(map_lat * 100)
    map_lon5 = round(map_lon * 100)
    filename = f"z{zoom_level}/%04d_%05d_z{zoom_level}.bmp" % (map_lat4, map_lon5)
    
    return filename


def calculate_pixel_coordinates(center_lat, center_lon, zoom_level, round_degrees):
    """
    Function to calculate pixel coordinates for given latitude and longitude,
    rounding them to the nearest specified degrees.
    """
    # Round latitude and longitude to the nearest x degrees
    map_lat = round_to_nearest_x_degrees(round_degrees, center_lat)
    map_lon = round_to_nearest_x_degrees(round_degrees, center_lon)
    filename = generate_filename(zoom_level,map_lat,map_lon)
    #print(filename)
    
    # Calculate pixel coordinates
    center_x = int(320.0 + (center_lon - map_lon) * pixels_per_degree(zoom_level))
    center_y = int(320.0 - (center_lat - map_lat) * pixels_per_degree_lat(zoom_level, center_lat))
    
    # Calculate top-left corner of 240x240 region
    start_x = center_x - 120
    start_y = center_y - 120
    
    return center_x, center_y, start_x, start_y

def print_at_pos(pos_lat,pos_lon,zoom_level,round_degrees):   
    center_x, center_y, start_x, start_y = calculate_pixel_coordinates(pos_lat, pos_lon, zoom_level, round_degrees)
    #print(f"Position: ({pos_lat}, {pos_lon}) with interval {round_degrees}")
    #print(f"Center pixel coordinates: ({center_x}, {center_y})")
    #print(f"Top-left corner of 240x240 region: ({start_x}, {start_y})")
    #print(f"right-bottom corner of 240x240 region: ({start_x+240}, {start_y+240})")
    if start_x < 0 or start_y < 0 or start_x+240 > 640 or start_y +240 > 640:
        print(f"!!!!!ERROR out of bound!!!!!")
    else:
        print(f"pass Zoom={zoom_level} Interval={round_degrees}")


def generate_sprite_id(zoom_level, map_lat, map_lon, start_x, start_y):
    """
    Generate a sprite ID based on the zoom level, map latitude, map longitude, 
    and the coordinates of the start point.
    """
    map_lat4 = round(map_lat * 100)
    map_lon5 = round(map_lon * 100)
    current_sprite_id = f"{zoom_level:02d}{map_lat4:04d}{map_lon5:05d}{start_x:03d}{start_y:03d}"
    
    print(current_sprite_id)
    return current_sprite_id


def test_position(test_lat,test_lon,zoom_level):

    test_pos_under = 0.49999
    test_pos_over = 0.50001

    center_lat = round_to_nearest_x_degrees(zoom_interval[zoom_level],test_lat)+zoom_interval[zoom_level]*test_pos_under
    center_lon = round_to_nearest_x_degrees(zoom_interval[zoom_level],test_lon)+zoom_interval[zoom_level]*test_pos_under
    print_at_pos(center_lat,center_lon,zoom_level,zoom_interval[zoom_level])
    center_lat = round_to_nearest_x_degrees(zoom_interval[zoom_level],test_lat)+zoom_interval[zoom_level]*test_pos_over
    center_lon = round_to_nearest_x_degrees(zoom_interval[zoom_level],test_lon)+zoom_interval[zoom_level]*test_pos_over
    print_at_pos(center_lat,center_lon,zoom_level,zoom_interval[zoom_level])


zoom_list = [5,7,11,13]
for each in zoom_list:
    test_lat = 35.29463196
    test_lon = 136.25637817
    test_position(test_lat,test_lon,each)

