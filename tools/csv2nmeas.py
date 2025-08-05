'''
このPythonコードは、CSVを、仮想のNMEA列に戻す。
リプレイモードのため作成。

'''
import pandas as pd
from datetime import datetime

# Read the input CSV file
df = pd.read_csv('./PONSv5_planelog_2025-07-26_1539.csv')

# Function to convert a row to NMEA GGA and RMC sentences
def row_to_nmea(row, utc_time):
    lat = abs(row['latitude'])
    lat_deg = int(lat)
    lat_min = (lat - lat_deg) * 60
    lat_dir = 'N' if row['latitude'] >= 0 else 'S'
    
    lon = abs(row['longitude'])
    lon_deg = int(lon)
    lon_min = (lon - lon_deg) * 60
    lon_dir = 'E' if row['longitude'] >= 0 else 'W'
    
    # Format time as HHMMSS and date as DDMMYY
    time_str = utc_time.strftime('%H%M%S')
    date_str = utc_time.strftime('%d%m%y')
    
    # Convert ground speed from m/s to knots (1 m/s = 1.94384 knots)
    speed_knots = row['gs'] * 1.94384
    
    # True track
    true_track = row['TrueTrack']
    
    # Create GGA sentence (simplified, with fixed values for some fields)
    gga = (f"$GPGGA,{time_str},{lat_deg:02d}{lat_min:07.4f},{lat_dir},"
           f"{lon_deg:03d}{lon_min:07.4f},{lon_dir},1,08,1.0,0.0,M,46.9,M,,*")
    
    # Create RMC sentence
    rmc = (f"$GPRMC,{time_str},A,{lat_deg:02d}{lat_min:07.4f},{lat_dir},"
           f"{lon_deg:03d}{lon_min:07.4f},{lon_dir},{speed_knots:.1f},"
           f"{true_track:.1f},{date_str},,,A*")
    
    # Calculate checksums
    gga_checksum = 0
    for char in gga[1:]:
        if char == '*':
            break
        gga_checksum ^= ord(char)
    gga += f"{gga_checksum:02X}"
    
    rmc_checksum = 0
    for char in rmc[1:]:
        if char == '*':
            break
        rmc_checksum ^= ord(char)
    rmc += f"{rmc_checksum:02X}"
    
    return gga, rmc

# Convert date and time to datetime object
df['datetime'] = pd.to_datetime(df['date'] + ' ' + df['time'])

# Calculate time in milliseconds relative to the first row
base_time = df['datetime'].iloc[0]
df['time_ms'] = (df['datetime'] - base_time).dt.total_seconds() * 1000

# Generate NMEA sentences (both GGA and RMC)
nmea_sentences = []
for _, row in df.iterrows():
    gga, rmc = row_to_nmea(row, row['datetime'])
    nmea_sentences.append({'time_ms': row['time_ms'], 'nmea': gga})
    nmea_sentences.append({'time_ms': row['time_ms'], 'nmea': rmc})

# Create output DataFrame
output_df = pd.DataFrame(nmea_sentences)
output_df['time_ms'] = output_df['time_ms'].astype(int)
output_df['nmea'] =  output_df['nmea'] 

# Save to output CSV
output_df.to_csv('output.csv', index=False, columns=['time_ms', 'nmea'])