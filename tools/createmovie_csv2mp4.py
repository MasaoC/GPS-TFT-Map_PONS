import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import FancyArrow
import matplotlib.patheffects as path_effects
from scipy.interpolate import interp1d

# Last update 2025/8/22

FILENAME = 'PONSv5_planelog_2025-07-26_1539.csv'
SAVENAME = '0726.mp4'
starttime = "15:42:10"
stoptime = "16:35:59"

# Function to convert time to seconds since the start time
def time_to_seconds(time_str, start_time_str):
    start_h, start_m, start_s = map(int, start_time_str.split(':'))
    h, m, s = map(int, time_str.split(':'))
    return (h * 3600 + m * 60 + s) - (start_h * 3600 + start_m * 60 + start_s)

# Data loading and preprocessing
data = pd.read_csv(FILENAME)
data['time_seconds'] = data['time'].apply(lambda x: time_to_seconds(x, starttime))

# Apply scaling to longitude
data['longitude'] = data['longitude'] * 0.8090169  # *cos(36deg)

# Interpolation setup
start = 0
stop = time_to_seconds(stoptime, starttime)
frame_interval = 0.25
new_time = np.arange(start, stop, frame_interval)

# Convert TrueTrack to radians for sine and cosine components
data['TrueTrack_rad'] = np.deg2rad(data['TrueTrack'])
data['sin_truetrack'] = np.sin(data['TrueTrack_rad'])
data['cos_truetrack'] = np.cos(data['TrueTrack_rad'])

# Create interpolation functions for sin(TrueTrack), cos(TrueTrack), gs, latitude, and longitude
interp_sin_truetrack = interp1d(data['time_seconds'], data['sin_truetrack'], kind='linear', fill_value="extrapolate")
interp_cos_truetrack = interp1d(data['time_seconds'], data['cos_truetrack'], kind='linear', fill_value="extrapolate")
interp_gs = interp1d(data['time_seconds'], data['gs'], kind='linear', fill_value="extrapolate")
interp_latitude = interp1d(data['time_seconds'], data['latitude'], kind='linear', fill_value="extrapolate")
interp_longitude = interp1d(data['time_seconds'], data['longitude'], kind='linear', fill_value="extrapolate")

# Initialize figure and axes
fig, (tt_ax, gs_ax, map_ax) = plt.subplots(3, 1, figsize=(8, 9), facecolor='green', gridspec_kw={'height_ratios': [0.3, 0.08, 0.5]})
tt_ax.set_xlim(-1, 1)
tt_ax.set_ylim(-1, 1)
tt_ax.set_aspect('equal')
tt_ax.set_facecolor('green')
tt_ax.grid(False)
tt_ax.xaxis.set_visible(False)
tt_ax.yaxis.set_visible(False)
for spine in tt_ax.spines.values():
    spine.set_visible(False)

gs_ax.set_xlim(2, 10)
gs_ax.set_xticks(range(11))
gs_ax.set_facecolor('lightgreen')
gs_ax.grid(axis='x')
gs_ax.yaxis.set_visible(False)

# Adjust the map subplot
map_ax.set_xlim(min(data['longitude']) - 0.01, max(data['longitude']) + 0.01)
map_ax.set_ylim(min(data['latitude']) - 0.01, max(data['latitude']) + 0.01)
map_ax.set_facecolor('green')
for spine in map_ax.spines.values():
    spine.set_visible(False)

# Plot settings
tt_text = tt_ax.text(0.5, 1.05, "", horizontalalignment='center', verticalalignment='center', transform=tt_ax.transAxes, color='black', fontsize=34, fontweight='bold', path_effects=[path_effects.withStroke(linewidth=6, foreground='white')])
latlon_text = tt_ax.text(-0.9, -0.13, "", horizontalalignment='left', verticalalignment='center', transform=tt_ax.transAxes, color='black', fontsize=20, fontweight='bold', path_effects=[path_effects.withStroke(linewidth=6, foreground='white')])
gs_text = gs_ax.text(0, 0.5, "", verticalalignment='center', horizontalalignment='left', transform=gs_ax.transAxes, color='black', fontsize=34, fontweight='bold', path_effects=[path_effects.withStroke(linewidth=3, foreground='white')])
time_text = tt_ax.text(0.5, 1.3, "", horizontalalignment='center', verticalalignment='center', transform=tt_ax.transAxes, color='black', fontsize=22, fontweight='bold', path_effects=[path_effects.withStroke(linewidth=6, foreground='white')])

# Function to convert seconds back to time string
def seconds_to_time(seconds, start_time_str):
    start_h, start_m, start_s = map(int, start_time_str.split(':'))
    total_seconds = int(seconds) + (start_h * 3600 + start_m * 60 + start_s)
    h = total_seconds // 3600
    m = (total_seconds % 3600) // 60
    s = total_seconds % 60
    return f"{h:02d}:{m:02d}:{s:02d}"

# Update function for animation
def update(i):
    tt_ax.clear()
    tt_ax.set_xlim(-1, 1)
    tt_ax.set_ylim(-1, 1)
    tt_ax.set_aspect('equal')
    tt_ax.set_facecolor('green')
    tt_ax.grid(False)
    tt_ax.xaxis.set_visible(False)
    tt_ax.yaxis.set_visible(False)
    for spine in tt_ax.spines.values():
        spine.set_visible(False)
    
    gs_ax.clear()
    gs_ax.set_xticks(range(11))
    gs_ax.set_xlim(2, 10)
    gs_ax.set_facecolor('lightgreen')
    gs_ax.grid(axis='x')
    gs_ax.yaxis.set_visible(False)

    # Interpolate values for the current time
    sin_truetrack = interp_sin_truetrack(i)
    cos_truetrack = interp_cos_truetrack(i)
    truetrack = np.mod(np.degrees(np.arctan2(sin_truetrack, cos_truetrack)), 360)  # Reconstruct angle
    gs = interp_gs(i)
    latitude = interp_latitude(i)
    longitude = interp_longitude(i)
    current_time = seconds_to_time(i, starttime)

    # TrueTrack arrow
    angle_rad = np.deg2rad(90 - truetrack)
    x = 0.7 * np.cos(angle_rad)
    y = 0.7 * np.sin(angle_rad)
    arrow = FancyArrow(0, 0, x, y, facecolor='black', width=0.05, edgecolor='white', linewidth=3)
    tt_ax.add_patch(arrow)
    
    # Ground speed bar
    gs_ax.barh(0.5, gs, height=0.05, color='blue')
    gs_text.set_text(f"PONS G/S {gs:.1f}m/s")
    
    # Text updates
    tt_text.set_text(f"TrueTrack: {int(truetrack)}°")
    latlon_text.set_text(f"{latitude:.6f}N {longitude:.6f}E")
    time_text.set_text(f"Time: {current_time}")
    
    tt_ax.add_artist(tt_text)
    tt_ax.add_artist(latlon_text)
    gs_ax.add_artist(gs_text)
    tt_ax.add_artist(time_text)
    
    # Map plot
    map_ax.clear()
    map_ax.set_aspect('equal')
    map_ax.set_xlim(min(data['longitude']) - 0.1, max(data['longitude']) + 0.01)
    map_ax.set_ylim(min(data['latitude']) - 0.01, max(data['latitude']) + 0.01)
    map_ax.set_facecolor('green')
    map_ax.grid(False)
    map_ax.xaxis.set_visible(False)
    map_ax.yaxis.set_visible(False)
    for spine in map_ax.spines.values():
        spine.set_visible(False)
    
    # Interpolate path up to current time
    times_up_to_i = new_time[new_time <= i]
    map_ax.plot(interp_longitude(times_up_to_i), interp_latitude(times_up_to_i), color='white', linewidth=2)
    map_ax.plot(longitude, latitude, marker='o', color='black', markersize=10, markeredgecolor='white', markeredgewidth=4)
    
    return [arrow, tt_text, latlon_text, gs_text, time_text] + gs_ax.patches + map_ax.lines + map_ax.patches

ani = FuncAnimation(fig, update, frames=new_time, interval=frame_interval*1000)
ani.save(SAVENAME, writer='ffmpeg', fps=1/frame_interval)