import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import FancyArrow
import matplotlib.patheffects as path_effects

FILENAME = '2024-07-27_0936_edited.csv'
SAVENAME = '0727.mp4'
starttime = "10:05:37"
stoptime = "10:18:40"

# Function to convert time to seconds since the start time
def time_to_seconds(time_str, start_time_str):
    start_h, start_m, start_s = map(int, start_time_str.split(':'))
    h, m, s = map(int, time_str.split(':'))
    return (h * 3600 + m * 60 + s) - (start_h * 3600 + start_m * 60 + start_s)

# Data loading and preprocessing
data = pd.read_csv(FILENAME)
data['time_seconds'] = data['time'].apply(lambda x: time_to_seconds(x, starttime))

# Convert ground speed from knots to meters per second
data['gs'] = data['gs'] * 0.514444

start = 0
stop = time_to_seconds(stoptime, starttime)
frame_interval = 1
new_time = np.arange(start, stop + frame_interval, frame_interval)

# Initialize figure and axes
fig, (tt_ax, gs_ax, map_ax) = plt.subplots(3, 1, figsize=(8, 16), facecolor='green')
tt_ax.set_xlim(-1, 1)
tt_ax.set_ylim(-1, 1)
tt_ax.set_facecolor('green')
tt_ax.grid(False)
tt_ax.xaxis.set_visible(False)
tt_ax.yaxis.set_visible(False)
for spine in tt_ax.spines.values():
    spine.set_visible(False)

gs_ax.set_xlim(0, 20)
gs_ax.set_ylim(0, 1)
gs_ax.set_yticks([0.5])
gs_ax.set_yticklabels(["SPD"])
gs_ax.set_facecolor('green')
gs_ax.grid(False)
gs_ax.xaxis.set_visible(False)
gs_ax.yaxis.set_visible(False)
for spine in gs_ax.spines.values():
    spine.set_visible(False)

# Adjust the map subplot
map_ax.set_xlim(min(data['longitude']) - 0.01, max(data['longitude']) + 0.01)
map_ax.set_ylim(min(data['latitude']) - 0.01, max(data['latitude']) + 0.01)
map_ax.set_facecolor('green')
map_ax.grid(False)
map_ax.xaxis.set_visible(False)
map_ax.yaxis.set_visible(False)
for spine in map_ax.spines.values():
    spine.set_visible(False)

# Plot settings
tt_text = tt_ax.text(0.5, 1.05, "", horizontalalignment='center', verticalalignment='center', transform=tt_ax.transAxes, color='black', fontsize=40, fontweight='bold', path_effects=[path_effects.withStroke(linewidth=6, foreground='white')])
lat_text = tt_ax.text(0.5, -0.1, "", horizontalalignment='center', verticalalignment='center', transform=tt_ax.transAxes, color='black', fontsize=20, fontweight='bold', path_effects=[path_effects.withStroke(linewidth=6, foreground='white')])
lon_text = tt_ax.text(0.5, -0.16, "", horizontalalignment='center', verticalalignment='center', transform=tt_ax.transAxes, color='black', fontsize=20, fontweight='bold', path_effects=[path_effects.withStroke(linewidth=6, foreground='white')])
gs_text = gs_ax.text(0.5, 0.75, "", horizontalalignment='center', verticalalignment='center', transform=gs_ax.transAxes, color='black', fontsize=40, fontweight='bold', path_effects=[path_effects.withStroke(linewidth=6, foreground='white')])
time_text = tt_ax.text(0.5, 1.15, "", horizontalalignment='center', verticalalignment='center', transform=tt_ax.transAxes, color='black', fontsize=30, fontweight='bold', path_effects=[path_effects.withStroke(linewidth=6, foreground='white')])

# Update function for animation
def update(i):
    tt_ax.clear()
    tt_ax.set_xlim(-1, 1)
    tt_ax.set_ylim(-1, 1)
    tt_ax.set_facecolor('green')
    tt_ax.grid(False)
    tt_ax.xaxis.set_visible(False)
    tt_ax.yaxis.set_visible(False)
    for spine in tt_ax.spines.values():
        spine.set_visible(False)
    
    gs_ax.clear()
    gs_ax.set_xlim(0, 20)
    gs_ax.set_ylim(0, 1)
    gs_ax.set_yticks([0.5])
    gs_ax.set_yticklabels(["SPD"])
    gs_ax.set_facecolor('green')
    gs_ax.grid(False)
    gs_ax.xaxis.set_visible(False)
    gs_ax.yaxis.set_visible(False)
    for spine in gs_ax.spines.values():
        spine.set_visible(False)
    
    idx = (data['time_seconds'] == i).idxmax()
    
    angle_rad = np.deg2rad(90 - data['TrueTrack'].iloc[idx])  # Adjust angle for compass
    x = 0.6 * np.cos(angle_rad)  # Make the arrow smaller to fit inside the screen
    y = 0.6 * np.sin(angle_rad)
    arrow = FancyArrow(0, 0, x, y, facecolor='black', width=0.05, edgecolor='white', linewidth=6)
    tt_ax.add_patch(arrow)
    
    gs_ax.barh(0.5, data['gs'].iloc[idx], height=0.1, facecolor='black', edgecolor='white', linewidth=12)
    
    tt_text.set_text(f"TrueTrack: {int(data['TrueTrack'].iloc[idx])}°")
    lat_text.set_text(f"Lat: {data['latitude'].iloc[idx]:.6f}")
    lon_text.set_text(f"Lon: {data['longitude'].iloc[idx]:.6f}")
    gs_text.set_text(f"{data['gs'].iloc[idx]:.1f} m/s")
    time_text.set_text(f"Time: {data['time'].iloc[idx]}")
    
    tt_ax.add_artist(tt_text)
    tt_ax.add_artist(lat_text)
    tt_ax.add_artist(lon_text)
    gs_ax.add_artist(gs_text)
    tt_ax.add_artist(time_text)
    
    # Clear the map axis
    map_ax.clear()
    map_ax.set_xlim(min(data['longitude']) - 0.01, max(data['longitude']) + 0.01)
    map_ax.set_ylim(min(data['latitude']) - 0.01, max(data['latitude']) + 0.01)
    map_ax.set_facecolor('green')
    map_ax.grid(False)
    map_ax.xaxis.set_visible(False)
    map_ax.yaxis.set_visible(False)
    for spine in map_ax.spines.values():
        spine.set_visible(False)
    
    # Plot the track
    map_ax.plot(data['longitude'][:idx+1], data['latitude'][:idx+1], color='white', linewidth=2)
    # Plot the current position
    map_ax.plot(data['longitude'].iloc[idx], data['latitude'].iloc[idx], marker='o', color='black', markersize=8, markeredgecolor='white', markeredgewidth=2)
    
    return [arrow, tt_text, lat_text, lon_text, gs_text, time_text] + gs_ax.patches + map_ax.lines + map_ax.patches

ani = FuncAnimation(fig, update, frames=new_time, interval=frame_interval*1000, blit=True)

# Save animation
ani.save(SAVENAME, writer='ffmpeg', fps=1)
