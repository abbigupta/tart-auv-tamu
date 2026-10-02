#!/usr/bin/env python3
from pylib import *
import pandas as pd
df=pd.read_csv('PIDlogs.csv')

olon,olat=-94.8167793,29.3125873
figure()
scatter(df['current_lon'],df['current_lat'],3,df['current_distance'],cmap='jet',vmin=0,vmax=100)
colorbar()
xlim([olon-0.0006,olon+0.0005])
ylim([olat-0.0003,olat+0.0012])
plot(olon,olat,'kx')
ylabel('Lat (deg)'); xlabel('Lon (deg)')
legend(['Track of robot','Preset destination'])
tight_layout()

#%%
import gmplot

# Define the latitude and longitude points
# Example: a few points in a 
lons,lats=df['current_lon'].values,df['current_lat'].values
distance=df['current_distance'].values
# latitudes = [34.0522, 34.0522, 34.0522]
# longitudes = [-118.2437, -118.2537, -118.2637]

# Initialize gmplot with the center latitude, longitude, and zoom level
# The center point can be an average of your points or a chosen central location
gmap = gmplot.GoogleMapPlotter(lats[-1], lons[-1], 20) # 13 is a good starting zoom level

# Plot the points as markers
# You can customize the color, size, etc.
gmap.scatter(lats,lons, '#FF0000', size=2, marker=True)

# Draw the map and save it as an HTML file
# This HTML file can then be opened in a web browser to view the map
gmap.draw('my_map_with_points.html')