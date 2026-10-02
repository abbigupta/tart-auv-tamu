import numpy as np
import math
#from math import sin,cos,atan2
def diff_yaw(yaw1,yaw2): #angle diff of yaw2 relative to yaw1
	return ((yaw2-yaw1)%360+180)%360-180

def mean_yaw(yaws): #get mean of yaws
	yaws = np.array(yaws)
	u = np.sin(np.deg2rad(yaws))
	v = np.cos(np.deg2rad(yaws))
	return np.rad2deg(math.atan2(np.mean(u),np.mean(v)))

def std_yaw(yaws): #get standard deviation of yaws
	yaws = np.array(yaws)
	myaw = mean_yaw(yaws)
	return np.std([diff_yaw(myaw,yaw) for yaw in yaws])

def calculate_yaw(lon1, lat1, lon2, lat2):
    """
    Calculate the yaw (bearing) from (lon1, lat1) to (lon2, lat2).
    Returns bearing in degrees, clockwise from North.
    """
    # Convert degrees to radians
    lon1_rad = math.radians(lon1)
    lat1_rad = math.radians(lat1)
    lon2_rad = math.radians(lon2)
    lat2_rad = math.radians(lat2)

    # Calculate bearing
    dlon = lon2_rad - lon1_rad
    x = math.sin(dlon) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
    
    initial_bearing = math.atan2(x, y)  # returns result in radians
    
    # Convert radian result to degrees and normalize to 0-360
    bearing_deg = (math.degrees(initial_bearing) + 360) % 360
    
    return bearing_deg
