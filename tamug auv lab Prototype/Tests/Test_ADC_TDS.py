import pigpio
import time
from adafruit_ads1x15.ads1115 import ADS1115
from adafruit_ads1x15.analog_in import AnalogIn
import board
import busio
import os

i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS1115(i2c)
channel = AnalogIn(ads, 1)

Kt = 0.02
Kc = 1.0

def read_tds(T = 25.0):
	cV = channel.voltage/(1 + Kt*(T-25.0))
	return round((133.42*cV**3 - 255.86*cV**3 + 857.39*cV)*Kc*0.5, 2)

try:
	while True:
		temp = 25.0
		tds_value = read_tds(temp)
		os.system('clear')
		print(f'TDS: {tds_value}ppm')
		time.sleep(0.2)
except KeyboardInterrupt:
	print('Stopped.')

