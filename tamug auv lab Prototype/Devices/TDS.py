import board
import busio
from adafruit_ads1x15.ads1115 import ADS1115
from adafruit_ads1x15.analog_in import AnalogIn

class TDS:
	
	ppm = 0.0
	
	def __init__(self, analog):
		self.ch = AnalogIn(ADS1115(busio.I2C(board.SCL, board.SDA)), analog)
	
	def tick(self, T = 25.0):
		Kt = 0.02
		Kc = 1.0
		cV = self.ch.voltage/(1 + Kt*(T-25.0))
		self.ppm = round((133.42*cV**3 - 255.86*cV**3 + 857.39*cV)*Kc*0.5, 2)
	
	def printData(self):
		print('\n---TDS---------------------')
		print(f'Total Dissolved Solids: {self.ppm}ppm')
