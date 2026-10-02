import time
from random import random
from RPLCD.i2c import CharLCD

lcd = CharLCD('PCF8574', 0x27)
for i in range(100):
	lcd.clear()
	seed = random()
	lcd.write_string(str(seed))
	print(i)
	time.sleep(0.2)
