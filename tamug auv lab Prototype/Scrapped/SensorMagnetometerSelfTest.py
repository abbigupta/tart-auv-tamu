import smbus
import time
from RPLCD.i2c import CharLCD
lcd = CharLCD('PCF8574', 0x27)

# I2C address of BMM150 Magnetometer
BMM150 = 0x13

# Initialize I2C bus
bus = smbus.SMBus(1)

# Function to write a byte to a register
def writeRegister(register, value):
	bus.write_byte_data(BMM150, register, value)

# Function to read multiple bytes from a register
def readRegisters(startRegister, length):
	return bus.read_i2c_block_data(BMM150, startRegister, length)

# Step 1: Enable normal mode (Power Control Register: 0x4B)
writeRegister(0x4B, 0x01)
print('Enabled normal mode')
time.sleep(1)

# Step 2: Enable self-test mode (Register 0x4C)
writeRegister(0x4C, 0x10)
print('Enabled self-test mode')
time.sleep(1)

# Step 3: Read self-test results (Registers 0x42 - 0x49)
data = readRegisters(0x42, 8)
print('Read self-test results')

# Convert raw data to signed 16-bit values
x = (data[1] << 8) | data[0]
y = (data[3] << 8) | data[2]
z = (data[5] << 8) | data[4]

# Handle signed values
if x > 32767: x -= 65536
if y > 32767: y -= 65536
if z > 32767: z -= 65536

# Print values
print(x)
print(y)
print(z)

# Check if values are within expected range
if abs(x) > 100 and abs(y) > 100 and abs(z) > 100:
	print('Self-Test Passed')
else:
	print('Self-Test Failed')

# Reset to normal mode
writeRegister(0x4C, 0x08)
