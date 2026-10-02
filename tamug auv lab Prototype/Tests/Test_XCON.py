from evdev import InputDevice, list_devices
import os
import time
import threading
from select import select

devices = [InputDevice(path) for path in list_devices()]
gamepad = None
stop_event = threading.Event()

for device in devices:
	if 'Xbox' in device.name or 'Gamepad' in device.name:
		gamepad = device
		break

if gamepad is None:
	raise RuntimeError('No Xbox controller found')

print(f'Using device: {gamepad.name} at {gamepad.path}')
time.sleep(1)

def thr(name):
	return threading.Thread(target = name, args = ())

A = False
B = False
X = False
Y = False

LB = False
RB = False

LC = False
RC = False

LJ = False
RJ = False

LX = 0
LY = 0
RX = 0
RY = 0

RT = 0
LT = 0

DE = False
DW = False
DS = False
DN = False

def getControllerInput():
	global A, B, X, Y, LB, RB, LC, RC, LJ, RJ, LX, LY, RX, RY, RT, LT, DE, DW, DS, DN
	
	while not stop_event.is_set():
		r, _, _ = select([gamepad.fd], [], [], 0.1)
		
		if r:
			for event in gamepad.read():
				
				et = event.type
				ec = event.code
				ev = event.value
				
				if et == 1:
					if ec == 304:
						if ev == 1:
							A = True
						else:
							A = False
					elif ec == 305:
						if ev == 1:
							B = True
						else:
							B = False
					elif ec == 306:
						if ev == 1:
							X = True
						else:
							X = False
					elif ec == 307:
						if ev == 1:
							Y = True
						else:
							Y = False
					elif ec == 308:
						if ev == 1:
							LB = True
						else:
							LB = False
					elif ec == 309:
						if ev == 1:
							RB = True
						else:
							RB = False
					elif ec == 310:
						if ev == 1:
							LC = True
						else:
							LC = False
					elif ec == 311:
						if ev == 1:
							RC = True
						else:
							RC = False
					elif ec == 312:
						if ev == 1:
							LJ = True
						else:
							LJ = False
					elif ec == 313:
						if ev == 1:
							RJ = True
						else:
							RJ = False
				
				elif et == 3:
					if ec == 0: LX = ev
					if ec == 1: LY = ev
					if ec == 2: RX = ev
					if ec == 5: RY = ev
					if ec == 9: RT = ev
					if ec == 10: LT = ev
					if ec == 16:
						if ev == 1:
							DE = True
							DW = False
						elif ev == -1:
							DE = False
							DW = True
						else:
							DE = False
							DW = False
					if ec == 17:
						if ev == 1:
							DS = True
							DN = False
						elif ev == -1:
							DS = False
							DN = True
						else:
							DS = False
							DN = False
		

def printData():
	runtime = 100
	fps = 10
	for i in range(runtime*fps):
		os.system('clear')
		print(f'---{i+1}/{runtime*fps}--------')
		print(f'A: {A}')
		print(f'B: {B}')
		print(f'X: {X}')
		print(f'Y: {Y}')
		print(f'LB: {LB}')
		print(f'RB: {RB}')
		print(f'LC: {LC}')
		print(f'RC: {RC}')
		print(f'LJ: {LJ}')
		print(f'RJ: {RJ}')
		print(f'LX: {LX}')
		print(f'LY: {LY}')
		print(f'RX: {RX}')
		print(f'RY: {RY}')
		print(f'LT: {LT}')
		print(f'RT: {RT}')
		print(f'DN: {DN}')
		print(f'DE: {DE}')
		print(f'DS: {DS}')
		print(f'DW: {DW}')
		time.sleep(1/fps)
	stop_event.set()
	t2.join()
	
t1 = thr(printData)
t2 = thr(getControllerInput)

t1.start()
t2.start()

t1.join()
