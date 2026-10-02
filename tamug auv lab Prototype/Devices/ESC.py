import pigpio
import time
import math
import numpy as np

class ESC:
	
	pi = pigpio.pi()
	
	frontThrottle = 2000
	stallThrottle = 1500
	backThrottle = 1000
	
	status = 'Initializing...'
	
	def __init__(self, pins, frequency = 500, speeding = 0.5, turn_offset = -0.08):
		for pin in pins:
			self.pi.set_mode(pin, pigpio.OUTPUT)
			self.pi.set_PWM_frequency(pin, frequency)
		self.pins = pins
		self.speeds = [0.0]*len(pins)
		self.motionModel = 'None'
		self.status = 'Initialized'
		self.speeding = min([1,speeding])
		self.turn_offset = turn_offset # this is the turn value to make the robot moving strafight
	
	def printData(self):
		#print('\n---Motors------------------')
		#print(self.status)
		speedsMessage = f'[ESC] Speeding: {self.speeding:.2f}; Motor 1: {self.speeds[0]}%'
		for i in range(1, len(self.pins)):
			speedsMessage += f'  |  Motor {i+1}: {self.speeds[i]}%'
		print(speedsMessage)
		print(f'[ESC] turn offset: {self.turn_offset:.2f}')
	
	def arm(self):
		self.status = f'Arming pulse (3s): {self.stallThrottle} µs'
		for pin in self.pins:
			self.pi.set_servo_pulsewidth(pin, self.stallThrottle)
		time.sleep(3)
		self.status = 'Armed'
	
	def setAll(self, speed):
		pulse = (speed*5 + 1500)
		if (pulse > 2000): pulse = 2000
		if (pulse < 1000): pulse = 1000
		for pin in self.pins:
			self.pi.set_servo_pulsewidth(pin, round(pulse))
		for i in range(len(self.pins)):
			self.speeds[i] = (round(pulse)-1500)/5
	
	def setMultiple(self, speed, motors):
		pulse = (speed*5 + 1500)
		if (pulse > 2000): pulse = 2000
		if (pulse < 1000): pulse = 1000
		for motor in motors:
			self.pi.set_servo_pulsewidth(self.pins[motor-1], round(pulse))
			self.speeds[motor-1] = (round(pulse)-1500)/5
	
	def setOne(self, speed, motor):
		pulse = (speed*5 + 1500)
		if (pulse > 2000): pulse = 2000
		if (pulse < 1000): pulse = 1000
		self.pi.set_servo_pulsewidth(self.pins[motor-1], round(pulse))
		self.speeds[motor-1] = (round(pulse)-1500)/5
	
	def setMotionModel(self, motionModel):
		if (motionModel == 'None'):
			self.motionModel = motionModel
		elif (motionModel == 'Differential'):
			self.motionModel = motionModel
		elif (motionModel == 'Holonomic2D'):
			self.motionModel = motionModel
		elif (motionModel == 'Holonomic3D'):
			self.motionModel = motionModel
		else:
			originalMessage = self.status
			self.status = f'Error: Non-existing motion model "{motionModel}"\nPlease use either "None", "Differential", "Holonomic2D", or "Holonomic3D"'
			time.sleep(2)
			self.status = originalMessage
	
	def setMotion(self, motion):
		if (self.motionModel == 'None'):
			originalMessage = self.status
			self.status = f'Error: No motion model set\nPlease set a motion model other than "None" with ESC.setMotionModel()'
			time.sleep(2)
			self.status = originalMessage
		elif (self.motionModel == 'Differential'):
			self.setMotionDifferential(motion)
		elif (self.motionModel == 'Holonomic2D'):
			self.setMotionHolonomic2D(motion)
		elif (self.motionModel == 'Holonomic3D'):
			self.setMotionHolonomic3D(motion)
		else:
			originalMessage = self.status
			self.status = f'Error: No motion model set\nPlease set a motion model other than "None" with ESC.setMotionModel()'
			time.sleep(2)
			self.status = originalMessage
	
	# Motion model for 2 motors
	# (array) 'motion' interpreted as [drive, turn]
	# (float) 'drive' ranges [-1.0, 1.0] meaning [full backward, full forward]
	# (float) 'turn' ranges [-1.0, 1.0] meaning [full CCW, full CW]
	def setMotionDifferential(self, motion):
		drive = np.clip(motion[0],-1,1)
		turn = np.clip(motion[1] + self.turn_offset, -1, 1)
		speed1 = +((drive+turn)/2)*100 #Formula (+ for CW motor)
		speed2 = -((drive-turn)/2)*100 #Formula (- for CCW motor)
		if self.speeding>0.5 and drive==-1 and turn==0:
			speed1*=self.speeding*2
			speed2*=self.speeding*2

		if self.speeding>0.5 and drive==1: 
			percent = 100*self.speeding - max([abs(speed1),abs(speed2)])
			if percent > 0 and speed1 > 0: 
				speed1 += percent
				speed2 -= percent

		self.setOne(speed1, 1)
		self.setOne(speed2, 2)
	
	# Motion model for 4 motors
	# (array) 'motion' interpreted as [dx, dy, dθ]
	def setMotionHolonomic2D(self, motion):
		dx = motion[0]
		dy = motion[1]
		dθ = motion[2]
		speed1 = 0 #Formula
		speed2 = 0 #Formula
		speed3 = 0 #Formula
		speed4 = 0 #Formula
		self.setOne(speed1, 1)
		self.setOne(speed2, 2)
		self.setOne(speed3, 3)
		self.setOne(speed4, 4)
	
	# Motion model for 6 motors
	def setMotionHolonomic3D(self, motion):
		speed1 = 0 #Formula
		speed2 = 0 #Formula
		speed3 = 0 #Formula
		speed4 = 0 #Formula
		speed5 = 0 #Formula
		speed6 = 0 #Formula
		self.setOne(speed1, 1)
		self.setOne(speed2, 2)
		self.setOne(speed3, 3)
		self.setOne(speed4, 4)
		self.setOne(speed5, 5)
		self.setOne(speed6, 6)
	
	def disarm(self):
		for pin in self.pins:
			self.pi.set_servo_pulsewidth(pin, 0)
		time.sleep(0.1)
		self.pi.stop()
		self.status = 'Initialized'
