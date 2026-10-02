import time
import threading
from select import select
from evdev import InputDevice, list_devices, ecodes

class XCON:
	def __init__(self, calibrate_duration=1.0, deadzone=0.08, alpha=0.35,
				invert_ly=True, invert_ry=True, lb_code=310, rb_code=311, ly_code=1, ry_code=4,
				debug=False):
		"""Create controller reader.

		Parameters:
		- calibrate_duration: seconds to sample for center calibration
		- deadzone: normalized deadzone (0..1)
		- alpha: smoothing factor for EMA (0..1)
		- invert_ly/invert_ry: flip axes if needed
		- lb_code/rb_code: button codes for LB and RB (defaults set to 310/311 per your note)
		- debug: enable raw event printing
		"""

		self.status = 'Initializing...'
		self.stop_reading = threading.Event()
		devices = [InputDevice(path) for path in list_devices()]
		self.gamepad = None
		for device in devices:
			# print device list only once
			#print([d.name for d in devices])
			if 'Xbox Wireless Controller' == device.name or 'Gamepad' in device.name:
				self.gamepad = device
				break
		if self.gamepad is None:
			self.status = 'No Xbox controller found'
		else:
			self.status = f'Using device: {self.gamepad.name} at {self.gamepad.path}'
		time.sleep(0.5)

		# public state
		self.A = False
		self.B = False
		self.X = False
		self.Y = False

		self.LB = False
		self.RB = False

		self.LC = False
		self.RC = False

		self.LJ = False
		self.RJ = False

		self.LX = 0.0
		self.LY = 0.0
		self.RX = 0.0
		self.RY = 0.0

		self.RT = 0.0
		self.LT = 0.0

		self.DE = False
		self.DW = False
		self.DS = False
		self.DN = False

		# parameters
		self.calibrate_duration = calibrate_duration
		self.deadzone = deadzone
		self.alpha = alpha
		self.invert_ly = invert_ly
		self.invert_ry = invert_ry
		self.lb_code = lb_code
		self.rb_code = rb_code
		self.debug = debug

		# detect axis codes (common defaults; use Test_XCON.py to find out)
		self.LX_CODE = 0       
		self.LY_CODE = ly_code 
		self.RX_CODE = 3 
		self.RY_CODE = ry_code 

		# internal smoothed values
		self._smoothed_ly = 0.0
		self._smoothed_ry = 0.0

		# calibration centers (will be set on start)
		self.center_ly = 0
		self.center_ry = 0

	def _get_abs_range(self, code):
		try:
			info = self.gamepad.absinfo(code)
			return info.min, info.max
		except Exception:
			return -32768, 32767

	def _normalize_axis(self, raw, center, code=None):
		if code is not None:
			rmin, rmax = self._get_abs_range(code)
		else:
			rmin, rmax = -32768, 32767
		span = max(abs(center - rmin), abs(rmax - center))
		if span == 0:
			return 0.0
		val = (raw - center) / float(span)
		if val > 1.0:
			val = 1.0
		if val < -1.0:
			val = -1.0
		if abs(val) < self.deadzone:
			return 0.0
		return val

	def _calibrate(self, duration=None):
		if duration is None:
			duration = self.calibrate_duration
		end_time = time.time() + duration
		counts = {self.LY_CODE: 0, self.RY_CODE: 0}
		sums = {self.LY_CODE: 0, self.RY_CODE: 0}
		while time.time() < end_time:
			r, _, _ = select([self.gamepad.fd], [], [], 0.05)
			if r:
				for ev in self.gamepad.read():
					if ev.type == ecodes.EV_ABS and ev.code in (self.LY_CODE, self.RY_CODE):
						sums[ev.code] += ev.value
						counts[ev.code] += 1
		# compute centers, fallback to midpoint if no samples
		if counts[self.LY_CODE]:
			self.center_ly = int(sums[self.LY_CODE] / counts[self.LY_CODE])
		else:
			rmin, rmax = self._get_abs_range(self.LY_CODE)
			self.center_ly = int((rmin + rmax) / 2)

		if counts[self.RY_CODE]:
			self.center_ry = int(sums[self.RY_CODE] / counts[self.RY_CODE])
		else:
			rmin, rmax = self._get_abs_range(self.RY_CODE)
			self.center_ry = int((rmin + rmax) / 2)
	
	def printData(self):
		print(f'[XCON] LY:{self.LY:.2f},RY:{self.RY:.2f},LB:{self.LB},RB:{self.RB}')
		

	def start(self):
		if self.gamepad is None:
			return
		# calibrate quickly
		self._calibrate()
		if self.debug:
			try:
				a1 = self.gamepad.absinfo(self.LY_CODE)
				a2 = self.gamepad.absinfo(self.RY_CODE)
				print(f'LY code {self.LY_CODE} absinfo: min={a1.min}, max={a1.max}, fuzz={a1.fuzz}, flat={a1.flat}')
				print(f'RY code {self.RY_CODE} absinfo: min={a2.min}, max={a2.max}, fuzz={a2.fuzz}, flat={a2.flat}')
			except Exception:
				print('Could not read absinfo for axes')

		# main loop
		while not self.stop_reading.is_set():
			r, _, _ = select([self.gamepad.fd], [], [], 0.1)
			if not r:
				continue
			for event in self.gamepad.read():
				et = event.type
				ec = event.code
				ev = event.value
				if self.debug:
					print(et, ec, ev)
				if et == ecodes.EV_KEY:
					if ec == 304: self.A = (ev == 1)
					elif ec == 305: self.B = (ev == 1)
					elif ec == 306: self.X = (ev == 1)
					elif ec == 307: self.Y = (ev == 1)
					elif ec == self.lb_code: self.LB = (ev == 1)
					elif ec == self.rb_code: self.RB = (ev == 1)
					elif ec == 310: self.LC = (ev == 1)
					elif ec == 311: self.RC = (ev == 1)
					elif ec == 312: self.LJ = (ev == 1)
					elif ec == 313: self.RJ = (ev == 1)
				elif et == ecodes.EV_ABS:
					if ec == self.LX_CODE:
						self.LX = self._normalize_axis(ev, self.center_ly, code=self.LX_CODE)
					elif ec == self.LY_CODE:
						#raw_ly = ev
						#self.LY=(32339-ev)/32339
						self.LY=-ev/32768
						#self.LY=(5000-ev)/5000
						#norm_ly = self._normalize_axis(raw_ly, self.center_ly, code=self.LY_CODE)
						#self._smoothed_ly = self.alpha * norm_ly + (1 - self.alpha) * self._smoothed_ly
						#self.LY = -self._smoothed_ly if self.invert_ly else self._smoothed_ly
					elif ec == self.RX_CODE:
						self.RX = self._normalize_axis(ev, self.center_ry, code=self.RX_CODE)
					elif ec == self.RY_CODE:
						#raw_ry = ev
						#self.RY=(32339-ev)/32339
						self.RY=-ev/32768
						#self.RY=(5000-ev)/5000
						#norm_ry = self._normalize_axis(raw_ry, self.center_ry, code=self.RY_CODE)
						#self._smoothed_ry = self.alpha * norm_ry + (1 - self.alpha) * self._smoothed_ry
						#self.RY = -self._smoothed_ry if self.invert_ry else self._smoothed_ry
					elif ec == 9:
						self.RT = ev / 1023.0
					elif ec == 10:
						self.LT = ev / 1023.0
					elif ec == 16:
						if ev == 1:
							self.DE = True
							self.DW = False
						elif ev == -1:
							self.DE = False
							self.DW = True
						else:
							self.DE = False
							self.DW = False
					elif ec == 17:
						if ev == 1:
							self.DS = True
							self.DN = False
						elif ev == -1:
							self.DS = False
							self.DN = True
						else:
							self.DS = False
							self.DN = False
	
