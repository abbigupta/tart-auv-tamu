import time
import threading
from select import select
from evdev import InputDevice, list_devices

class NINCON:
    """
    Template class for Nintendo Switch Pro or Joycon controllers.
    You must fill in button and axis codes according to your device using evtest.
    """
    def __init__(self):
        self.status = 'Initializing...'
        self.stop_reading = threading.Event()
        
        devices = [InputDevice(path) for path in list_devices()]
        self.gamepad = None
        
        # Nintendo controllers often appear with these names
        for device in devices:
            if 'Nintendo' in device.name or 'Pro Controller' in device.name:
                self.gamepad = device
                break
        
        if self.gamepad is None:
            self.status = 'No Nintendo controller found'
        else:
            self.status = f'Using device: {self.gamepad.name} at {self.gamepad.path}'
        
        time.sleep(1)
        
        # Buttons
        self.A = False
        self.B = False
        self.X = False
        self.Y = False
        self.L = False   # L button
        self.R = False   # R button
        self.ZL = False
        self.ZR = False
        self.LStick = False
        self.RStick = False
        self.Plus = False
        self.Minus = False
        self.Home = False
        
        # Joystick axes
        self.LX = 0.0
        self.LY = 0.0
        self.RX = 0.0
        self.RY = 0.0
        
        # Dpad
        self.DU = False
        self.DD = False
        self.DL = False
        self.DR = False
    
    def printData(self):
        print('\n---NINCON------------------')
        print(self.status)
        print(f'LX: {self.LX}, LY: {self.LY}')
        print(f'RX: {self.RX}, RY: {self.RY}')
        print(f'A:{self.A}, B:{self.B}, X:{self.X}, Y:{self.Y}')
        print(f'L:{self.L}, R:{self.R}, ZL:{self.ZL}, ZR:{self.ZR}')
        print(f'Dpad: U:{self.DU}, D:{self.DD}, L:{self.DL}, R:{self.DR}')
    
    def stop(self):
        self.stop_reading.set()
    
    def start(self):
        if not self.gamepad:
            return
        
        while not self.stop_reading.is_set():
            r, _, _ = select([self.gamepad.fd], [], [], 0.1)
            if r:
                for event in self.gamepad.read():
                    et = event.type
                    ec = event.code
                    ev = event.value
                    
                    # KEY
                    if et == 1:
                        # TODO: fill these codes using evtest results
                        if ec == 304:  # Example for A
                            self.A = (ev == 1)
                        elif ec == 305:
                            self.B = (ev == 1)
                        # Add more button mappings for X, Y, ZL, ZR, etc.
                    
                    # AXIS or ABS
                    elif et == 3:
                        # TODO: fill these with real axis codes and scaling
                        # Axis scales might be 0-255 or 0-32767
                        if ec == 0:   # Abs X for left stick
                            self.LX = (ev / 32767.5 - 1.0)
                        elif ec == 1:  # Abs Y left stick
                            self.LY = -(ev / 32767.5 - 1.0)
                        elif ec == 2:  # Abs X for right stick
                            self.RX = (ev / 32767.5 - 1.0)
                        elif ec == 5:  # Abs Y for right stick
                            self.RY = -(ev / 32767.5 - 1.0)
                        
                        # Dpad could be code 16 or 17 etc, similar to your Xbox class
                        # Fill in once you know the codes