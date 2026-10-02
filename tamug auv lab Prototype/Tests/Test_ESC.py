import pigpio
import time

# GPIO19 / Pin 35 (PWM1)
ESC_PIN = 20
pi = pigpio.pi()
if not pi.connected:
    exit()

# Set up PWM (2000 µs period)
pi.set_mode(ESC_PIN, pigpio.OUTPUT)
pi.set_PWM_frequency(ESC_PIN, 500)

# Define action range (µs)

# 100% backward -- 1000 µs
fullBack = 1000

# Zero throttle -- 1500 µs (also for arming)
arm = 1500

# 100% forward --- 2000 µs
fullFront = 2000

# Ramp test qualities
step = 10
interval = 0.1

# Send arming pulse to initiate the ESC
pi.set_servo_pulsewidth(ESC_PIN, arm)
print(f"Arming pulse (3s): {arm} µs")
time.sleep(3)

# Climb to fullFront
print("Starting throttle sweep...")
for pulse in range(arm, fullFront + 1, step):
    pi.set_servo_pulsewidth(ESC_PIN, pulse)
    print(f"Pulse width: {pulse} µs")
    time.sleep(interval)

print("Holding max throttle for 2 seconds...")
time.sleep(2)

# Fall past zero to fullBack
for pulse in range(fullFront, fullBack - 1, -step):
    pi.set_servo_pulsewidth(ESC_PIN, pulse)
    print(f"Pulse width: {pulse} µs")
    time.sleep(interval)

print("Holding max throttle for 2 seconds...")
time.sleep(2)

# Climb back to zero
for pulse in range(fullBack, arm + 1, step):
    pi.set_servo_pulsewidth(ESC_PIN, pulse)
    print(f"Pulse width: {pulse} µs")
    time.sleep(interval)

# Turn off signal
pi.set_servo_pulsewidth(ESC_PIN, 0)
pi.stop()
print("Throttle sweep complete.")
