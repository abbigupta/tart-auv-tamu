import time
import RPi.GPIO as GPIO

# Raspberry Pi GPIO pins (BCM numbering)
TRIG_PIN = 23
ECHO_PIN = 24

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

# Ensure trigger is low
GPIO.output(TRIG_PIN, False)
time.sleep(2)

def measure_distance():
    # Send 10us pulse to trigger
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)

    pulse_start = time.time()
    timeout = pulse_start + 0.04
    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()
        if pulse_start > timeout:
            return None

    pulse_end = time.time()
    timeout = pulse_end + 0.04
    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()
        if pulse_end > timeout:
            return None

    pulse_duration = pulse_end - pulse_start
    # Speed of sound constant; result in cm
    distance_cm = pulse_duration * 17150
    return round(distance_cm, 2)

try:
    while True:
        dist = measure_distance()
        if dist is None:
            print("Timeout / no echo received")
        else:
            print(f"Distance: {dist} cm")
        time.sleep(0.2)

except KeyboardInterrupt:
    print("Stopping ultrasonic test")

finally:
    GPIO.cleanup()