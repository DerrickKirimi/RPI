import logging
import math
import time

from imutils.video import VideoStream
import RPi.GPIO as GPIO


# https://github.com/pimoroni/pantilt-hat/blob/master/examples/smooth.py

GPIO.setmode(GPIO.BCM)
tilt_pin=13
pan_pin = 5

GPIO.setup(tilt_pin, GPIO.OUT)
GPIO.setup(pan_pin, GPIO.OUT)
pan_servo = GPIO.PWM(pan_pin, 50) # Replace 17 with the actual GPIO pin for pan
tilt_servo = GPIO.PWM(tilt_pin, 50)

def camera_test():
    
    logging.info('Starting USB Webcam')
    vs = VideoStream(usePiCamera=False).start()
    try:
        while True:
            continue
    except KeyboardInterrupt:
        logging.info('Stopping Raspberry Pi Camera')
        VideoStream.stop()

def set_servo(servo, angle):
    #assert angle > 30 and angle <= 150
    #tilt_servo = GPIO.PWM(tilt_pin, 50)
    #tilt_servo.start(8)
    dutyCycle = angle / 18. + 3.
    servo.ChangeDutyCycle(dutyCycle)
    print(dutyCycle)
    time.sleep(0.3)
    servo.stop()

def pantilt_test():
    logging.info('Starting Pan-Tilt test!')
    logging.info('Pan-Tilt  should follow a smooth sine wave')
    while True:
        # Get the time in seconds
        t = time.time()

        # G enerate an angle using a sine wave (-1 to 1) multiplied by 90 (-90 to 90)
        a = math.sin(t * 2) * 90
        # Cast a to int for v0.0.2
        a = int(a)
        set_servo (pan_servo,a)
        set_servo(tilt_servo,a)

        # Sleep for a bit so we're not hammering the HAT with updates
        time.sleep(0.005)


if __name__ == '__main__':
    pantilt_test()
