from multiprocessing import Value, Process, Manager
import logging
import numpy as np
from imutils.video import VideoStream
from rpi_deep_pantilt.control.pid import PID
import RPi.GPIO as GPIO
import argparse
import signal
import time
import sys
import cv2

from rpi_deep_pantilt.detect.ssd_mobilenet_v3_coco import SSDMobileNet_V3_Small_Coco_PostProcessed, SSDMobileNet_V3_Coco_EdgeTPU_Quant
from rpi_deep_pantilt.control.pid import PIDController

GPIO.setmode(GPIO.BCM)
#change pwm pins to 13 hardware pwm and other but 12=pwm0, 13=pwm1, same channel
#18 =pcm_clk, 19=pcm_fs
tilt_pin=13
#tilt_pin = 3
pan_pin = 5
#add config file to set pins from

GPIO.setup(tilt_pin, GPIO.OUT)
GPIO.setup(pan_pin, GPIO.OUT)
# Create AngularServo instances for pan and tilt
pan_servo = GPIO.PWM(pan_pin, 50) # Replace 17 with the actual GPIO pin for pan
tilt_servo = GPIO.PWM(tilt_pin, 50)  # Replace 18 with the actual GPIO pin for tilt
#tilt_servo.start(8) #check effects
#pan_servo.start(8)


# Define the range for the motors
servoRange = (-90, 90)

# Function to handle keyboard interrupt
def signal_handler(sig, frame):
    # Print a status message
    print("[INFO] You pressed `ctrl + c`! Exiting...")
    # Exit
    sys.exit()

# Function to set servo angles
def set_servo(servo, angle):
    #assert angle > 30 and angle <= 150
    #tilt_servo = GPIO.PWM(tilt_pin, 50)
    #tilt_servo.start(8)
    dutyCycle = angle / 18. + 3.
    servo.ChangeDutyCycle(dutyCycle)
    time.sleep(0.3)
    servo.stop()

def obj_center(args, objX, objY, centerX, centerY):
    # Signal trap to handle keyboard interrupt
    signal.signal(signal.SIGINT, signal_handler)
    # Start the video stream and wait for the camera to warm up
    vs = VideoStream(usePiCamera=False).start()
    time.sleep(2.0)
    # Initialize the object center finder. Our cascade path is passed to the constructor.
    obj = ObjCenter(args["cascade"])
    
    # Loop indefinitely
    rect = None
    while True:
        # Grab the frame from the threaded video stream and flip it
        frame = vs.read()
        #frame = cv2.flip(frame, 0)
        frame = cv2.flip(frame, 1)
        # Calculate the center of the frame as this is where we will
        # try to keep the object
        (H, W) = frame.shape[:2]
        centerX.value = W // 2
        centerY.value = H // 2
        # Find the object's location
        objectLoc = obj.update(frame, (centerX.value, centerY.value))
        if objectLoc is not None:
            ((objX.value, objY.value), rect) = objectLoc
        # Extract the bounding box and draw it
        #rect = None
        if rect is not None:
            (x, y, w, h) = rect
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        # Display the frame to the screen
        cv2.imshow("Pan-Tilt Face Tracking", frame)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

def pid_process(output, p, i, d, objCoord, centerCoord):
    # Signal trap to handle keyboard interrupt
    signal.signal(signal.SIGINT, signal_handler)
    # Create a PID and initialize it
    p = PID(p.value, i.value, d.value)
    p.initialise()
    # Loop indefinitely
    while True:
        # Calculate the error
        error = centerCoord.value - objCoord.value
        # Update the value
        output.value = p.update(error)

def in_range(val, start, end):
    # Determine if the input value is in the supplied range
    return start <= val <= end

def set_servos(tlt, pan):
    # Signal trap to handle keyboard interrupt
    signal.signal(signal.SIGINT, signal_handler)
    
    # Loop indefinitely
    while True:
        # The pan and tilt angles are reversed
        #panAngle = -1 * pan.value
        panAngle = pan.value
        #tiltAngle = -1 * tlt.value
        tiltAngle = tlt.value #removed -1 because vertical flipping is off
        # If the pan angle is within the range, pan
        if in_range(panAngle, servoRange[0], servoRange[1]):
            set_servo(pan_servo, panAngle)
        # If the tilt angle is within the range, tilt
        if in_range(tiltAngle, servoRange[0], servoRange[1]):
            set_servo(tilt_servo, tiltAngle)

# Check to see if this is the main body of execution
if __name__ == "__main__":
    # Construct the argument parser and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-c", "--cascade", type=str, required=True, help="path to input Haar cascade for face detection")
    args = vars(ap.parse_args())

    # Start a manager for managing process-safe variables
    with Manager() as manager:
        tilt_servo.start(8) #check effects
        pan_servo.start(8)
        # Set integer values for the object center (x, y)-coordinates
        centerX = manager.Value("i", 0)
        centerY = manager.Value("i", 0)
        # Set integer values for the object's (x, y)-coordinates
        objX = manager.Value("i", 0)
        objY = manager.Value("i", 0)
        # Pan and tilt values will be managed by independent PIDs
        pan = manager.Value("i", 0)
        tlt = manager.Value("i", 0)

        # Set PID values for panning
        panP = manager.Value("f", 0.09)
        panI = manager.Value("f", 0.08)
        panD = manager.Value("f", 0.002)
        # Set PID values for tilting
        tiltP = manager.Value("f", 0.11)
        tiltI = manager.Value("f", 0.10)
        tiltD = manager.Value("f", 0.002)

        # We have 4 independent processes
        # 1. objectCenter  - finds/localizes the object
        # 2. panning       - PID control loop determines panning angle
        # 3. tilting       - PID control loop determines tilting angle
        # 4. setServos     - drives the servos to proper angles based
        #                    on PID feedback to keep the object in the center

        processObjectCenter = Process(target=obj_center, args=(args, objX, objY, centerX, centerY))
        processPanning = Process(target=pid_process, args=(pan, panP, panI, panD, objX, centerX))
        processTilting = Process(target=pid_process, args=(tlt, tiltP, tiltI, tiltD, objY, centerY))
        processSetServos = Process(target=set_servos, args=(tlt, pan))

        # Start all 4 processes
        processObjectCenter.start()
        processPanning.start()
        processTilting.start()
        processSetServos.start()

        try:
            # Join all 4 processes
            processObjectCenter.join()
            #processPanning.join()
            processTilting.join()
            processSetServos.join()
        except KeyboardInterrupt:
            # Handle keyboard interrupt to exit gracefully
            print("Program stopped")
        finally:
            # Disable the servos
            # pan_servo.close()
            tilt_servo.ChangeDutyCycle(0)
            pan_servo.ChangeDutyCycle(0)
            tilt_servo.stop()
            pan_servo.stop()
            GPIO.cleanup()