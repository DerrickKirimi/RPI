import logging
from multiprocessing import Value, Process, Manager
from threading import Thread
import time
from imutils.video import VideoStream

import RPi.GPIO as GPIO
import signal
import sys
import numpy as np

#relic imports(artefacts)
import cv2
import argparse
import os
import importlib.util

#replace basic functionality with imutils videostream
#from rpi_deep_pantilt.detect.camera import PiCameraStream 
from rpi_deep_pantilt.detect.ssd_mobilenet_v3_coco import SSDMobileNet_V3_Small_Coco_PostProcessed, SSDMobileNet_V3_Coco_EdgeTPU_Quant
from rpi_deep_pantilt.control.pid import PIDController

logging.basicConfig()
LOGLEVEL = logging.getLogger().getEffectiveLevel()

RESOLUTION = (320, 320)

SERVO_MIN = -90
SERVO_MAX = 90

CENTER = (
    RESOLUTION[0] // 2,
    RESOLUTION[1] // 2
)

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
tilt_servo = GPIO.PWM(tilt_pin, 50)

# Define the range for the motors
servoRange = (-90, 90)

# function to handle keyboard interrupt


def signal_handler(sig, frame):
    # print a status message
    print("[INFO] You pressed `ctrl + c`! Exiting...")

    # disable the servos
    tilt_servo.ChangeDutyCycle(0)
    pan_servo.ChangeDutyCycle(0)
    tilt_servo.stop()
    pan_servo.stop()
    GPIO.cleanup()

    # exit
    sys.exit()


def run_detect(center_x, center_y, labels, edge_tpu):
    if edge_tpu:
        model = SSDMobileNet_V3_Coco_EdgeTPU_Quant()
    else:
        model = SSDMobileNet_V3_Small_Coco_PostProcessed()

    #capture_manager = PiCameraStream(resolution=RESOLUTION)
    #capture_manager.start()
    #capture_manager.start_overlay()
    vs = VideoStream(usePiCamera=False).start()

    #how to do the overlays now, with cv2 instead

    label_idxs = model.label_to_category_index(labels)
    start_time = time.time()
    fps_counter = 0
    while True:
            frame = vs.read()
            frame = cv2.flip(frame, 1)
            prediction = model.predict(frame)

            if not len(prediction.get('detection_boxes')):
                continue

            if any(item in label_idxs for item in prediction.get('detection_classes')):

                tracked = (
                    (i, x) for i, x in
                    enumerate(prediction.get('detection_classes'))
                    if x in label_idxs
                )
                tracked_idxs, tracked_classes = zip(*tracked)

                track_target = prediction.get('detection_boxes')[
                    tracked_idxs[0]]
                # [ymin, xmin, ymax, xmax]
                y = int(
                    RESOLUTION[1] - ((np.take(track_target, [0, 2])).mean() * RESOLUTION[1]))
                center_y.value = y
                x = int(
                    RESOLUTION[0] - ((np.take(track_target, [1, 3])).mean() * RESOLUTION[0]))
                center_x.value = x

                display_name = model.category_index[tracked_classes[0]]['name']
                logging.info(
                    f'Tracking {display_name} center_x {x} center_y {y}')

            #overlay = model.create_overlay(frame, prediction)
            #capture_manager.overlay_buff = overlay
            if LOGLEVEL is logging.DEBUG and (time.time() - start_time) > 1:
                fps_counter += 1
                fps = fps_counter / (time.time() - start_time)
                logging.debug(f'FPS: {fps}')
                fps_counter = 0
                start_time = time.time()


def in_range(val, start, end):
    # determine the input vale is in the supplied range
    return (val >= start and val <= end)

def set_servo(servo, angle):
    #assert angle > 30 and angle <= 150
    #tilt_servo = GPIO.PWM(tilt_pin, 50)
    #tilt_servo.start(8)
    dutyCycle = angle / 18. + 3.
    servo.ChangeDutyCycle(dutyCycle)
    time.sleep(0.3)
    servo.stop()

def set_servos(tlt, pan):
    # Signal trap to handle keyboard interrupt
    signal.signal(signal.SIGINT, signal_handler)
    
    # Loop indefinitely
    while True:
        # The pan and tilt angles are reversed
        #panAngle = -1 * pan.value
        pan_angle = pan.value
        #tiltAngle = -1 * tlt.value
        tilt_angle = tlt.value #removed -1 because vertical flipping is off
        # If the pan angle is within the range, pan
        if in_range(pan_angle, servoRange[0], servoRange[1]):
            set_servo(pan_servo, pan_angle)
        else:
            logging.info(f'pan_angle not in range {pan_angle}')
        # If the tilt angle is within the range, tilt
        if in_range(tilt_angle, servoRange[0], servoRange[1]):
            set_servo(tilt_servo, tilt_angle)
        else:
            logging.info(f'tilt_angle not in range {tilt_angle}')


def pid_process(output, p, i, d, box_coord, origin_coord, action):
    # signal trap to handle keyboard interrupt
    signal.signal(signal.SIGINT, signal_handler)

    # create a PID and initialize it
    p = PIDController(p.value, i.value, d.value)
    p.reset()

    # loop indefinitely
    while True:
        error = origin_coord - box_coord.value
        output.value = p.update(error)
        # logging.info(f'{action} error {error} angle: {output.value}')

# ('person',)
#('orange', 'apple', 'sports ball')


def pantilt_process_manager(
    edge_tpu=False,
    labels=('person',)
):

    tilt_servo.start(8) #check effects
    pan_servo.start(8)
    with Manager() as manager:
        # set initial bounding box (x, y)-coordinates to center of frame
        center_x = manager.Value('i', 0)
        center_y = manager.Value('i', 0)

        center_x.value = RESOLUTION[0] // 2
        center_y.value = RESOLUTION[1] // 2

        # pan and tilt angles updated by independent PID processes
        pan = manager.Value('i', 0)
        tilt = manager.Value('i', 0)

        # PID gains for panning

        pan_p = manager.Value('f', 0.05)
        # 0 time integral gain until inferencing is faster than ~50ms
        pan_i = manager.Value('f', 0.1)
        pan_d = manager.Value('f', 0)

        # PID gains for tilting
        tilt_p = manager.Value('f', 0.15)
        # 0 time integral gain until inferencing is faster than ~50ms
        tilt_i = manager.Value('f', 0.2)
        tilt_d = manager.Value('f', 0)

        detect_processr = Process(target=run_detect,
                                  args=(center_x, center_y, labels, edge_tpu))

        pan_process = Process(target=pid_process,
                              args=(pan, pan_p, pan_i, pan_d, center_x, CENTER[0], 'pan'))

        tilt_process = Process(target=pid_process,
                               args=(tilt, tilt_p, tilt_i, tilt_d, center_y, CENTER[1], 'tilt'))

        servo_process = Process(target=set_servos, args=(pan, tilt))

        detect_processr.start()
        pan_process.start()
        tilt_process.start()
        servo_process.start()

        detect_processr.join()
        pan_process.join()
        tilt_process.join()
        servo_process.join()


if __name__ == '__main__':
    pantilt_process_manager()
