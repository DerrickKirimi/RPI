Real-time object detection

The following will start a PiCamera preview and render detected objects as an overlay. Ensure you're able to detect an object before trying to track it.

`rpi-deep-pantilt detect`

```
rpi-deep-pantilt detect --help

Usage: rpi-deep-pantilt detect [OPTIONS]

Options:
  --loglevel TEXT  Run object detection without pan-tilt controls. Pass
                   --loglevel=DEBUG to inspect FPS.
  --help           Show this message and exit.
```

## Real-time object tracking

The following will start a PiCamera preview, render detected objects as an overlay, and track an object's movement with the pan-tilt HAT.

By default, this will track any `person` in the frame. You can track other objects by passing `--label <label>`. For a list of valid labels, run `rpi-deep-pantilt list-labels`.

`rpi-deep-pantilt track`

```
rpi-deep-pantilt track --help 
Usage: rpi-deep-pantilt track [OPTIONS]

Options:
  --label TEXT     The class label to track, e.g `orange`. Run `rpi-deep-
                   pantilt list-labels` to inspect all valid values
                   [required]
  --loglevel TEXT
  --help           Show this message and exit.
```

## Valid labels

`rpi-deep-pantilt list-labels`

The following labels are valid tracking targets.

```
['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush']
```

# Credits

The MobileNetV3-SSD model in this package was derived from [TensorFlow&#39;s model zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md), with [post-processing ops added](https://gist.github.com/leigh-johnson/155264e343402c761c03bc0640074d8c).

The PID control scheme in this package was inspired by [Adrian Rosebrock](https://github.com/jrosebr1) tutorial [Pan/tilt face tracking with a Raspberry Pi and OpenCV](https://www.pyimagesearch.com/2019/04/01/pan-tilt-face-tracking-with-a-raspberry-pi-and-opencv/)

This package was created with
[Cookiecutter](https://github.com/audreyr/cookiecutter) and the
[audreyr/cookiecutter-pypackage](https://github.com/audreyr/cookiecutter-pypackage)
project template.
