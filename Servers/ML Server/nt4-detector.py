"""Runs model using stream 0 (camera) and saves to out.mjpeg

TEST_DATA=../all_models

Run face detection model:
python3 detect.py \
--model ${TEST_DATA}/mobilenet_ssd_v2_face_quant_postprocess_edgetpu.tflite

Run coco model:
python3 detect.py \
--model ${TEST_DATA}/mobilenet_ssd_v2_coco_quant_postprocess_edgetpu.tflite \
--labels ${TEST_DATA}/coco_labels.txt

"""

# TODO UPDATE TO PYNTCORE FOR NT4!!!!!!!!!!!!!

import argparse
import cv2
import os
import time
import argparse
import logging
import psutil

import ntcore

import base64

from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.edgetpu import run_inference


def main():
    # TODO update defaults to actual models
    default_model_dir = '../all_models'
    default_model = 'mobilenet_ssd_v2_coco_quant_postprocess_edgetpu.tflite'
    default_labels = 'coco_labels.txt'
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', help='.tflite model path',
                        default=os.path.join(default_model_dir, default_model))
    parser.add_argument('--labels', help='label file path',
                        default=os.path.join(default_model_dir, default_labels))
    parser.add_argument('--top_k', type=int, default=3,
                        help='number of categories with highest score to display')
    parser.add_argument('--camera_idx', type=int, help='Index of which video source to use. ', default=0)
    parser.add_argument('--threshold', type=float, default=0.1,
                        help='classifier score threshold')
    parser.add_argument('--size', type=int, help='image size width height', default=(420, 420))
    parser.add_argument(
        "-p",
        "--protocol",
        type=int,
        choices=[3, 4],
        help="NT Protocol to use",
        default=4,
    )
    parser.add_argument("-ip", type=str, help="IP address to connect to")
    args = parser.parse_args()

    print('Loading {} with {} labels.'.format(args.model, args.labels))
    interpreter = make_interpreter(args.model)
    interpreter.allocate_tensors()
    inference_size = input_size(interpreter)

    cap = cv2.VideoCapture(args.camera_idx)

    # TODO Figure out if this not connecting to sim from seperate device is just a local issue or a major problem

    inst = ntcore.NetworkTableInstance.getDefault()

    identity = basename(__file__)
    if args.protocol == 3:
        inst.startClient3(identity)
    else:
        inst.startClient4(identity)

    inst.setServer(args.ip)

    ml = inst.getTable("ML Detector")

    t_s = time.time()

    # run loop to check for NT4 connection infinetly
    while True:
        # conditional run, while NT4 connected
        while inst.getConnections() != 0:
            ret, frame = cap.read()
            if not ret:
                break
            lt_s = time.time()
            cv2_im = frame

            cv2_im_rgb = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)
            cv2_im_rgb = cv2.resize(cv2_im_rgb, inference_size)
            run_inference(interpreter, cv2_im_rgb.tobytes())
            objs = get_objects(interpreter, args.threshold)[:args.top_k]

            """
            objs example
            
            objs:  [Object(id=66, score=0.26953125, bbox=BBox(xmin=15
            3, ymin=182, xmax=296, ymax=299)), Object(id=60, score=0.
            2109375, bbox=BBox(xmin=0, ymin=149, xmax=117, ymax=285))
            , Object(id=27, score=0.16015625, bbox=BBox(xmin=3, ymin=82
            , xmax=190, ymax=283))]
            """

            # publish objs to network tables as serialized string
            ml.putString("Detections", base64.b64encode(objs))

            print('objs: ', objs)
            print('latency: ', (time.time() - lt_s) * 1000, ' ms')

            ml.putNumber("Client Time", time.time() - t_s)
            ml.putNumber("CPU Usage", psutil.cpu_percent())
            ml.putNumber("RAM Usage", psutil.virtual_memory().percent)


if __name__ == '__main__':
    main()
