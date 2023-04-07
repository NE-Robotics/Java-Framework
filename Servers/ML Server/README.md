# OpenCV detectors MJPEG Stream & NT4 Options
This code works on Linux/macOS/Windows using a webcam, Raspberry Pi with the Pi Camera, and on the Coral Dev
Board using the Coral Camera or a webcam. For all settings other than the Coral Dev Board, you also need a Coral
USB/PCIe/M.2 Accelerator.

## Set up your device

1.  First, be sure you have completed the [setup instructions for your Coral
    device](https://coral.ai/docs/setup/). If it's been a while, repeat to be sure
    you have the latest software.

    Importantly, you should have the latest TensorFlow Lite runtime installed
    (as per the [Python quickstart](
    https://www.tensorflow.org/lite/guide/python)). You can check which version is installed
    using the ```pip3 show tflite_runtime``` command.

2.  Install other libraries:

    ```
    bash install_requirements.sh
    pip3 \ pip install networktables
    ```


## Run the detection demo (SSD models)

```
python3 stream-detector.py
```

By default, this uses the ```mobilenet_ssd_v2_coco_quant_postprocess_edgetpu.tflite``` model.

You can change the model and the labels file using flags ```--model``` and ```--labels```.

## Run NT4 System

```
python3 nt4-detector.py
```
or
```
python3 nt4-detector.py --model X --size 240, 240 --threshold 0.4 --roboRIO_IP 10.30.44.2
```