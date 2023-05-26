# Periscope package
This package is intended for use in the **MBZIRC 2024** competition. It contains the infrastructure for detecting and locating marine boats using a camera mounted on a drone. Only trained and/or customized model from the YoloV8 family needs to be provided (the current package includes 4 pre-trained models by default).

Package file structure:  
- **cam_details/** This folder contains camera intrinsics and distortion coefficients data.
- **nodes/** This folder contains all nodes scripts.
- **videos/**  This folder contains all videos for benchmark and testing.
- **weights/** This folder contains the actual weights of all trained models.
- `package.xml`, `setup.cfg` and `setup.py` are just configuration files require by ros.

If you train a new model the **.pt** file should be placed mandatory in **weights/** and only there, the same for any video used as a benchmark but in that case in **videos/**.

The second package, **periscope_msgs**, contains a custom interface for communicating the locations of marine boats.

## Requirements
Make sure to have installed [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html) on your system.  
Install all Python dependencies using the following command:
```bash
pip install -r requirements.txt

```

## Virtual environments
If you are using virtual environments with ros, you can leave the following code snippet in `setup.cfg` otherwise remove it.

```python
[build_scripts]
executable=/usr/bin/env python3	
```

## Stalker node
Main node responsible for the detection and localization of the boats.

- Subscriptions Topics: 
    - **/drone_posecov** Current pose of the drone and its respective covariance.
    - **/camera_posecov** Current pose of the camera and its respective covariance.

- Publications Topics: 
    - **/boats_location**  Polar coordinates of all detected objects in the image.
    - **/video_stream** Image frames with bounding boxes to visualize object detection.

**ULTRA Important**, before starting the node make sure to have define the appropriate params for your system when creating a Stalker node object inside main in `stalker.py`  
```bash
Args:
    device (int, optional): Camera to listen for input. 
                            Defaults to 0.
    model (string, optional): Model's file name.
                              Defaults to "yolo_demo.pt"
    classes (list, optional): Which clases to detect while using Yolo model. 
                              Defaults to [32].
    camera_resolution (tuple, optional): Resolution of input camera. 
                                         Defaults to (1920, 1080).
```
Apply your changes here:
```python
def main(args=None):
    rclpy.init(args=args) 
    stalker_node = Stalker(device=0, 
                           classes=[32, 49], 
                           camera_resolution=(640, 640))
    rclpy.spin(stalker_node)
    stalker_node.destroy_node()
    rclpy.shutdown()
```
Now you can initialize the stalker node.
```bash
ros2 run periscope stalker
```

## Video node
Node used solely for visualizing model detections.
```bash
ros2 run periscope video
```

## Demo node
Node used to test the detection capability of the trained model. It can be considered as a benchmark application.

**ULTRA Important**: Before starting the node, make sure to define the appropriate model to test and optionally specify the video inside the `main` function in `demo.py`. 

```python
def main():
    model_file_name = "best_medium.pt"
    video_file_name = "sail_amsterdam.mp4"
```
Now you can initialize the stalker node.
```bash
ros2 run periscope demo
```
