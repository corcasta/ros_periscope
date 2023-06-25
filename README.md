# Periscope package
This package is intended for use in the **MBZIRC 2024** competition. It contains the infrastructure for detecting and locating marine boats using a camera mounted on a drone. Only trained and/or customized model from the YoloV8 family needs to be provided (the current package includes 4 pre-trained models by default).

Package file structure:  
- **launch/** This directory contains all launch files.
- **cam_details/** This directory contains camera intrinsics and distortion coefficients data.
- **nodes/** This directory contains all nodes scripts.
- **videos/**  This directory contains all videos for benchmark and testing.
- **weights/** This directory contains the actual weights of all trained models.
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

```cfg
[build_scripts]
executable=/usr/bin/env python3	
```
## Launch Periscope
It is important to mentioned that before launching, either gazebo simulator must be initialized with the px4 agent/client or the real drone is on. Also make sure that the camera connection is establsihed appropriatly.
 ```bash
 ros2 launch periscope_demo.py
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
**Note**: If everything has been installed properly you should be able to run the node (DON'T FORGET TO BUILD THE PACKAGE)
```bash
ros2 run periscope demo
```


## Stalker node
Main node responsible for the detection and localization of the boats.
- Current Subscriptions Topics: 
    - **/ZR30/get_zoom_level** Current zoom level used by the camera.

- Optional Subscriptions Topics: 
    - **/drone_posecov** Current pose of the drone and its respective covariance.
    - **/camera_posecov** Current pose of the camera and its respective covariance.

- Publications Topics: 
    - **/boats_location**  Polar coordinates of all detected objects in the image.
    - **/video_stream** Image frames with bounding boxes to visualize object detection.

**ULTRA Important**, before starting the node make sure to have define the appropriate params for your system when creating a Stalker node object inside main in `stalker.py`  
```text
Args:
    source (int, rtsp, optional): Camera to listen for input. 
                            Defaults to "rtsp://192.168.144.25:8554/main.264".
    model (string, optional): Model's weights file name.
                              Defaults to "yolo_demo.pt"
    classes (list, optional): Which classes ids to detect while using Yolo model. 
                              Defaults to [32] (sports ball).
    camera_resolution (tuple, optional): Resolution of input camera. 
                                         Defaults to (1920, 1080).
```
Apply your changes here:
```python
def main(args=None):
    rclpy.init(args=args) 
    stalker_node = Stalker(source=0, 
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

## Transforms nodes
The following nodes: camera_orientation, gimbal_orientation and drone_odometry, are all related to listening to the appropriate sources for calculating their curresponding local poses. Then they are related between eacho other through the used of transforms. To understand their data sources please look at their scripts: camera_tf2_broadcaster.py, gimbal_tf2_broadcaster.py, px4_odometry_transform.py
