import os
import cv2
import time
import torch
import rclpy
import imutils
import numpy as np
import pandas as pd
from glob import glob
from pathlib import Path
import supervision as sv
from rclpy.node import Node
from ultralytics import YOLO
import matplotlib.pyplot as plt
from periscope_msgs.msg import PolarPoints
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32
import transforms3d as tf

from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener 



class Stalker(Node):
    def __init__(self, device=0, model="yolo_demo.pt", classes=[32], camera_resolution=(1920, 1080)):
        """
        In charge of doing all the setup and processing for target localization 
        and publishing coordinates across subscribed nodes.
        
        Args:
            device (int, optional): Camera to listen for input. 
                                    Defaults to 0.
            model (string, optional): Model's file name.
                                      Defaults to "yolo_demo.pt"
            classes (list, optional): Which clases to detect while using Yolo model. 
                                      Defaults to [32].
            camera_resolution (tuple, optional): Resolution of input camera. 
                                                 Defaults to (1920, 1080).
        """
        super().__init__('stalker')
        
        # Attributes for setting up YOLO
        self.__base_path = os.path.abspath(os.path.dirname(__file__))
        self.__parent_path = str(Path(self.__base_path).parent)
        self.__relative_path_model = "/weights/{}".format(model)
        self.__detector = YOLO(self.__parent_path  + self.__relative_path_model)
        self.__tracker = self.__detector.track(source="rtsp://192.168.144.25:8554/main.264",
                                               #source=device, 
                                               classes=classes,
                                               conf=0.3, 
                                               show=False,
                                               stream=True)
        
        
        self.__box_annotator = sv.BoxAnnotator(thickness=1,
                                               text_thickness=1,
                                               text_scale=0.5,
                                               text_padding=5)
              
        # Used to convert between ROS and OpenCV images
        self.__br = CvBridge()
        self.__tf_buffer = Buffer()
        self.__tf_listener = TransformListener(self.__tf_buffer, self)
        
        
        # Rotation matrix of camera frame wrt world/odom frame
        self.R_wc = 0
        
        # Position of camera frame wrt world/odom frame
        self.O_wc = 0
        
        # Message instance used to send all vessels locations.
        self.__msg = PolarPoints()

        # Camera details (resolution, intrinsincs, dist coefs, etc..)
        self.cam_width = camera_resolution[0]
        self.cam_height = camera_resolution[1]
        
        # The following two attributes come from the calibration script
        self.intrinsic_matrix = pd.read_csv(str(Path(os.path.abspath(os.path.dirname(__file__))).parent) 
                                            + "/cam_details/1.0x/intrinsic_matrix.txt", 
                                            delim_whitespace=True, 
                                            header=None).to_numpy()
        
        #self.intrinsic_newmatrix = pd.read_csv(str(Path(os.path.abspath(os.path.dirname(__file__))).parent) 
        #                                     + "/cam_details/newcam_intrinsics.txt", 
        #                                     delim_whitespace=True, 
        #                                     header=None).to_numpy()
        
        self.dist_coefficients = pd.read_csv(str(Path(os.path.abspath(os.path.dirname(__file__))).parent) 
                                             + "/cam_details/1.0x/dist_coefficients.txt", 
                                             delim_whitespace=True, 
                                             header=None).to_numpy()
        
        
        ## ROS components for communication
        #self._drone_pose = self.create_subscription(PoseWithCovarianceStamped, 
        #                                            'drone_posecov', 
        #                                            self.__update_drone_pose, 
        #                                            10)
        #self._camera_pose_cov = self.create_subscription(PoseWithCovarianceStamped, 
        #                                                 'camera_posecov', 
        #                                                 self.__update_camera_pose, 
        #                                                 10)
        
        self.zoom_level = self.create_subscription(Float32, 
                                                   'ZR30/get_zoom_level', 
                                                   self.__update_camera_details, 
                                                   10)
        
        self._polar_points_publisher = self.create_publisher(PolarPoints, 
                                                             'boats_location', 
                                                             10) 
        self._video_frames_publisher = self.create_publisher(Image, 
                                                             'video_stream',
                                                             10)
        
        # Create the timer
        self.timer = self.create_timer(0.01, self.stalking_callback)
          
    def __update_camera_details(self, msg):
        """
        Updates values representing the intrinsic matrix and the dist
        coefficients corresponding the lense being used.

        Args:
            msg (Float32): Zoom level used by the camera
            
        Returns:
            None
        """
        if np.round(msg.data, 1) == 1.0 or np.round(msg.data, 1) == 5.3 or np.round(msg.data, 1) == 10.2 or np.round(msg.data, 1) == 19.3:
            # The following two attributes come from the calibration script
            self.intrinsic_matrix = pd.read_csv(str(Path(os.path.abspath(os.path.dirname(__file__))).parent) 
                                                + "/cam_details/{}x/intrinsic_matrix.txt".format(np.round(msg.data, 1)), 
                                                delim_whitespace=True, 
                                                header=None).to_numpy()
            
            #self.intrinsic_newmatrix = pd.read_csv(str(Path(os.path.abspath(os.path.dirname(__file__))).parent) 
            #                                     + "/cam_details/newcam_intrinsics.txt", 
            #                                     delim_whitespace=True, 
            #                                     header=None).to_numpy()
            
            self.dist_coefficients = pd.read_csv(str(Path(os.path.abspath(os.path.dirname(__file__))).parent) 
                                                + "/cam_details/{}x/dist_coefficients.txt".format(np.round(msg.data, 1)), 
                                                delim_whitespace=True, 
                                                header=None).to_numpy() 
    
    def _denormalize(self, points):
        """
        Denormalize pixel coords of YOLO detected objects.
        Denormalize means retrieve the actual (x, y) of the image in pixels.
        
        Normalization Formulas:    
            Normalized(Xcentre) = (x_centre)/Image_Width
            Normalized(Ycentre) = (y_centre)/Image_Height
            Normalized(w) = w/Image_Width
            Normalized(h) = h/Image_Height
            
        Args:
            points (numpy.ndarray): Array of shape (N, 2).
                                    Array of normalized pixel coords (x, y) 
                                    of detected objects in the image. 
        
        Returns:
            points (numpy.ndarray): Array of shape (N, 2).
                                    Array of denormalized pixel coords (x, y)
                                    of detected objects in the image. 
        """
        
        # DENORMALIZE
        # x-center-bb
        points[:, 0] = points[:, 0]*self.cam_width 
        # y-center-bb
        points[:, 1] = points[:, 1]*self.cam_height
        
        ## width-bb 
        #objects[:, 2] = objects[:, 2]*self.cam_width 
        ## height-bb
        #objects[:, 3] = objects[:, 3]*self.cam_height 
        
        return points
    
    
    def _shift(self, points):
        #Shift the point of interest from center to floor
        #of bounding box
        #                   y           height     weight
        points[:, 1] = points[:, 1] + points[:, 3]*(0.75)
        return points
    

    def _undistort(self, points, cam_intrinsics, dist_coeffs, iter_num=5):
        """
        Undistort pixel point in image.
        Undistort means retrieve the real (x, y) of the frame in pixels 
        by taking into account camera intrinsics and distortion coefficients.
            
        Args:
            points (numpy.ndarray): Array of shape (N, 2).
                                    Array of raw pixel coords (x, y) 
                                    of detected objects in the image. 
        
        Returns:
            points (numpy.ndarray): Array of shape (N, 2).
                                    Array of undistorted pixel coords (x, y)
                                    of detected objects in the image. 
        """
        
        k1, k2, p1, p2, k3 = np.squeeze(dist_coeffs)
        fx, fy = cam_intrinsics[0, 0], cam_intrinsics[1, 1]
        cx, cy = cam_intrinsics[:2, 2]
        x = points[:, 0]
        y = points[:, 1]    
        
        x = (x - cx) / fx
        x0 = x
        y = (y - cy) / fy
        y0 = y
        
        for _ in range(iter_num):
            r2 = x ** 2 + y ** 2
            k_inv = 1 / (1 + k1 * r2 + k2 * r2**2 + k3 * r2**3)
            delta_x = 2 * p1 * x*y + p2 * (r2 + 2 * x**2)
            delta_y = p1 * (r2 + 2 * y**2) + 2 * p2 * x*y
            x = (x0 - delta_x) * k_inv
            y = (y0 - delta_y) * k_inv
        
        points = np.column_stack((x * fx + cx, y * fy + cy))
        return points
    
    
    def _cart2pol(self, points):
        """
        Transform cartesian points (x,y,z) in to polar points (rho, phi).
            
        Args:
            points (numpy.ndarray): Array of shape (N, 2) or (N, 3).
                                    Array of cartesian coords (x, y) or (x, y, z). 
        
        Returns:
            points (numpy.ndarray): Array of shape (N, 2).
                                    Array of polar coords (rho, phi).
        """
        
        x = points[:, 0]
        y = points[:, 1]
        
        rho = np.sqrt(x**2 + y**2).reshape(-1,1)
        phi = np.arctan2(y, x).reshape(-1,1)
        
        polar_points = np.hstack((rho, phi))
        
        return polar_points
    
    
    def _pol2cart(self, points):
        """
        Transform polar coords (x,y,z) in to cartesian coords (rho, phi).
            
        Args:
            points (numpy.ndarray): Array of shape (N, 2).
                                    Array of polar coords (rho, phi). 
        
        Returns:
            points (numpy.ndarray): Array of shape (N, 2).
                                    Array of cartesian coords (x, y).
        """
        
        rho = points[:, 0]
        phi = points[:, 1]
        
        x = (rho * np.cos(phi)).reshape(-1,1)
        y = (rho * np.sin(phi)).reshape(-1,1)
        
        cart_points = np.hstack((x, y))
        
        return cart_points

    def localize(self, points):
        """
        Localize points from image plane into world frame represented in polar coords.
        

        Args:
            points (numpy.ndarray): Array of shape (N, 2).
                                    Array of normalized pixel coords (x, y) 
                                    of detected objects in the image. 
        
        Returns:
            points (numpy.ndarray): Array of shape (N, 2).
                                    Array of polar coords (rho, phi).
         
        """
        
        points = self._denormalize(points) 
        #points = self._shift(points)
        points = self._undistort(points, self.intrinsic_matrix, self.dist_coefficients)
        
        # These points will be rearrange to be:
        #   row0: x
        #   row1: y
        #   columns: represents individual points   
        points = points.transpose()

        #Localize point in World Coords
        #Transform Point to a Homogeneous Point
        points = np.vstack((points, np.ones((1, len(points[0,:])))))
        self.get_logger().info('Points px_coords: \n{}'.format(points))
        
        A_ckpx = np.linalg.inv(self.intrinsic_matrix) @ points 
        #self.get_logger().info('intrinsic_matrix: \n{}'.format(self.intrinsic_matrix))
        #self.get_logger().info('A_ckpx: \n{}'.format(A_ckpx))
        #R_wc = t_wc[0:3, 0:3]
        #O_wc = t_wc[0:3, 3][:, np.newaxis]
        #Z = np.asarray([[0],[0],[1]])

        #Point K with respect to World Frame
        A_wk = self.O_wc + (-self.O_wc[2,:]/(np.dot(self.R_wc, A_ckpx)[2,:])) * np.dot(self.R_wc, A_ckpx)
        
        # This last transformation is just to have the points
        # in a format where each row represents a unique point
        # instead of the mathematical matrix representation
        # where each column represents a unique point.
        A_wk = A_wk.transpose()
        self.get_logger().info('Target coords: \n{}'.format(A_wk))
        
        #Point K with respect to World Frame in Polar coords
        A_wk_polar = self._cart2pol(A_wk)

        return A_wk_polar



    def localize2(self, points):
        """_summary_
        Args:
            points (_type_): raw pixel points from yolo model
        Returns:
            _type_: _description_
        """
        points = self._denormalize(points) 
        #points = self._shift(points)
        points = self._undistort(points, self.intrinsic_matrix, self.dist_coefficients)
        
        # These points will be rearrange to be:
        #   row0: x
        #   row1: y
        #   columns: represents individual points   
        points = points.transpose()

        #Localize point in World Coords
        #Transform Point to a Homogeneous Point
        points = np.vstack((points, np.ones((1, len(points[0,:])))))
        self.get_logger().info('Points px_coords: \n{}'.format(points))

        #Drone pose relative to World frame:
        phi   = 0
        theta = 0
        psi   = 0
        Od_x  = 0
        Od_y  = 0
        Od_z  = 0.235

        rot_x = np.asarray([[1,           0,            0],
                            [0, np.cos(phi), -np.sin(phi)],
                            [0, np.sin(phi),  np.cos(phi)]])

        rot_y = np.asarray([[np.cos(theta),  0, np.sin(theta)],
                            [0,              1,             0],
                            [-np.sin(theta), 0, np.cos(theta)]])

        rot_z = np.asarray([[np.cos(psi), -np.sin(psi), 0],
                            [np.sin(psi),  np.cos(psi), 0],
                            [0,                      0, 1]])

        rot_wd = rot_x @ rot_y @ rot_z
        t_wd = np.vstack((rot_wd, [0,0,0]))
        t_wd = np.hstack((t_wd, [[Od_x], [Od_y], [Od_z], [1]]))


        #Camera pose relative to Drone frame:
        #For the moment we are just assuming that the camera is fixed
        #to the drone's movement. Later we will need to provide the gimball
        #orientation to have the correct matrix for the camera relative to 
        #the drone.
        t_dc = np.asarray([[ 0,  0, 1, 0],
                           [-1,  0, 0, 0],
                           [ 0, -1, 0, 0],
                           [ 0,  0, 0, 1]])

        #Camera pose relative to World frame:
        t_wc = t_wd@t_dc
        
        A_ckpx = np.linalg.inv(self.intrinsic_matrix) @ points 
        self.get_logger().info('intrinsic_matrix: \n{}'.format(self.intrinsic_matrix))
        
        R_wc = t_wc[0:3, 0:3]
        O_wc = t_wc[0:3, 3][:, np.newaxis]
        Z = np.asarray([[0],[0],[1]])

        #Point K with respect to World Frame
        A_wk = O_wc + (-O_wc[2,:]/(np.dot(R_wc, A_ckpx)[2,:])) * np.dot(R_wc, A_ckpx)
        self.get_logger().info('DOT1: \n{}'.format((-O_wc[2,:]/(np.dot(R_wc, A_ckpx)[2,:]))))
        self.get_logger().info('DOT2: \n{}'.format(np.dot(R_wc, A_ckpx)))
        self.get_logger().info('DOT3: \n{}'.format((-O_wc[2,:]/(np.dot(R_wc, A_ckpx)[2,:])) * np.dot(R_wc, A_ckpx)))
        
        # This last transformation is just to have the points
        # in a format where each row represents a unique point
        # instead of the mathematical matrix representation
        # where each column represents a unique point.
        A_wk = np.transpose(A_wk)

        self.get_logger().info('A_wk: \n{}'.format(A_wk))
        #Point K with respect to World Frame in Polar coords
        #Point K with respect to World Frame in Polar coords
        A_wk_polar = self._cart2pol(A_wk)

 

        return A_wk_polar
    
    def stalking_callback(self):
        """
        Main loop of the ros node (stalker) 
        """
        
        # Updates current pose of camera frame with respect to world/odom frame
        try:
            t = self.__tf_buffer.lookup_transform(
                "odom",
                "camera",
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {"camera"} to {"odom"}: {ex}')
            return
        
        self.R_wc = tf.quaternions.quat2mat([t.transform.rotation.w,
                                             t.transform.rotation.x,
                                             t.transform.rotation.y,
                                             t.transform.rotation.z,])     
        #self.get_logger().info('R_wc: \n{}'.format(self.R_wc))   
        self.O_wc = np.array([t.transform.translation.x, 
                              t.transform.translation.y, 
                              t.transform.translation.z]).reshape((3,1))
        #self.get_logger().info('O_wc: \n{}'.format(self.O_wc)) 
        
        #self.get_logger().info('Camera position: {}'.format(self.O_wc))
        
        
        self.get_logger().info('intrinsic_matrix: {}'.format(self.intrinsic_matrix))
        # Process the current video frame to localize vessels
        for result in self.__tracker:
            frame = result.orig_img
            #self.get_logger().info('FRAME SIZE: \n{}'.format(frame.shape)) 
            detections = sv.Detections.from_yolov8(result)
            
            # Normalized Bounding Boxes of detected boats
            boats_bbn = result.boxes.xywhn.numpy()
            # Normalized X,Y of detected boats
            boats = boats_bbn[0:2]
            
            # If objects exist publish locations
            if boats.size != 0:
                polar_points_world = self.localize(boats)
                self.__msg.phi = list(polar_points_world[:, 0])
                self.__msg.rho = list(polar_points_world[:, 1])
                self._polar_points_publisher.publish(self.__msg)
                    
                if result.boxes.id is not None:
                    detections.tracker_id = result.boxes.id.cpu().numpy().astype(int) 
                    
                labels = [
                    f"#:{tracker_id}, {self.__detector.model.names[class_id]}: {confidence:0.2f}"
                    for xyxy, mask, confidence, class_id, tracker_id 
                    in detections
                ]
                
                frame = self.__box_annotator.annotate(scene=frame, detections=detections, labels=labels)
            self._video_frames_publisher.publish(self.__br.cv2_to_imgmsg(frame))
            break
            
    
def main(args=None):
    rclpy.init(args=args)
    stalker_node = Stalker(device=3, classes=[32, 49], camera_resolution=(1280, 720))
    rclpy.spin(stalker_node)
    stalker_node.destroy_node()
    rclpy.shutdown()
    
    
         
if __name__ == "__main__":    
    # Setting the starting path of the script
    script_path = os.path.realpath(__file__)
    os.chdir(Path(script_path).parent)
    print(os.getcwd())
    main()				
    
    
    
