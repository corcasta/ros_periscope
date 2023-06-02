import os
import cv2
import time
import imutils
from glob import glob
from pathlib import Path
import supervision as sv
from ultralytics import YOLO
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node

       
def main():
    model_file_name = "best_medium.pt"
    video_file_name = "sail_amsterdam.mp4"
    
    base_path = os.path.abspath(os.path.dirname(__file__))
    parent_path = str(Path(base_path).parent)
    relative_path_video = "videos/" + video_file_name
    relative_path_model = "weights/" + model_file_name
    
    #*********************FPS-DISPLAY-SETTINGS***********************
    # used to record the time when we processed last frame
    prev_frame_time = 0
    # used to record the time at which we processed current frame
    new_frame_time = 0
    # font which we will be using to display FPS
    font = cv2.FONT_HERSHEY_SIMPLEX
    #*********************FPS-DISPLAY-SETTINGS***********************
    
    #Loading video
    video = os.path.join(parent_path, relative_path_video)
           
    #Loading trained weights
    best_weights = os.path.join(parent_path, relative_path_model)

    #Model Instance
    model = YOLO(best_weights)

    box_annotator = sv.BoxAnnotator(
        thickness=1,
        text_thickness=1,
        text_scale=0.5,
        text_padding=5
    )
    
    
    for result in model.track(video, show=False, stream=True):        
        frame = result.orig_img
        detections = sv.Detections.from_yolov8(result)
        
        if result.boxes.id is not None:
            detections.tracker_id = result.boxes.id.cpu().numpy().astype(int) 
        
        #print(detections)
        labels = [
            f"#:{tracker_id}, {model.model.names[class_id]}: {confidence:0.2f}"
            for xyxy, mask, confidence, class_id, tracker_id 
            in detections
        ]
        
        frame = box_annotator.annotate(scene=frame, detections=detections, labels=labels)
        
        # time when we finish processing for this frame
        new_frame_time = time.time()
        # fps will be number of frame processed in given time frame
        # since their will be most of time error of 0.001 second
        # we will be subtracting it to get more accurate result
        fps = 1/(new_frame_time-prev_frame_time)
        prev_frame_time = new_frame_time
        # converting the fps into integer
        fps = int(fps)
        # converting the fps to string so that we can display it on frame
        # by using putText function
        fps = str(fps)
        
        # putting the FPS count on the frame
        cv2.putText(frame, "FPS: {}".format(fps), (7, 30), font, 1, (100, 255, 0), 3, cv2.LINE_AA)
  
        
        cv2.imshow("Ships-Detection-YoloV8n", imutils.resize(frame, width=1280))
        if (cv2.waitKey(20) == ord("q")):
            break
    
    
        
if __name__ == "__main__":
    # Setting the starting path of the script
    script_path = os.path.realpath(__file__)
    os.chdir(Path(script_path).parent)
    main()
