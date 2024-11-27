import pyrealsense2 as rs
from ultralytics import YOLO
import numpy as np
import cv2
import torch
from scipy.ndimage import gaussian_filter

class PilotDetector:
    def __init__(self):
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.available_person_models = ['yolov8m.engine', 'yolov8m.pt']
        self.available_vests_models = ['vest_large_default_hyp.engine', 'vest_large_default_hyp.pt']
        self.person_model_used = None
        self.vest_model_used = None
        self.W = 1280
        self.H = 720
        print(f"[INFO] System works on {self.device.type}")
        print(f"[INFO] Frame width: {self.W}")
        print(f"[INFO] Frame height: {self.H}")

        # color camera field of view of realsense d435i
        self.horizontal_fov = 69.4

        # Configuration of depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, self.W, self.H, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, self.W, self.H, rs.format.z16, 30)

        # Start of camera stream
        print("[INFO] Starting streaming...")
        self.pipeline.start(config)
        print("[INFO] Camera ready.")

        # Load retrained YOLOv8m model for person detection and trained YOLOv8 model for jacket detection
        print("[INFO] loading model...")
        if self.device.type == 'cuda': #if GPU is available take TensorRT model    
            self.model_coco = YOLO("./"+self.available_person_models[0], task="detect")
            self.person_model_used = self.available_person_models[0]
            self.model_safety_jacket_large_default_hyp = YOLO("./"+self.available_vests_models[0], task="detect")
            self.vest_model_used = self.available_vests_models[0]
        elif self.device.type == 'cpu':
            self.model_coco = YOLO("./"+self.available_person_models[1], task="detect")#
            self.person_model_used = self.available_person_models[1]
            self.model_safety_jacket_large_default_hyp = YOLO("./"+self.available_vests_models[1], task="detect")
            self.vest_model_used = self.available_vests_models[1]

        print(f"[INFO] Loaded model for persons: {self.person_model_used}")
        print(f"[INFO] Loaded model for vests: {self.vest_model_used}")
        
        self.labels_coco = self.model_coco.names #label "person"
        self.labels_jackets = self.model_safety_jacket_large_default_hyp.names #label "safety_jacket"
        self.color_person = (0,0,255) #color for person bounding box
        self.color_jacket = (0,255,0) #color for jacket bounding box
        self.confidence_threshold = 0.5

        # Show camera view
        #cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)

    def detect(self):
        # to align color and depth frames
        align = rs.align(rs.stream.color) 

        # Frames interception
        frames = self.pipeline.wait_for_frames()
        frames = align.process(frames)

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        # Convertion of images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Inference
        results_coco = self.model_coco(color_image, classes=[0], conf=self.confidence_threshold, stream=True)# return a list of Results objects
        results_jacket = self.model_safety_jacket_large_default_hyp(color_image, conf=self.confidence_threshold, classes=[1], stream=True) 

        #empty lists for future collecting of bounding boxes
        coco_boxes = []
        jacket_boxes = []
        # Process results list
        # This loop returns list of boxes for every person and every jacket identified on frame
        for model_results, color, bounding_boxes in [(results_coco, self.color_person, coco_boxes), (results_jacket, self.color_jacket, jacket_boxes)]:
            for result in model_results:
                boxes = result.boxes  
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    bounding_boxes.append((x1, y1, x2, y2))

                # Initiation of variables for distance and angle
                distance = None#0.0
                angle = None#0.0
                #looking for person box that has jacket box inside 
                # there is an assumption that it would be only one such object on frame
                for coco_box in coco_boxes:
                    for jacket_box in jacket_boxes:
                        if jacket_box[0]>=coco_box[0] and jacket_box[1]>=coco_box[1] and jacket_box[2]<=coco_box[2] and jacket_box[3]<=coco_box[3]:
                            x1_jacket=jacket_box[0]
                            y1_jacket=jacket_box[1]
                            x2_jacket=jacket_box[2]
                            y2_jacket=jacket_box[3]

                            x1_person=coco_box[0]
                            y1_person=coco_box[1]
                            x2_person=coco_box[2]
                            y2_person=coco_box[3]

                            # there will be gaussian filter on depth image to minimize noise in distance measurement
                            # filtered will be middle 10% area of jacket bounding box
                            bb_width = x2_jacket - x1_jacket
                            bb_height = y2_jacket - y1_jacket
                            c_x1 = x1_jacket + int(0.45 * bb_width)
                            c_x2 = x2_jacket - int(0.45 * bb_width)
                            c_y1 = y1_jacket + int(0.45 * bb_height)
                            c_y2 = y2_jacket - int(0.45 * bb_height)

                            distance_area_to_filter = depth_image[c_y1:c_y2, c_x1:c_x2]
                            filtered_distance_area = gaussian_filter(distance_area_to_filter/1000, sigma = 7)

                            jacket_box_center = ((x1_jacket+x2_jacket)//2, (y1_jacket+y2_jacket)//2)
                            # calculate horizontal angle to pilot
                            frame_center = self.W // 2
                            # negative on left side of frame, positive on right side od frame
                            angle = (frame_center - jacket_box_center[0]) / self.W * self.horizontal_fov

                            #calculate distance to detected pilot
                            # distance from camera is measured from the parallel plane of the imagers and not the absolute range 
                            # so to have absoulute distance it has to be calculated with cos
                            distance = round(filtered_distance_area[(c_y2 - c_y1) // 2, (c_x2 - c_x1) // 2], 2)
                            # angle as radians:
                            angle = round(angle * np.pi/180, 2)
                            distance = round(distance/np.cos(angle), 2)

                            # draw bounding box for visualization purposes
                            cv2.rectangle(color_image, (x1_person, y1_person), (x2_person, y2_person), color=(0, 255, 0), thickness=2)
                            cv2.putText(color_image, f'Pilot distance:{distance:.2f}m angle:{angle:.2f}rads', (x1_person, y1_person - 5), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=color, thickness=1)
        #cv2.imshow('RealSense', color_image)
        #cv2.waitKey(1)
        return distance, angle
            
    def stop(self):
        #cv2.destroyAllWindows()
        self.pipeline.stop()
