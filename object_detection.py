import argparse
import sys
import threading
from functools import lru_cache

import json
import cv2
import serial
import numpy as np
import time


from inverse_kinematic import run_robotics_script

from picamera2 import MappedArray, Picamera2
from picamera2.devices import IMX500
from picamera2.devices.imx500 import (NetworkIntrinsics,
                                      postprocess_nanodet_detection)

last_detections = []


ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  
time.sleep(2)  


class Detection:
    def __init__(self, coords, category, conf, metadata):
        """Create a Detection object, recording the bounding box, category and confidence."""
        self.category = category
        self.conf = conf
        self.box = imx500.convert_inference_coords(coords, metadata, picam2)

class CameraIntrinsics:
    def __init__(self, file_path=None):
        if file_path:
            with open(file_path, 'r') as f:
                data = json.load(f)
                self.fx = data["fx"]
                self.fy = data["fy"]
                self.cx = data["cx"]
                self.cy = data["cy"]
                self.distortion = np.array(data.get("distortion", [0, 0, 0, 0, 0])) 

intrinsics_parameters = CameraIntrinsics("intrinsics.json")


def parse_detections(metadata: dict):
    """Parse the output tensor into a number of detected objects, scaled to the ISP output."""
    global last_detections
    bbox_normalization = intrinsics.bbox_normalization
    bbox_order = intrinsics.bbox_order
    threshold = args.threshold
    iou = args.iou
    max_detections = args.max_detections

    np_outputs = imx500.get_outputs(metadata, add_batch=True)
    input_w, input_h = imx500.get_input_size()
    if np_outputs is None:
        return last_detections
    if intrinsics.postprocess == "nanodet":
        boxes, scores, classes = \
            postprocess_nanodet_detection(outputs=np_outputs[0], conf=threshold, iou_thres=iou,
                                          max_out_dets=max_detections)[0]
        from picamera2.devices.imx500.postprocess import scale_boxes
        boxes = scale_boxes(boxes, 1, 1, input_h, input_w, False, False)
    else:
        boxes, scores, classes = np_outputs[0][0], np_outputs[1][0], np_outputs[2][0]
        if bbox_normalization:
            boxes = boxes / input_h

        if bbox_order == "xy":
            boxes = boxes[:, [1, 0, 3, 2]]
        boxes = np.array_split(boxes, 4, axis=1)
        boxes = zip(*boxes)

    last_detections = [
        Detection(box, category, score, metadata)
        for box, score, category in zip(boxes, scores, classes)
        if score > threshold
    ]
    return last_detections


@lru_cache
def get_labels():
    labels = intrinsics.labels

    if intrinsics.ignore_dash_labels:
        labels = [label for label in labels if label and label != "-"]
    return labels

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1) 

coordinates_buffer = []
sent_once = False
flag = False 
    
def draw_detections(request, stream="main"):
    centers = []
    global sent_once
    global flag

    """Draw the detections for this request onto the ISP output."""
    detections = last_results
    if detections is None:
        return
    labels = get_labels()
    with MappedArray(request, stream) as m:
        object_centers = []
        for detection in detections:
            x, y, w, h = detection.box
            label = f"{labels[int(detection.category)]} ({detection.conf:.2f})"

            # Calculate text size and position
            (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            text_x = x + 5
            text_y = y + 15

            # Create a copy of the array to draw the background with opacity
            overlay = m.array.copy()

            # Draw the background rectangle on the overlay
            cv2.rectangle(overlay,
                          (text_x, text_y - text_height),
                          (text_x + text_width, text_y + baseline),
                          (255, 255, 255),  # Background color (white)
                          cv2.FILLED)

            alpha = 0.30
            cv2.addWeighted(overlay, alpha, m.array, 1 - alpha, 0, m.array)

            # Draw text on top of the background
            cv2.putText(m.array, label, (text_x, text_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            # Draw detection box
            cv2.rectangle(m.array, (x, y), (x + w, y + h), (0, 255, 0, 0), thickness=2)
            
            center_x = x + w // 2
            center_y = y + h // 2

            cv2.circle(m.array, (int(center_x), int(center_y)), 5, (255, 0, 0), -1)

            # Add the center to the list
            object_centers.append((center_x, center_y))
            
        if object_centers:
            centers = object_centers  # Keep only the last detections
            selected_center = min(centers, key=lambda c: c[1]) 
 
     

            u = int(centers[0][0])
            v = int(centers[0][1])
            fx = intrinsics_parameters.fx
            fy = intrinsics_parameters.fy
            principal_point = (intrinsics_parameters.cx, intrinsics_parameters.cy)
            depth=-0.31

            centers.clear()
            camera_coordinates = pixel_to_camera_coordinates(u, v, fx, fy, principal_point, depth)
            # Convert camera coordinates to world coordinates
            world_coordinates = camera_to_world_coordinates(camera_coordinates, translation_camera_to_world, rotation_camera_to_world)
            robot_base_coordinates = convert_to_robot_base(world_coordinates, translation_robot, ground_truth_rotation_angles)
            if len(coordinates_buffer) < 10 and not flag:
                coordinates_buffer.append(robot_base_coordinates)
        


            if len(coordinates_buffer) == 10 and not flag:
                # Compute the average of the coordinates once the buffer is full
                avg_robot_base_coordinates = np.mean(coordinates_buffer, axis=0)
                print('Averaged robot base coord:', avg_robot_base_coordinates)
                
                # Convert the averaged coordinates to a string and send via subprocess
                coordinates_str = avg_robot_base_coordinates.tolist()


                run_robotics_script(coordinates_str)
            
                print('before',coordinates_buffer)
                
                # Clear the buffer to collect new coordinates for the next object
                coordinates_buffer.clear()
                print("Buffer cleared for the next set of coordinates.")
                print('after',coordinates_buffer)
                
                # Reset the flag after sending
                flag = True
                print("Flag set to True after sending coordinates.")


        if intrinsics.preserve_aspect_ratio:
            b_x, b_y, b_w, b_h = imx500.get_roi_scaled(request)
            color = (255, 0, 0)  # red
            cv2.putText(m.array, "ROI", (b_x + 5, b_y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            cv2.rectangle(m.array, (b_x, b_y), (b_x + b_w, b_y + b_h), (255, 0, 0, 0))



def listen_for_done():
    global flag
    while True:
        response = ser.readline().decode().strip()  # Read incoming data
        if response == "DONE":
            flag = False  # Update the flag
            print("Sent once:", flag)  # Print confirmation
              # Exit loop after receiving "DONE
        


def rotation_matrix(yaw, pitch, roll):
    """
    Create a combined rotation matrix from yaw, pitch, and roll angles.
    """
    Rz_yaw = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    Ry_pitch = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    Rx_roll = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    # Combined rotation matrix
    R = np.dot(Rz_yaw, np.dot(Ry_pitch, Rx_roll))
    return R
    
def pixel_to_camera_coordinates(u, v, fx, fy, principal_point, depth ):
    """
    Convert pixel coordinates to camera coordinates using intrinsic parameters.
    """
    cx, cy = principal_point
    x = (u - cx) * depth / fx
    y = (v - cy) * depth / fy
    return np.array([x, y, depth])

def camera_to_world_coordinates(camera_coordinates, translation_camera_to_world, rotation_camera_to_world):
    """
    Convert camera coordinates to world coordinates.
    """
    world_coordinates = np.dot(rotation_camera_to_world, camera_coordinates) + translation_camera_to_world
    return world_coordinates

def convert_to_robot_base(world_coordinates, translation_robot, ground_truth_rotation_angles):
    """
    Convert world coordinates to robot base coordinates using translation and rotation.
    """
    yaw, pitch, roll = ground_truth_rotation_angles
    R = rotation_matrix(yaw, pitch, roll)
    
    # Convert to robot base coordinates
    robot_base_coordinates = R.T @ (world_coordinates - translation_robot)
    robot_base_coordinates[0] *= -1  # Flip x-coordinate
    return robot_base_coordinates         


translation_camera_to_world = np.array([0, 0, 0.31])  # Camera position in the world frame
rotation_camera_to_world = rotation_matrix(3.14159, 0, 0)  # Camera orientation in the world frame

translation_robot = np.array([0.03, -0.22, 0.0])  # Robot base position in the world frame
ground_truth_rotation_angles = np.array([0, 0, 0])  # Robot base orientation in the world frame

# Convert pixel coordinates to camera coordinates




def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=str, help="Path of the model",
                        default="/home/pi/Desktop/rpkfolder/network.rpk")
    parser.add_argument("--fps", type=int, default=20, help="Frames per second")
    parser.add_argument("--bbox-normalization", action=argparse.BooleanOptionalAction, default=True, help="Normalize bbox")
    parser.add_argument("--bbox-order", choices=["yx", "xy"], default="xy",
                        help="Set bbox order yx -> (y0, x0, y1, x1) xy -> (x0, y0, x1, y1)")
    parser.add_argument("--threshold", type=float, default=0.60, help="Detection threshold")
    parser.add_argument("--iou", type=float, default=0.65, help="Set iou threshold")
    parser.add_argument("--max-detections", type=int, default=1, help="Set max detections")
    parser.add_argument("--ignore-dash-labels", action=argparse.BooleanOptionalAction, help="Remove '-' labels ")
    parser.add_argument("--postprocess", choices=["", "nanodet"],
                        default=None, help="Run post process of type")
    parser.add_argument("-r", "--preserve-aspect-ratio", action=argparse.BooleanOptionalAction,
                        help="preserve the pixel aspect ratio of the input tensor")
    parser.add_argument("--labels", type=str, default="/home/pi/Desktop/rpkfolder/labels.txt",
                        help="Path to the labels file")
    parser.add_argument("--print-intrinsics", action="store_true",
                        help="Print JSON network_intrinsics then exit")
    return parser.parse_args()



if __name__ == "__main__":
    thread1 = threading.Thread(target=listen_for_done, daemon=True)
    thread1.start()   

    args = get_args()
            
    # This must be called before instantiation of Picamera2
    imx500 = IMX500(args.model)
    intrinsics = imx500.network_intrinsics
    if not intrinsics:
        intrinsics = NetworkIntrinsics()
        intrinsics.task = "object detection"
    elif intrinsics.task != "object detection":
        print("Network is not an object detection task", file=sys.stderr)
        exit()

    # Override intrinsics from args
    for key, value in vars(args).items():
        if key == 'labels' and value is not None:
            with open(value, 'r') as f:
                intrinsics.labels = f.read().splitlines()
        elif hasattr(intrinsics, key) and value is not None:
            setattr(intrinsics, key, value)

    # Defaults
    if intrinsics.labels is None:
        with open("assets/coco_labels.txt", "r") as f:
            intrinsics.labels = f.read().splitlines()
    intrinsics.update_with_defaults()

    if args.print_intrinsics:
        print(intrinsics)
        exit()

    picam2 = Picamera2(imx500.camera_num)
    config = picam2.create_preview_configuration(controls={"FrameRate": intrinsics.inference_rate}, buffer_count=12)

    imx500.show_network_fw_progress_bar()
    picam2.start(config, show_preview=True)

    if intrinsics.preserve_aspect_ratio:
        imx500.set_auto_aspect_ratio()

    last_results = None
    picam2.pre_callback = draw_detections
    while True:
        last_results = parse_detections(picam2.capture_metadata())
   
            