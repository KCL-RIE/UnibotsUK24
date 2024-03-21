# Ensure you have all necessary imports
from inference import get_roboflow_model
import supervision as sv
import cv2
import serial
import pandas as pd
import time 
import numpy as np
from enum import Enum,auto
class Position(Enum):
    CENTER=auto()
    CENTER_LEFT=auto()
    BOTTOM_RIGHT=auto()


# Initialize the webcam
camera = cv2.VideoCapture(0)
# Set resolution of camera
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
# Check if the resolution was set correctly
ret, frame = camera.read()
if ret:
    # Print the resolution of the frame
    height, width, channels = frame.shape
    print(f"Set Resolution: Width = 640, Height = 480")
    print(f"Actual Resolution: Width = {width}, Height = {height}")
else:
    print("Failed to grab frame from camera.")

# Initialize serial communication with Arduino
ser = serial.Serial(port='COM6', baudrate=9600, timeout=0.1)  # Update serial port as needed

# Load a pre-trained model of your choice, here using YOLOv8n as an example
model = get_roboflow_model(model_id="ping-pong-finder-w6mxk/9")

# Create supervision annotators
bounding_box_annotator = sv.BoundingBoxAnnotator()
label_annotator = sv.LabelAnnotator()


command_counts = {'right_move': 0, 'left_move': 0, 'forward_move': 0, 'stop': 0 ,'searching': 0}
#command_thresholds = {'right_move': 3, 'left_move': 3, 'forward_move': 3, 'stop': 3 'random' }
last_sent_command = None

while True:
    # Capture frame-by-frame
    ret, frame = camera.read()
    # Run inference on the current frame
    results = model.infer(frame)
    
    image_area = (frame.shape)[0] * (frame.shape)[1]
    # Load the results into the supervision Detections API
    detections = sv.Detections.from_inference(results[0].dict(by_alias=True, exclude_none=True))
    # confidence level 
    detections = detections[detections.confidence > 0.6]
    # specify class id, in this case id = 2 is white ping pong
    detections = detections[detections.class_id == 2]

    #detections = detections[]
    # Define thresholds for movement
    horizontal_threshold = frame.shape[1] / 10  # Adjust based on your needs
    vertical_threshold = frame.shape[0] / 10    # Adjust based on your needs

    frame_center_x = frame.shape[1] / 2
    frame_center_y = frame.shape[0] / 2            
    
    
    #Assume a single detected object for simplicity; adjust as needed for multiple detections
    if detections:
        max_index = 0
        #print(detections.box_area.max())
        for i in range(len(detections.box_area)):
            if detections.box_area[i] == detections.box_area.max():
                max_index = i
        detections = detections[max_index]
        #DONT NEED LABELLING
        areas = [str(round((i/image_area)*100,4)) for i in detections.box_area]

        for detection in detections:
            x, y, w, h = detection[0][0],   detection[0][1],    detection[0][2] - detection[0][0],    detection[0][3]- detection[0][1]
            cx = x + w / 2
            cy = y + h / 2
            
            # Horizontal movement
            #testing camera was mirrored while testing KEEPNOTE
             #mirrored directions
            if cx > frame_center_x and abs(cx - frame_center_x) > horizontal_threshold:
                command = "right_move"
                command_counts[command] += 1  # Increment the count every loop where the condition is true
                if command_counts[command] >= 5:  # Check if count has reached the threshold
                    ser.write(f"{command}\n".encode('utf-8'))
                    command_counts[command] = 0  # Reset the count for this command
                    print("L")
      
            elif cx < frame_center_x and abs(cx - frame_center_x) > horizontal_threshold:
                command = "left_move"
                command_counts[command] += 1
                if command_counts[command] >= 5:
                    ser.write(f"{command}\n".encode('utf-8'))
                    command_counts[command] = 0
                    print("R")
                
            elif float(areas[0]) < 1.5:
                command = "forward_move"
                command_counts[command] += 1
                if command_counts[command] >= 5:
                    ser.write(f"{command}\n".encode('utf-8'))
                    command_counts[command] = 0
                    print("F")
                
            elif float(areas[0]) > 1.5 or len(detections) == 0: 
                command = "stop"  # No significant horizontal movement needed
                command_counts[command] += 1  # This assumes you want to count 'stop' commands too
                if command_counts[command] >= 5:
                    ser.write(f"{command}\n".encode('utf-8'))
                    command_counts[command] = 0
                    print("stop")
             
                   # Vertical movement
            #REMOVE AND REPLACE WITH LOGIC FOR BOX AREA GETTING BIGGER AS ROBOT APPROACHES
            # Assuming moving forward is decreasing y (up in the frame) and backward is increasing y
            # if cy < frame_center_y - vertical_threshold:  # Adjust vertical_threshold
            #     command += " and forward"
            # elif cy > frame_center_y + vertical_threshold:
            #     command += " and backward"
    if not detections:
        command = "searching"  # No significant horizontal movement needed
        command_counts[command] += 1  # This assumes you want to count 'stop' commands too
        if command_counts[command] >= 5:
                ser.write(f"{command}\n".encode('utf-8'))
                command_counts[command] = 0
                print("searching")
                
         

    
    # Annotate the frame with inference results
    annotated_frame = bounding_box_annotator.annotate(scene=frame, detections=detections)
    annotated_frame = label_annotator.annotate(scene=annotated_frame, detections=detections)#,labels=areas)

# Display the resulting frame
    cv2.imshow('Frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

