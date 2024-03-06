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
#set resolution of camera
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
ser = serial.Serial(port='COM3', baudrate=9600, timeout=.1)  # Update serial port as needed

# Load a pre-trained model of your choice, here using YOLOv8n as an example
model = get_roboflow_model(model_id="ping-pong-finder-w6mxk/9")

# Create supervision annotators
bounding_box_annotator = sv.BoundingBoxAnnotator()
label_annotator = sv.LabelAnnotator()



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
             
            if cx > frame_center_x and abs(cx-frame_center_x) > horizontal_threshold:  # Adjust horizontal_threshold
                command = "left"
                print("L")
            elif cx < frame_center_x and abs(cx-frame_center_x) > horizontal_threshold:
                command = "right"
                print("R")
            elif float(areas[0])< 1.5:
                command = "forward"
                print("F")
            elif float(areas[0]) > 1.5: 
                command = "stop"  # No significant horizontal movement needed
                print("stop")
            
            # Vertical movement
            #REMOVE AND REPLACE WITH LOGIC FOR BOX AREA GETTING BIGGER AS ROBOT APPROACHES
            # Assuming moving forward is decreasing y (up in the frame) and backward is increasing y
            # if cy < frame_center_y - vertical_threshold:  # Adjust vertical_threshold
            #     command += " and forward"
            # elif cy > frame_center_y + vertical_threshold:
            #     command += " and backward"

            # Send the command to the Arduino
            ser.write(f"{command}\n".encode('utf-8'))
    else:
        command = "stop"  # No ball detected, stop movement
        ser.write(f"{command}\n".encode('utf-8'))

    
    # Annotate the frame with inference results
    annotated_frame = bounding_box_annotator.annotate(scene=frame, detections=detections)
    annotated_frame = label_annotator.annotate(scene=annotated_frame, detections=detections)#,labels=areas)

    # Display the resulting frame
    cv2.imshow('Frame', annotated_frame)

    # Break the loop with the 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture
camera.release()
cv2.destroyAllWindows()
