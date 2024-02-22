# Ensure you have all necessary imports
from inference import get_roboflow_model
import supervision as sv
import cv2
import serial
import time 
from enum import Enum,auto
class Position(Enum):
    CENTER=auto()
    CENTER_LEFT=auto()
    BOTTOM_RIGHT=auto()


# Initialize the webcam
camera = cv2.VideoCapture(0)

# Initialize serial communication with Arduino
ser = serial.Serial(port='COM6', baudrate=9600, timeout=.1)  # Update serial port as needed

# Load a pre-trained model of your choice, here using YOLOv8n as an example
model = get_roboflow_model(model_id="ping-pong-finder-w6mxk/9")
model.confidence = 90

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
    detections = detections[detections.confidence > 0.5]
    # specify class id, in this case id = 2 is white ping pong
    detections = detections[detections.class_id == 2]

    areas = [str(round((i/image_area)*100,4)) for i in detections.box_area]
    # Define thresholds for movement
    horizontal_threshold = frame.shape[1] / 10  # Adjust based on your needs
    vertical_threshold = frame.shape[0] / 10    # Adjust based on your needs

    frame_center_x = frame.shape[1] / 2
    frame_center_y = frame.shape[0] / 2
    
    print(detections.xyxy)
    #Assume a single detected object for simplicity; adjust as needed for multiple detections
    if detections:
        for detection in detections:
            x, y, w, h = detection[0][0],   detection[0][1],    detection[0][2] - detection[0][0],    detection[0][3]- detection[0][1]
            cx = x + w / 2
            cy = y + h / 2
            
            # Horizontal movement
            #testing camera was mirrored while testing KEEPNOTE
            if cx > frame_center_x and abs(cx-frame_center_x) > horizontal_threshold:  # Adjust horizontal_threshold
                command = "right"
                print("GO RIGHT")
            elif cx < frame_center_x and abs(cx-frame_center_x) > horizontal_threshold:
                command = "left"
                print("GO LEFT")
            else:
                command = "stop"  # No significant horizontal movement needed
                print("STOP AND GO FORWARD")
            
            # Vertical movement
            #REMOVE AND REPLACE WITH LOGIC FOR BOX AREA GETTING BIGGER AS ROBOT APPROACHES
            # Assuming moving forward is decreasing y (up in the frame) and backward is increasing y
            if cy < frame_center_y - vertical_threshold:  # Adjust vertical_threshold
                command += " and forward"
            elif cy > frame_center_y + vertical_threshold:
                command += " and backward"

            # Send the command to the Arduino
            ser.write(f"{command}\n".encode('utf-8'))
    else:
        command = "stop"  # No ball detected, stop movement
        ser.write(f"{command}\n".encode('utf-8'))

    # Annotate the frame with inference results
    annotated_frame = bounding_box_annotator.annotate(scene=frame, detections=detections)
    annotated_frame = label_annotator.annotate(scene=annotated_frame, detections=detections,labels=areas)

    # Display the resulting frame
    cv2.imshow('Frame', annotated_frame)

    # Break the loop with the 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture
camera.release()
cv2.destroyAllWindows()
