# Ensure you have all necessary imports
from inference import get_roboflow_model
import supervision as sv
import roboflow
import cv2

# Initialize the webcam
camera = cv2.VideoCapture(0)

# Load a pre-trained model of your choice, here using YOLOv8n as an example
model = get_roboflow_model(model_id="ping-pong-finder-w6mxk/9")
model.confidence = 90

# Create supervision annotators
bounding_box_annotator = sv.BoundingBoxAnnotator()
label_annotator = sv.LabelAnnotator()

while True:
    # Capture frame-by-frame
    ret, frame = camera.read()
    if not ret:
        break  # If the frame wasn't captured properly, exit the loop

    # Run inference on the current frame
    results = model.infer(frame)

    # Load the results into the supervision Detections API
    detections = sv.Detections.from_inference(results[0].dict(by_alias=True, exclude_none=True))
    # confidence level 
    detections = detections[detections.confidence > 0.4]
    # specify class id, in this case id = 2 is white ping pong
    detections = detections[detections.class_id == 2]
    

    # Annotate the frame with inference results
    annotated_frame = bounding_box_annotator.annotate(scene=frame, detections=detections)
    annotated_frame = label_annotator.annotate(scene=annotated_frame, detections=detections)

    # Display the resulting frame
    cv2.imshow('Frame', annotated_frame)

    # Break the loop with the 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture
camera.release()
cv2.destroyAllWindows()
