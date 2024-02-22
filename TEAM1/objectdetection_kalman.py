import cv2
import numpy as np
from scipy.linalg import block_diag
from filterpy.kalman import KalmanFilter

def initialize_kalman_filter():
    kf = KalmanFilter(dim_x=4, dim_z=2)
    
    # State transition matrix (assuming constant velocity model)
    dt = 1  # time step
    kf.F = np.array([[1, dt, 0,  0],
                     [0,  1, 0,  0],
                     [0,  0, 1, dt],
                     [0,  0, 0,  1]])
    
    # Measurement function: reflect the fact that we can only measure the position (x, y)
    kf.H = np.array([[1, 0, 0, 0],
                     [0, 0, 1, 0]])
    
    # Measurement uncertainty
    kf.R = np.array([[1, 0],
                     [0, 1]])
    
    # Process uncertainty
    kf.Q = block_diag(np.eye(2) * 0.01, np.eye(2) * 0.01)
    
    # Initial state
    kf.x = np.array([0., 0., 0., 0.])
    
    return kf

def detect_and_track_ping_pong_balls(frame, kf, lower_white, upper_white):
    # Convert frame to HSV color space for better color filtering
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define range for white color
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 25, 255])
    
    # Threshold the HSV image to get only white colors
    mask = cv2.inRange(hsv_frame, lower_white, upper_white)
    
    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # After detection, update the Kalman filter with the coordinates of the detected ball
    for contour in contours:
        # Create a mask for the contour
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        # Draw the contour in white on the mask
        cv2.drawContours(mask, [contour], -1, 255, -1)
        
        # Calculate the mean value of the contour area using the mask
        mean_val = cv2.mean(frame, mask=mask)
        
        # Convert mean_val to HSV
        mean_val_hsv = cv2.cvtColor(np.uint8([[mean_val]]), cv2.COLOR_BGR2HSV)[0][0]

        # Define a threshold for saturation
        some_saturation_threshold = 40  # Example value, adjust according to your needs

        # Check if the mean saturation is less than the defined threshold
        if mean_val_hsv[1] < some_saturation_threshold: 
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:  # Avoid division by zero
                continue  # Skip the rest of this iteration and proceed with the next contour

            circularity = (4 * np.pi * area) / (perimeter ** 2)
            if circularity > 0.6 and area > 40:
                # If a ping pong ball is detected...
                ((x, y), radius) = cv2.minEnclosingCircle(contour)
                kf.update(np.array([[x], [y]]))
                break  # Assuming only one ball, we can break after the first detection
            
    # Predict the next state (even if no ball is detected, we still predict based on the model)
    kf.predict()
    
    # Use kf.x (the state vector) to get the predicted position of the ball
    predicted_x, predicted_vx, predicted_y, predicted_vy = kf.x
    predicted_position = (int(predicted_x), int(predicted_y))
    
    # Draw the predicted position
    cv2.circle(frame, predicted_position, 10, (0, 0, 255), 2)
    
    return frame

def on_trackbar(val):
    pass

# Initialize your trackbars
def setup_trackbars(window_name):
    cv2.createTrackbar('Hue_low', window_name, 0, 180, on_trackbar)
    cv2.createTrackbar('Sat_low', window_name, 0, 255, on_trackbar)
    cv2.createTrackbar('Val_low', window_name, 200, 255, on_trackbar) #determines brightness of the colour
    cv2.createTrackbar('Hue_high', window_name, 180, 180, on_trackbar)
    cv2.createTrackbar('Sat_high', window_name, 25, 255, on_trackbar)
    cv2.createTrackbar('Val_high', window_name, 255, 255, on_trackbar)

# Function to get current trackbar positions
def get_trackbar_values(window_name):
    values = {
        'Hue_low': cv2.getTrackbarPos('Hue_low', window_name),
        'Sat_low': cv2.getTrackbarPos('Sat_low', window_name),
        'Val_low': cv2.getTrackbarPos('Val_low', window_name),
        'Hue_high': cv2.getTrackbarPos('Hue_high', window_name),
        'Sat_high': cv2.getTrackbarPos('Sat_high', window_name),
        'Val_high': cv2.getTrackbarPos('Val_high', window_name)
    }
    return values

# main
if __name__ == '__main__':
    # Initialize Kalman filter
    kf = initialize_kalman_filter()

    # Set up webcam
    capture = cv2.VideoCapture(0)

    # Create a named window and setup trackbars
    window_name = 'Trackbars'
    cv2.namedWindow(window_name)
    setup_trackbars(window_name)

    # Repeat until pressing a key "q"
    while(True):
        # Capture
        retval, frame = capture.read()
        if not retval:
            break

        # Get current positions of all trackbars
        trackbar_values = get_trackbar_values(window_name)

        # Use trackbar values to adjust the thresholds
        lower_white = np.array([trackbar_values['Hue_low'], trackbar_values['Sat_low'], trackbar_values['Val_low']])
        upper_white = np.array([trackbar_values['Hue_high'], trackbar_values['Sat_high'], trackbar_values['Val_high']])

        # Continue with detection and tracking
        detected_and_tracked_frame = detect_and_track_ping_pong_balls(frame, kf, lower_white, upper_white)
        
        # Visualize
        cv2.imshow('Detected and Tracked Ping Pong Balls', cv2.flip(detected_and_tracked_frame, 1))

        # Exit if the key "q" is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
