import cv2
import numpy as np

def detect_ping_pong_balls(frame):
    # Convert frame to HSV color space for better color filtering
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Adjusted range for white color for ping pong balls
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 25, 255])
    
    # Threshold the HSV image to get only white colors
    mask = cv2.inRange(hsv_frame, lower_white, upper_white)
    
    # Use morphological operations to remove noise
    kernel = np.ones((3,3),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
    
    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Loop over the contours
    for contour in contours:
        # Approximate the contour to a circle and calculate its area and perimeter
        ((x, y), radius) = cv2.minEnclosingCircle(contour)
        area = cv2.contourArea(contour)
        
        # Check if the area is not too small and the contour is circular enough
        if area > 40:
            circularity = (4 * np.pi * area) / (cv2.arcLength(contour, True) ** 2)
            if circularity > 0.6:  # circularity closer to 1 is a better circle
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)

    return frame


# main
if __name__ == '__main__':
    # set up webcams
    capture = cv2.VideoCapture(0)

    # repeat until pressing a key "q"
    while(True):
        # capture
        retval, frame = capture.read()
        if not retval:
            break

        # Detect ping pong balls
        detected_frame = detect_ping_pong_balls(frame)
        
        # visualize
        cv2.imshow('Detected Ping Pong Balls', cv2.flip(detected_frame, 1))

        # exit if the key "q" is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
