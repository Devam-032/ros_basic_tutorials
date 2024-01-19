import cv2
import numpy as np

def detect_cones_webcam():
    # Open the webcam
    cap = cv2.VideoCapture(0)

    cone_detected = 0  # Variable to store the detection result

    while True:
        # Read a frame from the webcam
        ret, frame = cap.read()

        # Convert the frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds for orange color in HSV
        lower_bound = np.array([0, 100, 100])
        upper_bound = np.array([20, 255, 255])

        # Create a mask using the color range
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # Apply morphological operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter contours based on area (height in this case)
        min_contour_area = 500  # Adjust based on your scenario
        max_contour_area = 3000  # Adjust based on your scenario

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if min_contour_area < area < max_contour_area:
                # Cone detected
                cone_detected = 1
                # Draw the contours on the original frame
                cv2.drawContours(frame, [cnt], -1, (0, 255, 0), 2)

        # Display the result
        cv2.imshow('Webcam - Cone Detection', frame)

        # Break the loop if 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the webcam and close all windows
    cap.release()
    cv2.destroyAllWindows()

    # Print the result
    print("Cone Detected:", cone_detected)

# Run the function
detect_cones_webcam()
