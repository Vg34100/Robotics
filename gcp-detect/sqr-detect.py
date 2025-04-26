import cv2
import numpy as np

def detect_squares():
    # Open the default camera (usually webcam)
    cap = cv2.VideoCapture(0)

    # Check if camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    print("Camera opened successfully. Press 'q' to quit.")

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        if not ret:
            print("Error: Failed to capture frame.")
            break

        # Create a copy for display
        display_frame = frame.copy()

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Find edges using Canny edge detector
        edges = cv2.Canny(blurred, 50, 150)

        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Loop through all contours
        for contour in contours:
            # Approximate the contour shape
            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)

            # If the shape has 4 vertices, it might be a square
            if len(approx) == 4:
                # Get bounding rectangle for the square
                x, y, w, h = cv2.boundingRect(approx)

                # Check if it's a square (width and height are similar)
                # You can adjust the ratio threshold as needed
                aspect_ratio = float(w) / h
                if 0.8 <= aspect_ratio <= 1.2:  # Allow some tolerance
                    # Draw the green box around the square
                    cv2.rectangle(display_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                    # Optional: Draw the square itself
                    cv2.drawContours(display_frame, [approx], 0, (0, 0, 255), 2)

        # Display the resulting frame
        cv2.imshow('Square Detection', display_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close all windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    detect_squares()
