import cv2
import numpy as np
import math

def detect_triangles_in_squares():
    # Open the default camera
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
        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Lists to store triangles and squares
        triangles = []
        squares = []

        # Loop through all contours to find triangles and squares
        for contour in contours:
            # Ignore very small contours
            if cv2.contourArea(contour) < 100:
                continue

            # Approximate the contour shape
            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)

            # Check for triangles (3 vertices)
            if len(approx) == 3:
                # Check if it's an equilateral triangle
                # Get the three sides
                side1 = np.linalg.norm(approx[0][0] - approx[1][0])
                side2 = np.linalg.norm(approx[1][0] - approx[2][0])
                side3 = np.linalg.norm(approx[2][0] - approx[0][0])

                # Calculate average side length
                avg_side = (side1 + side2 + side3) / 3

                # Check if all sides are similar in length (tolerance of 15%)
                if (abs(side1 - avg_side) / avg_side < 0.15 and
                    abs(side2 - avg_side) / avg_side < 0.15 and
                    abs(side3 - avg_side) / avg_side < 0.15):

                    # It's approximately equilateral
                    x, y, w, h = cv2.boundingRect(approx)
                    triangles.append((x, y, w, h, approx))

                    # Draw a blue box around each triangle
                    cv2.rectangle(display_frame, (x, y), (x + w, y + h), (255, 0, 0), 1)

            # Check for squares (4 vertices)
            elif len(approx) == 4:
                x, y, w, h = cv2.boundingRect(approx)

                # Check if it's a square (width and height are similar)
                aspect_ratio = float(w) / h
                if 0.8 <= aspect_ratio <= 1.2:
                    squares.append((x, y, w, h, approx))

        # Now check which squares contain at least 4 triangles
        for sx, sy, sw, sh, s_approx in squares:
            # Count triangles inside this square
            triangles_inside = 0
            for tx, ty, tw, th, t_approx in triangles:
                # Check if triangle center is inside square
                t_center_x = tx + tw/2
                t_center_y = ty + th/2

                if (sx <= t_center_x <= sx + sw and
                    sy <= t_center_y <= sy + sh):
                    triangles_inside += 1

            # If at least 4 triangles are inside the square
            if triangles_inside >= 4:
                # Draw a thick green box around the square
                cv2.rectangle(display_frame, (sx, sy), (sx + sw, sy + sh), (0, 255, 0), 3)

                # Optional: Label the square with triangle count
                cv2.putText(display_frame, f"{triangles_inside} triangles",
                           (sx, sy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            else:
                # Draw a thin yellow box around other squares
                cv2.rectangle(display_frame, (sx, sy), (sx + sw, sy + sh), (0, 255, 255), 1)

        # Display the resulting frame
        cv2.imshow('Triangle and Square Detection', display_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close all windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    detect_triangles_in_squares()
