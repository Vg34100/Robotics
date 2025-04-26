import cv2
import numpy as np
import math
import os
import time
import pytesseract  # Make sure to install: pip install pytesseract
# You'll also need to install Tesseract OCR on your system:
# Windows: https://github.com/UB-Mannheim/tesseract/wiki
# Linux: sudo apt install tesseract-ocr
# Mac: brew install tesseract

def detect_x_pattern_squares_with_numbers():
    # Open the default camera
    cap = cv2.VideoCapture(1)

    # Create debug folder if it doesn't exist
    debug_folder = "debug_frames"
    if not os.path.exists(debug_folder):
        os.makedirs(debug_folder)

    # Debug visualization function
    def save_debug_image(name, image):
        path = os.path.join(debug_folder, f"{name}.jpg")
        cv2.imwrite(path, image)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    print("Camera opened successfully. Press 'q' to quit.")
    print("\n\033[1;36m===== NUMBER DETECTION RESULTS WILL APPEAR BELOW =====\033[0m\n")

    # Parameters for detection
    MIN_CONTOUR_AREA = 500  # Reduced to catch smaller/distant markers
    SQUARE_RATIO_MIN = 0.7  # More lenient aspect ratio for squares
    SQUARE_RATIO_MAX = 1.3
    CANNY_LOW = 30  # Lower threshold for more edges
    CANNY_HIGH = 120
    HOUGH_THRESHOLD = 15  # Lower threshold for line detection
    MIN_LINE_LENGTH = 20  # Shorter lines can be detected
    MAX_LINE_GAP = 20  # Larger gaps allowed between line segments
    DIAGONAL_ANGLE_TOLERANCE = 15  # Wider angle tolerance (45±15 degrees)

    # Configure pytesseract to only look for digits
    custom_config = r'--oem 3 --psm 6 -c tessedit_char_whitelist=0123456789'

    frame_count = 0
    debug_interval = 30  # Save debug images every 30 frames

    # Store previously detected numbers to reduce flickering
    recent_detections = {}  # {(x,y,w,h): (number, confidence, last_seen_frame)}
    detection_persistence = 10  # How many frames a detection persists

    while True:
        # Capture frame
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame.")
            break

        display_frame = frame.copy()

        # Enhanced preprocessing for lower quality cameras
        # 1. Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 2. Apply bilateral filter (preserves edges better than Gaussian)
        filtered = cv2.bilateralFilter(gray, 9, 75, 75)

        # 3. Apply CLAHE (Contrast Limited Adaptive Histogram Equalization)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(filtered)

        # Save preprocessing debug images
        if frame_count % debug_interval == 0:
            save_debug_image("01_original", frame)
            save_debug_image("02_grayscale", gray)
            save_debug_image("03_filtered", filtered)
            save_debug_image("04_enhanced", enhanced)

        # 4. Multiple edge detection approaches combined
        # Canny edge detection
        edges_canny = cv2.Canny(enhanced, CANNY_LOW, CANNY_HIGH)

        # Sobel edge detection (alternative approach)
        sobelx = cv2.Sobel(enhanced, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(enhanced, cv2.CV_64F, 0, 1, ksize=3)
        sobel_edges = cv2.magnitude(sobelx, sobely)
        sobel_edges = cv2.normalize(sobel_edges, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        _, sobel_binary = cv2.threshold(sobel_edges, 50, 255, cv2.THRESH_BINARY)

        # Combine edge detection results
        edges = cv2.bitwise_or(edges_canny, sobel_binary)

        # Optional: Morphological operations to clean up edges
        kernel = np.ones((3,3), np.uint8)
        edges = cv2.dilate(edges, kernel, iterations=1)
        edges = cv2.erode(edges, kernel, iterations=1)

        if frame_count % debug_interval == 0:
            save_debug_image("05_edges_canny", edges_canny)
            save_debug_image("06_edges_sobel", sobel_binary)
            save_debug_image("07_edges_combined", edges)

        # Find contours with hierarchy to detect nested patterns
        contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Draw all contours for debugging
        contour_image = np.zeros_like(frame)
        cv2.drawContours(contour_image, contours, -1, (0, 255, 0), 1)
        if frame_count % debug_interval == 0:
            save_debug_image("08_all_contours", contour_image)

        # Process each contour to find potential squares
        potential_squares = []
        for i, contour in enumerate(contours):
            # Filter out small contours
            area = cv2.contourArea(contour)
            if area < MIN_CONTOUR_AREA:
                continue

            # Approximate the contour shape
            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)

            # Check if it has 4 vertices (potential square)
            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(approx)

                # Check aspect ratio for squareness
                aspect_ratio = float(w) / h
                if SQUARE_RATIO_MIN <= aspect_ratio <= SQUARE_RATIO_MAX:
                    potential_squares.append((x, y, w, h, approx, area))

        # Draw potential squares for debugging
        square_image = np.zeros_like(frame)
        for x, y, w, h, approx, area in potential_squares:
            cv2.drawContours(square_image, [approx], 0, (0, 255, 0), 2)
        if frame_count % debug_interval == 0:
            save_debug_image("09_potential_squares", square_image)

        # Check each potential square for X pattern
        x_pattern_squares = []
        for x, y, w, h, approx, area in potential_squares:
            # Create a mask for this square
            mask = np.zeros(gray.shape, dtype=np.uint8)
            cv2.drawContours(mask, [approx], 0, 255, -1)

            # Get the ROI (Region of Interest)
            roi = cv2.bitwise_and(enhanced, enhanced, mask=mask)
            roi_display = roi.copy()

            # Skip if ROI is empty
            if roi[mask > 0].size == 0:
                continue

            # Apply edge detection to the ROI
            roi_edges = cv2.Canny(roi, CANNY_LOW, CANNY_HIGH)

            # Look for diagonal lines using Hough transform
            lines = cv2.HoughLinesP(roi_edges, 1, np.pi/180,
                                  threshold=HOUGH_THRESHOLD,
                                  minLineLength=MIN_LINE_LENGTH,
                                  maxLineGap=MAX_LINE_GAP)

            # Prepare an image to draw detected lines for debugging
            line_image = np.zeros_like(frame)
            diagonal_lines = []

            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    global_x1, global_y1 = x1, y1  # For global coordinates
                    global_x2, global_y2 = x2, y2

                    # Calculate line length and angle
                    line_length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                    angle = math.degrees(math.atan2(y2 - y1, x2 - x1)) % 180

                    # Check if line is close to diagonal angles (45 or 135 degrees)
                    if ((45 - DIAGONAL_ANGLE_TOLERANCE <= angle <= 45 + DIAGONAL_ANGLE_TOLERANCE) or
                        (135 - DIAGONAL_ANGLE_TOLERANCE <= angle <= 135 + DIAGONAL_ANGLE_TOLERANCE)):
                        diagonal_lines.append((global_x1, global_y1, global_x2, global_y2, angle, line_length))
                        # Draw the diagonal line
                        cv2.line(line_image, (x + x1, y + y1), (x + x2, y + y2), (0, 0, 255), 2)

            # Check for intensity differences in quadrants (alternative method)
            h_half, w_half = h // 2, w // 2
            quadrant_pattern_detected = False

            if h_half > 0 and w_half > 0:
                # Create a mask for each quadrant
                masks = [
                    np.zeros(mask.shape, dtype=np.uint8) for _ in range(4)
                ]

                # Define quadrant regions
                regions = [
                    (x, y, x+w_half, y+h_half),             # Top-left
                    (x+w_half, y, x+w, y+h_half),           # Top-right
                    (x, y+h_half, x+w_half, y+h),           # Bottom-left
                    (x+w_half, y+h_half, x+w, y+h)          # Bottom-right
                ]

                # Fill quadrant masks
                for i, (x1, y1, x2, y2) in enumerate(regions):
                    masks[i][y1:y2, x1:x2] = mask[y1:y2, x1:x2]

                # Get mean intensity for each quadrant
                quadrant_intensities = [
                    cv2.mean(enhanced, mask=masks[i])[0] for i in range(4)
                ]

                # Calculate intensity differences
                avg_intensity = sum(quadrant_intensities) / 4
                intensity_diffs = [abs(q - avg_intensity) for q in quadrant_intensities]
                max_intensity_diff = max(intensity_diffs)

                # Check diagonal pattern (diagonal quadrants should have similar intensities)
                diagonal1_diff = abs(quadrant_intensities[0] - quadrant_intensities[3])
                diagonal2_diff = abs(quadrant_intensities[1] - quadrant_intensities[2])

                # If diagonals have similar intensities within each diagonal but
                # different between diagonals, it's likely an X pattern
                quadrant_pattern_detected = (
                    max_intensity_diff > 15 and
                    (diagonal1_diff < 10 or diagonal2_diff < 10)
                )

                # Create quadrant visualization for debugging
                quadrant_vis = np.zeros_like(frame)
                colors = [
                    (255, 0, 0),   # Blue for top-left
                    (0, 255, 0),   # Green for top-right
                    (0, 0, 255),   # Red for bottom-left
                    (255, 255, 0)  # Cyan for bottom-right
                ]

                for i, (x1, y1, x2, y2) in enumerate(regions):
                    intensity = int(quadrant_intensities[i])
                    cv2.rectangle(quadrant_vis, (x1, y1), (x2, y2), colors[i], -1)
                    cv2.putText(quadrant_vis, f"{intensity:.1f}",
                               (x1+5, y1+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # Score the square based on both diagonal lines and quadrant analysis
            has_enough_diagonals = len(diagonal_lines) >= 2

            # Final detection based on combined evidence
            if has_enough_diagonals or quadrant_pattern_detected:
                confidence = (len(diagonal_lines) * 0.3) + (1 if quadrant_pattern_detected else 0)
                x_pattern_squares.append((x, y, w, h, approx, confidence))

                # Draw debugging info for this square
                if frame_count % debug_interval == 0:
                    square_debug = np.zeros_like(frame)

                    # Draw the square
                    cv2.drawContours(square_debug, [approx], 0, (0, 255, 0), 2)

                    # Draw diagonal lines
                    for x1, y1, x2, y2, angle, length in diagonal_lines:
                        cv2.line(square_debug, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        # Add angle annotation
                        mid_x, mid_y = (x1 + x2) // 2, (y1 + y2) // 2
                        cv2.putText(square_debug, f"{angle:.1f}°",
                                   (mid_x, mid_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

                    # Add quadrant information
                    if h_half > 0 and w_half > 0:
                        for i, (x1, y1, x2, y2) in enumerate(regions):
                            intensity = int(quadrant_intensities[i])
                            cv2.rectangle(square_debug, (x1+5, y1+5), (x1+25, y1+25), colors[i], -1)
                            cv2.putText(square_debug, f"Q{i+1}:{intensity}",
                                       (x1+5, y1+45), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

                    save_debug_image(f"10_detected_square_{frame_count}_{x}_{y}", square_debug)

        # Display the detected X-pattern squares and try to read numbers
        for x, y, w, h, approx, confidence in x_pattern_squares:
            # Green box around the detected pattern
            cv2.rectangle(display_frame, (x, y), (x + w, y + h), (0, 255, 0), 3)

            # Extract the square region for number recognition
            square_roi = gray[y:y+h, x:x+w]
            if square_roi.size == 0:
                continue

            # Enlarge the ROI for better OCR (2x)
            enlarged_roi = cv2.resize(square_roi, (w*2, h*2), interpolation=cv2.INTER_CUBIC)

            # Enhance contrast for OCR
            _, thresholded = cv2.threshold(enlarged_roi, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

            # Zoom in the area and save debug image occasionally
            if frame_count % debug_interval == 0:
                save_debug_image(f"12_number_roi_{frame_count}_{x}_{y}", enlarged_roi)
                save_debug_image(f"13_number_thresh_{frame_count}_{x}_{y}", thresholded)

            # Try to read numbers using OCR
            try:
                text = pytesseract.image_to_string(thresholded, config=custom_config).strip()

                # Filter out non-digit characters and empty results
                digits = ''.join(filter(str.isdigit, text))

                # If we found digits, display and log them
                if digits:
                    # Store this detection with current frame number
                    detection_key = (x, y, w, h)
                    recent_detections[detection_key] = (digits, confidence, frame_count)

                    # Display detection with magenta color (different from the green box)
                    cv2.putText(display_frame, f"Number: {digits}",
                               (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

                    # Print to terminal with color and emphasis
                    print(f"\033[1;35m[FRAME {frame_count}] DETECTED NUMBER: {digits} (Confidence: {confidence:.2f})\033[0m")
            except Exception as e:
                # OCR can fail for various reasons
                print(f"OCR error: {e}")

        # Check for recently detected numbers that aren't in the current frame
        # (this helps display numbers even when detection flickers)
        keys_to_remove = []
        for (prev_x, prev_y, prev_w, prev_h), (number, conf, last_seen) in recent_detections.items():
            # If we've seen this detection recently but not in current frame
            if frame_count - last_seen < detection_persistence:
                # Show the last detected number, but with a different color to indicate it's persistent
                cv2.putText(display_frame, f"Last: {number}",
                           (prev_x, prev_y-30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
            else:
                # Remove old detections
                keys_to_remove.append((prev_x, prev_y, prev_w, prev_h))

        # Clean up old detections
        for key in keys_to_remove:
            recent_detections.pop(key, None)

        # Display results
        cv2.imshow('X Pattern Detection with Numbers', display_frame)

        # Save the final output every N frames
        if frame_count % debug_interval == 0:
            save_debug_image(f"14_final_output_{frame_count}", display_frame)

        frame_count += 1

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    detect_x_pattern_squares_with_numbers()
