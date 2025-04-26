import cv2
import numpy as np

def detect_x_pattern_tarp(frame):
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply adaptive thresholding
    thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                  cv2.THRESH_BINARY, 11, 2)

    # Find contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        # Filter by area
        area = cv2.contourArea(contour)
        if area < 1000 or area > 100000:  # Adjust based on expected size
            continue

        # Approximate the contour
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04 * peri, True)

        # Check if it's roughly a quadrilateral
        if len(approx) == 4:
            # Get the bounding rectangle
            x, y, w, h = cv2.boundingRect(approx)

            # Check if it's square-like
            aspect_ratio = float(w) / h
            if 0.8 <= aspect_ratio <= 1.2:
                # Extract the region
                roi = gray[y:y+h, x:x+w]
                if roi.size == 0:
                    continue

                # Resize to standard size
                roi_resized = cv2.resize(roi, (100, 100))

                # Check for X pattern by dividing into quarters
                h_roi, w_roi = roi_resized.shape
                tl = roi_resized[0:h_roi//2, 0:w_roi//2]
                tr = roi_resized[0:h_roi//2, w_roi//2:w_roi]
                bl = roi_resized[h_roi//2:h_roi, 0:w_roi//2]
                br = roi_resized[h_roi//2:h_roi, w_roi//2:w_roi]

                # Calculate average intensities
                tl_avg = np.mean(tl)
                tr_avg = np.mean(tr)
                bl_avg = np.mean(bl)
                br_avg = np.mean(br)

                # Check for diagonal pattern (either black-white-black-white or white-black-white-black)
                # Diagonal 1 similar, diagonal 2 similar, and diagonals different from each other
                if ((abs(tl_avg - br_avg) < 30) and
                    (abs(tr_avg - bl_avg) < 30) and
                    (abs(tl_avg - tr_avg) > 40)):
                    return True, (x, y, w, h)

    return False, None

def main():
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Cannot open camera")
        return

    print("Looking for X-pattern tarp... Press 'q' to quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        found, coords = detect_x_pattern_tarp(frame)

        if found:
            print("Found X-pattern tarp!")
            x, y, w, h = coords
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

        cv2.imshow('Tarp Detection', frame)

        if cv2.waitKey(1) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
