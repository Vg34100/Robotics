from ultralytics import YOLO
import cv2

# Load the trained YOLOv8 model
model = YOLO(r'runs\detect\train4\weights\best.pt')

# Open the webcam
cap = cv2.VideoCapture(0)

while True:
    # Read a frame from the webcam
    ret, frame = cap.read()

    if not ret:
        break

    # Perform object detection on the frame
    results = model.predict(frame)

    # Visualize the detected objects on the frame
    print(results)
    annotated_frame = results[0].plot()

    # Display the annotated frame
    cv2.imshow("YOLOv8 Detection", annotated_frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close the windows
cap.release()
cv2.destroyAllWindows()