from ultralytics import YOLO
import cv2

# Load the trained YOLOv8 model
model = YOLO(r'runs\detect\train4\weights\best.pt')
frame = r"yolohardhat\valid\images\000005_jpg.rf.3oZDOSnHROJtJC07iPjs.jpg"

# Perform object detection on the frame
results = model.predict(frame)

# Visualize the detected objects on the frame
annotated_frame = results[0].plot()

# Display the annotated frame
cv2.imshow("YOLOv8 Detection", annotated_frame)
