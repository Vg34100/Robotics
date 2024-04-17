import cv2
import tensorflow as tf
import numpy as np

# Load the TensorFlow model
model = tf.saved_model.load('hardhat')

# Function to run inference on a frame
def run_inference(model, image):
    input_tensor = tf.convert_to_tensor(np.expand_dims(image, 0), dtype=tf.float32)
    detections = model(input_tensor)
    return detections

# Initialize webcam
cap = cv2.VideoCapture(0)

try:
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            break

        # Model expects 640x640 images, adjust according to your model's requirement
        # Resize and normalize the image
        input_frame = cv2.resize(frame, (640, 640))
        input_frame = input_frame / 255.0

        # Run detection
        detections = run_inference(model, input_frame)

        # Process detections (example: print number of detections)
        # You'll need to adjust processing based on your model's specific output format
        print(f"Detections: {len(detections)}")

        # Display the resulting frame
        cv2.imshow('Frame', frame)

        # Break the loop with 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    cap.release()
    cv2.destroyAllWindows()
