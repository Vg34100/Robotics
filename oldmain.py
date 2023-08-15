import time, socket, cv2, logging, yaml, threading, pickle, struct
from datetime import datetime, timedelta
import numpy as np

import func

#* UPDATE - Can view webcame with use of receiver.py
#TODO - Export frames 
#TODO - Record video
#TODO - Frame queue

# Set up the log
logging.basicConfig(level=logging.DEBUG, format="%(asctime)s - %(levelname)s - %(message)s", filename="log.txt")
class Configuration:
    def __init__(self, config_file="config.yaml"):
        with open(config_file, 'r', encoding='UTF-8') as f:
            self.config = yaml.safe_load(f)

    def get(self, option):
        return self.config.get(option)
CONFIG = Configuration()  

# Adjust
MOCK = True
EXPORT_DETECTED = True




# Constants
GROUP, PORT, UID, IMAGE_DIR, mavlink, use_delay = func.set_constants(CONFIG)

logging.info(f"Session Started - UID: {UID}, GROUP: {GROUP}, PORT: {PORT}, MOCK: {MOCK}, IMAGES: {EXPORT_DETECTED}")

# Create a UDP socket for sending CoT messages
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
cot_last_sent = 0

#! Mavlink
if not MOCK: mavlink, process = func.mavConnect()

#! Object Detection Setup
net = cv2.dnn.readNetFromCaffe('SSD_MobileNet_prototxt.txt', 'SSD_MobileNet.caffemodel')
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]
COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

#! Start video capture
cap = cv2.VideoCapture(0)

# Verify if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

#! View webcam
video_stream_thread = threading.Thread(target=func.video_stream_server, args=(cap,))
video_stream_thread.start()

for result in func.gps_data(mavlink, MOCK):
    if result["class"] == "TPV":            
        func.cot_basic_message(result, use_delay, cot_last_sent, sock, GROUP, PORT, UID)
            
    #! Capture video frame
    ret, frame = cap.read()
    if ret:        
        #!Perform object detection on the frame
        (h, w) = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)
        net.setInput(blob)
        detections = net.forward()
        for i in np.arange(0, detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > 0.2:
                idx = int(detections[0,0,i,1])
                if CLASSES[idx] == "person":
                    box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                    (startX, startY, endX, endY) = box.astype("int")
                    label = "{}: {:.2f}%".format(CLASSES[idx], confidence * 100)
                    cv2.rectangle(frame, (startX, startY), (endX, endY), COLORS[idx], 2)
                    y = startY - 15 if startY - 15 > 15 else startY + 15
                    cv2.putText(frame, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)
        
                    if not MOCK and EXPORT_DETECTED: func.save_image_person(mavlink, frame, IMAGE_DIR)
                    func.cot_person_detected(result, sock, GROUP, PORT)
                    



# Release video capture when done
video_stream_thread.join()
cap.release()
cv2.destroyAllWindows()

