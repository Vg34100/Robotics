import cv2
import torch
import yaml
import socket
import logging
import threading
import numpy as np
import subprocess
from ultralytics import YOLO
from queue import Queue
from datetime import datetime, timedelta
import time
import os
from pymavlink import mavutil


# Set up logging
logging.basicConfig(filename='log.txt', level=logging.INFO, format='%(asctime)s - %(message)s')

# Load configuration from YAML file
with open('config_new.yaml', 'r') as file:
    config = yaml.safe_load(file)

# Extract configuration values
mock_mode = config['mock_mode']
export_frames = config['export_frames']
udp_ip = config['udp_ip']
udp_port = config['udp_port']
mavlink_connection_string = config['mavlink_connection_string']
gstreamer_pipeline = config['gstreamer_pipeline']
model_path = config['model_path']
device = 'cuda' if torch.cuda.is_available() else 'cpu'
threshold = config['threshold']
frame_skip = config['frame_skip']


# Create UDP socket for sending messages
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)

# Load YOLOv8 model
model = YOLO(model_path)
model.to(device)

# Initialize camera
# cap = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)
cap = cv2.VideoCapture(0)


# Create a queue to store frames
frame_queue = Queue()

# Flag to control the main loop
running = True

# Function to process frames
def process_frames():
    frame_counter = 0
    while running:
        if not frame_queue.empty():
            frame = frame_queue.get()
            frame_counter += 1

            if frame_counter % frame_skip == 0:
                # Perform object detection on the frame
                results = model.predict(frame, conf=threshold, show=True, stream=True, classes=0)
                # print(results[0])
                # for a, b, c, result in results[0]:
                #     print(a)
                # Check if an object is detected


                if results:
                    # Annotate the frame with detected objects
                    annotated_frame = results.plot()

                    # Save the annotated frame
                    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                    cv2.imwrite(f'detected_{timestamp}.jpg', annotated_frame)

                    # Send COT message
                    send_cot_message(results.boxes.xyxy.tolist())

                # Export the frame if enabled
                if export_frames:
                    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                    cv2.imwrite(f'frame_{timestamp}.jpg', frame)

            frame_queue.task_done()

# Function to send COT message
def send_cot_message(detections):
    # Construct COT message based on detections
    cot_message = construct_cot_message(detections)

    # Send COT message via UDP socket
    sock.sendto(cot_message.encode(), (udp_ip, udp_port))

# Function to construct COT message
def construct_cot_message(detections):
    cot_message = f'<?xml version="1.0" encoding="UTF-8"?>\n'
    cot_message += f'<event version="2.0" uid="{datetime.now().strftime("%Y%m%d%H%M%S")}" type="a-f-G-U-C-I" time="{datetime.utcnow().isoformat()}Z" start="{datetime.utcnow().isoformat()}Z" stale="{(datetime.utcnow() + timedelta(seconds=60)).isoformat()}Z" how="m-g">\n'
    
    for detection in detections:
        cot_message += f'  <point lat="{detection[1]}" lon="{detection[0]}" hae="{detection[2]}" ce="9999999" le="9999999"/>\n'
    
    cot_message += f'  <detail>\n'
    cot_message += f'    <precisionlocation altsrc="DTED0" geopointsrc="USER"/>\n'
    cot_message += f'    <contact callsign="Detected Object"/>\n'
    cot_message += f'    <__remarks>Detected by YOLOv8</__remarks>\n'
    cot_message += f'    <archive/>\n'
    cot_message += f'  </detail>\n'
    cot_message += f'</event>'

    return cot_message

# Function to send constant COT message for location
def send_location_cot_message(sock):
    while running:
        # Get current location (mock or real)
        if mock_mode:
            location = get_mock_location()
        else:
            location = get_real_location()

        # Construct location COT message
        location_cot_message = construct_location_cot_message(location)

        # Check if the socket is still open before sending the message
        if sock is not None:
            try:
                sock.sendto(location_cot_message.encode(), (udp_ip, udp_port))
            except OSError as e:
                print(f"Error sending location COT message: {e}")
                break
        else:
            print("Socket is not available. Exiting location COT message thread.")
            break

        # Delay before sending the next location update
        time.sleep(1)

# Function to get mock location
def get_mock_location():
    # Mock location coordinates
    latitude = 37.7749
    longitude = -122.4194
    altitude = 100.0

    return latitude, longitude, altitude

# Function to get real location
def get_real_location(mavlink):
    gps_data = next(gps_data_generator)
    latitude = float(gps_data['lat'])
    longitude = float(gps_data['lon'])
    altitude = float(gps_data['alt'])
    return latitude, longitude, altitude

# Function to construct location COT message
def construct_location_cot_message(location):
    latitude, longitude, altitude = location

    location_cot_message = f'<?xml version="1.0" encoding="UTF-8"?>\n'
    location_cot_message += f'<event version="2.0" uid="{datetime.now().strftime("%Y%m%d%H%M%S")}" type="a-f-G-U-C-I" time="{datetime.utcnow().isoformat()}Z" start="{datetime.utcnow().isoformat()}Z" stale="{(datetime.utcnow() + timedelta(seconds=60)).isoformat()}Z" how="m-g">\n'
    location_cot_message += f'  <point lat="{latitude}" lon="{longitude}" hae="{altitude}" ce="9999999" le="9999999"/>\n'
    location_cot_message += f'  <detail>\n'
    location_cot_message += f'    <precisionlocation altsrc="DTED0" geopointsrc="USER"/>\n'
    location_cot_message += f'    <contact callsign="UAV Location"/>\n'
    location_cot_message += f'    <archive/>\n'
    location_cot_message += f'  </detail>\n'
    location_cot_message += f'</event>'


    return location_cot_message

def mavConnect():
    # Create MAVLINK CONNECTION with to Computer and PI
    command = ['/home/pi/.local/bin/mavproxy.py', '--master=/dev/ttyACM0', '--out=tcpin:0.0.0.0:5760', '--out=tcpin:0.0.0.0:5761', '--aircraft', 'Electristar']
    
    if os.geteuid() == 0:
        process = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    else:
        print("Script is not being run as root. Please use sudo.")
        return None

    time.sleep(1)

    # Start a connection listening on a UDP port (PI)
    mavlink = mavutil.mavlink_connection('tcp:0.0.0.0:5761')

    # Wait for the first heartbeat
    mavlink.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (mavlink.target_system, mavlink.target_component))

    logging.info("Mavlink connection established")
    return mavlink, process

def gps_data(mavlink, mock):
    while running:
        # Fetch the GLOBAL_POSITION_INT message from MAVLink
        gps_data = mavlink.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        
        # Convert the received lat and lon values (they are in centidegrees)
        lat = gps_data.lat / 1e7
        lon = gps_data.lon / 1e7
        # Convert the received altitude (it's in millimeters)
        alt = gps_data.alt / 1000.0  # Convert to meters
        # Speed (ground speed in centimeters/second)
        speed = gps_data.vx**2 + gps_data.vy**2  # Calculating ground speed
        speed = (speed**0.5) / 100.0  # Convert to meters/second
        
        yield {
            "class": "TPV",
            "lat": str(lat),
            "lon": str(lon),
            "speed": str(speed),
            "alt": str(alt)
        }

try:
    # Connect to MAVLink
    if not mock_mode:
        mavlink, process = mavConnect()

        # Create a generator for GPS data
        gps_data_generator = gps_data(mavlink, mock_mode)

    # Start the frame processing thread
    frame_processing_thread = threading.Thread(target=process_frames)
    frame_processing_thread.start()

    # Start the location COT message thread
    location_cot_thread = threading.Thread(target=send_location_cot_message, args=(sock,))
    location_cot_thread.start()

    # Main loop to capture frames and add them to the queue
    while running:
        ret, frame = cap.read()
        if not ret:
            break
        # Resize the frame to 640x480
        frame = cv2.resize(frame, (640, 480))

        # Add the frame to the queue
        frame_queue.put(frame)

except KeyboardInterrupt:
    print("Keyboard interrupt received. Stopping the program...")
    running = False
    # Signal the frame processing thread to stop
    frame_queue.put(None)

finally:
    # Wait for the frame processing thread to finish
    frame_processing_thread.join()

    # Release the camera and close the socket
    cap.release()
    sock.close()

    print("Program stopped.")