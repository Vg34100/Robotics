from ultralytics import YOLO
from yaml import safe_load
from logging import basicConfig, INFO, info
import socket
from torch import cuda
from time import sleep
from threading import Thread
from datetime import datetime, timedelta

import subprocess
import os
from pymavlink import mavutil

from dronekit import connect, VehicleMode



def load_config():
    global config, model_path, threshold, export_frames, detection_log
    global export_video, export_detections, mock_mode
    global udp_ip, udp_port
    
    with open('config_new.yaml', 'r') as file:
        config = safe_load(file)
        
    basicConfig(filename='log.txt', level=INFO, format='%(asctime)s - %(message)s')

    info("Starting program...")

    mock_mode = config['mock_mode']

    model_path = config['model_path']
    threshold = config['threshold']
    export_frames = config['export_frames']
    detection_log = config['detection_log']
    export_video = config['export_video']
    export_detections = config['export_detections']
    
    udp_ip = config['udp_ip']
    udp_port = config['udp_port']
    
    info("Declaring variables")

def set_up():
    # global model, sock
    global model
    
    # Create UDP socket for sending messages
    # sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    # sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
    
    device = 'cuda' if cuda.is_available() else 'cpu'
    model = YOLO(model_path)
    model.to(device)
    
    info("Starting object detection file")



# Function to construct location COT message
# def construct_location_cot_message(location):
#     latitude, longitude, altitude = location

#     location_cot_message = f'<?xml version="1.0" encoding="UTF-8"?>\n'
#     location_cot_message += f'<event version="2.0" uid="{datetime.now().strftime("%Y%m%d%H%M%S")}" type="a-f-G-U-C-I" time="{datetime.utcnow().isoformat()}Z" start="{datetime.utcnow().isoformat()}Z" stale="{(datetime.utcnow() + timedelta(seconds=60)).isoformat()}Z" how="m-g">\n'
#     location_cot_message += f'  <point lat="{latitude}" lon="{longitude}" hae="{altitude}" ce="9999999" le="9999999"/>\n'
#     location_cot_message += f'  <detail>\n'
#     location_cot_message += f'    <precisionlocation altsrc="DTED0" geopointsrc="USER"/>\n'
#     location_cot_message += f'    <contact callsign="UAV Location"/>\n'
#     location_cot_message += f'    <archive/>\n'
#     location_cot_message += f'  </detail>\n'
#     location_cot_message += f'</event>'

#     return location_cot_message

# # Function to get mock location
# def get_mock_location():
#     # Mock location coordinates
#     latitude = 37.7749
#     longitude = -122.4194
#     altitude = 100.0

#     return latitude, longitude, altitude

# Function to get real location
# def get_real_location(mavlink):
#     gps_data = next(gps_data_generator)
#     latitude = float(gps_data['lat'])
#     longitude = float(gps_data['lon'])
#     altitude = float(gps_data['alt'])
#     return latitude, longitude, altitude

# Function to send constant COT message for location
# def send_location_cot_message(sock):
#     while running:
#         # Get current location (mock or real)
#         if mock_mode:
#             location = get_mock_location()
#         else:
#             location = get_real_location()

#         # Construct location COT message
#         location_cot_message = construct_location_cot_message(location)

#         # Check if the socket is still open before sending the message
#         if sock is not None:
#             try:
#                 sock.sendto(location_cot_message.encode(), (udp_ip, udp_port))
#             except OSError as e:
#                 print(f"Error sending location COT message: {e}")
#                 break
#         else:
#             print("Socket is not available. Exiting location COT message thread.")
#             break

#         # Delay before sending the next location update
#         sleep(1)

def mavConnect():
    # Create MAVLINK CONNECTION with to Computer and PI
    # command = ['/home/pi/.local/bin/mavproxy.py', '--master=/dev/ttyACM0', '--out=tcpin:0.0.0.0:5760', '--out=tcpin:0.0.0.0:5761', '--aircraft', 'Electristar']
    
    # if os.geteuid() == 0:
    #     process = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    # else:
    #     print("Script is not being run as root. Please use sudo.")
    #     return None

    # sleep(1)

    # # Start a connection listening on a UDP port (PI)
    # mavlink = mavutil.mavlink_connection('tcp:0.0.0.0:5761')

    # # Wait for the first heartbeat
    # mavlink.wait_heartbeat()
    # print("Heartbeat from system (system %u component %u)" % (mavlink.target_system, mavlink.target_component))

    # info("Mavlink connection established")
    # return mavlink, process
    
    
    # Connect to the vehicle

    connection_string = 'tcp:127.0.0.1:5761'
    vehicle = connect(connection_string, wait_ready=True)
    return vehicle
    
    
    
    
    
    
    
    
    

# def gps_data(mavlink, mock):
#     while running:
#         # Fetch the GLOBAL_POSITION_INT message from MAVLink
#         gps_data = mavlink.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        
#         # Convert the received lat and lon values (they are in centidegrees)
#         lat = gps_data.lat / 1e7
#         lon = gps_data.lon / 1e7
#         # Convert the received altitude (it's in millimeters)
#         alt = gps_data.alt / 1000.0  # Convert to meters
#         # Speed (ground speed in centimeters/second)
#         speed = gps_data.vx**2 + gps_data.vy**2  # Calculating ground speed
#         speed = (speed**0.5) / 100.0  # Convert to meters/second
        
#         yield {
#             "class": "TPV",
#             "lat": str(lat),
#             "lon": str(lon),
#             "speed": str(speed),
#             "alt": str(alt)
#         }

# Example: Arm and take off
def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        sleep(1)



if __name__ == '__main__':
    load_config()
    set_up()
    running = True

    if not mock_mode:
        vehicle = mavConnect()

    # Create a generator for GPS data
    # gps_data_generator = gps_data(mavlink, mock_mode)

# Start the location COT message thread
# location_cot_thread = Thread(target=send_location_cot_message, args=(sock,))
# location_cot_thread.start()



    results = model.predict(source="0", imgsz=(640,480), stream_buffer=False, conf=threshold, show=True, stream=True, classes=1, save=export_video, save_frames=export_frames, save_txt=detection_log, save_conf=detection_log, save_crop=export_detections)  # [0, 3, 5] for multiple classes

    try:
        for i, (result) in enumerate(results):
            # print('Do something with class 0')
            if result:
                print("Do something")
    except KeyboardInterrupt:
        print("Exiting program...")
        running = False
        
    finally:
        # location_cot_thread.join()
        vehicle.close()
        print("Program closed.")