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
    global model
    
    device = 'cuda' if cuda.is_available() else 'cpu'
    model = YOLO(model_path)
    model.to(device)
    
    info("Starting object detection file")

def mavConnect():
    connection_string = 'tcp:127.0.0.1:5761'
    vehicle = connect(connection_string, wait_ready=True)
    return vehicle

def arm_and_takeoff(vehicle, aTargetAltitude):
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

def run_object_detection():
    results = model.predict(source="0", imgsz=(640,480), stream_buffer=False, conf=threshold, show=True, stream=True, classes=1, save=export_video, save_frames=export_frames, save_txt=detection_log, save_conf=detection_log, save_crop=export_detections)  # [0, 3, 5] for multiple classes

    try:
        for i, (result) in enumerate(results):
            if result:
                print("Do something")
    except KeyboardInterrupt:
        print("Exiting program...")
        running = False
        
    finally:
        print("Object detection finished.")

if __name__ == '__main__':
    load_config()
    set_up()
    running = True

    if not mock_mode:
        vehicle = mavConnect()

        # Create a thread for the takeoff function
        takeoff_thread = Thread(target=arm_and_takeoff, args=(vehicle, 10))
        takeoff_thread.start()

    # Create a thread for the object detection function
    detection_thread = Thread(target=run_object_detection)
    detection_thread.start()

    try:
        # Wait for both threads to complete
        if not mock_mode:
            takeoff_thread.join()
        detection_thread.join()

    except KeyboardInterrupt:
        print("Exiting program...")
        running = False
        
    finally:
        if not mock_mode:
            vehicle.close()
        print("Program closed.")
