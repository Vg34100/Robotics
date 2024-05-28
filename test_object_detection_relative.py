from ultralytics import YOLO
from yaml import safe_load
from logging import basicConfig, INFO, info
from torch import cuda
from time import sleep
from threading import Thread
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import math
import cv2

def load_config():
    global config, model_path, threshold, export_frames, detection_log
    global export_video, export_detections, mock_mode, listofwaypoints
    
    with open('config_relative.yaml', 'r') as file:
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
    
    listofwaypoints = config['listofwaypoints']
    
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

def return_to_launch(vehicle):
    print("Returning to Launch")
    vehicle.mode = VehicleMode("RTL")

def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` meters from the specified `original_location`. 
    The returned Location has the same `alt` value as `original_location`.
    """
    earth_radius = 6378137.0  # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180.0))

    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180.0 / math.pi)
    newlon = original_location.lon + (dLon * 180.0 / math.pi)
    return LocationGlobal(newlat, newlon, original_location.alt)

def navigate_to_waypoint_and_check(vehicle, waypoint, original_location):
    waypoint_location = get_location_metres(original_location, waypoint[0], waypoint[1])
    vehicle.simple_goto(waypoint_location)
    
    while vehicle.mode.name == "GUIDED":
        current_distance = get_distance_metres(vehicle.location.global_frame, waypoint_location)
        print(f"Distance to waypoint: {current_distance}")
        if current_distance < 1:
            print("Reached waypoint")
            break
        sleep(1)

def align_and_drop(vehicle, centerX, centerY, cameraX, cameraY):
    print('Aligning drone to target center')
    offsetX = centerX - cameraX
    offsetY = centerY - cameraY

    if abs(offsetX) > 10 or abs(offsetY) > 10:
        print(f'Offset detected: X={offsetX}, Y={offsetY}')
        while abs(offsetX) > 10 or abs(offsetY) > 10:
            if centerX is not None and centerY is not None:
                print(f'Re-adjusting: X={offsetX}, Y={offsetY}')
                send_ned_velocity(vehicle, 0.5 * (offsetY / abs(offsetY)), 0.5 * (offsetX / abs(offsetX)), 0)
                offsetX = centerX - cameraX
                offsetY = centerY - cameraY
            sleep(1)

    print('Aligned. Dropping payload.')
    # Implement drop logic here
    # servo_drop()  # You can define this function to control the servo for payload drop

def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        velocity_x, velocity_y, velocity_z,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)

def run_object_detection(vehicle):
    cap = cv2.VideoCapture(0)
    global centerX, centerY, cameraX, cameraY, hastargetbeendetected
    
    cameraX = 640 / 2
    cameraY = 480 / 2
    hastargetbeendetected = False

    results = model.predict(source=0, imgsz=(640, 480), conf=threshold, stream=True, show=True, save=export_video, save_frames=export_frames, save_txt=detection_log, save_conf=detection_log, save_crop=export_detections)
    
    for result in results:
        frame = result.orig_img
        if len(result.boxes) > 0:
            box = result.boxes[0]
            centerX = int((box.xmin + box.xmax) / 2)
            centerY = int((box.ymin + box.ymax) / 2)
            hastargetbeendetected = True
            print(f'Target detected at X={centerX}, Y={centerY}')
        else:
            hastargetbeendetected = False
            centerX = None
            centerY = None
        sleep(1)
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    load_config()
    set_up()
    running = True

    if not mock_mode:
        vehicle = mavConnect()

        takeoff_thread = Thread(target=arm_and_takeoff, args=(vehicle, 10))
        takeoff_thread.start()

    detection_thread = Thread(target=run_object_detection, args=(vehicle,))
    detection_thread.start()

    try:
        if not mock_mode:
            takeoff_thread.join()
        
        detection_thread.join()

        # Store the original location for relative waypoints
        original_location = vehicle.location.global_frame

        # Navigate through waypoints
        for waypoint in listofwaypoints:
            navigate_to_waypoint_and_check(vehicle, waypoint, original_location)
            if hastargetbeendetected:
                align_and_drop(vehicle, centerX, centerY, cameraX, cameraY)
                break
        
        # Return to launch
        return_to_launch(vehicle)
        while vehicle.armed:
            print("Returning to Launch...")
            sleep(1)

    except KeyboardInterrupt:
        print("Exiting program...")
        running = False
        
    finally:
        if not mock_mode:
            vehicle.close()
        print("Program closed.")
