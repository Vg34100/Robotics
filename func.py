import os, subprocess, time, cv2, logging, yaml
from pymavlink import mavutil
from datetime import datetime, timedelta

logging.basicConfig(level=logging.DEBUG, format="%(asctime)s - %(levelname)s - %(message)s", filename="log.txt")

class Configuration:
    def __init__(self, config_file="config.yaml"):
        with open(config_file, 'r', encoding='UTF-8') as f:
            self.config = yaml.safe_load(f)

    def get(self, option):
        return self.config.get(option)
CONFIG = Configuration()  

def set_constants(CONFIG):
    mavlink = None
    GROUP = CONFIG.get("GROUP")
    PORT = CONFIG.get("PORT")
    UID = CONFIG.get("UID")
    IMAGE_DIR = CONFIG.get("IMAGE_DIR")
    use_delay = CONFIG.get("FAST_DELAY")
    return GROUP, PORT, UID, IMAGE_DIR, mavlink, use_delay

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
      
def cot_basic_message(result, delay, last, sock, group, port, uid):
    current_time = datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%SZ')
    stale_time = (datetime.utcnow() + timedelta(days=1)).strftime('%Y-%m-%dT%H:%M:%SZ')
    lat = result.get("lat")
    lon = result.get("lon")
    speed = int(result.get("speed"))
    altitude = result.get("alt")
    
    # Construct the CoT message
    cot = f"""<?xml version='1.0' encoding='UTF-8' standalone='yes'?>
    <event uid="{uid}" type="a-f-A-M-F-Q-H" how="" start="{current_time}" time="{current_time}" stale="{stale_time}">
        <point le="0" ce="0" hae="{altitude}" lon="{lon}" lat="{lat}"/>
    </event>"""
    
    print(cot)
    logging.info(cot)

    # Send the CoT message if enough time has passed
    if time.time() - last >= delay:
        sock.sendto(bytes(cot, "utf-8"), (group, port))
        last = time.time()

def cot_person_detected(result, sock, group, port, callsign="Detected Person"):
    """
    Build a Cursor on Target (CoT) message for a detected person.

    :param lat: Latitude of the detected object.
    :param lon: Longitude of the detected object.
    :param altitude: Altitude of the detected object.
    :param callsign: Callsign for the detected object. Default is "DetectedPerson".
    :return: CoT message string.
    """
    
    lat, lon, altitude = result["lat"], result["lon"], result["alt"]  

    # Event details for CoT
    event_time = datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%S.%fZ')  # Current UTC time
    start_time = event_time
    stale_time = (datetime.utcnow() + timedelta(minutes=1)).strftime('%Y-%m-%dT%H:%M:%S.%fZ')

    # CoT message format
    cot_template = """<event version='2.0' uid='{callsign}' type='a-h-G-U-C' time='{event_time}' start='{start_time}' stale='{stale_time}' how='m-g'>
    <point lat='{lat}' lon='{lon}' hae='9999999.0' ce='9999999.0' le='9999999.0'/>
    <detail>
        <contact callsign='{callsign}'/>
    </detail>
</event>"""

    # Fill in the CoT template with data
    cot_message = cot_template.format(
        callsign=callsign,
        event_time=event_time,
        start_time=start_time,
        stale_time=stale_time,
        lat=lat,
        lon=lon,
        altitude=altitude
    )

    # Transmit the CoT message to ATAK
    sock.sendto(bytes(cot_message, "utf-8"), (group, port))

    print(cot_message)  # Also print the CoT message to the console for debugging.
    logging.info(cot_message)


    return cot_message

def gps_data(mavlink, mock):
    if mock: 
        #mock_gps_data()
        for _ in range(1):  # Simulating 10 GPS updates
            # Mock data: incrementally moving from lat 0, lon 0 to lat 10, lon 10
            # with a constant speed of 5 (for simplicity)
            for i in range(10):
                yield {
                    "class": "TPV",
                    "lat": str(i),
                    "lon": str(i),
                    "speed": "5",
                    "alt": "1000"  # altitude is constant at 1000m for simplicity
                }
                time.sleep(1)  # Simulate 1 second delay between each GPS update
    else:
        #real_gps_data(mavlink) 
        while True:
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

def save_image_person(result, frame, directory):
    time_string = datetime.now().strftime("%Y%m%d-%H%M%S")
    lat, lon, altitude = result["lat"], result["lon"], result["alt"]  
    location = f"lat_{lat}-lon_{lon}-alt_{altitude}"
    cv2.imwrite(directory + '/DETECTED-Person-C_{}-GPS_{}.jpg'.format(time_string, location), frame)
    
def video_stream_server(cap):
    import pickle, struct, socket, threading
    host = '0.0.0.0'  # Listen on all available interfaces
    port = 8485       # Choose an appropriate port

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(5)
    print("Video stream server waiting for a connection...")


    def handle_client(connection, client_address):
        print(f"Accepted video stream connection from {client_address}")
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            data = pickle.dumps(frame)
            message_size = struct.pack("L", len(data))
            try:
                connection.sendall(message_size + data)
            except:
                # Handle disconnections
                break
        connection.close()
        print(f"Connection with {client_address} closed.")

    while True:
        connection, client_address = server_socket.accept()
        client_thread = threading.Thread(target=handle_client, args=(connection, client_address))
        client_thread.start()

    # connection, client_address = server_socket.accept()
    # print(f"Accepted video stream connection from {client_address}")

    # while True:
    #     ret, frame = cap.read()
    #     if not ret:
    #         break

    #     data = pickle.dumps(frame)
    #     message_size = struct.pack("L", len(data))

    #     #Send message size and data
    #     connection.sendall(message_size + data)

    server_socket.close()