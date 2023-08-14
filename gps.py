# import gi
# gi.require_version('Gst', '1.0')
# from gi.repository import Gst, GLib


# import time
# import socket
# from datetime import datetime, timedelta

# import cv2
# import numpy as np
# import os, subprocess
# from pymavlink import mavutil

# # Initialize GStreamer
# Gst.init(None)

# # Constants
# GROUP = "239.2.3.1"
# PORT = 6969
# STATIONARY_DELAY = 45
# SLOW_DELAY = 25
# FAST_DELAY = 2

# EXPORT_IMAGE = False
# IMAGE_DIR = r"image"

# #? TEMP - Mock
# UID = "ELECTRISTAR"
# #?

# # Create a UDP socket for sending CoT messages
# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
# sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)

# #? TEMP - Mock
# # Generate Mock GPS Data
# def real_gps_data(mavlink):
#     while True:
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
# #?


# # Determine delay based on speed
# def get_delay_based_on_speed(speed):
#     if speed <= 1:
#         return STATIONARY_DELAY
#     elif speed <= 6:
#         return SLOW_DELAY
#     else:
#         return FAST_DELAY


# def build_cot_message(lat, lon, altitude, callsign="DetectedPerson"):
#     """
#     Build a Cursor on Target (CoT) message for a detected person.

#     :param lat: Latitude of the detected object.
#     :param lon: Longitude of the detected object.
#     :param altitude: Altitude of the detected object.
#     :param callsign: Callsign for the detected object. Default is "DetectedPerson".
#     :return: CoT message string.
#     """
#     # Event details for CoT
#     event_time = datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%S.%fZ')  # Current UTC time
#     start_time = event_time
#     stale_time = (datetime.utcnow() + timedelta(minutes=1)).strftime('%Y-%m-%dT%H:%M:%S.%fZ')

#     # CoT message format
#     cot_template = """<event version='2.0' uid='{callsign}' type='a-h-G-U-C' time='{event_time}' start='{start_time}' stale='{stale_time}' how='m-g'>
#     <point lat='{lat}' lon='{lon}' hae='{altitude}' ce='9999999.0' le='9999999.0'/>
#     <detail>
#         <contact callsign='{callsign}'/>
#     </detail>
# </event>"""

#     # Fill in the CoT template with data
#     cot_message = cot_template.format(
#         callsign=callsign,
#         event_time=event_time,
#         start_time=start_time,
#         stale_time=stale_time,
#         lat=lat,
#         lon=lon,
#         altitude=altitude
#     )

#     return cot_message



# cot_last_sent = 0
# use_delay = FAST_DELAY

# #! - Object Detection
# # Object Detection Setup
# net = cv2.dnn.readNetFromCaffe('SSD_MobileNet_prototxt.txt', 'SSD_MobileNet.caffemodel')
# CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]
# COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))
# #!

# #! - Video Capture
# # Start video capture
# cap = cv2.VideoCapture(0)

# #! - Mavlink Connection


# #!

# # GStreamer setup
# pipeline_str = (
#     "v4l2src device=/dev/video0 ! tee name=t "
#     "t. ! queue ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay ! udpsink host=192.168.11.146 port=5000 "
#     "t. ! queue ! videoconvert ! appsink emit-signals=True max-buffers=1 drop=True"
# )
# pipeline = Gst.parse_launch(pipeline_str)

# # Function to handle new samples from appsink
# def on_new_sample(sink):
#     sample = sink.emit("pull-sample")
#     buf = sample.get_buffer()
#     caps = sample.get_caps()
#     height = caps.get_structure(0).get_int("height")[1]
#     width = caps.get_structure(0).get_int("width")[1]
    
#     # Extract the frame data from the buffer
#     result, mapinfo = buf.map(Gst.MapFlags.READ)
#     img_array = np.asarray(bytearray(mapinfo.data), dtype=np.uint8)
#     frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
    
#     # Continue with your OpenCV processing here...
#     # [ ... Your OpenCV code ... ]
#     for result in real_gps_data(mavlink):
#         if result["class"] == "TPV":
#             current_time = datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%SZ')
#             stale_time = (datetime.utcnow() + timedelta(days=1)).strftime('%Y-%m-%dT%H:%M:%SZ')
#             lat = result.get("lat")
#             lon = result.get("lon")
#             #speed = int(result.get("speed"))
#             altitude = result.get("alt")
            
#             # Construct the CoT message
#             cot = f"""<?xml version='1.0' encoding='UTF-8' standalone='yes'?>
#             <event uid="{UID}" type="a-f-A-M-F-Q-H" how="" start="{current_time}" time="{current_time}" stale="{stale_time}">
#                 <point le="0" ce="0" hae="{altitude}" lon="{lon}" lat="{lat}"/>
#             </event>"""
            
#             print(cot)
            
#             # Send the CoT message if enough time has passed
#             if time.time() - cot_last_sent >= use_delay:
#                 sock.sendto(bytes(cot, "utf-8"), (GROUP, PORT))
#                 cot_last_sent = time.time()
            
#             # Adjust the delay for the next message based on the plane's speed
#             #use_delay = get_delay_based_on_speed(speed)
            
#         #! - Frame Detection
#         # Capture video frame
#         ret, frame = cap.read()
#         if ret:
#             # Perform object detection on the frame
#             (h, w) = frame.shape[:2]
#             blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)
#             net.setInput(blob)
#             detections = net.forward()
#             for i in np.arange(0, detections.shape[2]):
#                 confidence = detections[0, 0, i, 2]
#                 if confidence > 0.2:
#                     idx = int(detections[0,0,i,1])
                    
#                     #! - If "person"
#                     if CLASSES[idx] == "person":
#                         box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
#                         (startX, startY, endX, endY) = box.astype("int")
#                         label = "{}: {:.2f}%".format(CLASSES[idx], confidence * 100)
#                         cv2.rectangle(frame, (startX, startY), (endX, endY), COLORS[idx], 2)
#                         y = startY - 15 if startY - 15 > 15 else startY + 15
#                         cv2.putText(frame, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)
                        
#                         if EXPORT_IMAGE:
#                             time_string = datetime.now().strftime("%Y%m%d-%H%M%S")
#                             cv2.imwrite(IMAGE_DIR+'/DETECTED-Person-C_{}-GPS_{}.jpg'.format(time_string, mavlink.location()), frame)

#                         # HERE: You can send a unique CoT message for detected person's location
#                         # (Use the mock GPS data for now, but replace with MAVLink data later)
#                         # I am assuming that your detection system also has an associated GPS module.

#                         # Build your CoT message here for the detected person
#                         # and send it to ATAK
                        
#                         #! - Send Message
#                         # Create a CoT message for the detected person
#                         lat, lon, altitude = result["lat"], result["lon"], result["alt"]  
#                         cot_msg = build_cot_message(lat, lon, altitude)
                        
#                         # Transmit the CoT message to ATAK
#                         sock.sendto(bytes(cot_msg, "utf-8"), (GROUP, PORT))

#                         print(cot_msg)  # Also print the CoT message to the console for debugging.

#     buf.unmap(mapinfo)
#     return Gst.FlowReturn.OK


# appsink = pipeline.get_by_element_factory_name("appsink")
# appsink.connect("new-sample", on_new_sample)

# # Start the GStreamer pipeline
# pipeline.set_state(Gst.State.PLAYING)

# # Start the main loop
# GLib.MainLoop().run()
# # Release video capture when done
# pipeline.set_state(Gst.State.NULL)

# cap.release()
# process.terminate()
# #!