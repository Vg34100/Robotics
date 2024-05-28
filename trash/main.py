import log_config as log
import general as g
import cot_msg as cot
import mavlink_connect as mav
import cam_record as cam
import stream_server as stream

from queue import Queue
import os, time
import threading
import asyncio
from datetime import datetime

async def main_async(cap, CONFIG):
    await stream.video_stream_server(cap, CONFIG)

async def main():
    # Config Initialization
    CONFIG = log.Configuration()  
    
    # Generate Directories
    g.create_path(CONFIG.get("EXPORT_DIR"))
    IMAGE_DIR = g.create_path(os.path.join(CONFIG.get("EXPORT_DIR"),CONFIG.get("IMAGE_DIR")))
    VIDEO_DIR = g.create_path(os.path.join(CONFIG.get("EXPORT_DIR"),CONFIG.get("VIDEO_DIR")))
    LOG_DIR = g.create_path(os.path.join(CONFIG.get("EXPORT_DIR"),CONFIG.get("LOG_DIR")))

    # Log Initialization
    log.log_init(LOG_DIR)
    g.session_info(CONFIG)



    # UDP socket for sending CoT messages
    cot_last_sent = 0
    sock = cot.cot_init() if CONFIG.get("COT_MSG") else None
    
    # MAVLINK Connection
    mavlink, process = mav.mavConnect(CONFIG) if CONFIG.get("MAVLINK_CONNECT") else None, None

    # Camera Setup
    cap, fps = cam.setup_cam()
    if CONFIG.get('CAM_RECORD'): 
        filename, extension = os.path.splitext(CONFIG.get('VIDEO_FILE'))
        VIDEO_FILE = os.path.join(VIDEO_DIR, f"{filename}-{datetime.now().strftime('%m%d%y_%H%M%S%f')}{extension}")
        out = cam.record_init(cap, VIDEO_FILE, fps)

    import frame_queue as queue
    capture_thread = queue.start_frame_thread(cap)

    import frame_process as processing
    process_loop, processing_thread = processing.start_frame_processing(CONFIG, out)



    # G-Streamer Compatibility
    # video_stream_thread = stream.start_stream(cap, CONFIG) if CONFIG.get("STREAM_SERVER") else None

    # Frame Queue
    #FRAME_QUEUE = Queue()

    # Video Rendering Thread
    # recording_thread = cam.start_recording(cap, fps, CONFIG)

    try:
        while True:
            time.sleep(1)
            print("WORKING")
    except KeyboardInterrupt:
        # stream.stop_server()
        queue.end_frame_thread(capture_thread)
        processing.end_frame_processing(process_loop, processing_thread)
        # if CONFIG.get("STREAM_SERVER"): stream.end_stream(video_stream_thread)
        # cam.end_recording(recording_thread)
        cam.close_cam(cap)     





# In the script's entry point
if __name__ == "__main__":
    #CONFIG = log.Configuration()
    #cap = cam.setup_cam()

    # Run the main asyncio function
    asyncio.run(main())