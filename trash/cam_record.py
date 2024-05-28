import cv2, logging, os, threading
from datetime import datetime
import log_config as log
import general as g

# Global shutdown event
shutdown_event = threading.Event()

def setup_cam():
    logging.info("CAM: Starting camera setup...")
    try:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            logging.error("CAM Error: Could not open camera.")
            return None, None
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 27)

        # Attempt to get the FPS of the camera
        fps = cap.get(cv2.CAP_PROP_FPS)
        if fps == 0:
            logging.warning("CAM Warning: Could not determine FPS. Defaulting to 30.")
            fps = 30  # Default FPS if unable to determine

        logging.info(f"CAM: Found camera with fps of {fps}")
        return cap, fps

    except Exception as e:
        logging.error(f"CAM: An error occurred during camera setup: {e}")
        return None, None
    
def record_init(cap, filename='output.avi', fps=30, codec='XVID'):
    logging.info("RECORD: Starting recording script...")
    if not cap.isOpened():
        logging.error("RECORD: Capture device is not opened.")
        return None

    # Defaulting to 30 FPS if not specified or if cap FPS is not available
    if fps is None or fps == 0:
        fps = cap.get(cv2.CAP_PROP_FPS) or 30
        logging.info(f"RECORD: Using fps: {fps}")

    # Get frame width and height from the capture device
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # Setup the codec
    try:
        fourcc = cv2.VideoWriter_fourcc(*codec)
    except Exception as e:
        logging.error(f"RECORD: Failed to set codec {codec}: {e}")
        return None

    # Initialize the VideoWriter object
    try:
        out = cv2.VideoWriter(filename, fourcc, fps, (frame_width, frame_height))
        if not out.isOpened():
            logging.error("RECORD: VideoWriter failed to open.")
            return None
        return out
    except Exception as e:
        logging.error(f"RECORD: Error initializing VideoWriter: {e}")
        return None
 
import time 
def timed_frame(time_interval, frame, image_dir):
    # Use a mutable object like a list to store the last export time
    # since non-primitive types can maintain state across function calls
    if 'last_export_time' not in timed_frame.__dict__:
        timed_frame.last_export_time = [0]

    current_time = time.time()
    if current_time - timed_frame.last_export_time[0] >= time_interval:
        export_frame(frame, image_dir)
        timed_frame.last_export_time[0] = current_time
    
def export_frame(frame, image_dir):
    filename = os.path.join(image_dir, f"frame_{datetime.now().strftime('%m%d%y_%H%M%S%f')}.jpg")
    cv2.imwrite(filename, frame)
    logging.info(f"RECORD: Exported frame to {filename}")

    
def close_cam(cap):
    cap.release()

def end_recording(out):
    out.release()
    
def start_recording(cap, fps, CONFIG):
    recording_thread = threading.Thread(target=recording_process, args=(cap, fps, CONFIG))
    recording_thread.start()
    return recording_thread 
    
def end_recording(recording_thread):
    shutdown_event.set()
    recording_thread.join()
    logging.info("RECORD: Recording thread has been shut down.")
    
def recording_process(cap, fps, CONFIG):
    frame_interval = CONFIG.get("FRAME_INTERVAL")  # or set a default
    VIDEO_DIR = g.create_path(os.path.join(CONFIG.get("EXPORT_DIR"),CONFIG.get("VIDEO_DIR")))
    IMAGE_DIR = g.create_path(os.path.join(CONFIG.get("EXPORT_DIR"),CONFIG.get("IMAGE_DIR")))

    out = record_init(cap, os.path.join(VIDEO_DIR, CONFIG.get('VIDEO_FILE')), fps)
    logging.info("RECORD: Recording has been initialized")
    frame_count = 0

    try:
        while not shutdown_event.is_set():
            ret, frame = cap.read()
            if ret:
                if out is not None:
                    if CONFIG.get('CAM_RECORD'): out.write(frame)
                if frame_count % frame_interval == 0:
                    if CONFIG.get('EXPORT_FRAMES'): export_frame(frame, IMAGE_DIR)
                frame_count += 1
    except Exception as e:
        logging.error(f"RECORD: An exception occurred in the recording process: {e}")
    finally:
        if out is not None:
            out.release()
        logging.info("RECORD: Recording process ended.")
    
def main():
    CONFIG = log.Configuration()  
    stop_queue = False
    frame_interval = 30
    
    # Generate Paths
    g.create_path(CONFIG.get("EXPORT_DIR"))
    IMAGE_DIR = g.create_path(os.path.join(CONFIG.get("EXPORT_DIR"),CONFIG.get("IMAGE_DIR")))
    VIDEO_DIR = g.create_path(os.path.join(CONFIG.get("EXPORT_DIR"),CONFIG.get("VIDEO_DIR")))

    # Initiate Camera
    cap, fps = setup_cam()
    
    # Start Recording
    out = record_init(cap, os.path.join(VIDEO_DIR, CONFIG.get('VIDEO_FILE')), fps)
    
    def end_program():
        nonlocal stop_queue
        end_recording(out)
        close_cam(cap)
        stop_queue = True     
    
    frame_count = 0
    try:
        while not stop_queue:        
            ret, frame = cap.read()
            if ret:    
                if out is not None:
                    out.write(frame)                
                if frame_count % frame_interval == 0: 
                    export_frame(frame, IMAGE_DIR)
                frame_count += 1  
    except KeyboardInterrupt:
        end_program()
    except Exception as e:
        print(f"An Exception occurred: {e}")
        end_program()

if __name__ == "__main__":
    main()