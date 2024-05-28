import queue
import threading
import cv2
import asyncio
import socket
import pickle
import struct
import logging
shutdown_event = threading.Event()

# Create a thread-safe queue
frame_queue = queue.Queue(maxsize=10)  # Adjust maxsize as needed

def capture_frames(cap):
    
    try:
        while not shutdown_event.is_set():
            ret, frame = cap.read()
            if not ret:
                break
            if not frame_queue.full():
                frame_queue.put_nowait((ret, frame))
            else:
                # Skip the frame if the queue is full
                pass
    except Exception as e:
        logging.error(f"FRAMES: An exception occurred in the frame process: {e}")
    finally:
        logging.info("FRAMES: Frame process ended.")
    cap.release()
    
async def get_frame():
    try:
        # Wait for a frame to be available in the queue
        return frame_queue.get_nowait()
    except queue.Empty:
        # Return None or a default value if the queue is empty
        return None, None
    
def start_frame_thread(cap):
    logging.info("FRAMES: Opening frame thread")
    capture_thread = threading.Thread(target=capture_frames, args=(cap,), daemon=True)
    capture_thread.start()
    return capture_thread

def end_frame_thread(capture_thread):
    shutdown_event.set()
    capture_thread.join()
    logging.info("FRAMES: Closing frame thread")