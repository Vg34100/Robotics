import pickle, struct, socket, threading, logging, time
import cam_record as cam
import log_config as log

import frame_queue as frames
import asyncio
shutdown_event = asyncio.Event()


async def handle_client(reader, writer):
    client_address = writer.get_extra_info('peername')
    logging.info(f"STREAM: Accepted video stream connection from {client_address}")
    try:
        while True:
            # Read frame from camera (this needs to be non-blocking or in a separate thread)
            ret, frame = await frames.get_frame()  # You need to define `get_frame()`
            if not ret:
                logging.warning(f"STREAM: Failed to read frame for {client_address}")
                break

            data = pickle.dumps(frame)
            message_size = struct.pack("L", len(data))

            try:
                writer.write(message_size + data)
                await writer.drain()
            except Exception as e:
                logging.error(f"STREAM: Error sending data to {client_address}: {e}")
                break
    finally:
        writer.close()
        await writer.wait_closed()
        logging.info(f"STREAM: Connection with {client_address} closed.")
        
async def video_stream_server(cap, CONFIG):
    logging.info("STREAM: Starting video streaming program")
    server = await asyncio.start_server(
        lambda r, w: handle_client(r, w),
        CONFIG.get("G-HOST"),
        CONFIG.get("G-PORT")
    )

    async with server:
        await shutdown_event.wait()
        server.close()
        await server.wait_closed()

def run_server(cap, CONFIG):
    asyncio.run(video_stream_server(cap, CONFIG))

def stop_server():
    logging.info("STREAM: Attempting to close video streaming program")
    shutdown_event.set()


# Global shutdown event
# shutdown_event = threading.Event()

# def video_stream_server(cap, CONFIG):
#     host = CONFIG.get("G-HOST") # Listen on all available interfaces
#     port = CONFIG.get("G-PORT") # Choose an appropriate port
#     timeout = CONFIG.get("TIMEOUT_TIMER")

#     server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     try:
#         server_socket.bind((host, port))
#         server_socket.listen(5)
#         server_socket.settimeout(timeout)  # Set a timeout for the accept call
#         logging.info("STREAM: Video stream server waiting for a connection...")
#     except Exception as e:
#         logging.error(f"STREAM: Failed to bind or listen on socket: {e}")
#         return

#     def handle_client(connection, client_address):
#         logging.info(f"STREAM: Accepted video stream connection from {client_address}")
#         try:
#             while True:
#                 ret, frame = cap.read()
#                 if not ret:
#                     logging.warning(f"STREAM: Failed to read frame from camera for {client_address}")
#                     break

#                 data = pickle.dumps(frame)
#                 message_size = struct.pack("L", len(data))
#                 try:
#                     connection.sendall(message_size + data)
#                 except Exception as e:
#                     logging.error(f"STREAM: Error sending data to {client_address}: {e}")
#                     break
#         finally:
#             connection.close()
#             logging.info(f"STREAM: Connection with {client_address} closed.")

#     try:
#         logging.info(f"STREAM: Attempting to Connect with a timeout of {timeout} seconds")
#         while not shutdown_event.is_set():  # Check if shutdown is signaled
#             try:
#                 connection, client_address = server_socket.accept()
#                 client_thread = threading.Thread(target=handle_client, args=(connection, client_address))
#                 client_thread.daemon = True
#                 client_thread.start()
#             except socket.timeout:
#                 continue  # Continue the loop if accept times out
#             except Exception as e:
#                 logging.error(f"STREAM: Error accepting connections: {e}")
#                 break  # Break the loop on other exceptions
#     finally:
#         server_socket.close()
#         logging.info("STREAM: Server socket closed.")
        
# def start_stream(cap, CONFIG):
#     video_stream_thread = threading.Thread(target=video_stream_server, args=(cap, CONFIG))
#     video_stream_thread.start()
#     return video_stream_thread
    
# def end_stream(video_stream_thread):
#     shutdown_event.set()  # Signal the server thread to shut down
#     video_stream_thread.join()  # Wait for the server thread to finish
#     logging.info("STREAM: Video stream server has been shut down.")

# def main():
#     CONFIG = log.Configuration()  
#     cap, fps = cam.setup_cam()
#     print("Camera Ready")
#     # G-Streamer Compatibility
#     video_stream_thread = start_stream(cap, CONFIG)
#     try:
#         while True:
#             time.sleep(1)
#     except KeyboardInterrupt:
#         end_stream(video_stream_thread)


# if __name__ == "__main__":
#     main()