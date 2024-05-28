import threading, asyncio, os
from frame_queue import get_frame
from cam_record import timed_frame, record_init

shutdown_event = threading.Event()

def start_async_loop(loop):
    asyncio.set_event_loop(loop)
    loop.run_forever()
    
def start_frame_processing(CONFIG, out):
    async_loop = asyncio.new_event_loop()
    async_loop_thread = threading.Thread(target=start_async_loop, args=(async_loop,), daemon=True)
    async_loop_thread.start()
    # Run the process_frames_async coroutine in the async loop
    asyncio.run_coroutine_threadsafe(process_frames_async(CONFIG, out), async_loop)
    return async_loop, async_loop_thread

def end_frame_processing(async_loop, async_loop_thread):
    shutdown_event.set()
    async_loop.call_soon_threadsafe(async_loop.stop)
    async_loop_thread.join()

async def process_frames_async(CONFIG, out):
    IMAGE_DIR = os.path.join(CONFIG.get('EXPORT_DIR'),CONFIG.get('IMAGE_DIR'))
    FRAME_INTERVAL = CONFIG.get('TIME_INTERVAL')

    while not shutdown_event.is_set():
        ret, frame = await get_frame()
        if frame is not None:
            # Process the frame here
            if CONFIG.get('CAM_RECORD'): out.write(frame)
            if CONFIG.get('EXPORT_FRAMES'): timed_frame(FRAME_INTERVAL, frame, IMAGE_DIR)
            pass
        await asyncio.sleep(0.01)  # Prevents the loop from hogging CPU