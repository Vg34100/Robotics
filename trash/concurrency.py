import multiprocessing
import time

def print_hello():
    while True:
        print("Hello")
        time.sleep(1)  # Adding a slight delay for readability

def print_world():
    while True:
        print("World")
        time.sleep(1)  # Adding a slight delay for readability

if __name__ == '__main__':
    # Create the Process objects
    hello_process = multiprocessing.Process(target=print_hello)
    world_process = multiprocessing.Process(target=print_world)

    # Start the processes
    hello_process.start()
    world_process.start()
    
    try: 
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        hello_process.join()
        world_process.join()