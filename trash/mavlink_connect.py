from pymavlink import mavutil
import time, os, subprocess, logging, platform

def is_admin():
    """Check if the script is running with root/administrative privileges."""
    try:
        return os.geteuid() == 0
    except Exception as e:
        logging.error(f"MAV: Error checking administrative privileges: {e}")
        return False

def mavConnect(CONFIG):
    if platform.system() == "Windows":
        logging.error("MAV: MAVLINK connection not designed to run on Windows")
        return None
    
    try:
        # Log the start of the MAVLINK connection attempt
        logging.info("MAV: Attempting to establish MAVLINK connection...")
        command = [CONFIG.get('MAVLINK_DIR'), CONFIG.get('MASTER'), '--out=tcpin:0.0.0.0:5760', '--out=tcpin:0.0.0.0:5761', '--aircraft', CONFIG.get('UID')]

        # Checking for root privileges
        if is_admin():
            logging.info("MAV: Script running with administrative privileges, proceeding with connection.")
            process = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        else:
            logging.error("MAV: Script is not being run with administrative privileges. Please run as admin or use sudo.")
            return None

        time.sleep(1)  # Waiting before starting the connection

        # Start a connection listening on a UDP port (PI)
        mavlink = mavutil.mavlink_connection('tcp:0.0.0.0:5761')

        # Wait for the first heartbeat and log it
        mavlink.wait_heartbeat()
        logging.info(f"MAV: Heartbeat received from system (system {mavlink.target_system} component {mavlink.target_component})")

        logging.info("MAV: Mavlink connection established successfully.")
        return mavlink, process

    except Exception as e:
        # Log any exceptions or errors during the process
        logging.error(f"MAV: Error in establishing MAVLINK connection: {e}")
        return None
    
