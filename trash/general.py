import logging, os
def session_info(CONFIG):
    session_info = f"Session Started:\n"

    session_info += f"----------------\n"
    session_info += f"* Session Info:\n"
    session_info += f"**MOCK FLIGHT**\n" if CONFIG.get('MOCK') else ""
    session_info += f"UID: {CONFIG.get('UID')}\n"
    session_info += f"GROUP: {CONFIG.get('GROUP')}\n"
    session_info += f"PORT: {CONFIG.get('PORT')}\n"

    session_info += f"----------------\n"
    session_info += f"* Directories:\n"
    session_info += f"IMAGE DIR: {CONFIG.get('IMAGE_DIR')}\n"
    session_info += f"----------------\n"

    session_info += f"* Active:\n"

    session_info += f"-MAVLINK_CONNECT\n" if CONFIG.get("MAVLINK_CONNECT") else ""

    session_info += f"-STREAM_SERVER\n" if CONFIG.get("STREAM_SERVER") else ""

    session_info += f"-CAM_RECORD\n" if CONFIG.get("CAM_RECORD") else ""
    session_info += f"-EXPORT_FRAMES\n" if CONFIG.get("EXPORT_FRAMES") else ""

    session_info += f"-OBJECT_DETECT\n" if CONFIG.get("OBJECT_DETECT") else ""
    session_info += f"-EXPORT_DETECTED\n" if CONFIG.get("EXPORT_DETECTED") else ""

    session_info += f"-COT_MSG\n" if CONFIG.get("COT_MSG") else ""
    session_info += f"----------------\n"
    logging.info(session_info)

def create_path(directory):
    # Check if the directory path is not empty
    if not directory:
        print("Provided directory path is empty. Please check your configuration.")
        return

    # Check if the directory already exists
    if not os.path.exists(directory):
        try:
            os.makedirs(directory)
            print(f"Created directory: {directory}")
        except Exception as e:
            print(f"Error creating directory {directory}: {e}")

    return directory
