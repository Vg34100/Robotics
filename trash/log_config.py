import logging, yaml, datetime

class Configuration:
    def __init__(self, config_file="config.yaml"):
        with open(config_file, 'r', encoding='UTF-8') as f:
            self.config = yaml.safe_load(f)
            

    def get(self, option):
        return self.config.get(option)
  
def log_init(LOG_DIR):
    current_date = datetime.datetime.now().strftime("%Y-%m-%d")

    # Set up the logging with a filename that includes the current date
    log_filename = f"{LOG_DIR}/log_{current_date}.txt"
    logging.basicConfig(level=logging.DEBUG, format="%(asctime)s - %(levelname)s - %(message)s", filename=log_filename)
    
    
# def set_constants(CONFIG):
#     mavlink = None
#     GROUP = CONFIG.get("GROUP")
#     PORT = CONFIG.get("PORT")
#     UID = CONFIG.get("UID")
#     IMAGE_DIR = CONFIG.get("IMAGE_DIR")
#     use_delay = CONFIG.get("FAST_DELAY")
#     return GROUP, PORT, UID, IMAGE_DIR, mavlink, use_delay