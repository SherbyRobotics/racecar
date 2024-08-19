import logging
import os
from datetime import datetime
from importlib import reload
from pathlib import Path

FORMAT = '[%(asctime)s] %(name)s %(levelname)s :  %(message)s'
LOGGING_FORMAT = logging.Formatter(FORMAT)

def setup_logger(log_parent_file, file_level=logging.DEBUG, print_level=logging.INFO):
    # Need to reload logging after init_node because of a ROS bug
    reload(logging)

    filepath = datetime.now().strftime(
        str(Path(log_parent_file).parent.parent) + "/logs/" + Path(log_parent_file).name[:-3] + '_%H_%M_%d_%m_%Y.log')

    # Create the directory if it doesnt exits
    Path(os.path.split(filepath)[0]).mkdir(parents=True, exist_ok=True)

    logging.basicConfig(level=file_level,
                        format=FORMAT,
                        datefmt='%m-%d %H:%M',
                        filename=filepath,
                        filemode='w')

    console = logging.StreamHandler()
    console.setLevel(print_level)
    console.setFormatter(LOGGING_FORMAT)
    logging.getLogger('').addHandler(console)

def get_logger(name):
    return logging.getLogger(name)
