# general defines

from enum import Enum

DEBUG = True
DATA_DIR = 'data'
WINDOW_NAME = "Drone final"

class RunMode(Enum):
    ONLINE = 0
    OFFLINE = 1
    CREATE_VIDEO = 2

STATE = RunMode.CREATE_VIDEO