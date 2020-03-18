from enum import Enum

class PathState(Enum):
    INITIALIZE = 0
    LOAD_FILES = 1
    EXTRACT = 2
    EXECUTE = 3