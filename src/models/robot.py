from enum import Enum, auto

class RobotStatus(Enum):
    IDLE = auto()
    MOVING = auto()
    WAITING = auto()
    CHARGING = auto()
    TASK_COMPLETE = auto()
    ERROR = auto()
    MOVING_TO_CHARGER = auto()

class Robot:
    def __init__(self, id: int, current_vertex: int):
        self.id = id
        self.current_vertex = current_vertex
        self.destination_vertex = None
        self.path = []
        self.status = RobotStatus.IDLE
        self.battery = 100.0
        self.color = "red"
        self.speed = 0.2  # units per second
        self.progress = 0.0
        self.task_start_time = 0.0
        self.total_distance = 0.0