from enum import Enum, auto
from typing import Tuple

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
        self.color = self._get_simple_color()
        self.speed = 0.2  # units per second
        self.progress = 0.0
        self.task_start_time = 0.0
        self.total_distance = 0.0

    def _get_simple_color(self) -> str:
        """Simple color assignment using a predefined list"""
        colors = ['red', 'blue', 'purple', 'orange', 
              'cyan', 'magenta', 'yellow', 'lime', 'pink',
              'darkred', 'darkgreen', 'darkblue', 'darkcyan', 
              'darkmagenta', 'darkorange', 'teal', 'indigo', 
              'coral', 'tomato', 'gold', 'royalblue', 'slateblue',
              'mediumpurple', 'olive', 'chocolate', 'skyblue', 
              'salmon', 'peru', 'sienna']
        return colors[self.id % len(colors)]