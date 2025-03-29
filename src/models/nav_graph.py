from dataclasses import dataclass, field
from typing import List, Dict, Optional

@dataclass
class Vertex:
    index: int
    x: float
    y: float
    name: str = ""
    is_charger: bool = False
    occupied_by: Optional[int] = None

@dataclass
class Lane:
    start: int
    end: int
    speed_limit: float = 0
    occupied_by: Optional[int] = None
    reservation_time: float = 0

@dataclass
class NavigationGraph:
    vertices: List[Vertex] = field(default_factory=list)
    lanes: List[Lane] = field(default_factory=list)
    adjacency_list: Dict[int, List[int]] = field(default_factory=dict)
    name: str = ""

    def build_adjacency_list(self):
        self.adjacency_list = {}
        for lane in self.lanes:
            if lane.start not in self.adjacency_list:
                self.adjacency_list[lane.start] = []
            self.adjacency_list[lane.start].append(lane.end)