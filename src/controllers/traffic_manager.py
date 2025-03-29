from threading import Lock
from typing import Dict, List, Set, Tuple, Optional
from queue import Queue

from ..models import NavigationGraph

class TrafficManager:
    """Manages traffic and collision avoidance"""
    def __init__(self, nav_graph: NavigationGraph):
        self.nav_graph = nav_graph
        self.lane_reservations: Dict[Tuple[int, int], int] = {}  # (start, end) -> robot_id
        self.vertex_reservations: Dict[int, int] = {}  # vertex_index -> robot_id
        self.lock = Lock() #threading.Lock()
        self.waiting_robots = Queue()
    
    def reserve_path(self, robot_id: int, path: List[int]) -> bool:
        """Attempt to reserve a path for a robot, returns True if successful"""
        with self.lock:
            # Check if all vertices and lanes in the path are available
            for i in range(len(path)):
                vertex = path[i]
                # Check vertex availability
                if self.vertex_reservations.get(vertex, None) not in [None, robot_id]:
                    return False
                
                # Check lane availability (except for the last vertex)
                if i < len(path) - 1:
                    next_vertex = path[i+1]
                    lane_key = (vertex, next_vertex)
                    if self.lane_reservations.get(lane_key, None) not in [None, robot_id]:
                        return False
            
            # If all available, reserve them
            for i in range(len(path)):
                vertex = path[i]
                self.vertex_reservations[vertex] = robot_id
                
                if i < len(path) - 1:
                    next_vertex = path[i+1]
                    lane_key = (vertex, next_vertex)
                    self.lane_reservations[lane_key] = robot_id
            
            return True
    
    def release_path(self, robot_id: int, path: List[int]):
        """Release reservations for a robot's path"""
        with self.lock:
            for i in range(len(path)):
                vertex = path[i]
                if self.vertex_reservations.get(vertex) == robot_id:
                    self.vertex_reservations[vertex] = None
                
                if i < len(path) - 1:
                    next_vertex = path[i+1]
                    lane_key = (vertex, next_vertex)
                    if self.lane_reservations.get(lane_key) == robot_id:
                        self.lane_reservations[lane_key] = None
    
    def find_alternative_path(self, robot_id: int, start: int, end: int, blocked_vertices: Set[int]) -> List[int]:
        """Find an alternative path avoiding blocked vertices"""
        # Create a temporary graph without blocked vertices
        temp_adj = {}
        for v, neighbors in self.nav_graph.adjacency_list.items():
            if v not in blocked_vertices:
                temp_adj[v] = [n for n in neighbors if n not in blocked_vertices]
        
        # Use BFS to find an alternative path
        queue = [(start, [start])]
        visited = set()
        
        while queue:
            current, path = queue.pop(0)
            if current == end:
                return path
            
            if current not in visited:
                visited.add(current)
                for neighbor in temp_adj.get(current, []):
                    if neighbor not in visited:
                        queue.append((neighbor, path + [neighbor]))
        
        return []