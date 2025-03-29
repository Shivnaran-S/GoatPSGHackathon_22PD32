import logging
import time
from typing import Dict, List, Optional, Set, Tuple
from threading import Lock

from models.robot import Robot, RobotStatus
from models.nav_graph import NavigationGraph

from .traffic_manager import TrafficManager

from utils.helpers import calculate_distance, find_shortest_path

class FleetManager:
    """Manages the fleet of robots and their tasks"""
    def __init__(self, nav_graph: NavigationGraph, traffic_manager: TrafficManager):
        self.nav_graph = nav_graph
        self.traffic_manager = traffic_manager
        self.robots: Dict[int, Robot] = {}
        self.next_robot_id = 1
        self.logger = self._setup_logger()
        self.lock = Lock()

    def _setup_logger(self):
        logger = logging.getLogger('fleet_manager')
        logger.setLevel(logging.INFO)
        
        # Create file handler
        fh = logging.FileHandler('fleet_logs.txt')
        fh.setLevel(logging.INFO)
        
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        fh.setFormatter(formatter)
        
        logger.addHandler(fh)
        return logger

    def spawn_robot(self, vertex_index: int) -> Optional[Robot]:
        if vertex_index < 0 or vertex_index >= len(self.nav_graph.vertices):
            return None
        
        with self.lock:
            if self.nav_graph.vertices[vertex_index].occupied_by is not None:
                return None
            
            robot_id = self.next_robot_id
            self.next_robot_id += 1
            
            robot = Robot(robot_id, vertex_index)
            self.robots[robot_id] = robot
            self.nav_graph.vertices[vertex_index].occupied_by = robot_id
            
            self.logger.info(f"Spawned Robot {robot_id} at vertex {vertex_index}")
            return robot
    
    def assign_task(self, robot_id: int, destination_vertex: int) -> bool:
        """Assign a navigation task to a robot"""
        if robot_id not in self.robots:
            return False
        
        if destination_vertex < 0 or destination_vertex >= len(self.nav_graph.vertices):
            return False
        
        robot = self.robots[robot_id]
        
        if robot.current_vertex == destination_vertex:
            self.logger.info(f"Robot {robot_id} is already at destination {destination_vertex}")
            return False
        
        # Find the shortest path
        path = find_shortest_path(self.nav_graph, robot.current_vertex, destination_vertex)
        
        if not path:
            self.logger.warning(f"No path found from {robot.current_vertex} to {destination_vertex}")
            return False
        
        # Try to reserve the path
        if not self.traffic_manager.reserve_path(robot_id, path):
            # If path is blocked, try to find an alternative
            blocked_vertices = {
                v for v, rid in self.traffic_manager.vertex_reservations.items() 
                if rid is not None and rid != robot_id
            }
            
            alt_path = self.traffic_manager.find_alternative_path(robot_id, robot.current_vertex, destination_vertex, blocked_vertices)
            
            if alt_path and self.traffic_manager.reserve_path(robot_id, alt_path):
                path = alt_path
                self.logger.info(f"Robot {robot_id} taking alternative path to avoid congestion")
            else:
                self.logger.warning(f"Path blocked for Robot {robot_id} to {destination_vertex}")
                return False
        
        # Update robot state
        robot.destination_vertex = destination_vertex
        robot.path = path
        robot.status = RobotStatus.MOVING
        robot.progress = 0.0
        robot.task_start_time = time.time()
        
        # Calculate total distance for the path
        total_distance = 0.0
        for i in range(len(path) - 1):
            v1 = self.nav_graph.vertices[path[i]]
            v2 = self.nav_graph.vertices[path[i+1]]
            total_distance += calculate_distance(v1, v2)
        robot.total_distance = total_distance
        
        self.logger.info(f"Assigned task to Robot {robot_id}: {robot.current_vertex} -> {destination_vertex} via {path}")
        return True
    
    def update_robot_positions(self, delta_time: float):
        """Update positions of all moving robots"""
        with self.lock:
            for robot in list(self.robots.values()):
                if robot.status == RobotStatus.MOVING and robot.path:
                    # Calculate movement for this time step
                    movement = robot.speed * delta_time
                    
                    # Get current and next vertices
                    current_vertex_idx = robot.path[0]
                    next_vertex_idx = robot.path[1]
                    
                    v1 = self.nav_graph.vertices[current_vertex_idx]
                    v2 = self.nav_graph.vertices[next_vertex_idx]
                    distance = calculate_distance(v1, v2)
                    
                    # Update progress along the current lane
                    robot.progress += movement / distance
                    
                    # If reached the next vertex
                    if robot.progress >= 1.0:
                        # Move to next vertex
                        self.nav_graph.vertices[current_vertex_idx].occupied_by = None
                        self.nav_graph.vertices[next_vertex_idx].occupied_by = robot.id
                        
                        # Update robot's current vertex
                        robot.current_vertex = next_vertex_idx
                        robot.path.pop(0)
                        robot.progress = 0.0
                        
                        # If reached destination
                        if len(robot.path) == 1:  # Last element is destination
                            robot.status = RobotStatus.TASK_COMPLETE
                            self.traffic_manager.release_path(robot.id, robot.path)
                            robot.path = []
                            self.logger.info(f"Robot {robot.id} reached destination {robot.destination_vertex}")
                        
                        # Check if next vertex is a charger
                        elif self.nav_graph.vertices[next_vertex_idx].is_charger and robot.battery < 30.0:
                            robot.status = RobotStatus.CHARGING
                            self.logger.info(f"Robot {robot.id} is charging at vertex {next_vertex_idx}")
                    
                    # Update battery (consumption proportional to movement)
                    robot.battery = max(0, robot.battery - movement * 0.5)
                    
                    # If battery is critically low, try to find nearest charger
                    if robot.battery < 10.0 and robot.status != RobotStatus.CHARGING:
                        self._handle_low_battery(robot)
                
                elif robot.status == RobotStatus.CHARGING:
                    # Charge the battery
                    robot.battery = min(100.0, robot.battery + delta_time * 10.0)
                    if robot.battery >= 95.0:
                        robot.status = RobotStatus.IDLE
                        self.logger.info(f"Robot {robot.id} finished charging (battery: {robot.battery:.1f}%)")
    
    def _handle_low_battery(self, robot: Robot):
        """Handle low battery situation by redirecting to nearest charger"""
        # Find all charger vertices
        chargers = [v.index for v in self.nav_graph.vertices if v.is_charger]
        
        if not chargers:
            self.logger.error("No charger vertices found in the navigation graph!")
            robot.status = RobotStatus.ERROR
            return
        
        # Find nearest charger
        min_distance = float('inf')
        nearest_charger = None
        
        for charger_idx in chargers:
            path = find_shortest_path(self.nav_graph, robot.current_vertex, charger_idx)
            if path:
                distance = sum(
                    calculate_distance(self.nav_graph.vertices[path[i]], self.nav_graph.vertices[path[i+1]])
                    for i in range(len(path) - 1)
                )
                if distance < min_distance:
                    min_distance = distance
                    nearest_charger = charger_idx
        
        if nearest_charger is None:
            self.logger.error(f"Robot {robot.id} can't reach any charger!")
            robot.status = RobotStatus.ERROR
            return
        
        # Assign new task to the charger
        self.logger.warning(f"Robot {robot.id} battery critically low ({robot.battery:.1f}%), redirecting to charger at {nearest_charger}")
        self.assign_task(robot.id, nearest_charger)
        if robot.status == RobotStatus.MOVING:
            robot.status = RobotStatus.MOVING_TO_CHARGER
    
    def get_robot_position(self, robot_id: int) -> Tuple[float, float]:
        """Get current position of a robot (interpolated if between vertices)"""
        if robot_id not in self.robots:
            return (0, 0)
        
        robot = self.robots[robot_id]
        
        if not robot.path or robot.status not in [RobotStatus.MOVING, RobotStatus.MOVING_TO_CHARGER]:
            v = self.nav_graph.vertices[robot.current_vertex]
            return (v.x, v.y)
        else:
            v1 = self.nav_graph.vertices[robot.path[0]]
            v2 = self.nav_graph.vertices[robot.path[1]]
            x = v1.x + (v2.x - v1.x) * robot.progress
            y = v1.y + (v2.y - v1.y) * robot.progress
            return (x, y)
    
    def get_robot_info(self, robot_id: int) -> Dict:
        """Get information about a robot for display"""
        if robot_id not in self.robots:
            return None
        
        robot = self.robots[robot_id]
        pos = self.get_robot_position(robot_id)
        
        return {
            'id': robot.id,
            'x': pos[0],
            'y': pos[1],
            'status': robot.status.name,
            'battery': f"{robot.battery:.1f}%",
            'color': robot.color,
            'current_vertex': robot.current_vertex,
            'destination': robot.destination_vertex,
            'path': robot.path,
            'progress': f"{robot.progress*100:.1f}%",
            'distance_remaining': self._calculate_remaining_distance(robot),
            'time_elapsed': time.time() - robot.task_start_time if robot.task_start_time > 0 else 0
        }
    
    def _calculate_remaining_distance(self, robot: Robot) -> float:
        """Calculate remaining distance for a robot's current path"""
        if not robot.path or len(robot.path) < 2:
            return 0.0
        
        remaining_distance = 0.0
        
        # Add distance from current position to next vertex
        v1 = self.nav_graph.vertices[robot.path[0]]
        v2 = self.nav_graph.vertices[robot.path[1]]
        remaining_distance += calculate_distance(v1, v2) * (1 - robot.progress)
        
        # Add distance for remaining path
        for i in range(1, len(robot.path) - 1):
            v1 = self.nav_graph.vertices[robot.path[i]]
            v2 = self.nav_graph.vertices[robot.path[i+1]]
            remaining_distance += calculate_distance(v1, v2)
        
        return remaining_distance