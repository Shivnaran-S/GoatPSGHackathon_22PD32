import json
import math
from typing import List, Tuple
from queue import PriorityQueue
import os

from models.nav_graph import NavigationGraph, Vertex, Lane

def load_nav_graph(file_name: str) -> NavigationGraph:
    """Load navigation graph from JSON file"""
    base_dir = os.path.dirname(__file__)
    data_dir = os.path.join(base_dir, '..', '..', 'data')  # Navigate two directories up to 'data'
    file_path = os.path.join(data_dir, file_name)
    #print(file_path)

    with open(file_path, 'r') as f:
        data = json.load(f)
    
    nav_graph = NavigationGraph()
    nav_graph.name = data.get("building_name", "")
    
    # Get the first level (assuming single level for simplicity)
    level_data = data["levels"][next(iter(data["levels"]))]  # It extracts the first key from the data["levels"] dictionary, then uses that key to get the associated value from the dictionary. The result is stored in level_data
                                                             # level data ("level1", "l0", "l1") will be a dictionary with vertices and lanes as keys
                                                             
    # Process vertices
    for idx, vertex_data in enumerate(level_data["vertices"]):
        x, y, attrs = vertex_data[0], vertex_data[1], vertex_data[2]
        nav_graph.vertices.append(Vertex(
            index=idx,
            x=x,
            y=y,
            name=attrs.get("name", ""),
            is_charger=attrs.get("is_charger", False)
        ))
    
    # Process lanes
    for lane_data in level_data["lanes"]:
        start, end = lane_data[0], lane_data[1]
        attrs = lane_data[2] if len(lane_data) > 2 else {}
        nav_graph.lanes.append(Lane(
            start=start,
            end=end,
            speed_limit=attrs.get("speed_limit", 0)
        ))
    
    nav_graph.build_adjacency_list()
    return nav_graph

def calculate_distance(v1: Vertex, v2: Vertex) -> float:
    """Calculate Euclidean distance between two vertices"""
    return math.sqrt((v2.x - v1.x)**2 + (v2.y - v1.y)**2)

def find_shortest_path(graph: NavigationGraph, start: int, end: int) -> List[int]:
    """Find shortest path using A* algorithm"""
    # Heuristic function (Euclidean distance)
    def heuristic(a: int, b: int) -> float:
        v1 = graph.vertices[a]
        v2 = graph.vertices[b]
        return calculate_distance(v1, v2)
    
    open_set = PriorityQueue()
    open_set.put((0, start))  # 0 represents the f_score of the starting node
    came_from = {}
    g_score = {vertex.index: float('inf') for vertex in graph.vertices}
    g_score[start] = 0
    f_score = {vertex.index: float('inf') for vertex in graph.vertices}
    f_score[start] = heuristic(start, end)  # g_score[start] + heuristic(start, end) = 0 + heuristic(start, end) = heuristic(start, end)
    
    open_set_hash = {start}  # set that keeps track of the nodes currently in the priority queue (open_set)
    
    while not open_set.empty():
        current = open_set.get()[1]
        open_set_hash.remove(current)
        
        if current == end:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path
        
        for neighbor in graph.adjacency_list.get(current, []):
            tentative_g_score = g_score[current] + calculate_distance(
                graph.vertices[current], 
                graph.vertices[neighbor]
            )
            
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, end)
                if neighbor not in open_set_hash:
                    open_set.put((f_score[neighbor], neighbor))
                    open_set_hash.add(neighbor)
    
    return []  # No path found