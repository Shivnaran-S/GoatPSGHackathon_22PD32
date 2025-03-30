import tkinter as tk
from tkinter import ttk, messagebox
import math
import time
import random
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from models.nav_graph import NavigationGraph
from models.robot import  RobotStatus
from controllers.fleet_manager import FleetManager

class FleetGUI:
    def __init__(self, master, nav_graph: NavigationGraph, fleet_manager: FleetManager):
        self.master = master
        self.nav_graph = nav_graph
        self.fleet_manager = fleet_manager
        self.selected_robot = None
        self.selected_vertex = None
        self.last_update_time = time.time()
        
        self._setup_gui()
        self._draw_nav_graph()
        self._update()

    def _setup_gui(self):
        self.master.title("Goat Robotics Fleet Management")
        self.master.geometry("1200x800")
        
        self.main_frame = ttk.Frame(self.master)
        self.main_frame.pack(fill=tk.BOTH, expand=True)
        
        self.canvas_frame = ttk.Frame(self.main_frame)
        self.canvas_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        self.control_frame = ttk.Frame(self.main_frame, width=300)
        self.control_frame.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.canvas_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        self._create_control_widgets()

    def _create_control_widgets(self):
        """Create control panel widgets"""
        # Title
        ttk.Label(self.control_frame, text="Fleet Controls", font=('Helvetica', 14, 'bold')).pack(pady=(0, 10))
        
        # Robot spawning controls
        spawn_frame = ttk.LabelFrame(self.control_frame, text="Spawn Robot", padding=10)
        spawn_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(spawn_frame, text="Click on a vertex to spawn a robot").pack()
        ttk.Button(spawn_frame, text="Spawn Random Robot", command=self._spawn_random_robot).pack(fill=tk.X, pady=5)
        
        # Task assignment controls
        task_frame = ttk.LabelFrame(self.control_frame, text="Assign Task", padding=10)
        task_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(task_frame, text="1. Select robot (click on it)").pack()
        ttk.Label(task_frame, text="2. Click destination vertex").pack()
        ttk.Button(task_frame, text="Assign Random Task", command=self._assign_random_task).pack(fill=tk.X, pady=5)
        
        # Robot info display
        self.info_frame = ttk.LabelFrame(self.control_frame, text="Robot Information", padding=10)
        self.info_frame.pack(fill=tk.X, pady=5)
        
        self.info_text = tk.Text(self.info_frame, height=15, width=30, state=tk.DISABLED)
        self.info_text.pack(fill=tk.BOTH)
        
        # System controls
        control_frame = ttk.Frame(self.control_frame)
        control_frame.pack(fill=tk.X, pady=10)
        
        ttk.Button(control_frame, text="Pause All", command=self._pause_all).pack(side=tk.LEFT, expand=True)
        ttk.Button(control_frame, text="Resume All", command=self._resume_all).pack(side=tk.LEFT, expand=True)
        
        # Statistics
        stats_frame = ttk.LabelFrame(self.control_frame, text="System Statistics", padding=10)
        stats_frame.pack(fill=tk.X, pady=5)
        
        self.stats_text = tk.Text(stats_frame, height=5, width=30, state=tk.DISABLED)
        self.stats_text.pack(fill=tk.BOTH)
        
        # Help button
        ttk.Button(self.control_frame, text="Help", command=self._show_help).pack(fill=tk.X, pady=10)
    
    def _draw_nav_graph(self):
        """Draw the navigation graph on the canvas"""
        self.ax.clear()
        
        # Draw lanes
        for lane in self.nav_graph.lanes:
            v1 = self.nav_graph.vertices[lane.start]
            v2 = self.nav_graph.vertices[lane.end]
            
            # Draw lane
            self.ax.plot([v1.x, v2.x], [v1.y, v2.y], 'k-', alpha=0.3, linewidth=2)
        
        # Draw vertices
        for vertex in self.nav_graph.vertices:
            color = 'green' if vertex.is_charger else 'black'
            marker = 's' if vertex.is_charger else 'o'
            self.ax.plot(vertex.x, vertex.y, marker=marker, color=color, markersize=10)
            
            # Add vertex label
            if vertex.name:
                self.ax.text(vertex.x + 0.2, vertex.y + 0.2, vertex.name, 
                            fontsize=9, bbox=dict(facecolor='white', alpha=0.7))
        
        # Draw robots
        for robot_id in self.fleet_manager.robots:
            robot_info = self.fleet_manager.get_robot_info(robot_id)
            x, y = robot_info['x'], robot_info['y']
            
            '''
            # Choose marker based on status
            marker = {
                'MOVING': 'D',        # Diamond
                'IDLE': 'o',          # Circle
                'WAITING': 's',       # Square
                'CHARGING': 'p',      # Pentagon
                'TASK_COMPLETE': '*', # Star
                'ERROR': 'X',         # X
                'MOVING_TO_CHARGER': 'D' # Diamond
            }.get(robot_info['status'], 'o')
            '''
            # Special handling for MOVING_TO_CHARGER status
            if robot_info['status'] == 'MOVING_TO_CHARGER':
                # Draw robot as triangle with flashing red color
                flash_factor = 0.5 + 0.5 * math.sin(time.time() * 5)  # Creates pulsing effect
                color = (1.0, flash_factor, flash_factor)  # Red with pulsing brightness
                self.ax.plot(x, y, marker='^', color=color, 
                            markersize=14, markeredgecolor='red', markeredgewidth=2)
            else:
                # Normal robot drawing
                self.ax.plot(x, y, marker='^', color=robot_info['color'], 
                            markersize=12, markeredgecolor='black')
            
            # Draw robot ID and status
            status_text = f"{robot_id}: {robot_info['status']}"
            if robot_info['status'] == 'WAITING':
                status_text += f"\nBlocked at {robot_info['current_vertex']}"
            self.ax.text(x + 0.2, y + 0.2, status_text, 
                        fontsize=8, bbox=dict(facecolor='white', alpha=0.7))
            
            # Draw path if robot is moving
            if robot_info['status'] in ['MOVING', 'MOVING_TO_CHARGER'] and robot_info['path']:
                path_x = [self.nav_graph.vertices[v].x for v in robot_info['path']]
                path_y = [self.nav_graph.vertices[v].y for v in robot_info['path']]
                path_color = 'red' if robot_info['status'] == 'MOVING_TO_CHARGER' else robot_info['color']
                self.ax.plot(path_x, path_y, '--', color=path_color, alpha=0.7, linewidth=2)
        
        # Highlight selected robot and vertex
        if self.selected_robot:
            robot_info = self.fleet_manager.get_robot_info(self.selected_robot)
            x, y = robot_info['x'], robot_info['y']
            self.ax.plot(x, y, 'o', color=robot_info['color'], markersize=16, 
                        markeredgecolor='gold', markeredgewidth=2)
        
        if self.selected_vertex is not None:
            v = self.nav_graph.vertices[self.selected_vertex]
            self.ax.plot(v.x, v.y, 'o', color='gold', markersize=14, alpha=0.7)
        
        self.ax.set_title(f"Goat Robotics Fleet Management - {self.nav_graph.name}")
        self.ax.set_xlabel("X Coordinate")
        self.ax.set_ylabel("Y Coordinate")
        self.ax.grid(True, alpha=0.3)
        self.ax.axis('equal')
    
    def _update(self):
        """Update the visualization and robot states"""
        current_time = time.time()
        delta_time = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Update robot positions
        self.fleet_manager.update_robot_positions(delta_time)
        
        # Redraw the visualization
        self._draw_nav_graph()
        self.canvas.draw()
        
        # Update robot info display
        self._update_info_display()
        
        # Update statistics
        self._update_statistics()
        
        # Schedule next update
        self.master.after(50, self._update)  # UPDATE_INTERVAL
    
    def _update_info_display(self):
        """Update the robot information display"""
        self.info_text.config(state=tk.NORMAL)
        self.info_text.delete(1.0, tk.END)
        
        if self.selected_robot:
            robot_info = self.fleet_manager.get_robot_info(self.selected_robot)
            
            self.info_text.insert(tk.END, f"Robot ID: {robot_info['id']}\n")
            self.info_text.insert(tk.END, f"Status: {robot_info['status']}\n")
            self.info_text.insert(tk.END, f"Battery: {robot_info['battery']}\n")
            self.info_text.insert(tk.END, f"Current Vertex: {robot_info['current_vertex']}\n")
            
            if robot_info['destination'] is not None:
                self.info_text.insert(tk.END, f"Destination: {robot_info['destination']}\n")
                self.info_text.insert(tk.END, f"Distance Remaining: {robot_info['distance_remaining']:.2f} units\n")
                self.info_text.insert(tk.END, f"Time Elapsed: {robot_info['time_elapsed']:.1f} sec\n")
            
            if robot_info['path']:
                self.info_text.insert(tk.END, "\nPath:\n")
                for i, vertex in enumerate(robot_info['path']):
                    prefix = "→ " if i > 0 else ""
                    v = self.nav_graph.vertices[vertex]
                    name = v.name if v.name else str(vertex)
                    self.info_text.insert(tk.END, f"{prefix}{name}\n")
        else:
            self.info_text.insert(tk.END, "No robot selected\n\n")
            self.info_text.insert(tk.END, "Instructions:\n")
            self.info_text.insert(tk.END, "1. Click on a robot to select it\n")
            self.info_text.insert(tk.END, "2. Click on a vertex to assign destination\n")
            self.info_text.insert(tk.END, "3. Click on empty vertex to spawn new robot")
        
        self.info_text.config(state=tk.DISABLED)
    
    def _update_statistics(self):
        """Update system statistics display"""
        self.stats_text.config(state=tk.NORMAL)
        self.stats_text.delete(1.0, tk.END)
        
        num_robots = len(self.fleet_manager.robots)
        moving = sum(1 for r in self.fleet_manager.robots.values() if r.status == RobotStatus.MOVING)
        idle = sum(1 for r in self.fleet_manager.robots.values() if r.status == RobotStatus.IDLE)
        charging = sum(1 for r in self.fleet_manager.robots.values() if r.status == RobotStatus.CHARGING)
        complete = sum(1 for r in self.fleet_manager.robots.values() if r.status == RobotStatus.TASK_COMPLETE)
        
        self.stats_text.insert(tk.END, f"Total Robots: {num_robots}/{num_robots}\n")
        self.stats_text.insert(tk.END, f"Moving: {moving}\n")
        self.stats_text.insert(tk.END, f"Idle: {idle}\n")
        self.stats_text.insert(tk.END, f"Charging: {charging}\n")
        self.stats_text.insert(tk.END, f"Tasks Complete: {complete}")
        
        self.stats_text.config(state=tk.DISABLED)
    
    def on_canvas_click(self, event):
        """Handle clicks on the canvas"""
        if not event.inaxes:
            return
        
        # Find closest vertex
        min_dist = float('inf')
        closest_vertex = None
        
        for i, vertex in enumerate(self.nav_graph.vertices):
            dist = math.sqrt((event.xdata - vertex.x)**2 + (event.ydata - vertex.y)**2)
            if dist < min_dist and dist < 0.5:  # Max click distance threshold
                min_dist = dist
                closest_vertex = i
        
        # Find closest robot
        min_robot_dist = float('inf')
        closest_robot = None
        
        for robot_id in self.fleet_manager.robots:
            robot_info = self.fleet_manager.get_robot_info(robot_id)
            dist = math.sqrt((event.xdata - robot_info['x'])**2 + (event.ydata - robot_info['y'])**2)
            if dist < min_robot_dist and dist < 0.5:  # Max click distance threshold
                min_robot_dist = dist
                closest_robot = robot_id
        
        # Prioritize robot selection over vertex selection
        if closest_robot is not None:
            self.selected_robot = closest_robot
            self.selected_vertex = None
        elif closest_vertex is not None:
            if self.selected_robot:
                # Assign task to selected robot
                if self.fleet_manager.assign_task(self.selected_robot, closest_vertex):
                    messagebox.showinfo("Task Assigned", 
                                      f"Robot {self.selected_robot} assigned to vertex {closest_vertex}")
                else:
                    messagebox.showerror("Assignment Failed", 
                                       f"Could not assign Robot {self.selected_robot} to vertex {closest_vertex}")
                self.selected_robot = None
            else:
                # Spawn new robot or select vertex
                if self.nav_graph.vertices[closest_vertex].occupied_by is None:
                    robot = self.fleet_manager.spawn_robot(closest_vertex)
                    if robot:
                        messagebox.showinfo("Robot Spawned", 
                                          f"Robot {robot.id} spawned at vertex {closest_vertex}")
                    else:
                        messagebox.showerror("Spawn Failed", 
                                           f"Could not spawn robot at vertex {closest_vertex}")
                else:
                    self.selected_vertex = closest_vertex
                    messagebox.showinfo("Vertex Occupied", 
                                      f"Vertex {closest_vertex} is occupied by Robot {self.nav_graph.vertices[closest_vertex].occupied_by}")
    
    def _spawn_random_robot(self):
        """Spawn a robot at a random unoccupied vertex"""
        available_vertices = [
            i for i, v in enumerate(self.nav_graph.vertices) 
            if v.occupied_by is None
        ]
        
        if not available_vertices:
            messagebox.showerror("No Space", "All vertices are occupied!")
            return
        
        vertex = random.choice(available_vertices)
        robot = self.fleet_manager.spawn_robot(vertex)
        
        if robot:
            messagebox.showinfo("Robot Spawned", f"Robot {robot.id} spawned at vertex {vertex}")
            self.selected_robot = robot.id
        else:
            messagebox.showerror("Spawn Failed", "Could not spawn robot")
    
    def _assign_random_task(self):
        """Assign a random task to the selected robot"""
        if not self.selected_robot:
            messagebox.showerror("No Selection", "Please select a robot first")
            return
        
        available_vertices = [
            i for i, v in enumerate(self.nav_graph.vertices) 
            if i != self.fleet_manager.robots[self.selected_robot].current_vertex
        ]
        
        if not available_vertices:
            messagebox.showerror("No Destination", "No available destination vertices")
            return
        
        destination = random.choice(available_vertices)
        
        if self.fleet_manager.assign_task(self.selected_robot, destination):
            messagebox.showinfo("Task Assigned", 
                              f"Robot {self.selected_robot} assigned to vertex {destination}")
        else:
            messagebox.showerror("Assignment Failed", 
                               f"Could not assign Robot {self.selected_robot} to vertex {destination}")
    
    def _pause_all(self):
        """Pause all moving robots"""
        for robot in self.fleet_manager.robots.values():
            if robot.status == RobotStatus.MOVING:
                robot.status = RobotStatus.WAITING
        messagebox.showinfo("System Paused", "All robots have been paused")
    
    def _resume_all(self):
        """Resume all paused robots"""
        for robot in self.fleet_manager.robots.values():
            if robot.status == RobotStatus.WAITING:
                robot.status = RobotStatus.MOVING
        messagebox.showinfo("System Resumed", "All robots have been resumed")
    
    def _show_help(self):
        """Show help information"""
        help_text = """
        Goat Robotics Fleet Management System
        
        Instructions:
        1. Click on an empty vertex to spawn a new robot
        2. Click on a robot to select it
        3. Click on a destination vertex to assign a task
        4. Use the control panel for additional functions
        
        Robot Status Colors:
        - Moving: Solid color
        - Idle: Small white center
        - Charging: Pulsing animation
        - Task Complete: Green ring
        - Error: Red X
        
        Vertex Types:
        - Blue circle: Regular vertex
        - Green square: Charging station
        """
        messagebox.showinfo("Help", help_text)