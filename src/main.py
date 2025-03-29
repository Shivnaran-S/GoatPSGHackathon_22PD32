import tkinter as tk
from tkinter import messagebox

from models.nav_graph import NavigationGraph

from controllers.traffic_manager import TrafficManager
from controllers.fleet_manager import FleetManager

from gui.fleet_gui import FleetGUI

from utils.helpers import load_nav_graph

def main():
    try:
        nav_graph = load_nav_graph("../data/nav_graph_1.json")
    except FileNotFoundError:
        messagebox.showerror("Error", "Could not load navigation graph file")
        return
    
    traffic_manager = TrafficManager(nav_graph)
    fleet_manager = FleetManager(nav_graph, traffic_manager)
    
    root = tk.Tk()
    app = FleetGUI(root, nav_graph, fleet_manager)
    app.canvas.mpl_connect('button_press_event', app.on_canvas_click)
    root.mainloop()

if __name__ == "__main__":
    main()