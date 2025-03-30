# 🚀 Fleet Management System with Traffic Negotiation for Multi-Robots

## 🌟 Key Innovations

### 🖥️ Immersive Visual Interface
- **Interactive environment** with clickable vertices and real-time robot tracking
- **Distinct visual identifiers** for each robot (unique colors + status indicators)
- **Dynamic path visualization** showing optimal routes 

### 🤖 Intelligent Robot Management
- **One-click spawning** at any vertex with automatic ID assignment
- **Drag-and-drop task assignment** (click robot → click destination)
- **Battery-aware routing** with automatic charger redirection
- **Direct-to-charger navigation** when battery critical

### 🚦 Traffic Control
- **Real-time lane reservations** preventing collisions
- **Alternative path finding** when primary route blocked
- **Visual waiting indicators** with blockage details

### ⚡ Performance Optimizations
- **Efficient A* pathfinding** with Euclidean heuristics
- **Thread-safe operations** using locking mechanisms
- **Continuous logging** of all system events (`fleet_logs.txt`)

## 🏆 Evaluation Criteria Met

| Criteria                | Implementation Status |
|-------------------------|-----------------------|
| Functionality           | ✅ Visual and Functional requirements met|
| Code Quality            | ✅ Modular, Readable, Maintainable, and Commented |
| Visual Interface        | ✅ User-friendly interface |
| Traffic Negotiation     | ✅ Collision avoidance |
| Dynamic Interaction     | ✅ Real-time control |
| Robustness              | ✅ Visual feedback, error handling |
| Creative Enhancements   | ✅ Battery optimization, direct charging |

## 🛠️ Technical Stack
- Python 3.8+
- Tkinter (GUI)
- Matplotlib (Visualization)
- JSON (Graph configuration)

## 🚀 Getting Started

1. **Clone the repository**
   ```bash
   git clone https://github.com/Shivnaran-S/GoatPSGHackathon_22PD32.git
   ```
2. **Install dependencies**
   ```bash
   pip install -r requirements.txt
   ```
3. **Run visual GUI system**
   ```bash
   python src/main.py
   ```
4. **Interact with the system**:
   - Click empty vertex → Spawn robot
   - Click robot → Select it
   - Click destination → Assign task
   - Buttons: Spawn random robot, Assign random task  
5. If you want to use any other Nav Graph (any other .json file), add the .json file to data directory and in src/main.py file in main function give the name of the file in load_nav_graph(file_name) function of try block
