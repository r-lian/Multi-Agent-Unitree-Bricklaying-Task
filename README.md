# Multi-Robot Bricklaying System

## Overview

This system provides a **comprehensive high-level scheduler** for coordinating **N robots** (scalable from 1 to 50) to perform bricklaying tasks. The system features **persistent operation**, **real-time terminal controls**, **dynamic scaling**, **advanced path planning**, and **collision avoidance** with comprehensive **RViz2 visualization** for research, testing, and monitoring.

## Key Features

### 🎮 Real-Time Terminal Control System
- **Persistent operation** - System runs indefinitely until shutdown command
- **Start/Pause/Shutdown** - Full runtime control over robot operation
- **Dynamic entity management** - Add/remove robots, bricks, and targets during operation
- **Live configuration** - Change system parameters without restart
- **Interactive help system** - Built-in command reference and status monitoring

### 🎲 N-Robot & N-Brick Scaling
- **Configurable quantities** - Support for 1-50 robots and 1-100 bricks
- **Dynamic spawning** - Add entities at runtime with `spawn_robots N` and `spawn_bricks N`
- **Automatic color generation** - HSV color space ensures unique colors for any number of robots
- **Performance optimization** - Safety limits and efficient algorithms for large-scale coordination
- **Real-time scaling** - Add or remove entities while system is running

### 🎯 Advanced Target Management
- **Multiple input modes** - Manual coordinates or batch spawning
- **Ordered vs closest filling** - Choose between sequential (FIFO) or distance-based target selection
- **Live mode switching** - Toggle target filling strategy during operation
- **Dynamic target addition** - Add targets while robots are working

### 🤖 Multi-Robot Coordination
- **N robots** working simultaneously with unique colors
- Independent FSM states and path planning per robot
- Advanced collision avoidance and path reservation
- Task allocation optimization for efficiency

### 🗺️ Advanced Path Planning
- **A* pathfinding algorithm** with collision avoidance
- **Spatial-temporal planning** - considers both space and time conflicts
- **Path reservation system** - prevents robot conflicts
- **Dynamic replanning** - adaptive pathfinding under changing conditions
- **Fallback mechanisms** - robust operation when paths are blocked

### 📊 Comprehensive Visualization
- **Dynamic color system** - Unique colors for unlimited robots using HSV color space
- **Real-time state display** - Floating text above each robot showing status
- **Path visualization** - Color-coded lines for each robot's planned route
- **Planning grid** and **reserved areas** - Debug pathfinding algorithms
- **Trajectory arrows** - Movement direction indicators
- **Entity status** - Visual distinction between different brick and target states

### 🔄 Robust State Management
- **Persistent FSM** - Robot states survive pause/resume cycles
- **Task completion handling** - Automatic detection with continuous operation
- **Error recovery** - Graceful handling of edge cases
- **Thread-safe operations** - Safe concurrent access to all data structures

## Getting Started

### 1. Start the Persistent System
```bash
python3 robot_brick_visualizer.py
```

The system starts in **persistent mode** with default entities (3 robots, 5 bricks, 0 targets).

### 2. Terminal Control Interface
```
=== Multi-Robot Bricklaying System (Persistent Mode) ===
🤖 3 robots initialized
🧱 5 bricks to place
🎯 0 target locations

🎮 TERMINAL CONTROL COMMANDS:
   📍 add_target x,y,z     - Add target location
   🧱 add_brick x,y,z      - Add unplaced brick
   🤖 add_robot x,y,z      - Add new robot
   ❌ remove_target ID     - Remove target by ID
   ❌ remove_brick ID      - Remove brick by ID
   ❌ remove_robot ID      - Remove robot by ID
   🎲 spawn_robots N       - Spawn N robots randomly
   🎲 spawn_bricks N       - Spawn N bricks randomly
   🗑️  clear_all           - Remove all entities
   ⚙️  set_defaults N,M    - Set default robots,bricks
   ▶️  start               - Start/resume robot operation
   ⏸️  pause               - Pause robot operation
   🛑 shutdown             - Exit system
   🔄 order                - Toggle target filling mode
   ❓ help                 - Show this help
   📊 status               - Show system status

🎮 Command: 
```

## Command Reference

### Entity Management Commands

#### **Adding Entities**
```bash
🎮 Command: add_target 2.0,2.0,0.0      # Add specific target location
🎮 Command: add_brick 1.5,1.5,0.0       # Add specific brick location  
🎮 Command: add_robot 0.5,0.5,0.0       # Add specific robot location
🎮 Command: spawn_robots 10             # Add 10 robots at random locations
🎮 Command: spawn_bricks 20             # Add 20 bricks at random locations
```

#### **Removing Entities**
```bash
🎮 Command: remove_target 1             # Remove target by ID
🎮 Command: remove_brick 5              # Remove brick by ID
🎮 Command: remove_robot 3              # Remove robot by ID
🎮 Command: clear_all                   # Remove ALL entities
```

### System Control Commands

#### **Operation Control**
```bash
🎮 Command: start                       # Start/resume robot operation
🎮 Command: pause                       # Pause all robot activity
🎮 Command: shutdown                    # Exit system gracefully
```

#### **Configuration**
```bash
🎮 Command: set_defaults 8,15           # Set 8 robots, 15 bricks as defaults
🎮 Command: order                       # Toggle ORDERED/CLOSEST target mode
```

#### **Information**
```bash
🎮 Command: status                      # Show detailed system status
🎮 Command: help                        # Show all available commands
```

## Usage Scenarios

### Small Scale Testing (3-5 robots)
```bash
🎮 Command: clear_all                   # Start fresh
🎮 Command: spawn_robots 3              # Add 3 robots
🎮 Command: spawn_bricks 5              # Add 5 bricks
🎮 Command: add_target 2.0,2.0,0.0      # Add target
🎮 Command: start                       # Begin operation
🎮 Command: status                      # Monitor progress
```

### Medium Scale Coordination (8-15 robots)
```bash
🎮 Command: set_defaults 10,20          # Set higher defaults
🎮 Command: clear_all                   # Clear existing
🎮 Command: spawn_robots 10             # 10-robot swarm
🎮 Command: spawn_bricks 20             # Plenty of work
🎮 Command: add_target 1.0,1.0,0.0      # Multiple targets
🎮 Command: add_target 3.0,3.0,0.0
🎮 Command: add_target 2.0,4.0,0.0
🎮 Command: start                       # Watch coordination!
```

### Large Scale Stress Testing (20-50 robots)
```bash
🎮 Command: clear_all
🎮 Command: spawn_robots 30             # Large robot swarm  
🎮 Command: spawn_bricks 60             # High brick count
🎮 Command: add_target 2.5,2.5,0.0      # Central stacking point
🎮 Command: start                       # Observe emergent behavior
🎮 Command: status                      # Monitor coordination
```

### Dynamic Scaling During Operation
```bash
# While robots are working:
🎮 Command: spawn_robots 5              # Add more workers
🎮 Command: spawn_bricks 10             # Add more work
🎮 Command: add_target 4.0,1.0,0.0      # Add more targets
🎮 Command: pause                       # Pause to observe
🎮 Command: remove_robot 15             # Remove specific robot
🎮 Command: start                       # Resume operation
```

### Target Filling Modes

#### **ORDERED Mode (Default)**
- Robots fill targets in **exact order** they were added
- Target 1 → Target 2 → Target 3 (always)
- Ideal for **sequential construction** where build order matters

#### **CLOSEST Mode**
- Robots choose **nearest available** target
- Optimizes for **efficiency** and **faster completion**
- Better for **throughput-focused** operations

**Toggle between modes:**
```bash
🎮 Command: order                       # Switch modes
🔄 Target filling mode changed to: CLOSEST (distance-based)
```

## RViz2 Visualization

### Quick Setup
1. **Start RViz2**: `rviz2`
2. **Set Fixed Frame**: `map` 
3. **Add MarkerArray Display**:
   - Topic: `/debug_markers`
   - Enable all namespaces for full visualization
4. **Add Path Displays** (for each active robot):
   - `/robot_1_path`, `/robot_2_path`, etc.
   - Set colors to match robot colors

### Visualization Elements

#### **Entity Visualization**
- **🌈 Colored Cylinders**: Robots with unique HSV-generated colors
- **🟥 Red Cubes**: Unplaced bricks (available for pickup)
- **🟧 Orange Cubes**: Reserved bricks (robot targeting them)
- **🟩 Green Cubes**: Target locations (pending placement)
- **🟪 Purple Cubes**: Placed bricks (auto-stacked vertically)

#### **Advanced Visualization**
- **📍 Robot State Text**: Real-time status above each robot
  - Current FSM state (IDLE, MOVING_TO_BRICK, etc.)
  - Target brick/location IDs
  - Path progress (waypoint X/Y)
  
- **🛤️ Colored Path Lines**: Planned routes for each robot
  - Color-matched to robot for easy tracking
  - Real-time updates as paths change
  
- **📐 Planning Grid**: Gray overlay showing pathfinding resolution
  - 0.2m grid spacing visualization
  - Shows discrete movement space
  
- **🚫 Reserved Areas**: Collision avoidance zones
  - Colored squares showing reserved path cells
  - Color-matched to reserving robot
  
- **➡️ Trajectory Arrows**: Movement direction indicators
  - Point toward next waypoint
  - Only visible when robots are moving

### Topics Published
- **`/debug_markers`** - All visual elements (robots, bricks, targets, text, grid, etc.)
- **`/robot_N_path`** - Individual planned path for robot N (dynamically created)

### Namespace Organization
- `robots` - Robot cylinders
- `unplaced_bricks` - Red/orange brick cubes  
- `placed_bricks` - Purple placed brick cubes
- `targets` - Green target cubes
- `robot_states` - State text displays
- `robot_N_path` - Path lines for robot N
- `planning_grid` - Grid overlay
- `reserved_areas` - Collision avoidance cells
- `trajectory_arrows` - Movement arrows

## Algorithm Details

### Target Selection Strategies

#### **ORDERED Mode**
```python
# Always select first available target in list
selected_target = available_targets[0]
```
- **Sequential processing**: Target 1 → Target 2 → Target 3
- **Predictable behavior**: Build order guaranteed
- **Use case**: Sequential construction, assembly lines

#### **CLOSEST Mode**  
```python
# Select target with minimum distance to robot
distances = [distance(robot.location, target.location) for target in available_targets]
selected_target = available_targets[distances.index(min(distances))]
```
- **Efficiency optimization**: Minimize travel time
- **Dynamic selection**: Based on robot positions
- **Use case**: Throughput maximization, parallel construction

### Dynamic Color Generation
```python
def get_robot_color(self, robot_id):
    import colorsys
    hue = ((robot_id - 1) * 0.618034) % 1.0  # Golden ratio spacing
    saturation = 0.8
    value = 1.0
    return colorsys.hsv_to_rgb(hue, saturation, value)
```
- **HSV color space**: Ensures good visual separation
- **Golden ratio spacing**: Optimal color distribution
- **Unlimited robots**: Unique color for any robot count
- **Consistent mapping**: Same robot ID → same color

### Path Planning & Collision Avoidance
1. **Grid-based A***: 0.2m resolution over 5x5m world
2. **Robot safety radius**: 15cm collision detection
3. **Temporal reservations**: 10-second path reservations
4. **Spatial-temporal conflicts**: Prevents robot collisions
5. **Fallback planning**: Retry without avoidance if blocked
6. **Dynamic replanning**: 1-second retry delays

### Multi-Robot Coordination
1. **Task allocation**: Closest available brick assignment
2. **Load balancing**: Dynamic work distribution
3. **Collision avoidance**: Path reservation system
4. **State synchronization**: Thread-safe FSM updates
5. **Resource management**: Brick reservation prevents conflicts

## System Architecture

### Core Components
1. **`robot_brick_visualizer.py`** - Main ROS2 node with persistent terminal control
2. **`path_planner.py`** - A* pathfinding with spatial-temporal reservations
3. **`models.py`** - Data structures (Robot, Brick, Target, Location, States)

### Key Classes
- **`RobotBrickVisualizer`** - Main coordinator with FSM, visualization, and terminal control
- **`PathPlanner`** - A* pathfinding with collision avoidance and reservations
- **`Robot`** - Enhanced model with FSM state and path planning attributes
- **`Target`** - Location data with PENDING/PLACED state tracking

### Data Management
```python
# Scalable entity storage
self.robots = []              # List of Robot objects (1-50)
self.unplaced_bricks = []     # List of Brick objects (1-100)
self.placed_bricks = []       # Completed bricks
self.targets = []             # List of Target objects

# Configuration
self.default_num_robots = 3   # Configurable defaults
self.default_num_bricks = 5
self.ordered_target_filling = True  # Target selection mode

# System state
self.system_paused = False    # Runtime pause control
self.robots_can_start = False # Activation control
self.shutdown_requested = False # Graceful shutdown
```

### Threading Architecture
- **Main thread**: ROS2 node, FSM updates, visualization
- **Terminal control thread**: Non-blocking command input handling  
- **Completion handling**: Separate thread for restart prompts (legacy mode)

## Performance & Scaling

### Entity Limits
- **Robots**: 1-50 (safety limit prevents visualization overload)
- **Bricks**: 1-100 (memory efficiency limit)
- **Targets**: Unlimited (dynamically managed)

### Performance Characteristics
- **1-5 robots**: Optimal visualization and coordination
- **6-15 robots**: Good performance, rich color variety
- **16-30 robots**: High throughput, coordinated swarms  
- **31-50 robots**: Maximum capacity, stress testing

### Memory & CPU Optimization
- **Efficient marker updates**: Only publish changes
- **Thread-safe operations**: Minimal locking overhead
- **Dynamic path publishers**: Created/destroyed as needed
- **Spatial indexing**: Grid-based collision detection

## Configuration Options

### System Parameters
```python
# Pathfinding
grid_size = 0.2              # meters
robot_radius = 0.15          # meters
path_reservation_time = 10.0 # seconds

# FSM Timing  
state_duration = 0.5         # minimum state time
move_interval = 0.3          # waypoint following rate
retry_delay = 1.0            # WAITING state duration

# Defaults (configurable via set_defaults)
default_num_robots = 3       # Initial robot count
default_num_bricks = 5       # Initial brick count
```

### Runtime Configuration
- **Entity quantities**: Use `spawn_robots N` and `spawn_bricks N`
- **Target mode**: Toggle with `order` command
- **System state**: Control with `start`, `pause`, `shutdown`
- **Defaults**: Set with `set_defaults N,M`

## Testing & Validation

### Unit Testing
```bash
# Test basic functionality
🎮 Command: clear_all
🎮 Command: spawn_robots 1
🎮 Command: spawn_bricks 1  
🎮 Command: add_target 2.0,2.0,0.0
🎮 Command: start
# Verify: Single robot picks up brick and places at target
```

### Coordination Testing  
```bash
# Test multi-robot coordination
🎮 Command: clear_all
🎮 Command: spawn_robots 5
🎮 Command: spawn_bricks 10
🎮 Command: add_target 2.0,2.0,0.0
🎮 Command: start
# Verify: No collisions, efficient task distribution
```

### Scaling Testing
```bash
# Test large-scale performance
🎮 Command: clear_all
🎮 Command: spawn_robots 30
🎮 Command: spawn_bricks 60
🎮 Command: add_target 2.5,2.5,0.0
🎮 Command: start
# Verify: System remains responsive, coordination works
```

### Stress Testing
```bash
# Test dynamic scaling under load
🎮 Command: spawn_robots 20
🎮 Command: spawn_bricks 40
# While running:
🎮 Command: spawn_robots 10  # Add more workers
🎮 Command: spawn_bricks 20  # Add more work
# Verify: Smooth integration of new entities
```

## Research Applications

### Multi-Robot Coordination Studies
- **Swarm behavior analysis** with 10+ robots
- **Task allocation algorithms** testing
- **Collision avoidance** evaluation under high density
- **Emergent behavior** observation in large groups

### Pathfinding Algorithm Development
- **A* algorithm** performance under different robot densities
- **Spatial-temporal planning** effectiveness
- **Dynamic replanning** response times
- **Reservation system** efficiency analysis

### System Performance Analysis
- **Throughput measurements** with varying robot/brick ratios
- **Coordination overhead** analysis
- **Scaling behavior** characterization
- **Resource utilization** profiling

## Future Extensions

### Enhanced Algorithms
- **Machine learning** for task allocation optimization
- **Predictive pathfinding** using robot behavior models
- **Adaptive coordination** based on performance metrics
- **Multi-objective optimization** (time, energy, coordination)

### Advanced Visualization
- **3D visualization** with height-based operations
- **Performance dashboards** with real-time metrics
- **Heatmaps** for coordination efficiency analysis
- **Animation recording** for presentation and analysis

### Integration Capabilities
- **Physical robot communication** protocols
- **External sensor integration** (cameras, lidar)
- **Cloud-based coordination** for distributed systems
- **API interfaces** for external control systems

### Simulation Features
- **Physics simulation** integration (Gazebo, MuJoCo)
- **Realistic robot dynamics** modeling  
- **Environmental obstacles** and complex scenarios
- **Failure simulation** and recovery testing

## Troubleshooting

### Common Issues

#### **No robots moving after 'start'**
```bash
🎮 Command: status                      # Check robot activation status
# If "Robots active: NO":
🎮 Command: start                       # Ensure robots are started
# If no targets:
🎮 Command: add_target 2.0,2.0,0.0      # Add at least one target
```

#### **RViz2 not showing markers**
1. Check **Fixed Frame** is set to `map`
2. Add **MarkerArray** display with topic `/debug_markers`
3. Ensure **all namespaces** are enabled in MarkerArray display
4. Run: `ros2 topic echo /debug_markers` to verify messages

#### **Performance issues with many robots**
```bash
🎮 Command: status                      # Check entity counts
# If too many entities:
🎮 Command: remove_robot 25             # Remove some robots
# Or restart with lower defaults:
🎮 Command: set_defaults 10,20
🎮 Command: clear_all
🎮 Command: spawn_robots 10
```

#### **Path planning failures**
- **Symptom**: Robots stuck in WAITING state
- **Solution**: Reduce robot density or add more space
```bash
🎮 Command: remove_robot 5              # Reduce density
🎮 Command: spawn_bricks 5              # Add more distributed work
```

### Performance Optimization
- **Reduce entity counts** for smoother operation
- **Disable planning grid** visualization for better performance  
- **Use CLOSEST mode** for more efficient pathfinding
- **Monitor system resources** with `status` command

## Conclusion

This Multi-Robot Bricklaying System provides a **comprehensive platform** for multi-robot coordination research, algorithm development, and system testing. With **persistent operation**, **real-time controls**, **N-robot scaling**, and **advanced visualization**, it supports everything from **small-scale algorithm testing** to **large-scale swarm behavior analysis**.

The system's **modular architecture**, **robust error handling**, and **extensive configuration options** make it suitable for both **research applications** and **educational demonstrations**. The **terminal control interface** enables **interactive exploration** of multi-robot coordination phenomena while maintaining **scientific rigor** through **reproducible configurations** and **comprehensive monitoring**.

**Start exploring multi-robot coordination today:**
```bash
python3 robot_brick_visualizer.py
🎮 Command: help
``` 