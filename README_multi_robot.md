# Multi-Robot Bricklaying System

## Overview

This system extends the single-robot bricklaying simulator to support **3 coordinated robots** with **path planning** and **collision avoidance**. Each robot independently plans paths while avoiding conflicts with other robots' reserved paths.

## Key Features

### 🤖 Multi-Robot Coordination
- **3 robots** working simultaneously (Blue, Green, Red)
- Each robot has independent FSM states and path planning
- Robots coordinate to avoid collisions

### 🗺️ Advanced Path Planning
- **A* pathfinding algorithm** with collision avoidance
- **Path reservation system** - robots reserve paths to prevent conflicts
- **Dynamic obstacle avoidance** based on other robots' movements
- **Spatial-temporal planning** - considers both space and time conflicts

### 📊 Enhanced Visualization
- **Different colored robots**: Blue (Robot 1), Green (Robot 2), Red (Robot 3)
- **Path visualization** in RViz2 (published to `/robot_paths` topic)
- **Real-time coordination** display

### 🔄 Robot State Machine
Extended FSM with new states:
- `IDLE` - Waiting for tasks
- `MOVING_TO_BRICK` - Following path to target brick
- `MOVING_TO_TARGET` - Following path to placement location
- `GRABBING` - Picking up brick
- `PLACING` - Placing brick with stacking
- `WAITING` - Waiting for clear path

## Algorithm Details

### Path Planning (A* with Collision Avoidance)
1. **Grid-based planning**: 0.2m grid resolution over 5x5m world
2. **Collision detection**: 30cm safety radius around each robot
3. **Path reservation**: Robots reserve paths for 10 seconds
4. **Temporal conflicts**: Checks both spatial and time overlaps

### Multi-Robot Coordination
1. **Task allocation**: Closest available brick assignment
2. **Path conflicts**: If path blocked, robot enters WAITING state
3. **Dynamic replanning**: Robots retry path planning after delays
4. **Brick stacking**: Multiple robots can stack at same target location

## Testing the System

### 1. Start the System
```bash
python3 robot_brick_visualizer.py
```

### 2. Add Target Locations
When prompted, enter target coordinates to see robots coordinate:
```
2.0,2.0,0.5
4.0,1.0,0.5
1.0,4.0,0.5
3.5,3.5,0.5
0.5,1.5,0.5
```

### 3. Observe Coordination
- **Multiple robots** will start moving simultaneously
- **Different colored paths** show each robot's planned route
- **Collision avoidance** - robots wait if paths conflict
- **Efficient task allocation** - closest robot gets each brick

### 4. Test Stacking (Same Location)
Add multiple targets at the same location to see stacking:
```
2.0,2.0,0.5
2.0,2.0,0.5
2.0,2.0,0.5
```

### 5. Configure RViz2 for Full Visualization
To see all features, add these display types in RViz2:

1. **MarkerArray** display:
   - Topic: `/debug_markers`
   - Shows robots, bricks, targets, paths, grid, text, arrows

2. **Path** displays (add 3 separate ones):
   - Topic: `/robot_1_path` (set color to Blue)
   - Topic: `/robot_2_path` (set color to Green)
   - Topic: `/robot_3_path` (set color to Red)

3. **Optional**: Filter namespaces in MarkerArray display:
   - Enable/disable specific namespaces to focus on different aspects
   - Toggle `planning_grid` for cleaner view
   - Toggle `reserved_areas` to see collision avoidance in action

## RViz2 Visualization

### Topics Published
- `/debug_markers` - Robot, brick, target markers, paths, grid, and state text
- `/robot_1_path` - Individual path for Robot 1 (Blue)
- `/robot_2_path` - Individual path for Robot 2 (Green)  
- `/robot_3_path` - Individual path for Robot 3 (Red)

### Enhanced Visualization Elements

#### **Basic Objects**
- **Blue/Green/Red Cylinders**: The 3 robots
- **Red Cubes**: Unplaced bricks (ungrabbed)
- **Orange Cubes**: Reserved bricks (robot targeting them)
- **Green Cubes**: Target locations
- **Purple Cubes**: Placed bricks (stacked vertically)

#### **Advanced Visualizations**
- **📍 Robot State Text**: Floating text above each robot showing:
  - Current state (idle, moving_to_brick, etc.)
  - Target brick ID
  - Target location ID
  - Path progress (current/total waypoints)

- **🛤️ Path Lines**: Colored LINE_STRIP markers showing planned routes
  - Blue lines for Robot 1
  - Green lines for Robot 2
  - Red lines for Robot 3
  - Semi-transparent, update in real-time

- **📐 Planning Grid**: Gray grid lines showing pathfinding resolution
  - 0.2m grid spacing
  - Very transparent overlay
  - Shows discrete planning space

- **🚫 Reserved Areas**: Colored squares showing collision avoidance zones
  - Color-matched to robot (Blue/Green/Red)
  - Semi-transparent cells
  - Real-time updates as paths are reserved/released

- **➡️ Trajectory Arrows**: Direction indicators for moving robots
  - Point toward next waypoint
  - Color-matched to robot
  - Only visible when robots are moving

### RViz2 Display Setup
Each visualization type uses separate namespaces for easy filtering:
- `robots` - Robot cylinders
- `unplaced_bricks` - Red/orange brick cubes
- `placed_bricks` - Purple placed brick cubes
- `targets` - Green target cubes
- `robot_states` - White floating text
- `robot_1_path`, `robot_2_path`, `robot_3_path` - Colored path lines
- `planning_grid` - Gray grid lines
- `reserved_areas` - Colored reserved cells
- `trajectory_arrows` - Movement direction arrows

## Architecture

### Core Components
1. **`path_planner.py`** - A* algorithm with collision avoidance
2. **`models.py`** - Extended robot models with path planning attributes
3. **`robot_brick_visualizer.py`** - Main coordinator with multi-robot FSM

### Key Classes
- **`PathPlanner`** - Handles pathfinding and path reservation
- **`Robot`** - Extended with path planning attributes
- **`ReservedPath`** - Tracks spatial-temporal path reservations

## Performance Features

- **Efficient pathfinding**: A* with early termination
- **Scalable collision detection**: Grid-based spatial indexing
- **Real-time coordination**: 10Hz update rate
- **Memory efficient**: Automatic cleanup of expired reservations

## Future Extensions

- **N-robot support**: Easily scalable to more robots
- **Complex environments**: Add static obstacles
- **Task priorities**: Priority-based task allocation
- **Load balancing**: Distribute tasks optimally
- **3D pathfinding**: Extend to full 3D movement 