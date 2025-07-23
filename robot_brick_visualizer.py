import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
import random
import threading
import time
import sys
import select
from models import Robot, Brick, Target, Location, RobotState, BrickState, TargetState
from path_planner import PathPlanner

class RobotBrickVisualizer(Node):
    def __init__(self):
        super().__init__('robot_brick_visualizer')
        self.publisher_ = self.create_publisher(MarkerArray, 'debug_markers', 10)
        
        # Create separate path publishers for each robot
        self.path_publishers_ = {}
        for robot_id in range(1, 4):
            topic_name = f'robot_{robot_id}_path'
            self.path_publishers_[robot_id] = self.create_publisher(Path, topic_name, 10)
        
        # Initialize data structures using proper classes - start completely empty
        self.robots = []
        self.unplaced_bricks = []
        self.placed_bricks = []  # Should start empty
        self.targets = []  # Should start empty
        self.target_lock = threading.Lock()
        
        # Track how many bricks are stacked at each target location
        self.target_brick_counts = {}  # target_id -> number of bricks placed
        
        # Initialize path planner
        self.path_planner = PathPlanner(grid_size=0.2, world_size=(5.0, 5.0))
        
        # Task completion tracking
        self.task_completed = False
        self.completion_announced = False
        self.simulation_running = True
        self.input_thread_active = False
        self.robots_can_start = False  # Robots wait until target input is complete
        self.ordered_target_filling = True  # Fill targets in the order they were added
        
        # Terminal control system
        self.system_paused = False
        self.shutdown_requested = False
        self.persistent_mode = True  # Run indefinitely until shutdown
        
        # Configurable entity quantities
        self.default_num_robots = 3
        self.default_num_bricks = 5
        
        # Initialize the simulation
        self.reset_simulation()
        
    def reset_simulation(self):
        """Reset the simulation to initial state with random positions"""
        print("\n=== Initializing New Simulation ===")
        
        # Clear existing data
        self.robots.clear()
        self.unplaced_bricks.clear()
        self.placed_bricks.clear()
        self.targets.clear()
        self.target_brick_counts.clear()
        
        # Reset completion tracking
        self.task_completed = False
        self.completion_announced = False
        self.robots_can_start = False  # Reset robot activation flag
        self.system_paused = False  # Reset pause state
        
        # Clear any existing markers from previous runs
        self.clear_all_markers()
        
        # Create configurable number of robots with random positions
        for robot_id in range(1, self.default_num_robots + 1):
            robot_loc = self.random_location()
            robot = Robot(robot_id=robot_id, location=robot_loc, state=RobotState.IDLE)
            robot.state_start_time = time.time()
            self.robots.append(robot)
        
        # Create configurable number of initial unplaced bricks
        for i in range(self.default_num_bricks):
            brick_loc = self.random_location()
            brick = Brick(brick_id=i+1, location=brick_loc, state=BrickState.UNGRABBED)
            self.unplaced_bricks.append(brick)
        
        # Start with NO targets - user must add them manually
        print("=== Multi-Robot Bricklaying System (Persistent Mode) ===")
        print(f"🤖 {len(self.robots)} robots initialized")
        print(f"🧱 {len(self.unplaced_bricks)} bricks to place")
        print(f"🎯 {len(self.targets)} target locations")
        print()
        print("🎮 TERMINAL CONTROL COMMANDS:")
        print("   📍 add_target x,y,z     - Add target location")
        print("   🧱 add_brick x,y,z      - Add unplaced brick")
        print("   🤖 add_robot x,y,z      - Add new robot")
        print("   ❌ remove_target ID     - Remove target by ID")
        print("   ❌ remove_brick ID      - Remove brick by ID")
        print("   ❌ remove_robot ID      - Remove robot by ID")
        print("   🎲 spawn_robots N       - Spawn N robots randomly")
        print("   🎲 spawn_bricks N       - Spawn N bricks randomly")
        print("   🗑️  clear_all           - Remove all entities")
        print("   ⚙️  set_defaults N,M    - Set default robots,bricks (e.g., 5,10)")
        print("   ▶️  start               - Start/resume robot operation")
        print("   ⏸️  pause               - Pause robot operation")
        print("   🛑 shutdown             - Exit system")
        print("   🔄 order                - Toggle target filling mode")
        print("   ❓ help                 - Show this help")
        print("   📊 status               - Show system status")
        print(f"🎯 Target filling mode: {'ORDERED (first-added first)' if self.ordered_target_filling else 'CLOSEST (distance-based)'}")
        print()
        
        # Start terminal control thread
        self.input_thread_active = True
        threading.Thread(target=self.terminal_control_thread, daemon=False).start()

    def check_task_completion(self):
        """Check if all bricks have been placed and announce completion"""
        if self.task_completed:
            return
            
        # Task is complete when all bricks are placed and all robots are idle
        all_bricks_placed = len(self.unplaced_bricks) == 0
        all_robots_idle = all(robot.state == RobotState.IDLE for robot in self.robots)
        
        if all_bricks_placed and all_robots_idle and not self.completion_announced:
            self.task_completed = True
            self.completion_announced = True
            
            # Calculate some stats
            total_bricks = len(self.placed_bricks)
            num_targets = len(self.targets)
            
            print(f"\n🎉 TASK COMPLETED! 🎉")
            print(f"✅ Successfully placed {total_bricks} bricks across {num_targets} target locations")
            print(f"📊 Final stacking distribution:")
            for target_id, count in self.target_brick_counts.items():
                print(f"   Target {target_id}: {count} bricks")
            
            # In persistent mode, just announce completion and continue running
            if self.persistent_mode:
                print(f"🎮 System continues running. Add more bricks/targets to start new tasks.")
                print(f"   Use 'help' to see available commands.")
                # Reset task completion state to allow future completions
                self.task_completed = False
                self.completion_announced = False
            else:
                # Legacy mode - stop and ask for restart
                self.input_thread_active = False
                input_thread = threading.Thread(target=self.completion_input_thread, daemon=True)
                input_thread.start()
            
    def completion_input_thread(self):
        """Handle user input for restarting simulation"""
        while self.task_completed and self.simulation_running:
            try:
                print(f"\n🔄 Would you like to run another simulation? (y/n): ", end="", flush=True)
                
                # Wait for input with timeout
                response = input().strip().lower()
                
                if response in ['y', 'yes']:
                    print("🚀 Starting new simulation...")
                    print("   New robots and bricks will be spawned.")
                    print("   You'll need to add target locations manually again.")
                    self.reset_simulation()
                    break
                elif response in ['n', 'no']:
                    print("👋 Exiting simulation. Thanks for using the robot brick system!")
                    self.simulation_running = False
                    break
                else:
                    print("Please enter 'y' for yes or 'n' for no.")
                    
            except (EOFError, KeyboardInterrupt):
                print("\n👋 Exiting simulation. Thanks for using the robot brick system!")
                self.simulation_running = False
                break

    def clear_all_markers(self):
        """Clear all existing markers from previous runs by sending DELETEALL action"""
        marker_array = MarkerArray()
        
        # Use DELETEALL action to clear all markers in each namespace
        namespaces = ['robots', 'unplaced_bricks', 'placed_bricks', 'targets']
        markers = []
        
        for ns in namespaces:
            delete_all_marker = Marker()
            delete_all_marker.header.frame_id = 'map'
            delete_all_marker.header.stamp = self.get_clock().now().to_msg()
            delete_all_marker.ns = ns
            delete_all_marker.id = 0  # ID doesn't matter for DELETEALL
            delete_all_marker.action = Marker.DELETEALL
            markers.append(delete_all_marker)
        
        marker_array.markers = markers
        # Publish the delete commands
        self.publisher_.publish(marker_array)
        
        # Also send a broader DELETEALL without namespace to catch any stray markers
        general_delete = Marker()
        general_delete.header.frame_id = 'map'
        general_delete.header.stamp = self.get_clock().now().to_msg()
        general_delete.ns = ''  # Empty namespace to catch all
        general_delete.id = 0
        general_delete.action = Marker.DELETEALL
        
        general_array = MarkerArray()
        general_array.markers = [general_delete]
        self.publisher_.publish(general_array)
        
        print("Cleared all existing markers from previous runs using DELETEALL")

    def random_location(self):
        """Generate a random location within the world bounds"""
        return Location(
            x=random.uniform(0.5, 4.5),  # Keep within pathfinding grid bounds
            y=random.uniform(0.5, 4.5),  # Keep within pathfinding grid bounds
            z=0.0
        )

    def distance(self, loc1, loc2):
        return ((loc1.x - loc2.x)**2 + (loc1.y - loc2.y)**2 + (loc1.z - loc2.z)**2)**0.5

    def get_robot_color(self, robot_id):
        """Generate a unique color for each robot ID using HSV color space"""
        import colorsys
        
        # Use HSV color space to generate evenly spaced colors
        # This ensures good color separation for any number of robots
        hue = ((robot_id - 1) * 0.618034) % 1.0  # Golden ratio for good spacing
        saturation = 0.8
        value = 1.0
        
        # Convert HSV to RGB
        rgb = colorsys.hsv_to_rgb(hue, saturation, value)
        return rgb

    def select_target(self, robot_location, available_targets):
        """Select target based on ordering preference"""
        if not available_targets:
            return None
            
        if self.ordered_target_filling:
            # Return the first target in the list (oldest added)
            return available_targets[0]
        else:
            # Return closest target by distance
            distances = [self.distance(robot_location, target.location) for target in available_targets]
            return available_targets[distances.index(min(distances))]

    def update_fsm(self):
        """Update FSM for all robots with path planning and coordination"""
        current_time = time.time()
        
        # Check for task completion
        self.check_task_completion()
        
        # Only update FSM if simulation is still running
        if not self.simulation_running:
            return
            
        # Update each robot independently
        for robot in self.robots:
            self.update_robot_fsm(robot, current_time)
    
    def update_robot_fsm(self, robot: Robot, current_time: float):
        """Update FSM for a single robot"""
        # Check if enough time has passed for state transition
        if current_time - robot.state_start_time < 0.5:  # Minimum state duration
            return
        
        # Handle path following for moving states
        if robot.state in [RobotState.MOVING_TO_BRICK, RobotState.MOVING_TO_TARGET]:
            if robot.current_path and robot.path_index < len(robot.current_path):
                # Move along the path
                if current_time - robot.last_move_time > 0.3:  # Move every 0.3 seconds
                    target_waypoint = robot.current_path[robot.path_index]
                    robot.location = Location(target_waypoint.x, target_waypoint.y, target_waypoint.z)
                    robot.path_index += 1
                    robot.last_move_time = current_time
                    
                    if robot.path_index >= len(robot.current_path):
                        # Reached destination
                        if robot.state == RobotState.MOVING_TO_BRICK:
                            robot.state = RobotState.GRABBING
                        else:  # MOVING_TO_TARGET
                            robot.state = RobotState.PLACING
                        robot.current_path = None
                        robot.path_index = 0
                        robot.state_start_time = current_time
                        print(f"Robot {robot.robot_id} reached destination")
                return
        
        # FSM state transitions
        if robot.state == RobotState.IDLE:
            # Only start working if robots are allowed to start and system is not paused
            if not self.robots_can_start or self.system_paused:
                return
                
            # Check if there are targets and bricks available
            with self.target_lock:
                available_targets = [t for t in self.targets if t.state == TargetState.PENDING]
                available_bricks = [b for b in self.unplaced_bricks if b.state == BrickState.UNGRABBED]
                
                if available_targets and available_bricks:
                    # Find closest available brick
                    distances = [self.distance(robot.location, brick.location) for brick in available_bricks]
                    closest_brick = available_bricks[distances.index(min(distances))]
                    
                    # Plan path to brick
                    path = self.path_planner.find_path(robot.location, closest_brick.location, robot.robot_id)
                    if path and self.path_planner.reserve_path(robot.robot_id, path, 10.0):
                        closest_brick.state = BrickState.RESERVED
                        robot.target_brick = closest_brick
                        robot.current_path = path
                        robot.path_index = 0
                        robot.state = RobotState.MOVING_TO_BRICK
                        robot.state_start_time = current_time
                        print(f"Robot {robot.robot_id} planning path to brick {closest_brick.brick_id}")
                    else:
                        robot.state = RobotState.WAITING
                        robot.state_start_time = current_time
                        print(f"Robot {robot.robot_id} waiting - no clear path to brick")
                        
        elif robot.state == RobotState.GRABBING:
            # Grab the brick
            if robot.target_brick and robot.target_brick in self.unplaced_bricks:
                self.unplaced_bricks.remove(robot.target_brick)
                print(f"Robot {robot.robot_id} grabbed brick {robot.target_brick.brick_id}")
                
                # Select target based on ordering preference
                with self.target_lock:
                    available_targets = [t for t in self.targets if t.state == TargetState.PENDING]
                    if available_targets:
                        selected_target = self.select_target(robot.location, available_targets)
                        if not selected_target:
                            return
                            
                        # Plan path to target
                        path = self.path_planner.find_path(robot.location, selected_target.location, robot.robot_id)
                        if path and self.path_planner.reserve_path(robot.robot_id, path, 10.0):
                            robot.target_location = selected_target
                            robot.current_path = path
                            robot.path_index = 0
                            robot.state = RobotState.MOVING_TO_TARGET
                            robot.state_start_time = current_time
                            print(f"Robot {robot.robot_id} planning path to target {selected_target.target_id}")
                        else:
                            robot.state = RobotState.WAITING
                            robot.state_start_time = current_time
                            
        elif robot.state == RobotState.PLACING:
            # Place the brick
            if robot.target_location and robot.target_brick:
                target_loc = robot.target_location.location
                target_id = robot.target_location.target_id
                
                # Calculate stacking height
                if target_id not in self.target_brick_counts:
                    self.target_brick_counts[target_id] = 0
                
                brick_height = 0.1
                stack_height = self.target_brick_counts[target_id] * brick_height
                stacked_z = target_loc.z + stack_height
                
                # Create placed brick
                placed_brick = Brick(
                    brick_id=robot.target_brick.brick_id,
                    location=Location(target_loc.x, target_loc.y, stacked_z),
                    state=BrickState.UNGRABBED
                )
                self.placed_bricks.append(placed_brick)
                self.target_brick_counts[target_id] += 1
                
                print(f"Robot {robot.robot_id} placed brick {robot.target_brick.brick_id} at target {target_id} (stack height: {self.target_brick_counts[target_id]})")
                
                # Reset robot state
                robot.target_brick = None
                robot.target_location = None
                robot.state = RobotState.IDLE
                robot.state_start_time = current_time
                
        elif robot.state == RobotState.WAITING:
            # Try again after a shorter delay
            if current_time - robot.state_start_time > 1.0:  # Wait 1 second (reduced from 2)
                robot.state = RobotState.IDLE
                robot.state_start_time = current_time

    def terminal_control_thread(self):
        """Handle real-time terminal commands for system control"""
        print("🎮 Terminal control ready - type 'help' for commands...")
        
        while self.input_thread_active and self.simulation_running and not self.shutdown_requested:
            try:
                user_input = input("🎮 Command: ").strip()
                parts = user_input.split()
                
                if not parts:
                    continue
                    
                command = parts[0].lower()
                
                if command == 'help':
                    self.show_help()
                elif command == 'status':
                    self.show_status()
                elif command == 'start':
                    self.handle_start()
                elif command == 'pause':
                    self.handle_pause()
                elif command == 'shutdown':
                    self.handle_shutdown()
                    break
                elif command == 'order':
                    self.handle_order_toggle()
                elif command == 'add_target':
                    self.handle_add_target(parts[1:])
                elif command == 'add_brick':
                    self.handle_add_brick(parts[1:])
                elif command == 'add_robot':
                    self.handle_add_robot(parts[1:])
                elif command == 'remove_target':
                    self.handle_remove_target(parts[1:])
                elif command == 'remove_brick':
                    self.handle_remove_brick(parts[1:])
                elif command == 'remove_robot':
                    self.handle_remove_robot(parts[1:])
                elif command == 'spawn_robots':
                    self.handle_spawn_robots(parts[1:])
                elif command == 'spawn_bricks':
                    self.handle_spawn_bricks(parts[1:])
                elif command == 'clear_all':
                    self.handle_clear_all()
                elif command == 'set_defaults':
                    self.handle_set_defaults(parts[1:])
                else:
                    print(f"❌ Unknown command: {command}. Type 'help' for available commands.")
                    
            except (EOFError, KeyboardInterrupt):
                print("\n🛑 Terminal control interrupted. Shutting down...")
                self.handle_shutdown()
                break
            except Exception as e:
                print(f"❌ Error processing command: {e}")
                
        print("🛑 Terminal control stopped.")

    # Command handler methods
    def show_help(self):
        """Display all available commands"""
        print("\n🎮 AVAILABLE COMMANDS:")
        print("   📍 add_target x,y,z     - Add target location")
        print("   🧱 add_brick x,y,z      - Add unplaced brick") 
        print("   🤖 add_robot x,y,z      - Add new robot")
        print("   ❌ remove_target ID     - Remove target by ID")
        print("   ❌ remove_brick ID      - Remove brick by ID")
        print("   ❌ remove_robot ID      - Remove robot by ID")
        print("   🎲 spawn_robots N       - Spawn N robots randomly")
        print("   🎲 spawn_bricks N       - Spawn N bricks randomly")
        print("   🗑️  clear_all           - Remove all entities")
        print("   ⚙️  set_defaults N,M    - Set default robots,bricks (e.g., 5,10)")
        print("   ▶️  start               - Start/resume robot operation")
        print("   ⏸️  pause               - Pause robot operation")
        print("   🛑 shutdown             - Exit system")
        print("   🔄 order                - Toggle target filling mode")
        print("   ❓ help                 - Show this help")
        print("   📊 status               - Show system status")
        print()

    def show_status(self):
        """Display current system status"""
        print(f"\n📊 SYSTEM STATUS:")
        print(f"   🤖 Robots: {len(self.robots)}")
        for robot in self.robots:
            print(f"      Robot {robot.robot_id}: {robot.state.name} at ({robot.location.x:.1f}, {robot.location.y:.1f})")
        print(f"   🧱 Unplaced bricks: {len(self.unplaced_bricks)}")
        print(f"   🟪 Placed bricks: {len(self.placed_bricks)}")
        print(f"   🎯 Targets: {len(self.targets)} ({sum(1 for t in self.targets if t.state == TargetState.PENDING)} pending)")
        print(f"   ▶️  System state: {'PAUSED' if self.system_paused else 'RUNNING'}")
        print(f"   🚀 Robots active: {'YES' if self.robots_can_start else 'NO'}")
        print(f"   🎯 Target mode: {'ORDERED' if self.ordered_target_filling else 'CLOSEST'}")
        print(f"   ⚙️  Defaults: {self.default_num_robots} robots, {self.default_num_bricks} bricks")
        print()

    def handle_start(self):
        """Start or resume robot operation"""
        self.system_paused = False
        self.robots_can_start = True
        print("▶️  System resumed - robots can now work!")

    def handle_pause(self):
        """Pause robot operation"""
        self.system_paused = True
        self.robots_can_start = False
        print("⏸️  System paused - robots will become idle")

    def handle_shutdown(self):
        """Shutdown the entire system"""
        print("🛑 Shutting down system...")
        self.shutdown_requested = True
        self.simulation_running = False
        self.input_thread_active = False

    def handle_order_toggle(self):
        """Toggle target filling order mode"""
        self.ordered_target_filling = not self.ordered_target_filling
        mode = 'ORDERED (first-added first)' if self.ordered_target_filling else 'CLOSEST (distance-based)'
        print(f"🔄 Target filling mode changed to: {mode}")

    def handle_add_target(self, args):
        """Add a new target location"""
        try:
            if len(args) != 1:
                print("❌ Usage: add_target x,y,z (e.g., add_target 2.5,3.0,0.0)")
                return
                
            coords = [float(x.strip()) for x in args[0].split(',')]
            if len(coords) != 3:
                print("❌ Please provide exactly 3 coordinates (x,y,z)")
                return
                
            x, y, z = coords
            
            # Validate coordinates
            if not (0 <= x <= 5 and 0 <= y <= 5 and 0 <= z <= 5):
                print("⚠️  Warning: Coordinates outside typical range (0-5). Continuing anyway...")
            
            with self.target_lock:
                target_id = len(self.targets) + 1
                target_loc = Location(x, y, z)
                target = Target(target_id=target_id, location=target_loc, state=TargetState.PENDING)
                self.targets.append(target)
                self.target_brick_counts[target_id] = 0
                
            print(f"✅ Added Target {target_id} at: ({x:.1f}, {y:.1f}, {z:.1f})")
            
        except ValueError:
            print("❌ Invalid coordinates. Use format: x,y,z (e.g., 2.5,3.0,0.0)")
        except Exception as e:
            print(f"❌ Error adding target: {e}")

    def handle_add_brick(self, args):
        """Add a new unplaced brick"""
        try:
            if len(args) != 1:
                print("❌ Usage: add_brick x,y,z (e.g., add_brick 1.5,2.0,0.0)")
                return
                
            coords = [float(x.strip()) for x in args[0].split(',')]
            if len(coords) != 3:
                print("❌ Please provide exactly 3 coordinates (x,y,z)")
                return
                
            x, y, z = coords
            
            # Validate coordinates
            if not (0 <= x <= 5 and 0 <= y <= 5 and 0 <= z <= 5):
                print("⚠️  Warning: Coordinates outside typical range (0-5). Continuing anyway...")
            
            brick_id = max([b.brick_id for b in self.unplaced_bricks + self.placed_bricks], default=0) + 1
            brick_loc = Location(x, y, z)
            brick = Brick(brick_id=brick_id, location=brick_loc, state=BrickState.UNGRABBED)
            self.unplaced_bricks.append(brick)
            
            print(f"✅ Added Brick {brick_id} at: ({x:.1f}, {y:.1f}, {z:.1f})")
            
        except ValueError:
            print("❌ Invalid coordinates. Use format: x,y,z (e.g., 1.5,2.0,0.0)")
        except Exception as e:
            print(f"❌ Error adding brick: {e}")

    def handle_add_robot(self, args):
        """Add a new robot"""
        try:
            if len(args) != 1:
                print("❌ Usage: add_robot x,y,z (e.g., add_robot 0.5,0.5,0.0)")
                return
                
            coords = [float(x.strip()) for x in args[0].split(',')]
            if len(coords) != 3:
                print("❌ Please provide exactly 3 coordinates (x,y,z)")
                return
                
            x, y, z = coords
            
            # Validate coordinates
            if not (0 <= x <= 5 and 0 <= y <= 5 and 0 <= z <= 5):
                print("⚠️  Warning: Coordinates outside typical range (0-5). Continuing anyway...")
            
            robot_id = max([r.robot_id for r in self.robots], default=0) + 1
            robot_loc = Location(x, y, z)
            robot = Robot(robot_id=robot_id, location=robot_loc, state=RobotState.IDLE)
            self.robots.append(robot)
            
            # Create path publisher for new robot
            topic_name = f'/robot_{robot_id}_path'
            self.path_publishers_[robot_id] = self.create_publisher(Path, topic_name, 10)
            
            print(f"✅ Added Robot {robot_id} at: ({x:.1f}, {y:.1f}, {z:.1f})")
            
        except ValueError:
            print("❌ Invalid coordinates. Use format: x,y,z (e.g., 0.5,0.5,0.0)")
        except Exception as e:
            print(f"❌ Error adding robot: {e}")

    def handle_remove_target(self, args):
        """Remove a target by ID"""
        try:
            if len(args) != 1:
                print("❌ Usage: remove_target ID (e.g., remove_target 1)")
                return
                
            target_id = int(args[0])
            
            with self.target_lock:
                target_to_remove = None
                for target in self.targets:
                    if target.target_id == target_id:
                        target_to_remove = target
                        break
                
                if target_to_remove:
                    self.targets.remove(target_to_remove)
                    if target_id in self.target_brick_counts:
                        del self.target_brick_counts[target_id]
                    print(f"✅ Removed Target {target_id}")
                else:
                    print(f"❌ Target {target_id} not found")
                    
        except ValueError:
            print("❌ Invalid ID. Use a number (e.g., remove_target 1)")
        except Exception as e:
            print(f"❌ Error removing target: {e}")

    def handle_remove_brick(self, args):
        """Remove a brick by ID"""
        try:
            if len(args) != 1:
                print("❌ Usage: remove_brick ID (e.g., remove_brick 1)")
                return
                
            brick_id = int(args[0])
            
            brick_to_remove = None
            for brick in self.unplaced_bricks:
                if brick.brick_id == brick_id:
                    brick_to_remove = brick
                    break
            
            if brick_to_remove:
                if brick_to_remove.state == BrickState.RESERVED:
                    print(f"⚠️  Warning: Brick {brick_id} is reserved by a robot")
                self.unplaced_bricks.remove(brick_to_remove)
                print(f"✅ Removed Brick {brick_id}")
            else:
                print(f"❌ Unplaced brick {brick_id} not found")
                
        except ValueError:
            print("❌ Invalid ID. Use a number (e.g., remove_brick 1)")
        except Exception as e:
            print(f"❌ Error removing brick: {e}")

    def handle_remove_robot(self, args):
        """Remove a robot by ID"""
        try:
            if len(args) != 1:
                print("❌ Usage: remove_robot ID (e.g., remove_robot 1)")
                return
                
            robot_id = int(args[0])
            
            robot_to_remove = None
            for robot in self.robots:
                if robot.robot_id == robot_id:
                    robot_to_remove = robot
                    break
            
            if robot_to_remove:
                if robot_to_remove.state != RobotState.IDLE:
                    print(f"⚠️  Warning: Robot {robot_id} is currently {robot_to_remove.state.name}")
                    
                self.robots.remove(robot_to_remove)
                
                # Remove path publisher
                if robot_id in self.path_publishers_:
                    del self.path_publishers_[robot_id]
                    
                print(f"✅ Removed Robot {robot_id}")
            else:
                print(f"❌ Robot {robot_id} not found")
                
        except ValueError:
            print("❌ Invalid ID. Use a number (e.g., remove_robot 1)")
        except Exception as e:
            print(f"❌ Error removing robot: {e}")

    def handle_spawn_robots(self, args):
        """Spawn N robots at random locations"""
        try:
            if len(args) != 1:
                print("❌ Usage: spawn_robots N (e.g., spawn_robots 5)")
                return
                
            num_robots = int(args[0])
            if num_robots <= 0:
                print("❌ Number of robots must be positive")
                return
                
            if num_robots > 50:
                print("❌ Maximum 50 robots allowed to prevent performance issues")
                return
                
            next_robot_id = max([r.robot_id for r in self.robots], default=0) + 1
            spawned_count = 0
            
            for i in range(num_robots):
                robot_id = next_robot_id + i
                robot_loc = self.random_location()
                robot = Robot(robot_id=robot_id, location=robot_loc, state=RobotState.IDLE)
                robot.state_start_time = time.time()
                self.robots.append(robot)
                
                # Create path publisher for new robot
                topic_name = f'/robot_{robot_id}_path'
                self.path_publishers_[robot_id] = self.create_publisher(Path, topic_name, 10)
                spawned_count += 1
                
            print(f"✅ Spawned {spawned_count} robots (IDs {next_robot_id}-{next_robot_id + num_robots - 1})")
            
        except ValueError:
            print("❌ Invalid number. Use a positive integer (e.g., spawn_robots 5)")
        except Exception as e:
            print(f"❌ Error spawning robots: {e}")

    def handle_spawn_bricks(self, args):
        """Spawn N bricks at random locations"""
        try:
            if len(args) != 1:
                print("❌ Usage: spawn_bricks N (e.g., spawn_bricks 10)")
                return
                
            num_bricks = int(args[0])
            if num_bricks <= 0:
                print("❌ Number of bricks must be positive")
                return
                
            if num_bricks > 100:
                print("❌ Maximum 100 bricks allowed to prevent performance issues")
                return
                
            next_brick_id = max([b.brick_id for b in self.unplaced_bricks + self.placed_bricks], default=0) + 1
            spawned_count = 0
            
            for i in range(num_bricks):
                brick_id = next_brick_id + i
                brick_loc = self.random_location()
                brick = Brick(brick_id=brick_id, location=brick_loc, state=BrickState.UNGRABBED)
                self.unplaced_bricks.append(brick)
                spawned_count += 1
                
            print(f"✅ Spawned {spawned_count} bricks (IDs {next_brick_id}-{next_brick_id + num_bricks - 1})")
            
        except ValueError:
            print("❌ Invalid number. Use a positive integer (e.g., spawn_bricks 10)")
        except Exception as e:
            print(f"❌ Error spawning bricks: {e}")

    def handle_clear_all(self):
        """Remove all robots, bricks, and targets"""
        try:
            # Count entities before clearing
            num_robots = len(self.robots)
            num_bricks = len(self.unplaced_bricks) + len(self.placed_bricks)
            num_targets = len(self.targets)
            
            # Clear all entity lists
            self.robots.clear()
            self.unplaced_bricks.clear()
            self.placed_bricks.clear()
            with self.target_lock:
                self.targets.clear()
                self.target_brick_counts.clear()
            
            # Clear path publishers
            self.path_publishers_.clear()
            
            # Clear visualization
            self.clear_all_markers()
            
            print(f"✅ Cleared all entities:")
            print(f"   🤖 {num_robots} robots removed")
            print(f"   🧱 {num_bricks} bricks removed")
            print(f"   🎯 {num_targets} targets removed")
            
        except Exception as e:
            print(f"❌ Error clearing entities: {e}")

    def handle_set_defaults(self, args):
        """Set default number of robots and bricks for reset_simulation"""
        try:
            if len(args) != 1:
                print("❌ Usage: set_defaults N,M (e.g., set_defaults 5,10)")
                return
                
            nums = [int(x.strip()) for x in args[0].split(',')]
            if len(nums) != 2:
                print("❌ Please provide exactly 2 numbers: robots,bricks")
                return
                
            num_robots, num_bricks = nums
            
            if num_robots <= 0 or num_bricks <= 0:
                print("❌ Both numbers must be positive")
                return
                
            if num_robots > 50:
                print("❌ Maximum 50 robots allowed")
                return
                
            if num_bricks > 100:
                print("❌ Maximum 100 bricks allowed")
                return
                
            self.default_num_robots = num_robots
            self.default_num_bricks = num_bricks
            
            print(f"✅ Set defaults: {num_robots} robots, {num_bricks} bricks")
            print(f"   These will be used when system restarts or resets")
            
        except ValueError:
            print("❌ Invalid format. Use: N,M (e.g., set_defaults 5,10)")
        except Exception as e:
            print(f"❌ Error setting defaults: {e}")

    def publish_markers(self):
        """Publish all markers and update FSM"""
        # Only continue if simulation is running
        if not self.simulation_running:
            return
            
        # Update FSM before publishing
        self.update_fsm()
        
        # Publish robot paths
        self.publish_robot_paths()
        
        marker_array = MarkerArray()
        
        # Robot markers (different colored cylinders)
        for robot in self.robots:
            robot_marker = Marker()
            robot_marker.header.frame_id = 'map'
            robot_marker.header.stamp = self.get_clock().now().to_msg()
            robot_marker.ns = 'robots'
            robot_marker.id = robot.robot_id
            robot_marker.type = Marker.CYLINDER
            robot_marker.action = Marker.ADD
            robot_marker.pose.position.x = robot.location.x
            robot_marker.pose.position.y = robot.location.y
            robot_marker.pose.position.z = robot.location.z
            robot_marker.scale.x = 0.4
            robot_marker.scale.y = 0.4
            robot_marker.scale.z = 0.6
            color = self.get_robot_color(robot.robot_id)
            robot_marker.color.r = color[0]
            robot_marker.color.g = color[1]
            robot_marker.color.b = color[2]
            robot_marker.color.a = 1.0
            marker_array.markers.append(robot_marker)
        
        # Unplaced brick markers (red/orange cubes)
        for brick in self.unplaced_bricks:
            brick_marker = Marker()
            brick_marker.header.frame_id = 'map'
            brick_marker.header.stamp = self.get_clock().now().to_msg()
            brick_marker.ns = 'unplaced_bricks'
            brick_marker.id = brick.brick_id
            brick_marker.type = Marker.CUBE
            brick_marker.action = Marker.ADD
            brick_marker.pose.position.x = brick.location.x
            brick_marker.pose.position.y = brick.location.y
            brick_marker.pose.position.z = brick.location.z
            brick_marker.scale.x = 0.3
            brick_marker.scale.y = 0.15
            brick_marker.scale.z = 0.1
            # Color based on state: red for ungrabbed, orange for reserved
            if brick.state == BrickState.RESERVED:
                brick_marker.color.r = 1.0
                brick_marker.color.g = 0.5
                brick_marker.color.b = 0.0
            else:
                brick_marker.color.r = 1.0
                brick_marker.color.g = 0.0
                brick_marker.color.b = 0.0
            brick_marker.color.a = 1.0
            marker_array.markers.append(brick_marker)
        
        # Placed brick markers (purple cubes at target locations)
        for brick in self.placed_bricks:
            brick_marker = Marker()
            brick_marker.header.frame_id = 'map'
            brick_marker.header.stamp = self.get_clock().now().to_msg()
            brick_marker.ns = 'placed_bricks'
            brick_marker.id = brick.brick_id + 1000  # Avoid ID collision with unplaced bricks
            brick_marker.type = Marker.CUBE
            brick_marker.action = Marker.ADD
            brick_marker.pose.position.x = brick.location.x
            brick_marker.pose.position.y = brick.location.y
            brick_marker.pose.position.z = brick.location.z
            brick_marker.scale.x = 0.3
            brick_marker.scale.y = 0.15
            brick_marker.scale.z = 0.1
            brick_marker.color.r = 0.5  # Purple color
            brick_marker.color.g = 0.0
            brick_marker.color.b = 1.0
            brick_marker.color.a = 1.0
            marker_array.markers.append(brick_marker)
            
        # Target markers - always green (don't change to yellow)
        with self.target_lock:
            for target in self.targets:
                target_marker = Marker()
                target_marker.header.frame_id = 'map'
                target_marker.header.stamp = self.get_clock().now().to_msg()
                target_marker.ns = 'targets'
                target_marker.id = target.target_id + 100  # Avoid ID collision
                target_marker.type = Marker.CUBE
                target_marker.action = Marker.ADD
                target_marker.pose.position.x = target.location.x
                target_marker.pose.position.y = target.location.y
                target_marker.pose.position.z = target.location.z
                target_marker.scale.x = 0.3
                target_marker.scale.y = 0.15
                target_marker.scale.z = 0.1
                # Always green - user knows it's filled by purple brick presence
                target_marker.color.r = 0.0  # Green
                target_marker.color.g = 1.0
                target_marker.color.b = 0.0
                target_marker.color.a = 1.0
                marker_array.markers.append(target_marker)
        
        # Add path visualization as LINE_STRIP markers
        self.add_path_markers(marker_array)
        
        # Add robot state text markers
        self.add_robot_state_text(marker_array)
        
        # Add planning grid visualization
        self.add_planning_grid(marker_array)
        
        # Add reserved area markers
        self.add_reserved_areas(marker_array)
        
        # Add trajectory arrows
        self.add_trajectory_arrows(marker_array)
                
        self.publisher_.publish(marker_array)

    def add_path_markers(self, marker_array: MarkerArray):
        """Add robot path visualization as LINE_STRIP markers to the marker array"""
        reserved_paths = self.path_planner.get_reserved_paths()
        
        for robot_id, path_locations in reserved_paths.items():
            if not path_locations or len(path_locations) < 2:
                continue
                
            # Create LINE_STRIP marker for robot path
            path_marker = Marker()
            path_marker.header.frame_id = 'map'
            path_marker.header.stamp = self.get_clock().now().to_msg()
            path_marker.ns = f'robot_{robot_id}_path'
            path_marker.id = robot_id
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            path_marker.scale.x = 0.05  # Line width
            
            # Set color based on robot ID
            color = self.get_robot_color(robot_id)
            path_marker.color.r = color[0]
            path_marker.color.g = color[1] 
            path_marker.color.b = color[2]
            path_marker.color.a = 0.8  # Semi-transparent
            
            # Add path points
            path_marker.points = []
            for location in path_locations:
                point = Point()
                point.x = location.x
                point.y = location.y
                point.z = location.z + 0.1  # Slightly above ground
                path_marker.points.append(point)
            
            marker_array.markers.append(path_marker)

    def add_robot_state_text(self, marker_array: MarkerArray):
        """Add floating text above each robot showing their current state"""
        for robot in self.robots:
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'robot_states'
            text_marker.id = robot.robot_id + 200  # Avoid ID collisions
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = robot.location.x
            text_marker.pose.position.y = robot.location.y
            text_marker.pose.position.z = robot.location.z + 0.8  # Above robot
            text_marker.scale.z = 0.15  # Text size
            text_marker.color.r = 1.0  # White text
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            # Create informative text based on robot state
            state_text = f"R{robot.robot_id}: {robot.state.value}"
            if robot.target_brick:
                state_text += f"\n→ Brick {robot.target_brick.brick_id}"
            if robot.target_location:
                state_text += f"\n→ Target {robot.target_location.target_id}"
            if robot.current_path:
                progress = f"{robot.path_index}/{len(robot.current_path)}"
                state_text += f"\nPath: {progress}"
            
            text_marker.text = state_text
            marker_array.markers.append(text_marker)

    def add_planning_grid(self, marker_array: MarkerArray):
        """Add grid lines to show planning resolution"""
        grid_size = self.path_planner.grid_size
        world_size = self.path_planner.world_size
        
        # Vertical lines
        for i in range(0, int(world_size[0] / grid_size) + 1):
            x = i * grid_size
            line_marker = Marker()
            line_marker.header.frame_id = 'map'
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = 'planning_grid'
            line_marker.id = i
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.005  # Very thin lines
            line_marker.color.r = 0.5
            line_marker.color.g = 0.5
            line_marker.color.b = 0.5
            line_marker.color.a = 0.2  # Very transparent
            
            # Add points for vertical line
            line_marker.points = []
            for y in [0.0, world_size[1]]:
                point = Point()
                point.x = x
                point.y = y
                point.z = 0.0
                line_marker.points.append(point)
            
            marker_array.markers.append(line_marker)
        
        # Horizontal lines
        for j in range(0, int(world_size[1] / grid_size) + 1):
            y = j * grid_size
            line_marker = Marker()
            line_marker.header.frame_id = 'map'
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = 'planning_grid'
            line_marker.id = j + 1000  # Offset to avoid collision with vertical lines
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.005  # Very thin lines
            line_marker.color.r = 0.5
            line_marker.color.g = 0.5
            line_marker.color.b = 0.5
            line_marker.color.a = 0.2  # Very transparent
            
            # Add points for horizontal line
            line_marker.points = []
            for x in [0.0, world_size[0]]:
                point = Point()
                point.x = x
                point.y = y
                point.z = 0.0
                line_marker.points.append(point)
            
            marker_array.markers.append(line_marker)

    def add_reserved_areas(self, marker_array: MarkerArray):
        """Show reserved path cells as colored squares"""
        # Get current reserved paths
        current_time = time.time()
        marker_id = 0
        
        for robot_id, reserved_path in self.path_planner.reserved_paths.items():
            # Check if path is still active
            if current_time > reserved_path.start_time + reserved_path.duration:
                continue
                
            color = self.get_robot_color(robot_id)
            
            # Create markers for reserved cells
            for grid_x, grid_y in reserved_path.cells:
                # Convert grid coordinates to world coordinates
                world_x, world_y = self.path_planner.grid_to_world(grid_x, grid_y)
                
                cell_marker = Marker()
                cell_marker.header.frame_id = 'map'
                cell_marker.header.stamp = self.get_clock().now().to_msg()
                cell_marker.ns = 'reserved_areas'
                cell_marker.id = marker_id
                cell_marker.type = Marker.CUBE
                cell_marker.action = Marker.ADD
                cell_marker.pose.position.x = world_x
                cell_marker.pose.position.y = world_y
                cell_marker.pose.position.z = 0.01  # Just above ground
                cell_marker.scale.x = self.path_planner.grid_size * 0.8  # Slightly smaller than grid
                cell_marker.scale.y = self.path_planner.grid_size * 0.8
                cell_marker.scale.z = 0.02  # Very thin
                cell_marker.color.r = color[0]
                cell_marker.color.g = color[1]
                cell_marker.color.b = color[2]
                cell_marker.color.a = 0.3  # Semi-transparent
                
                marker_array.markers.append(cell_marker)
                marker_id += 1

    def add_trajectory_arrows(self, marker_array: MarkerArray):
        """Add arrows showing movement direction for each robot"""
        
        for robot in self.robots:
            # Only show arrows for robots that are moving
            if robot.state not in [RobotState.MOVING_TO_BRICK, RobotState.MOVING_TO_TARGET]:
                continue
                
            if not robot.current_path or robot.path_index >= len(robot.current_path):
                continue
                
            # Get current position and next waypoint
            current_pos = robot.location
            if robot.path_index < len(robot.current_path):
                next_waypoint = robot.current_path[robot.path_index]
                
                # Calculate direction vector
                dx = next_waypoint.x - current_pos.x
                dy = next_waypoint.y - current_pos.y
                
                # Create arrow marker
                arrow_marker = Marker()
                arrow_marker.header.frame_id = 'map'
                arrow_marker.header.stamp = self.get_clock().now().to_msg()
                arrow_marker.ns = 'trajectory_arrows'
                arrow_marker.id = robot.robot_id + 300  # Avoid ID collisions
                arrow_marker.type = Marker.ARROW
                arrow_marker.action = Marker.ADD
                
                # Set arrow position and orientation
                arrow_marker.pose.position.x = current_pos.x
                arrow_marker.pose.position.y = current_pos.y
                arrow_marker.pose.position.z = current_pos.z + 0.2  # Above robot
                
                # Calculate orientation from direction vector
                import math
                yaw = math.atan2(dy, dx)
                # Convert to quaternion (simplified - only yaw rotation)
                arrow_marker.pose.orientation.z = math.sin(yaw / 2.0)
                arrow_marker.pose.orientation.w = math.cos(yaw / 2.0)
                
                # Set arrow size
                arrow_marker.scale.x = 0.3  # Length
                arrow_marker.scale.y = 0.05  # Width
                arrow_marker.scale.z = 0.05  # Height
                
                # Set color based on robot
                color = self.get_robot_color(robot.robot_id)
                arrow_marker.color.r = color[0]
                arrow_marker.color.g = color[1]
                arrow_marker.color.b = color[2]
                arrow_marker.color.a = 0.8
                
                marker_array.markers.append(arrow_marker)

    def publish_robot_paths(self):
        """Publish robot paths for visualization in RViz2 on separate topics"""
        reserved_paths = self.path_planner.get_reserved_paths()
        
        for robot_id, path_locations in reserved_paths.items():
            if not path_locations or robot_id not in self.path_publishers_:
                continue
                
            # Create Path message
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            path_msg.header.stamp = self.get_clock().now().to_msg()
            
            # Convert locations to poses
            for location in path_locations:
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = 'map'
                pose_stamped.header.stamp = self.get_clock().now().to_msg()
                pose_stamped.pose.position.x = location.x
                pose_stamped.pose.position.y = location.y
                pose_stamped.pose.position.z = location.z
                pose_stamped.pose.orientation.w = 1.0  # No rotation
                path_msg.poses.append(pose_stamped)
            
            # Publish to robot-specific topic
            self.path_publishers_[robot_id].publish(path_msg)


def main(args=None):
    print("Starting robot_brick_visualizer in persistent mode...")
    rclpy.init(args=args)
    node = RobotBrickVisualizer()
    try:
        while rclpy.ok() and node.simulation_running and not node.shutdown_requested:
            node.publish_markers()
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("\n🛑 Keyboard interrupt received. Shutting down...")
        node.handle_shutdown()
    
    print("🛑 System shutdown complete.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 