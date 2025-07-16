#!/usr/bin/env python3
"""
Multi-Robot Path Planning Module

Implements A* pathfinding with collision avoidance and path reservation system
for coordinating multiple robots in the bricklaying environment.
"""

import math
import heapq
import time
from typing import List, Tuple, Optional, Set, Dict
from dataclasses import dataclass
from models import Location

@dataclass
class PathNode:
    """Represents a node in the path planning grid"""
    x: float
    y: float
    g_cost: float = 0.0  # Cost from start
    h_cost: float = 0.0  # Heuristic cost to goal
    f_cost: float = 0.0  # Total cost (g + h)
    parent: Optional['PathNode'] = None
    
    def __lt__(self, other):
        return self.f_cost < other.f_cost

@dataclass
class ReservedPath:
    """Represents a path reserved by a robot"""
    robot_id: int
    path: List[Location]
    start_time: float
    duration: float
    cells: Set[Tuple[int, int]]  # Grid cells occupied by this path

class PathPlanner:
    """
    Multi-robot path planner with collision avoidance
    
    Uses A* algorithm with dynamic obstacle avoidance based on
    other robots' reserved paths.
    """
    
    def __init__(self, grid_size: float = 0.1, world_size: Tuple[float, float] = (5.0, 5.0)):
        """
        Initialize the path planner
        
        Args:
            grid_size: Resolution of the planning grid (meters)
            world_size: Size of the world (width, height) in meters
        """
        self.grid_size = grid_size
        self.world_size = world_size
        self.grid_width = int(world_size[0] / grid_size)
        self.grid_height = int(world_size[1] / grid_size)
        
        # Path reservation system
        self.reserved_paths: Dict[int, ReservedPath] = {}
        self.path_lock = None  # Will be set from main thread
        
        # Static obstacles (can be extended for environment obstacles)
        self.static_obstacles: Set[Tuple[int, int]] = set()
        
        # Robot safety radius in grid cells - reduced for less conservative planning
        self.robot_radius_cells = max(1, int(0.15 / grid_size))  # 15cm safety radius (reduced from 30cm)
        
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates"""
        grid_x = int(x / self.grid_size)
        grid_y = int(y / self.grid_size)
        # Clamp to grid bounds
        grid_x = max(0, min(self.grid_width - 1, grid_x))
        grid_y = max(0, min(self.grid_height - 1, grid_y))
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates"""
        x = (grid_x + 0.5) * self.grid_size
        y = (grid_y + 0.5) * self.grid_size
        return x, y
    
    def heuristic(self, node: PathNode, goal: PathNode) -> float:
        """Calculate heuristic distance (Euclidean)"""
        dx = node.x - goal.x
        dy = node.y - goal.y
        return math.sqrt(dx * dx + dy * dy)
    
    def get_neighbors(self, node: PathNode) -> List[PathNode]:
        """Get valid neighboring nodes (8-connected grid)"""
        neighbors = []
        grid_x, grid_y = self.world_to_grid(node.x, node.y)
        
        # 8-connected neighbors
        directions = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1)
        ]
        
        for dx, dy in directions:
            new_grid_x = grid_x + dx
            new_grid_y = grid_y + dy
            
            # Check bounds
            if (0 <= new_grid_x < self.grid_width and 
                0 <= new_grid_y < self.grid_height):
                
                world_x, world_y = self.grid_to_world(new_grid_x, new_grid_y)
                neighbor = PathNode(world_x, world_y)
                neighbors.append(neighbor)
        
        return neighbors
    
    def is_cell_occupied(self, grid_x: int, grid_y: int, robot_id: int, 
                        start_time: float, duration: float) -> bool:
        """
        Check if a grid cell is occupied by another robot's reserved path
        
        Args:
            grid_x, grid_y: Grid coordinates to check
            robot_id: ID of the robot requesting the path
            start_time: When the robot would occupy this cell
            duration: How long the robot would occupy this cell
        """
        # Check static obstacles
        if (grid_x, grid_y) in self.static_obstacles:
            return True
        
        # Check reserved paths from other robots
        current_time = time.time()
        for other_robot_id, reserved_path in self.reserved_paths.items():
            if other_robot_id == robot_id:
                continue  # Don't check against own path
            
            # Check if the reserved path is still active
            if current_time > reserved_path.start_time + reserved_path.duration:
                continue  # Path has expired
            
            # Check for spatial-temporal conflict
            if (grid_x, grid_y) in reserved_path.cells:
                # Check for temporal overlap - be more lenient with timing
                other_start = reserved_path.start_time
                other_end = reserved_path.start_time + reserved_path.duration
                our_start = start_time
                our_end = start_time + duration
                
                # Add buffer time to reduce conflicts
                buffer_time = 0.5  # 0.5 second buffer
                
                # Check if time intervals overlap (with buffer)
                if not (our_end + buffer_time <= other_start or our_start >= other_end + buffer_time):
                    return True  # Temporal conflict
        
        return False
    
    def calculate_path_cells(self, path: List[Location]) -> Set[Tuple[int, int]]:
        """Calculate all grid cells that a path passes through"""
        cells = set()
        
        for location in path:
            grid_x, grid_y = self.world_to_grid(location.x, location.y)
            
            # Add cells in robot radius - reduced radius for less conservative planning
            for dx in range(-self.robot_radius_cells, self.robot_radius_cells + 1):
                for dy in range(-self.robot_radius_cells, self.robot_radius_cells + 1):
                    if dx * dx + dy * dy <= self.robot_radius_cells * self.robot_radius_cells:
                        cell_x = grid_x + dx
                        cell_y = grid_y + dy
                        if (0 <= cell_x < self.grid_width and 
                            0 <= cell_y < self.grid_height):
                            cells.add((cell_x, cell_y))
        
        return cells
    
    def find_path(self, start: Location, goal: Location, robot_id: int) -> Optional[List[Location]]:
        """
        Find a path from start to goal using A* algorithm with collision avoidance
        
        Args:
            start: Starting location
            goal: Goal location  
            robot_id: ID of the robot requesting the path
            
        Returns:
            List of waypoints if path found, None otherwise
        """
        # First, try to find a path with collision avoidance
        path = self._find_path_with_avoidance(start, goal, robot_id)
        
        # If no path found with collision avoidance, try without (emergency fallback)
        if not path:
            print(f"Robot {robot_id}: No path with collision avoidance, trying fallback...")
            path = self._find_path_without_avoidance(start, goal)
        
        return path
    
    def _find_path_with_avoidance(self, start: Location, goal: Location, robot_id: int) -> Optional[List[Location]]:
        """Find path with collision avoidance"""
        start_node = PathNode(start.x, start.y)
        goal_node = PathNode(goal.x, goal.y)
        
        open_set = []
        closed_set = set()
        
        start_node.h_cost = self.heuristic(start_node, goal_node)
        start_node.f_cost = start_node.h_cost
        
        heapq.heappush(open_set, start_node)
        
        # For tracking visited nodes
        visited = {}
        
        max_iterations = 1000  # Prevent infinite loops
        iterations = 0
        
        while open_set and iterations < max_iterations:
            iterations += 1
            
            current = heapq.heappop(open_set)
            current_key = (round(current.x, 3), round(current.y, 3))
            
            if current_key in closed_set:
                continue
                
            closed_set.add(current_key)
            
            # Check if we reached the goal (within tolerance)
            distance_to_goal = self.heuristic(current, goal_node)
            if distance_to_goal < self.grid_size * 1.5:  # Slightly more tolerant
                # Reconstruct path
                path = []
                node = current
                while node:
                    path.append(Location(node.x, node.y, start.z))
                    node = node.parent
                path.reverse()
                return path
            
            # Explore neighbors
            for neighbor in self.get_neighbors(current):
                neighbor_key = (round(neighbor.x, 3), round(neighbor.y, 3))
                
                if neighbor_key in closed_set:
                    continue
                
                # Check if this cell would be occupied by other robots
                grid_x, grid_y = self.world_to_grid(neighbor.x, neighbor.y)
                estimated_arrival_time = time.time() + current.g_cost * 0.5  # Faster movement estimate
                cell_occupation_time = 0.5  # Reduced occupation time
                
                if self.is_cell_occupied(grid_x, grid_y, robot_id, 
                                       estimated_arrival_time, cell_occupation_time):
                    continue  # Skip occupied cells
                
                # Calculate costs
                move_cost = self.heuristic(current, neighbor)
                tentative_g = current.g_cost + move_cost
                
                neighbor_in_open = neighbor_key in visited
                if not neighbor_in_open or tentative_g < visited[neighbor_key].g_cost:
                    neighbor.parent = current
                    neighbor.g_cost = tentative_g
                    neighbor.h_cost = self.heuristic(neighbor, goal_node)
                    neighbor.f_cost = neighbor.g_cost + neighbor.h_cost
                    
                    visited[neighbor_key] = neighbor
                    heapq.heappush(open_set, neighbor)
        
        return None  # No path found
    
    def _find_path_without_avoidance(self, start: Location, goal: Location) -> Optional[List[Location]]:
        """Find path without collision avoidance (emergency fallback)"""
        start_node = PathNode(start.x, start.y)
        goal_node = PathNode(goal.x, goal.y)
        
        open_set = []
        closed_set = set()
        
        start_node.h_cost = self.heuristic(start_node, goal_node)
        start_node.f_cost = start_node.h_cost
        
        heapq.heappush(open_set, start_node)
        
        # For tracking visited nodes
        visited = {}
        
        max_iterations = 500  # Reduced for fallback
        iterations = 0
        
        while open_set and iterations < max_iterations:
            iterations += 1
            
            current = heapq.heappop(open_set)
            current_key = (round(current.x, 3), round(current.y, 3))
            
            if current_key in closed_set:
                continue
                
            closed_set.add(current_key)
            
            # Check if we reached the goal (within tolerance)
            distance_to_goal = self.heuristic(current, goal_node)
            if distance_to_goal < self.grid_size * 1.5:
                # Reconstruct path
                path = []
                node = current
                while node:
                    path.append(Location(node.x, node.y, start.z))
                    node = node.parent
                path.reverse()
                return path
            
            # Explore neighbors (no collision checking)
            for neighbor in self.get_neighbors(current):
                neighbor_key = (round(neighbor.x, 3), round(neighbor.y, 3))
                
                if neighbor_key in closed_set:
                    continue
                
                # Calculate costs
                move_cost = self.heuristic(current, neighbor)
                tentative_g = current.g_cost + move_cost
                
                neighbor_in_open = neighbor_key in visited
                if not neighbor_in_open or tentative_g < visited[neighbor_key].g_cost:
                    neighbor.parent = current
                    neighbor.g_cost = tentative_g
                    neighbor.h_cost = self.heuristic(neighbor, goal_node)
                    neighbor.f_cost = neighbor.g_cost + neighbor.h_cost
                    
                    visited[neighbor_key] = neighbor
                    heapq.heappush(open_set, neighbor)
        
        return None  # No path found
    
    def reserve_path(self, robot_id: int, path: List[Location], duration: float) -> bool:
        """
        Reserve a path for a robot
        
        Args:
            robot_id: ID of the robot
            path: Path to reserve
            duration: How long to reserve the path (seconds)
            
        Returns:
            True if reservation successful, False otherwise
        """
        if not path:
            return False
        
        start_time = time.time()
        cells = self.calculate_path_cells(path)
        
        # Check if path conflicts with existing reservations
        for cell in cells:
            if self.is_cell_occupied(cell[0], cell[1], robot_id, start_time, duration):
                return False  # Conflict detected
        
        # Reserve the path
        reserved_path = ReservedPath(
            robot_id=robot_id,
            path=path,
            start_time=start_time,
            duration=duration,
            cells=cells
        )
        
        self.reserved_paths[robot_id] = reserved_path
        print(f"Robot {robot_id}: Reserved path with {len(path)} waypoints for {duration}s")
        return True
    
    def clear_expired_reservations(self):
        """Remove expired path reservations"""
        current_time = time.time()
        expired_robots = []
        
        for robot_id, reserved_path in self.reserved_paths.items():
            if current_time > reserved_path.start_time + reserved_path.duration:
                expired_robots.append(robot_id)
        
        for robot_id in expired_robots:
            del self.reserved_paths[robot_id]
    
    def get_reserved_paths(self) -> Dict[int, List[Location]]:
        """Get all currently reserved paths for visualization"""
        self.clear_expired_reservations()
        return {robot_id: path.path for robot_id, path in self.reserved_paths.items()} 