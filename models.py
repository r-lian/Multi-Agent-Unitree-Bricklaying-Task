from enum import Enum
from typing import List, Optional

class RobotState(Enum):
    IDLE = 'idle'
    MOVING_TO_BRICK = 'moving_to_brick'
    MOVING_TO_TARGET = 'moving_to_target'
    GRABBING = 'grabbing'
    PLACING = 'placing'
    WAITING = 'waiting'  # Waiting for path or other robots

class BrickState(Enum):
    UNGRABBED = 'ungrabbed'
    RESERVED = 'reserved'
    # Optionally add GRABBED if needed

class TargetState(Enum):
    PENDING = 'pending'
    PLACED = 'placed'

class Location:
    def __init__(self, x, y, z, rotation=None):
        self.x = x
        self.y = y
        self.z = z
        self.rotation = rotation  # Can be a tuple or list (e.g., quaternion or Euler)

    def __repr__(self):
        return f"Location(x={self.x}, y={self.y}, z={self.z}, rotation={self.rotation})"

class Robot:
    def __init__(self, robot_id, location, state=RobotState.IDLE):
        self.robot_id = robot_id
        self.location = location
        self.state = state
        
        # Path planning attributes
        self.current_path: Optional[List['Location']] = None
        self.path_index: int = 0
        self.target_brick: Optional['Brick'] = None
        self.target_location: Optional['Target'] = None
        
        # Timing for coordination
        self.state_start_time: float = 0.0
        self.last_move_time: float = 0.0

    def __repr__(self):
        return f"Robot(id={self.robot_id}, location={self.location}, state={self.state})"

class Brick:
    def __init__(self, brick_id, location, state=BrickState.UNGRABBED):
        self.brick_id = brick_id
        self.location = location
        self.state = state

    def __repr__(self):
        return f"Brick(id={self.brick_id}, location={self.location}, state={self.state})"

class Target:
    def __init__(self, target_id, location, state=TargetState.PENDING):
        self.target_id = target_id
        self.location = location
        self.state = state

    def __repr__(self):
        return f"Target(id={self.target_id}, location={self.location}, state={self.state})" 