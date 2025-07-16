from models import Robot, Brick, Target, Location, RobotState, BrickState, TargetState

# Initialize collections
robots = []
unplaced_bricks = []
targets = []

# Example: Add 2 robots at different locations
robots.append(Robot(robot_id=1, location=Location(0, 0, 0)))
robots.append(Robot(robot_id=2, location=Location(5, 5, 0)))

# Example: Add 5 unplaced bricks in a pile
for i in range(5):
    unplaced_bricks.append(Brick(brick_id=i+1, location=Location(x=i, y=0, z=0)))

# targets starts empty; will be added via CLI

def print_status():
    print("\nRobots:")
    for robot in robots:
        print(robot)
    print("\nUnplaced Bricks:")
    for brick in unplaced_bricks:
        print(brick)
    print("\nTargets:")
    for target in targets:
        print(target)

def add_target():
    try:
        coords = input("Enter target brick x,y,z: ").strip()
        x, y, z = map(float, coords.split(','))
        target_id = len(targets) + 1
        targets.append(Target(target_id=target_id, location=Location(x, y, z)))
        print(f"Added target {target_id} at ({x}, {y}, {z})")
    except Exception as e:
        print(f"Invalid input: {e}")

def mark_target_placed():
    try:
        target_id = int(input("Enter target ID to mark as placed: ").strip())
        for target in targets:
            if target.target_id == target_id:
                target.state = TargetState.PLACED
                print(f"Target {target_id} marked as placed.")
                return
        print("Target ID not found.")
    except Exception as e:
        print(f"Invalid input: {e}")

def main():
    while True:
        print("\nOptions:")
        print("1. Show status")
        print("2. Add target location")
        print("3. Mark target as placed")
        print("4. Exit")
        choice = input("Select option: ").strip()
        if choice == '1':
            print_status()
        elif choice == '2':
            add_target()
        elif choice == '3':
            mark_target_placed()
        elif choice == '4':
            break
        else:
            print("Invalid option.")

if __name__ == "__main__":
    main() 