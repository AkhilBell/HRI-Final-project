#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time

# --- CONFIGURATION ---
# EDIT THESE COORDINATES AFTER MAPPING!
# Use "ros2 topic echo /clicked_point" in a separate terminal
# and click "Publish Point" in Rviz to find these values.
ISLAND_1_X = 1.0 
ISLAND_1_Y = 0.0

def main():
    rclpy.init()
    
    # Initialize the Navigator
    navigator = BasicNavigator()
    print("Waiting for Nav2 to activate...")
    navigator.waitUntilNav2Active()
    print("Nav2 Active!")

    # --- STEP 1: Go to Island ---
    print(f"Navigating to Island 1 ({ISLAND_1_X}, {ISLAND_1_Y})...")
    goal_pose = navigator.getPoseStamped([ISLAND_1_X, ISLAND_1_Y], 0.0) # 0.0 is orientation
    navigator.goToPose(goal_pose)

    while not navigator.isTaskComplete():
        # Optional: Print feedback
        pass

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Arrived at Island!")
    else:
        print("Failed to reach Island. Check map/obstacles.")
        # If we fail, we stop here to avoid crashing into things
        rclpy.shutdown()
        return

    time.sleep(3) # Wait at island for 3 seconds

    # --- STEP 2: Return Home ---
    print("Returning Home (0,0)...")
    home_pose = navigator.getPoseStamped([0.0, 0.0], 0.0)
    navigator.goToPose(home_pose)

    while not navigator.isTaskComplete():
        pass

    if navigator.getResult() == TaskResult.SUCCEEDED:
        print("Returned Home Successfully. Test Passed!")
    else:
        print("Failed to return home.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()