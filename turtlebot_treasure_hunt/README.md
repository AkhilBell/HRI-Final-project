# TurtleBot4 Pirate Treasure Hunt Game

## Overview
A fun ROS2-based game where a TurtleBot4 robot navigates to a person standing 6 feet away, announces the probability of finding treasure at that location, and returns to the starting position. This sequence repeats 10 times.

## Game Flow
1. **Navigate to Island**: Robot travels 6 feet (1.83m) to reach the person
2. **Calculate Probability**: Robot generates a random treasure probability (10%-90%)
3. **Announce Result**: Robot announces the probability to the person
4. **Determine Outcome**: Based on the probability, treasure may or may not be found
5. **Return Home**: Robot navigates back to the starting position
6. **Repeat**: Steps 1-5 are repeated 10 times

## Project Structure
```
turtlebot_treasure_hunt/
├── turtlebot_treasure_hunt/
│   ├── __init__.py                    # Package initialization
│   ├── treasure_hunt_game.py          # Main game orchestrator
│   ├── navigation_controller.py       # Navigation to/from target
│   └── treasure_generator.py          # Treasure probability logic
├── launch/
│   └── treasure_hunt.launch.py        # ROS2 launch file
├── config/
│   └── game_params.yaml               # Game configuration parameters
├── package.xml                        # ROS2 package manifest
├── setup.py                           # Python package setup
└── setup.cfg                          # Python setup configuration
```

## Requirements
- ROS2 (Humble or newer)
- Python 3.8+
- TurtleBot4
- Nav2 (Navigation2 stack)
- scipy

## Installation

### 1. Clone/Copy the Package
```bash
cd ~/ros2_ws/src
# Copy the turtlebot_treasure_hunt folder here
```

### 2. Install Dependencies
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select turtlebot_treasure_hunt
```

### 4. Source the Workspace
```bash
source install/setup.bash
```

## Running the Game

### Option 1: Using Launch File
```bash
ros2 launch turtlebot_treasure_hunt treasure_hunt.launch.py
```

### Option 2: With Custom Parameters
```bash
ros2 launch turtlebot_treasure_hunt treasure_hunt.launch.py num_rounds:=5 target_distance:=2.0
```

### Option 3: Direct Node Execution
```bash
ros2 run turtlebot_treasure_hunt treasure_hunt_game
```

## Configuration

Edit `config/game_params.yaml` to adjust:
- **num_rounds**: Number of treasure hunt rounds (default: 10)
- **treasure_probability**: Base probability of treasure (default: 0.5)
- **target_distance**: Distance to target in meters (default: 1.83m = 6 feet)

## Features
- ✅ Automated navigation to target and back
- ✅ Random treasure probability generation (10%-90%)
- ✅ Treasure outcome determination based on probability
- ✅ Game summary with success statistics
- ✅ Configurable via parameters
- ✅ Detailed logging for debugging
- ✅ ROS2 launch file for easy deployment

## Game Output
The game provides detailed console output including:
- Round-by-round progress
- Navigation status
- Treasure probability announcements
- Treasure found/not found status
- Final game statistics

Example output:
```
============================================================
🏴‍☠️  PIRATE TREASURE HUNT - TURTLEBOT4 EDITION  🏴‍☠️
Starting 10 rounds of treasure hunting!
============================================================

------------------------------------------------------------
🎮 ROUND 1/10
------------------------------------------------------------
📍 Step 1: Navigating 1.83m to the island...
✓ Reached target position!
🎲 Step 2: Announcing treasure probability...
⚓ TREASURE PROBABILITY: 65% ⚓
💰 TREASURE FOUND! 💰
🏠 Step 3: Returning to start position...
✓ Returned to start position!
✅ Round 1 completed successfully!
```

## Troubleshooting

### Navigation Fails
- Ensure Nav2 stack is running
- Check that the map is properly initialized
- Verify TurtleBot4 is localized on the map

### Import Errors
- Make sure all ROS2 dependencies are installed: `rosdep install --from-paths src --ignore-src -r -y`
- Verify you've sourced the ROS2 setup: `source /opt/ros/<distro>/setup.bash`

### Parameters Not Loading
- Check the YAML file syntax is correct
- Ensure the path to `game_params.yaml` is correct in the launch file

## Future Enhancements
- Add text-to-speech for treasure announcements
- Integrate with TurtleBot4 camera for person detection
- Add game difficulty levels
- Create a GUI for game monitoring
- Record treasure hunt statistics to a file

## License
Apache 2.0
