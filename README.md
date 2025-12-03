# Pirate Treasure Game

this is it boys

A ROS2-based interactive treasure hunting game for TurtleBot4 that combines robot movement, text-to-speech, and voice commands. The robot navigates to different islands, searches for treasure, and returns home, all controlled through voice commands.

## Features

- **Voice-Controlled Gameplay**: Use voice commands to select islands and control the game
- **Robot Movement**: Robot physically moves to different island positions (1.5m away at different angles)
- **Text-to-Speech**: Robot speaks all game events and prompts
- **Treasure Hunting**: Three islands with different treasure probabilities (30%, 50%, 70%)
- **Game Statistics**: Track rounds played and treasures found per island
- **Keyboard Fallback**: Falls back to keyboard input if voice recognition fails

## Game Mechanics

### Islands

- **Island 1**: 0° (straight ahead) - 30% treasure probability
- **Island 2**: +45° (right) - 50% treasure probability
- **Island 3**: -45° (left) - 70% treasure probability

All islands are 1.5 meters from the starting position.

### Game Flow

1. Robot asks which island you want to visit (via TTS)
2. You speak the island number (or type it)
3. Robot moves to the selected island
4. Robot searches for treasure (waits 3 seconds)
5. Robot announces if treasure was found
6. Robot returns to starting position
7. Statistics are displayed
8. You choose to continue, reset, or quit

## Prerequisites

### System Requirements

- ROS2 (tested on ROS2 Humble/Jazzy)
- TurtleBot4 or compatible robot
- Working microphone for voice commands
- Internet connection (for Google Speech Recognition API)

### Python Dependencies

Install the following Python packages:

```bash
pip3 install SpeechRecognition pyaudio
```

Or on Ubuntu/Debian:

```bash
sudo apt-get install python3-pyaudio python3-speechrecognition
```

### System Dependencies

For text-to-speech (Linux):

```bash
sudo apt-get install espeak
```

## Building the Package

```bash
cd ~/HRI-Final-project/turtlebot4_ws
colcon build --packages-select pirate_game
source install/setup.bash
```

## Running the Game

### Main Game

```bash
ros2 run pirate_game pirate_game
```

The game will:

- Welcome you with TTS
- Ask which island to visit (voice or keyboard)
- Move the robot to the island
- Search for treasure
- Return to starting position
- Ask if you want to continue, reset, or quit

### Voice Commands

**Island Selection:**

- Say: "one", "two", "three"
- Say: "1", "2", "3"
- Say: "island one", "island two", "island three"
- Say: "go to island one", etc.

**Game Control:**

- Continue: "continue", "c", "next", "play again"
- Reset: "reset", "r", "restart", "start over", "new game"
- Quit: "quit", "q", "exit", "stop", "end"

The game will retry voice recognition up to 5 times before falling back to keyboard input.

## Test Scripts

### Test Text-to-Speech

```bash
ros2 run pirate_game tts_test
```

Tests TTS functionality using espeak.

### Test Speech Recognition

```bash
ros2 run pirate_game speech_test
```

Tests speech recognition. Say something and it will be displayed. Say "quit" to exit.

**Note**: To suppress ALSA warnings, run:

```bash
ros2 run pirate_game speech_test 2>/dev/null
```

### Test Movement

```bash
ros2 run pirate_game movement_test
```

Tests robot movement: moves forward 2 seconds, turns around, returns to start.

### Test Navigation (Nav2)

```bash
ros2 run pirate_game test_nav
```

Tests Nav2 navigation (requires lidar and mapping).

## Package Structure

```
pirate_game/
├── pirate_game/
│   ├── game_logic.py          # Core game logic and island management
│   ├── game_state.py           # Game state tracking and statistics
│   ├── island.py               # Island class with position and probability
│   ├── pirate_game.py          # Main game class with voice/TTS integration
│   ├── simple_movement.py      # Robot movement controller (cmd_vel)
│   ├── speech_controller.py   # Speech recognition controller
│   ├── tts_controller.py       # Text-to-speech controller
│   ├── movement_test.py        # Movement test script
│   ├── speech_test.py          # Speech recognition test script
│   ├── tts_test.py             # TTS test script
│   └── simple_nav_test.py     # Nav2 navigation test
├── package.xml
├── setup.py
└── README.md
```

## Configuration

### Movement Speeds

Edit `simple_movement.py` to adjust:

- `forward_speed`: Default 0.3 m/s
- `angular_speed`: Default 0.7 rad/s
- `island_distance`: Default 1.5 meters

### Island Positions

Edit `game_logic.py` to change island positions:

- Island 1: `position_angle=0.0` (forward)
- Island 2: `position_angle=math.pi/4` (+45°)
- Island 3: `position_angle=-math.pi/4` (-45°)

### Treasure Probabilities

Edit `game_logic.py` to change probabilities:

- Island 1: 0.3 (30%)
- Island 2: 0.5 (50%)
- Island 3: 0.7 (70%)

## Troubleshooting

### Speech Recognition Not Working

- Check microphone is connected and working
- Ensure internet connection (Google Speech Recognition requires internet)
- Verify SpeechRecognition and pyaudio are installed
- Try running `speech_test` to debug

### TTS Not Working

- Install espeak: `sudo apt-get install espeak`
- Check audio output is working

### Movement Not Working

- Ensure robot is powered on and ROS2 is running
- Check `/cmd_vel` topic is available: `ros2 topic list`
- Verify robot base controller is running

### ALSA Warnings

ALSA warnings are harmless but noisy. Suppress them by running:

```bash
ros2 run pirate_game pirate_game 2>/dev/null
```

## License

TODO: License declaration
