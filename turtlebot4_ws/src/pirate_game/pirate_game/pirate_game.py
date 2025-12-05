"""
Main Pirate Game class with CLI interface and robot movement.
"""
import rclpy
from rclpy.node import Node
import time
import threading
import queue

from pirate_game.game_logic import GameLogic
from pirate_game.simple_movement import SimpleMovement
from pirate_game.tts_controller import TTSController
from pirate_game.speech_controller import SpeechRecognitionController


class PirateGame(Node):
    """Main game class with CLI interface and robot movement."""
    
    def __init__(self):
        """Initialize the game with game logic and movement controller."""
        super().__init__('pirate_game')
        self.game_logic = GameLogic()
        self.movement = SimpleMovement()
        self.tts = TTSController(logger=self.get_logger())
        self.speech = SpeechRecognitionController(logger=self.get_logger())
        self.get_logger().info("Pirate game initialized with movement controller, TTS, and speech recognition")
    
    def _number_to_word(self, num):
        """Convert number to word for better TTS."""
        words = {1: "one", 2: "two", 3: "three", 4: "four", 5: "five",
                 6: "six", 7: "seven", 8: "eight", 9: "nine", 10: "ten"}
        return words.get(num, str(num))
    
    def _get_island_selection(self):
        """
        Get island selection via voice command with keyboard fallback.
        Keyboard input can override voice input at any time.
        
        Returns:
            int: Selected island ID (1-3)
        """
        max_speech_attempts = 3
        speech_attempt = 0
        
        # Queue to receive keyboard input from thread
        keyboard_queue = queue.Queue()
        keyboard_done = threading.Event()
        
        def keyboard_listener():
            """Thread function to listen for keyboard input."""
            try:
                selection = input("Select an island (1, 2, or 3): ").strip()
                keyboard_queue.put(selection)
            except EOFError:
                keyboard_queue.put(None)
            finally:
                keyboard_done.set()
        
        # Try speech recognition with keyboard override
        if self.speech.available:
            self.tts.speak("Listening for island selection. Say island one, island two, or island three.")
            print("\n🎤 Listening for voice command...")
            print("💡 Tip: Type a number (1, 2, or 3) at any time to skip voice input")
            print("   Or press Enter to immediately switch to keyboard input\n")
            
            # Start keyboard listener thread immediately
            keyboard_thread = threading.Thread(target=keyboard_listener, daemon=True)
            keyboard_thread.start()
            
            while speech_attempt < max_speech_attempts:
                # Check if keyboard input is available (non-blocking)
                try:
                    selection = keyboard_queue.get_nowait()
                    keyboard_done.set()
                    
                    # Empty input means user wants to skip voice and use keyboard
                    if selection == "" or selection is None:
                        print("⏭️  Skipping voice input, switching to keyboard...")
                        break  # Exit speech loop and go to keyboard input
                    
                    # Process keyboard input
                    try:
                        island_id = int(selection)
                        if island_id in [1, 2, 3]:
                            print(f"✅ Keyboard input: Island {island_id}")
                            return island_id
                        else:
                            print("Invalid selection. Please enter 1, 2, or 3.")
                            # Restart keyboard listener
                            keyboard_thread = threading.Thread(target=keyboard_listener, daemon=True)
                            keyboard_thread.start()
                    except (ValueError, TypeError):
                        print("Invalid input. Please enter a number (1, 2, or 3).")
                        # Restart keyboard listener
                        keyboard_thread = threading.Thread(target=keyboard_listener, daemon=True)
                        keyboard_thread.start()
                except queue.Empty:
                    pass  # No keyboard input yet, continue with speech
                
                # Listen for speech with shorter timeout to check keyboard more frequently
                text = self.speech.listen(timeout=3)
                
                # Check keyboard again after speech listening
                try:
                    selection = keyboard_queue.get_nowait()
                    keyboard_done.set()
                    
                    # Empty input means user wants to skip voice and use keyboard
                    if selection == "" or selection is None:
                        print("⏭️  Skipping voice input, switching to keyboard...")
                        break  # Exit speech loop and go to keyboard input
                    
                    # Process keyboard input
                    try:
                        island_id = int(selection)
                        if island_id in [1, 2, 3]:
                            print(f"✅ Keyboard input: Island {island_id}")
                            return island_id
                        else:
                            print("Invalid selection. Please enter 1, 2, or 3.")
                            # Restart keyboard listener
                            keyboard_thread = threading.Thread(target=keyboard_listener, daemon=True)
                            keyboard_thread.start()
                    except (ValueError, TypeError):
                        print("Invalid input. Please enter a number (1, 2, or 3).")
                        # Restart keyboard listener
                        keyboard_thread = threading.Thread(target=keyboard_listener, daemon=True)
                        keyboard_thread.start()
                except queue.Empty:
                    pass  # No keyboard input, process speech result
                
                if text:
                    island_id = self.speech.parse_island_command(text)
                    if island_id:
                        keyboard_done.set()  # Cancel keyboard listener
                        island_word = self._number_to_word(island_id)
                        print(f"✅ Voice command recognized: Island {island_id}")
                        return island_id
                    else:
                        speech_attempt += 1
                        if speech_attempt < max_speech_attempts:
                            self.tts.speak("I didn't understand. Please say one, two, or three.")
                            print(f"❓ Could not parse command: '{text}'. Please try again. ({speech_attempt}/{max_speech_attempts})")
                        else:
                            print(f"❓ Could not parse command: '{text}'. Switching to keyboard input.")
                else:
                    speech_attempt += 1
                    if speech_attempt < max_speech_attempts:
                        self.tts.speak("I didn't hear anything. Please try again.")
                        print(f"⏱️  No speech detected. Retrying... ({speech_attempt}/{max_speech_attempts})")
                    else:
                        print("⏱️  No speech detected. Switching to keyboard input.")
            
            keyboard_done.set()  # Cancel keyboard listener if speech attempts exhausted
        
        # Fallback to keyboard input after speech attempts exhausted or if speech not available
        if not keyboard_done.is_set():
            keyboard_done.set()  # Cancel any existing keyboard thread
        
        print("\nUsing keyboard input for island selection...")
        while True:
            try:
                selection = input("Select an island (1, 2, or 3): ").strip()
                island_id = int(selection)
                
                if island_id in [1, 2, 3]:
                    return island_id
                else:
                    print("Invalid selection. Please enter 1, 2, or 3.")
            except ValueError:
                print("Invalid input. Please enter a number (1, 2, or 3).")
    
    def _display_round_result(self, island, treasure_found):
        """
        Display the result of a round.
        
        Args:
            island: The Island object that was searched
            treasure_found: Whether treasure was found
        """
        print(f"\n{'='*60}")
        print(f"Round {self.game_logic.game_state.current_round} Results")
        print(f"{'='*60}")
        print(f"Island: {island.name} (Probability: {island.treasure_probability*100:.0f}%)")
        
        if treasure_found:
            print("💰 TREASURE FOUND! 💰")
        else:
            print("No treasure at this location.")
        print(f"{'='*60}\n")
        self.get_logger().info(f"Round {self.game_logic.game_state.current_round}: {island.name} - {'Treasure found!' if treasure_found else 'No treasure'}")
        
        # TTS announcement for treasure result
        round_word = self._number_to_word(self.game_logic.game_state.current_round)
        island_word = self._number_to_word(island.id)
        if treasure_found:
            self.tts.speak(f"Treasure found!")
        else:
            self.tts.speak(f"No treasure at this location.")
    
    def _display_statistics(self):
        """Display current game statistics."""
        stats = self.game_logic.game_state.get_statistics()
        
        print(f"\n{'='*60}")
        print("Game Statistics")
        print(f"{'='*60}")
        print(f"Total Rounds: {stats['total_rounds']}")
        print(f"Total Treasures Found: {stats['total_treasures']}")
        print(f"\nTreasures per Island:")
        for island_id in [1, 2, 3]:
            island = self.game_logic.islands[island_id - 1]
            count = stats['treasures_found'][island_id]
            print(f"  {island.name}: {count}")
        print(f"{'='*60}\n")
    
    def run(self):
        """Main game loop with CLI prompts and robot movement."""
        print("\n" + "="*60)
        print("🏴‍☠️  PIRATE TREASURE GAME  🏴‍☠️")
        print("="*60)
        print("Welcome! Search for treasure on 3 islands.")
        print("Each island has a different probability of treasure.")
        print("The robot will move to each island you select!")
        print("="*60 + "\n")
        
        # TTS welcome message
        self.tts.speak("Welcome to the pirate treasure game. Search for treasure on 3 islands.")
        
        while True:
            try:
                # Get island selection (includes TTS prompt)
                island_id = self._get_island_selection()
                island = self.game_logic.select_island(island_id)
                
                # Start a new round
                self.game_logic.game_state.start_round()
                
                # Step 1: Move robot to island
                print(f"\n🚢 Navigating to {island.name}...")
                self.get_logger().info(f"Moving to Island {island_id}")
                island_word = self._number_to_word(island_id)
                self.tts.speak("Aye aye captain")
                try:
                    self.movement.go_to_island(island_id, island.position_angle)
                    # TTS announcement for arrival
                    self.tts.speak(f"Arrived at Island {island_word}")
                except Exception as e:
                    self.get_logger().error(f"Movement error: {e}")
                    print(f"⚠️  Movement error: {e}. Continuing with treasure check...")
                
                # Step 2: Wait 1.5 seconds at island and check for treasure
                print(f"\n🔍 Searching for treasure at {island.name}...")
                print("(Waiting 1.5 seconds...)")
                self.get_logger().info("Waiting 1.5 seconds at island to check for treasure")
                
                # Check for treasure during the wait
                treasure_found = self.game_logic.check_for_treasure(island)
                
                # Wait the full 1.5 seconds
                time.sleep(1.5)
                
                # Record the result
                self.game_logic.game_state.record_treasure_find(island_id, treasure_found)
                
                # Display results
                self._display_round_result(island, treasure_found)
                
                # Step 3: Return robot to starting position
                print(f"\n🏠 Returning to starting position...")
                self.get_logger().info("Returning to starting position")
                self.tts.speak("Returning to starting position")
                try:
                    self.movement.return_to_start(island.position_angle)
                    # TTS announcement for return complete
                    self.tts.speak("Returned to starting position")
                except Exception as e:
                    self.get_logger().error(f"Return movement error: {e}")
                    print(f"⚠️  Return movement error: {e}. Continuing...")
                
                # Display statistics
                self._display_statistics()
                
                # Only ask to continue/reset/quit after the 10th round
                if self.game_logic.game_state.current_round < 10:
                    # Automatically continue to next round
                    print(f"\nRound {self.game_logic.game_state.current_round} complete. Continuing to next round...\n")
                    continue
                
                # Ask to continue, reset, or quit (voice or keyboard) after 10th round
                max_speech_attempts = 1
                speech_attempt = 0
                command_handled = False
                
                # Queue to receive keyboard input from thread
                keyboard_queue = queue.Queue()
                keyboard_done = threading.Event()
                
                def keyboard_listener():
                    """Thread function to listen for keyboard input."""
                    try:
                        choice = input("Continue (c), Reset (r), or Quit (q)? ").strip().lower()
                        keyboard_queue.put(choice)
                    except EOFError:
                        keyboard_queue.put(None)
                    finally:
                        keyboard_done.set()
                
                # Try speech recognition with keyboard override
                if self.speech.available:
                    self.tts.speak("Say continue to play again, reset to start over, or quit to exit.")
                    print("\n🎤 Listening for command... (or type c/r/q to override)")
                    
                    # Start keyboard listener thread
                    keyboard_thread = threading.Thread(target=keyboard_listener, daemon=True)
                    keyboard_thread.start()
                    
                    while speech_attempt < max_speech_attempts and not command_handled:
                        # Check if keyboard input is available (non-blocking)
                        try:
                            choice = keyboard_queue.get_nowait()
                            keyboard_done.set()
                            # Process keyboard input
                            if choice in ['c', 'continue']:
                                print("✅ Keyboard input: Continue")
                                command_handled = True
                                break
                            elif choice in ['r', 'reset']:
                                self.game_logic.reset_game()
                                print("\nGame reset! Starting fresh...\n")
                                self.tts.speak("Game reset. Starting fresh")
                                command_handled = True
                                break
                            elif choice in ['q', 'quit']:
                                print("\nThanks for playing! 🏴‍☠️\n")
                                self.tts.speak("Thanks for playing")
                                return
                            else:
                                print("Invalid choice. Please enter 'c' (continue), 'r' (reset), or 'q' (quit).")
                                # Restart keyboard listener
                                keyboard_thread = threading.Thread(target=keyboard_listener, daemon=True)
                                keyboard_thread.start()
                        except queue.Empty:
                            pass  # No keyboard input yet, continue with speech
                        
                        # Listen for speech with timeout
                        text = self.speech.listen(timeout=8)
                        
                        # Check keyboard again after speech listening
                        try:
                            choice = keyboard_queue.get_nowait()
                            keyboard_done.set()
                            # Process keyboard input
                            if choice in ['c', 'continue']:
                                print("✅ Keyboard input: Continue")
                                command_handled = True
                                break
                            elif choice in ['r', 'reset']:
                                self.game_logic.reset_game()
                                print("\nGame reset! Starting fresh...\n")
                                self.tts.speak("Game reset. Starting fresh")
                                command_handled = True
                                break
                            elif choice in ['q', 'quit']:
                                print("\nThanks for playing! 🏴‍☠️\n")
                                self.tts.speak("Thanks for playing")
                                return
                            else:
                                print("Invalid choice. Please enter 'c' (continue), 'r' (reset), or 'q' (quit).")
                                # Restart keyboard listener
                                keyboard_thread = threading.Thread(target=keyboard_listener, daemon=True)
                                keyboard_thread.start()
                        except queue.Empty:
                            pass  # No keyboard input, process speech result
                        
                        if text:
                            command = self.speech.parse_game_command(text)
                            if command == "continue":
                                keyboard_done.set()  # Cancel keyboard listener
                                print("✅ Voice command: Continue")
                                command_handled = True
                                break
                            elif command == "reset":
                                keyboard_done.set()  # Cancel keyboard listener
                                self.game_logic.reset_game()
                                print("\nGame reset! Starting fresh...\n")
                                self.tts.speak("Game reset. Starting fresh")
                                command_handled = True
                                break
                            elif command == "quit":
                                keyboard_done.set()  # Cancel keyboard listener
                                print("\nThanks for playing! 🏴‍☠️\n")
                                self.tts.speak("Thanks for playing")
                                return
                            else:
                                speech_attempt += 1
                                if speech_attempt < max_speech_attempts:
                                    self.tts.speak("I didn't understand. Please say continue, reset, or quit.")
                                    print(f"❓ Could not parse command: '{text}'. Please try again. ({speech_attempt}/{max_speech_attempts})")
                                else:
                                    print(f"❓ Could not parse command: '{text}'. Switching to keyboard input.")
                        else:
                            speech_attempt += 1
                            if speech_attempt < max_speech_attempts:
                                print(f"⏱️  No speech detected. Retrying... ({speech_attempt}/{max_speech_attempts})")
                            else:
                                print("⏱️  No speech detected. Switching to keyboard input.")
                    
                    keyboard_done.set()  # Cancel keyboard listener if speech attempts exhausted
                    
                    # If command was handled, continue to next round
                    if command_handled:
                        continue
                
                # Fallback to keyboard input after speech attempts exhausted or if speech not available
                if not keyboard_done.is_set():
                    keyboard_done.set()  # Cancel any existing keyboard thread
                
                print("\nUsing keyboard input...")
                while True:
                    choice = input("Continue (c), Reset (r), or Quit (q)? ").strip().lower()
                    if choice in ['c', 'continue']:
                        break
                    elif choice in ['r', 'reset']:
                        self.game_logic.reset_game()
                        print("\nGame reset! Starting fresh...\n")
                        self.tts.speak("Game reset. Starting fresh")
                        break
                    elif choice in ['q', 'quit']:
                        print("\nThanks for playing! 🏴‍☠️\n")
                        self.tts.speak("Thanks for playing")
                        return
                    else:
                        print("Invalid choice. Please enter 'c' (continue), 'r' (reset), or 'q' (quit).")
                
            except KeyboardInterrupt:
                print("\n\nGame interrupted. Thanks for playing! 🏴‍☠️\n")
                self.movement.stop()
                self.tts.speak("Thanks for playing")
                return
            except Exception as e:
                self.get_logger().error(f"Error in game loop: {e}")
                print(f"\nError: {e}\n")
                self.movement.stop()
                continue


def main(args=None):
    """Main entry point for the game."""
    rclpy.init(args=args)
    
    game = PirateGame()
    
    try:
        game.run()
    except KeyboardInterrupt:
        game.get_logger().info("\nGame interrupted by user")
        game.movement.stop()
    except Exception as e:
        game.get_logger().error(f"Fatal error: {e}")
        game.movement.stop()
    finally:
        game.movement.destroy_node()
        game.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

