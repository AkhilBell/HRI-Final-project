"""
Main Pirate Game class with CLI interface and robot movement.
"""
import rclpy
from rclpy.node import Node
import time

from pirate_game.game_logic import GameLogic
from pirate_game.simple_movement import SimpleMovement
from pirate_game.tts_controller import TTSController


class PirateGame(Node):
    """Main game class with CLI interface and robot movement."""
    
    def __init__(self):
        """Initialize the game with game logic and movement controller."""
        super().__init__('pirate_game')
        self.game_logic = GameLogic()
        self.movement = SimpleMovement()
        self.tts = TTSController(logger=self.get_logger())
        self.get_logger().info("Pirate game initialized with movement controller and TTS")
    
    def _number_to_word(self, num):
        """Convert number to word for better TTS."""
        words = {1: "one", 2: "two", 3: "three", 4: "four", 5: "five",
                 6: "six", 7: "seven", 8: "eight", 9: "nine", 10: "ten"}
        return words.get(num, str(num))
    
    def _get_island_selection(self):
        """
        Prompt user for island selection.
        
        Returns:
            int: Selected island ID (1-3)
        """
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
            self.tts.speak(f"Round {round_word} complete. Island {island_word}. Treasure found!")
        else:
            self.tts.speak(f"Round {round_word} complete. Island {island_word}. No treasure at this location.")
    
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
                # Ask user which island they want to go to (TTS)
                self.tts.speak("Which island would you like to go to? Choose island one, two, or three.")
                
                # Get island selection
                island_id = self._get_island_selection()
                island = self.game_logic.select_island(island_id)
                
                # Start a new round
                self.game_logic.game_state.start_round()
                
                # Step 1: Move robot to island
                print(f"\n🚢 Navigating to {island.name}...")
                self.get_logger().info(f"Moving to Island {island_id}")
                island_word = self._number_to_word(island_id)
                self.tts.speak(f"Navigating to Island {island_word}")
                try:
                    self.movement.go_to_island(island_id, island.position_angle)
                    # TTS announcement for arrival
                    self.tts.speak(f"Arrived at Island {island_word}")
                except Exception as e:
                    self.get_logger().error(f"Movement error: {e}")
                    print(f"⚠️  Movement error: {e}. Continuing with treasure check...")
                
                # Step 2: Wait 3 seconds at island and check for treasure
                print(f"\n🔍 Searching for treasure at {island.name}...")
                print("(Waiting 3 seconds...)")
                self.get_logger().info("Waiting 3 seconds at island to check for treasure")
                self.tts.speak(f"Searching for treasure at Island {island_word}")
                
                # Check for treasure during the wait
                treasure_found = self.game_logic.check_for_treasure(island)
                
                # Wait the full 3 seconds
                time.sleep(3.0)
                
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
                
                # Ask to continue, reset, or quit
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

