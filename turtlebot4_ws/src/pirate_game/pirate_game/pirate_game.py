"""
Main Pirate Game class with CLI interface.
"""
from pirate_game.game_logic import GameLogic


class PirateGame:
    """Main game class with CLI interface."""
    
    def __init__(self):
        """Initialize the game with game logic."""
        self.game_logic = GameLogic()
    
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
        """Main game loop with CLI prompts."""
        print("\n" + "="*60)
        print("🏴‍☠️  PIRATE TREASURE GAME  🏴‍☠️")
        print("="*60)
        print("Welcome! Search for treasure on 3 islands.")
        print("Each island has a different probability of treasure.")
        print("="*60 + "\n")
        
        while True:
            try:
                # Get island selection
                island_id = self._get_island_selection()
                
                # Play the round
                island, treasure_found = self.game_logic.play_round(island_id)
                
                # Display results
                self._display_round_result(island, treasure_found)
                
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
                        break
                    elif choice in ['q', 'quit']:
                        print("\nThanks for playing! 🏴‍☠️\n")
                        return
                    else:
                        print("Invalid choice. Please enter 'c' (continue), 'r' (reset), or 'q' (quit).")
                
            except KeyboardInterrupt:
                print("\n\nGame interrupted. Thanks for playing! 🏴‍☠️\n")
                return
            except Exception as e:
                print(f"\nError: {e}\n")
                continue


def main(args=None):
    """Main entry point for the game."""
    game = PirateGame()
    game.run()


if __name__ == '__main__':
    main()

