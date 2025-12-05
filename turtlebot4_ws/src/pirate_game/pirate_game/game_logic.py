"""
GameLogic class for core game mechanics.
Handles island management and treasure checking.
"""
import random
import math

from pirate_game.island import Island
from pirate_game.game_state import GameState


class GameLogic:
    """Core game logic for the pirate treasure game."""
    
    def __init__(self):
        """Initialize game logic with 3 islands and hardcoded probabilities."""
        # Hardcoded probabilities: Island 1: 0.3, Island 2: 0.5, Island 3: 0.7
        # Position angles: Island 1: 0° (forward), Island 2: +45°, Island 3: -45°
        self.islands = [
            Island(1, "Island 1", 0.3, position_angle=0.0),  # 0 radians = forward (1.5m)
            Island(2, "Island 2", 0.5, position_angle=math.pi/4),  # +45 degrees (π/4 radians)
            Island(3, "Island 3", 0.7, position_angle=-math.pi/4)   # -45 degrees (-π/4 radians)
        ]
        self.game_state = GameState()
    
    def select_island(self, island_id: int):
        """
        Validate and return the Island object for the given ID.
        
        Args:
            island_id: The island ID to select (1-3)
            
        Returns:
            Island: The selected Island object
            
        Raises:
            ValueError: If island_id is not 1, 2, or 3
        """
        if island_id not in [1, 2, 3]:
            raise ValueError(f"Invalid island_id: {island_id}. Must be 1, 2, or 3.")
        
        return self.islands[island_id - 1]  # Convert to 0-based index
    
    def check_for_treasure(self, island: Island):
        """
        Check if treasure is found at the given island based on probability.
        
        Args:
            island: The Island object to check
            
        Returns:
            bool: True if treasure is found, False otherwise
        """
        return random.random() < island.treasure_probability
    
    def play_round(self, island_id: int):
        """
        Execute one round of the game.
        
        Args:
            island_id: The island to search (1-3)
            
        Returns:
            tuple: (island, treasure_found) where island is the Island object
                   and treasure_found is a boolean
        """
        # Start a new round
        self.game_state.start_round()
        
        # Select the island
        island = self.select_island(island_id)
        
        # Check for treasure
        treasure_found = self.check_for_treasure(island)
        
        # Record the result
        self.game_state.record_treasure_find(island_id, treasure_found)
        
        return island, treasure_found
    
    def reset_game(self):
        """Reset the game state."""
        self.game_state.reset()

