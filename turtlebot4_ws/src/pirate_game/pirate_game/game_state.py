"""
GameState class for tracking game progress and statistics.
"""


class GameState:
    """Tracks game progress, rounds, and treasure statistics."""
    
    def __init__(self):
        """Initialize game state with empty statistics."""
        self.current_round = 0
        self.total_rounds = 0
        self.treasures_found = {1: 0, 2: 0, 3: 0}  # Map island_id -> count
        self.round_history = []  # List of (round_num, island_id, treasure_found) tuples
    
    def start_round(self):
        """Increment the round counter."""
        self.current_round += 1
        self.total_rounds += 1
    
    def record_treasure_find(self, island_id: int, found: bool):
        """
        Record the result of a treasure hunt.
        
        Args:
            island_id: The island that was searched (1-3)
            found: Whether treasure was found
        """
        if island_id not in [1, 2, 3]:
            raise ValueError("island_id must be 1, 2, or 3")
        
        if found:
            self.treasures_found[island_id] += 1
        
        self.round_history.append((self.current_round, island_id, found))
    
    def reset(self):
        """Reset all game state to initial values."""
        self.current_round = 0
        self.total_rounds = 0
        self.treasures_found = {1: 0, 2: 0, 3: 0}
        self.round_history = []
    
    def get_statistics(self):
        """
        Get a summary of game statistics.
        
        Returns:
            dict: Statistics including total rounds, treasures per island, etc.
        """
        total_treasures = sum(self.treasures_found.values())
        
        return {
            'current_round': self.current_round,
            'total_rounds': self.total_rounds,
            'treasures_found': self.treasures_found.copy(),
            'total_treasures': total_treasures,
            'round_history': self.round_history.copy()
        }

