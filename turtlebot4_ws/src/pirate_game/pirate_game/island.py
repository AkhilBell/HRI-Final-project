"""
Island class for the Pirate Treasure Game.
Represents an island with a treasure probability.
"""


class Island:
    """Represents an island with a treasure probability."""
    
    def __init__(self, island_id: int, name: str = None, treasure_probability: float = 0.5):
        """
        Initialize an Island.
        
        Args:
            island_id: Unique identifier for the island (1-3)
            name: Optional name for the island
            treasure_probability: Probability of finding treasure (0.0-1.0)
        """
        if not (1 <= island_id <= 3):
            raise ValueError("island_id must be between 1 and 3")
        if not (0.0 <= treasure_probability <= 1.0):
            raise ValueError("treasure_probability must be between 0.0 and 1.0")
        
        self.id = island_id
        self.name = name if name else f"Island {island_id}"
        self.treasure_probability = treasure_probability
    
    def __repr__(self):
        """String representation of the Island."""
        return f"Island(id={self.id}, name='{self.name}', probability={self.treasure_probability})"

