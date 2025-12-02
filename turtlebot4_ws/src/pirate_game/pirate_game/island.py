"""
Island class for the Pirate Treasure Game.
Represents an island with a treasure probability and position.
"""
import math


class Island:
    """Represents an island with a treasure probability and position."""
    
    def __init__(self, island_id: int, name: str = None, treasure_probability: float = 0.5, position_angle: float = 0.0):
        """
        Initialize an Island.
        
        Args:
            island_id: Unique identifier for the island (1-3)
            name: Optional name for the island
            treasure_probability: Probability of finding treasure (0.0-1.0)
            position_angle: Angle in radians from starting position (0 = forward)
        """
        if not (1 <= island_id <= 3):
            raise ValueError("island_id must be between 1 and 3")
        if not (0.0 <= treasure_probability <= 1.0):
            raise ValueError("treasure_probability must be between 0.0 and 1.0")
        
        self.id = island_id
        self.name = name if name else f"Island {island_id}"
        self.treasure_probability = treasure_probability
        self.position_angle = position_angle
    
    def __repr__(self):
        """String representation of the Island."""
        angle_deg = math.degrees(self.position_angle)
        return f"Island(id={self.id}, name='{self.name}', probability={self.treasure_probability}, angle={angle_deg:.1f}°)"

