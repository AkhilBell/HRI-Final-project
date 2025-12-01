"""
Main Treasure Hunt Game Node for TurtleBot4
Orchestrates the complete game loop: navigate -> announce probability -> return -> repeat
"""
import rclpy
from rclpy.node import Node
import time
import sys

from turtlebot_treasure_hunt.navigation_controller import NavigationController
from turtlebot_treasure_hunt.treasure_generator import TreasureGenerator


class TreasureHuntGame(Node):
    def __init__(self):
        super().__init__('treasure_hunt_game')
        
        # Parameters
        self.declare_parameter('num_rounds', 10)
        self.declare_parameter('treasure_probability', 0.5)
        self.declare_parameter('target_distance', 1.83)
        
        self.num_rounds = self.get_parameter('num_rounds').value
        self.treasure_prob = self.get_parameter('treasure_probability').value
        self.target_distance = self.get_parameter('target_distance').value
        
        # Initialize sub-systems
        self.nav_controller = NavigationController()
        self.treasure_gen = TreasureGenerator()
        
        self.current_round = 0
        self.successful_rounds = 0
        
    def run_game(self):
        """Main game loop"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("🏴‍☠️  PIRATE TREASURE HUNT - TURTLEBOT4 EDITION  🏴‍☠️")
        self.get_logger().info(f"Starting {self.num_rounds} rounds of treasure hunting!")
        self.get_logger().info("=" * 60)
        
        for round_num in range(1, self.num_rounds + 1):
            self.current_round = round_num
            self.get_logger().info("\n" + "-" * 60)
            self.get_logger().info(f"🎮 ROUND {round_num}/{self.num_rounds}")
            self.get_logger().info("-" * 60)
            
            try:
                # Step 1: Navigate to target
                self.get_logger().info(f"📍 Step 1: Navigating {self.target_distance}m to the island...")
                nav_success = self.nav_controller.go_to_target()
                
                if not nav_success:
                    self.get_logger().warn(f"⚠️  Round {round_num}: Navigation failed, skipping to next round")
                    continue
                
                # Small delay for robot to settle
                time.sleep(1)
                
                # Step 2: Calculate and announce treasure probability
                probability = self.treasure_gen.calculate_treasure_probability()
                self.get_logger().info(f"🎲 Step 2: Announcing treasure probability...")
                self.treasure_gen.announce_treasure_probability(probability)
                
                # Step 3: Announce if treasure is found (based on probability)
                treasure_found = self.treasure_gen.get_treasure_status(probability)
                if treasure_found:
                    self.get_logger().info("💰 TREASURE FOUND! 💰")
                else:
                    self.get_logger().info("No treasure at this location")
                
                time.sleep(2)  # Give time for announcement to be heard
                
                # Step 4: Return to start
                self.get_logger().info(f"🏠 Step 3: Returning to start position...")
                return_success = self.nav_controller.return_to_start()
                
                if return_success:
                    self.successful_rounds += 1
                    self.get_logger().info(f"✅ Round {round_num} completed successfully!")
                else:
                    self.get_logger().warn(f"⚠️  Round {round_num}: Return navigation failed")
                
                # Delay between rounds
                if round_num < self.num_rounds:
                    self.get_logger().info(f"⏳ Preparing for next round...")
                    time.sleep(2)
                    
            except Exception as e:
                self.get_logger().error(f"❌ Error during round {round_num}: {str(e)}")
                continue
        
        # Game summary
        self.print_game_summary()
    
    def print_game_summary(self):
        """Print final game statistics"""
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("🏁 GAME COMPLETE - FINAL RESULTS 🏁")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Total Rounds: {self.num_rounds}")
        self.get_logger().info(f"Successful Rounds: {self.successful_rounds}")
        self.get_logger().info(f"Success Rate: {(self.successful_rounds/self.num_rounds)*100:.1f}%")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Thanks for playing! 🏴‍☠️")
        self.get_logger().info("=" * 60 + "\n")


def main(args=None):
    rclpy.init(args=args)
    
    game = TreasureHuntGame()
    
    try:
        game.run_game()
    except KeyboardInterrupt:
        game.get_logger().info("\nGame interrupted by user")
    except Exception as e:
        game.get_logger().error(f"Fatal error: {str(e)}")
    finally:
        game.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
