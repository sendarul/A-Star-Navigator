import unittest
import numpy as np
from robot_agent import RobotAgent
from environment import Environment

class TestEnvironments(unittest.TestCase):
    def test_empty_grid(self):
        """Test pathfinding in an empty grid."""
        env = Environment.create_empty(10, 10)
        grid = env.get_grid()
        bot = RobotAgent((1, 1), (8, 8))
        path = bot.a_star_search(grid)
        self.assertIsNotNone(path)
        self.assertEqual(path[-1], (8, 8))

    def test_full_obstacle(self):
        """Test pathfinding in a grid full of obstacles."""
        env = Environment.create_full_obstacle(10, 10)
        grid = env.get_grid()
        # Ensure start and goal are walkable for the sake of the test setup,
        # but if they are surrounded, no path should be found.
        # Actually, if start is an obstacle, is_valid_move might fail immediately or A* might not start.
        # Let's just trust the grid is all 1s.
        bot = RobotAgent((1, 1), (8, 8))
        
        # In a full obstacle grid, start and goal are also obstacles by default logic of create_full_obstacle
        # RobotAgent logic says "grid[new_y, new_x] == 1" is invalid.
        # If start is invalid, A* neighbor expansion will fail immediately.
        path = bot.a_star_search(grid)
        self.assertIsNone(path)

    def test_impossible_path_solid_wall(self):
        """Test immediate wall blocking path."""
        env = Environment.create_empty(10, 10)
        grid = env.get_grid()
        # Create a solid vertical wall
        grid[:, 5] = 1
        
        bot = RobotAgent((1, 1), (1, 8)) # Start left of wall, goal right of wall
        path = bot.a_star_search(grid)
        self.assertIsNone(path)

    def test_no_boundary(self):
        """Test grid without perimeter walls."""
        env = Environment.create_no_boundary(10, 10)
        grid = env.get_grid()
        bot = RobotAgent((0, 0), (9, 9))
        path = bot.a_star_search(grid)
        self.assertIsNotNone(path)
        self.assertEqual(path[-1], (9, 9))
        
    def test_start_is_goal(self):
        """Test when start point is the same as goal."""
        env = Environment.create_empty(10, 10)
        grid = env.get_grid()
        bot = RobotAgent((5, 5), (5, 5))
        path = bot.a_star_search(grid)
        self.assertIsNotNone(path)
        self.assertEqual(len(path), 1)
        self.assertEqual(path[0], (5, 5))

if __name__ == '__main__':
    unittest.main()
