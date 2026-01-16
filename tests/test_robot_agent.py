import unittest
import numpy as np
from robot_agent import RobotAgent

class TestRobotAgent(unittest.TestCase):
    def setUp(self):
        self.grid = np.zeros((10, 10), dtype=int)
        self.grid[0, :] = 1
        self.grid[-1, :] = 1
        self.grid[:, 0] = 1
        self.grid[:, -1] = 1
        
        # Simple obstacle
        self.grid[5, 5] = 1

    def test_initialization(self):
        bot = RobotAgent((1, 1), (8, 8))
        self.assertEqual(bot.y, 1)
        self.assertEqual(bot.x, 1)
        self.assertEqual(bot.goal_y, 8)
        self.assertEqual(bot.goal_x, 8)
        self.assertEqual(bot.path, [(1, 1)])

    def test_is_valid_move(self):
        bot = RobotAgent((1, 1), (8, 8))
        # Valid move
        self.assertTrue(bot.is_valid_move(1, 2, self.grid))
        # Obstacle
        self.assertFalse(bot.is_valid_move(5, 5, self.grid))
        # Out of bounds
        self.assertFalse(bot.is_valid_move(-1, 0, self.grid))
        self.assertFalse(bot.is_valid_move(0, 0, self.grid)) # Wall

    def test_move(self):
        bot = RobotAgent((1, 1), (8, 8))
        bot.move("right", self.grid)
        self.assertEqual(bot.y, 1)
        self.assertEqual(bot.x, 2)
        self.assertEqual(bot.path[-1], (1, 2))

        # Try hitting a wall
        bot.y, bot.x = 1, 1 # Reset
        bot.move("up", self.grid) # Should hit Top wall (y=0 is wall)
        self.assertEqual(bot.y, 1) # Should not have moved
        self.assertEqual(bot.x, 1)

    def test_a_star_success(self):
        bot = RobotAgent((1, 1), (2, 2))
        path = bot.a_star_search(self.grid)
        self.assertIsNotNone(path)
        self.assertEqual(path[0], (1, 1))
        self.assertEqual(path[-1], (2, 2))

    def test_a_star_no_path(self):
        # Surround (8,8) with walls
        self.grid[7, 8] = 1
        self.grid[9, 8] = 1
        self.grid[8, 7] = 1
        self.grid[8, 9] = 1
        
        bot = RobotAgent((1, 1), (8, 8))
        path = bot.a_star_search(self.grid)
        self.assertIsNone(path)

if __name__ == '__main__':
    unittest.main()
