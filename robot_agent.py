import heapq
from typing import List, Tuple, Optional, Dict, Set
import numpy as np

class RobotAgent:
    """
    Represents a robot agent capable of moving in a 2D grid environment
    and finding paths using the A* search algorithm.
    """

    def __init__(self, start_pos: Tuple[int, int], goal_pos: Tuple[int, int]):
        """
        Initialize the robot's position, goal, and path history.

        Args:
            start_pos (Tuple[int, int]): (y, x) starting coordinates.
            goal_pos (Tuple[int, int]): (y, x) goal coordinates.
        """
        self.y, self.x = start_pos
        self.goal_y, self.goal_x = goal_pos
        self.path: List[Tuple[int, int]] = [(self.y, self.x)]

    def is_valid_move(self, new_y: int, new_x: int, grid: np.ndarray) -> bool:
        """
        Check if a move to (new_y, new_x) is within bounds and not blocked.

        Args:
            new_y (int): Target y-coordinate.
            new_x (int): Target x-coordinate.
            grid (np.ndarray): 2D grid representing obstacles.

        Returns:
            bool: True if the move is valid, False otherwise.
        """
        rows, cols = grid.shape
        if (new_y < 0 or new_y >= rows or
            new_x < 0 or new_x >= cols or
            grid[new_y, new_x] == 1):
            return False
        return True

    def move(self, direction: str, grid: np.ndarray) -> None:
        """
        Move the robot one step in the specified direction if possible.

        Args:
            direction (str): "up", "down", "left", or "right".
            grid (np.ndarray): 2D grid representing obstacles.
        """
        direction = direction.lower()
        temp_y, temp_x = self.y, self.x

        if direction == "up":
            temp_y -= 1
        elif direction == "down":
            temp_y += 1
        elif direction == "left":
            temp_x -= 1
        elif direction == "right":
            temp_x += 1

        if self.is_valid_move(temp_y, temp_x, grid):
            self.y, self.x = temp_y, temp_x
            self.path.append((self.y, self.x))
            print(f"Moved to: ({self.y}, {self.x})")
        else:
            print("BUMP! Collision detected.")

    def has_reached_goal(self) -> bool:
        """
        Check if the robot has reached its goal position.

        Returns:
            bool: True if goal is reached, False otherwise.
        """
        return (self.y == self.goal_y) and (self.x == self.goal_x)

    def a_star_search(self, grid: np.ndarray) -> Optional[List[Tuple[int, int]]]:
        """
        Perform A* search to find a path from start to goal.

        Args:
            grid (np.ndarray): 2D grid representing obstacles.

        Returns:
            Optional[List[Tuple[int, int]]]: List of (y, x) tuples representing the path, or None if no path.
        """
        open_set: List[Tuple[int, int, int]] = []
        start: Tuple[int, int] = (self.y, self.x)
        goal: Tuple[int, int] = (self.goal_y, self.goal_x)

        # Push the start node with f_score = 0
        # Heap elements: (f_score, y, x)
        heapq.heappush(open_set, (0, start[0], start[1]))

        g_score: Dict[Tuple[int, int], int] = {start: 0}
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}

        while open_set:
            current_f, curr_y, curr_x = heapq.heappop(open_set)
            current = (curr_y, curr_x)

            if current == goal:
                print("Path Found!")
                return self.reconstruct_path(came_from, current)

            # Neighbors: right, left, down, up
            directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]

            for dy, dx in directions:
                neighbor_y = current[0] + dy
                neighbor_x = current[1] + dx
                neighbor = (neighbor_y, neighbor_x)

                if self.is_valid_move(neighbor_y, neighbor_x, grid):
                    tentative_g_score = g_score[current] + 1

                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score

                        # Manhattan distance heuristic
                        h_score = abs(neighbor[0] - goal[0]) + abs(neighbor[1] - goal[1])
                        f_score = tentative_g_score + h_score
                        heapq.heappush(open_set, (f_score, neighbor[0], neighbor[1]))

        return None

    def reconstruct_path(self, came_from: Dict[Tuple[int, int], Tuple[int, int]], current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Reconstruct the path from start to goal.

        Args:
            came_from (Dict): Map of node -> parent node.
            current (Tuple): Goal node coordinates.

        Returns:
            List[Tuple[int, int]]: Ordered list of (y, x) tuples forming the path.
        """
        total_path = []

        while current in came_from:
            total_path.append(current)
            current = came_from[current]

        total_path.append(current)  # Add start node
        return total_path[::-1]     # Reverse to get start -> goal
