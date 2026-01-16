import numpy as np
from typing import Tuple

class Environment:
    """
    Manages the grid environment dealing with obstacles and boundaries.
    """
    def __init__(self, rows: int = 20, cols: int = 20):
        self.rows = rows
        self.cols = cols
        self.grid = np.zeros((rows, cols), dtype=int)

    @classmethod
    def create_c_shape(cls) -> 'Environment':
        """Creates the default environment with a 'C' shaped obstacle."""
        env = cls(20, 20)
        # Perimeter
        env.grid[0, :] = 1
        env.grid[-1, :] = 1
        env.grid[:, 0] = 1
        env.grid[:, -1] = 1

        # 'C' Obstacle
        env.grid[3, 4:16] = 1
        env.grid[3:15, 4] = 1
        env.grid[14, 4:16] = 1
        return env

    @classmethod
    def create_empty(cls, rows=10, cols=10) -> 'Environment':
        """Creates an empty environment with boundaries."""
        env = cls(rows, cols)
        env.grid[0, :] = 1
        env.grid[-1, :] = 1
        env.grid[:, 0] = 1
        env.grid[:, -1] = 1
        return env

    @classmethod
    def create_full_obstacle(cls, rows=10, cols=10) -> 'Environment':
        """Creates an environment completely filled with obstacles (except potential start/goal)."""
        env = cls(rows, cols)
        env.grid.fill(1)
        return env

    @classmethod
    def create_no_boundary(cls, rows=10, cols=10) -> 'Environment':
        """Creates an empty environment WITHOUT perimeter boundaries."""
        env = cls(rows, cols)
        return env

    def get_grid(self) -> np.ndarray:
        return self.grid
