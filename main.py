import numpy as np
import matplotlib.pyplot as plt
from robot_agent import RobotAgent

# --- Environment Setup ---
from environment import Environment

# Use the 'C' shaped environment factory by default
env = Environment.create_c_shape()
grid = env.get_grid()


# --- Visualization Function ---
def visualize_game(grid, robot, title):
    """
    Display the grid, robot path, start, and goal positions.

    Args:
        grid (np.ndarray): 2D grid with obstacles
        robot (RobotAgent): The robot agent
        title (str): Title for the plot
    """
    plt.figure(figsize=(8, 8))
    plt.imshow(grid, cmap='binary', origin='upper')

    # Plot the path taken by the robot
    if len(robot.path) > 1:
        path_y, path_x = zip(*robot.path)
        plt.plot(
            path_x, path_y,
            color='blue',
            linewidth=2,
            label="Path Taken",
            alpha=0.6
        )

    # Plot start and goal points
    plt.scatter(robot.x, robot.y, color='red', s=200, edgecolors='black', label='Robot', zorder=5)
    plt.scatter(robot.goal_x, robot.goal_y, color='green', marker='X', s=200, label='Goal', zorder=5)

    # Grid and labels
    plt.title(title, fontsize=15)
    plt.legend(loc='upper right')
    plt.grid(True, which='both', color='gray', linestyle='-', linewidth=0.5, alpha=0.3)
    plt.xticks(range(grid.shape[1]))
    plt.yticks(range(grid.shape[0]))
    plt.show()


# --- Robot Setup and Execution ---

def main():
    # Initialize robot at (1,1) with goal at (18,18)
    bot = RobotAgent(start_pos=(1, 1), goal_pos=(18, 18))

    # Perform A* search
    path = bot.a_star_search(grid)

    title = "A* Pathfinding Result: No Path Found"
    if path:
        bot.path = path  # Update robot with the found path
        print(f"Success! Path found with {len(path)} steps.")
        title = f"A* Pathfinding Result: Success ({len(path)} steps)"
    else:
        print("No path found. Is the robot trapped?")
    
    visualize_game(grid, bot, title)

if __name__ == "__main__":
    main()
