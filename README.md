# Robot Pathfinding with A* Search

A Python project demonstrating a simple 2D grid environment where a robot navigates around obstacles using the **A* search algorithm**. This project includes both manual movement and automated pathfinding with visualization.

## Table of Contents

- [Overview](#overview)
- [Project Structure](#project-structure)
- [Features](#features)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Usage](#usage)
- [Testing](#testing)

---

## Overview

This project simulates a robot moving in a **20x20 grid environment**:

- `0` represents free space.
- `1` represents obstacles.

The robot starts at a specified position and attempts to reach a goal location, avoiding obstacles. The **A* search algorithm** is used for automatic path planning, leveraging the **Manhattan distance heuristic**.

## Project Structure

```
A-Star-Navigator/
├── main.py            # Entry point for visualization
├── robot_agent.py     # Robot class and A* implementation
├── requirements.txt   # Project dependencies
├── tests/             # Unit tests
│   └── test_robot_agent.py
└── README.md          # Project documentation
```

## Features

- **Manual movement**: Move the robot using "up", "down", "left", "right" commands (via `RobotAgent.move`).
- **Automated pathfinding**: Use A* to find the optimal path from start to goal.
- **Obstacle handling**: Detects collisions and avoids blocked cells.
- **Visualization**: Displays the grid, robot path, start, and goal points with Matplotlib.

## Dependencies

- Python 3.8+
- [NumPy](https://numpy.org/)
- [Matplotlib](https://matplotlib.org/)

## Installation

1. Clone this repository.
2. Install the required packages:

```bash
pip install -r requirements.txt
```

## Usage

Run the main script to see the A* algorithm in action:

```bash
python main.py
```

It will confirm if a path is found and open a Matplotlib window showing the robot's journey.

## Testing

Run the unit tests to verify the logic:

```bash
python -m unittest discover tests
```

---
