# Smart Delivery Robot Navigation using A*

## Overview
This project demonstrates the A* (A-Star) Search Algorithm for autonomous robot navigation in a 10×10 grid environment. The robot finds the shortest path from a start position to a goal while avoiding obstacles using the Manhattan Distance heuristic.

## Features
- A* Search Algorithm
- Manhattan Distance Heuristic
- Obstacle Avoidance
- Path Visualization using Matplotlib
- Displays explored nodes and shortest path

## Technologies Used
- Python
- NumPy
- Matplotlib

## Project Structure
```
Smart-Delivery-Robot
│── astar_navigation.py
│── requirements.txt
│── README.md
│── output.png
```

## Output
The visualization shows:
- 🔵 Start Node
- 🔴 Goal Node
- ⚫ Obstacles
- 🔷 Explored Nodes
- 🟢 Shortest Path

## How to Run

```bash
pip install -r requirements.txt
python astar_navigation.py
```
