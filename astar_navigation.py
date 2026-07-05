import matplotlib.pyplot as plt
import numpy as np
from queue import PriorityQueue

# Grid size
ROWS = 10
COLS = 10

# Create grid
grid = np.zeros((ROWS, COLS))

# Obstacles
obstacles = [
    (1, 3), (2, 3), (3, 3),
    (4, 5), (5, 5), (6, 5),
    (7, 2), (7, 3), (7, 4)
]

for obs in obstacles:
    grid[obs] = 1

# Start and Goal
start = (0, 0)
goal = (9, 9)

# Heuristic Function (Manhattan Distance)
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# A* Algorithm
def astar(grid, start, goal):

    rows, cols = grid.shape
    open_set = PriorityQueue()
    open_set.put((0, start))

    came_from = {}

    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    visited = []

    while not open_set.empty():

        current = open_set.get()[1]

        visited.append(current)

        if current == goal:
            path = []

            while current in came_from:
                path.append(current)
                current = came_from[current]

            path.append(start)
            path.reverse()

            return path, visited

        neighbors = [
            (current[0] - 1, current[1]),
            (current[0] + 1, current[1]),
            (current[0], current[1] - 1),
            (current[0], current[1] + 1)
        ]

        for neighbor in neighbors:

            r, c = neighbor

            if 0 <= r < rows and 0 <= c < cols:

                if grid[r][c] == 1:
                    continue

                temp_g_score = g_score[current] + 1

                if neighbor not in g_score or temp_g_score < g_score[neighbor]:

                    came_from[neighbor] = current
                    g_score[neighbor] = temp_g_score
                    f_score[neighbor] = temp_g_score + heuristic(neighbor, goal)

                    open_set.put((f_score[neighbor], neighbor))

    return None, visited

# Run Algorithm
path, visited = astar(grid, start, goal)

# Visualization
fig, ax = plt.subplots(figsize=(8, 8))

# Draw Grid
for i in range(ROWS):
    for j in range(COLS):

        if grid[i][j] == 1:
            ax.scatter(j, ROWS - i - 1, color='black', s=500)

# Draw Visited Nodes
for node in visited:
    ax.scatter(node[1], ROWS - node[0] - 1, color='lightblue', s=200)

# Draw Path
if path:
    for node in path:
        ax.scatter(node[1], ROWS - node[0] - 1, color='green', s=300)

# Start and Goal
ax.scatter(start[1], ROWS - start[0] - 1, color='blue', s=500, label='Start')
ax.scatter(goal[1], ROWS - goal[0] - 1, color='red', s=500, label='Goal')

# Grid settings
ax.set_xticks(range(COLS))
ax.set_yticks(range(ROWS))
ax.grid(True)

plt.title("Smart Delivery Robot Navigation using A*")
plt.legend()
plt.show()

# Print Results
print("Path Found:")
print(path)

print("\nNodes Explored:")
print(len(visited))