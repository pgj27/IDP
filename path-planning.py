import numpy as np
import matplotlib.pyplot as plt

grid = np.random.randint(300, size=(50, 50))
to_visit = [[0,0]]
num_nodes = 1
for i in range(len(grid)):
    for j in range(len(grid[i])):
        grid[i,j] = -10*bool(grid[i,j])
        if grid[i][j] == 0:
            to_visit.append([i,j])
            num_nodes += 1

weights = np.zeros((num_nodes, num_nodes), dtype=int)
for i in range(num_nodes):
    for j in range(num_nodes):
        weights[i][j] = abs(to_visit[i][0] - to_visit[j][0]) + abs(to_visit[i][1] - to_visit[j][1])
        if i == j:
            weights[i][j] = 1000

route = [[0, 0]]
pos = 0
for i in range(num_nodes-1):
    next = np.argmin(weights[pos])
    print("Closest point to {} is {} (distance: {})".format(pos, next, weights[pos, next]))
    xstart = to_visit[pos][1]
    ystart = to_visit[pos][0]
    xend = to_visit[next][1]
    yend = to_visit[next][0]
    if xstart > xend:
        xend = xstart+1
        xstart = to_visit[next][1]+1
    if ystart > yend:
        yend = ystart+1
        ystart = to_visit[next][0]+1
    print("{}, {} to {}, {}".format(xstart, ystart, xend, yend))
    grid[ystart:yend, to_visit[pos][1]] += 1
    grid[to_visit[next][0], xstart:xend] += 1
    weights[:, pos] = 1000
    pos = next
    route.append(to_visit[pos])
route.append([0, 0])
print(route)

plt.matshow(grid)
plt.show()