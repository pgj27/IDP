def plan_path(self, grid):
    to_visit = [[0, 0]]
    num_nodes = 1
    for i in range(len(grid)):
        for j in range(len(grid[i])):
            if grid[i][j] == 1:
                to_visit.append([i, j])
                num_nodes += 1

    weights = np.zeros((num_nodes, num_nodes), dtype=int)
    for i in range(num_nodes):
        for j in range(num_nodes):
            weights[i][j] = abs(to_visit[i][0] - to_visit[j][0]) + abs(to_visit[i][1] - to_visit[j][1])
            if i == j:
                weights[i][j] = 1000

    route = [[0, 0]]
    pos = 0
    for i in range(num_nodes - 1):
        next = np.argmin(weights[pos])
        print("Closest point to {} is {} (distance: {})".format(pos, next, weights[pos, next]))
        xstart = to_visit[pos][1]
        ystart = to_visit[pos][0]
        xend = to_visit[next][1]
        yend = to_visit[next][0]
        if xstart > xend:
            xend = xstart + 1
            xstart = to_visit[next][1] + 1
        if ystart > yend:
            yend = ystart + 1
            ystart = to_visit[next][0] + 1
        print("{}, {} to {}, {}".format(xstart, ystart, xend, yend))
        grid[ystart:yend, to_visit[pos][1]] += 0.1
        grid[to_visit[next][0], xstart:xend] += 0.1
        weights[:, pos] = 1000
        pos = next
        route.append(to_visit[pos])
    route.append([0, 0])
    print(route)

    plt.matshow(grid)
    plt.show()


scale = self.find_line(conts)
grid_scale = 3 * scale
grid = np.zeros(((int(img.shape[0] / grid_scale)), int(img.shape[1] / grid_scale)))
print(grid.shape)
# print("Cells found at: ")
for i in range(len(conts)):
    xi = int(conts[i][0][0][0] / grid_scale + 1)
    yi = int(conts[i][0][0][1] / grid_scale + 1)
    # print("{}, {}".format(xi, yi))
    grid[yi, xi] = 1  # numpy arrays are row-col