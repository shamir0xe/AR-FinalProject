import os
import queue
import random
import math
import copy
from enum import Enum

import time

from sklearn.neighbors import KDTree
import numpy as np

import graphics


class ShowProgress:
    def __init__(self):
        self.before_percent = -1

    def show_progress(self, percent):
        if percent - self.before_percent > 0.01:
            print('\r' + '{0:.02f}%'.format(100 * percent), end='')
            self.before_percent = percent

    def end_progress(self):
        self.before_percent = -1
        print('\r\t\t\t\r100%')


class SamplingStrategy(Enum):
    UNIFORM = 0
    GAUSSIAN = 1
    HYBRID = 2
    BRIDGE_TEST = 3


N, M = 0, 0
board = list()
obstacles = list()
knn = 20
total_milestones = 77777
strategy = SamplingStrategy.BRIDGE_TEST


def read_input():
    global N, M, obstacles, board
    file = open(os.path.join('Inputs', 'map.txt'), 'r+')
    i, j = -1, -1
    with file as f:
        for line in f:
            if (line is None) or (line is '\n'):
                continue
            i += 1
            tokens = line.split(',')
            cur = list()
            j = -1
            for token in tokens:
                j += 1
                cur.append(int(token))
                if int(token) == 1:
                    obstacles.append((i, j))
            board.append(cur)
    N = len(board)
    M = len(board[0])


def in_range(node):
    global N, M
    x, y, alpha, beta = node
    return 0 <= x < N and 0 <= y < M and -1 <= alpha <= +1 and -1 <= beta <= +1


def is_valid(node):
    global N, M, board
    if not in_range(node):
        return False
    x, y, alpha, beta = node
    # swapping x & y
    x, y = y, x

    wall_e = []
    for i in range(-1, +2):
        for j in range(-1, +2):
            xx = x + i
            yy = y + j
            wall_e.append([xx, yy])
    wall_e.append([x - 2, y])
    wall_e.append([x + 2, y])
    if alpha is 0:
        wall_e.append([x - 3, y])
        wall_e.append([x - 4, y])
        wall_e.append([x - 5, y])
    elif alpha is +1:
        wall_e.append([x - 2, y - 1])
        wall_e.append([x - 2, y - 2])
        wall_e.append([x - 2, y - 3])
    elif alpha is -1:
        wall_e.append([x - 2, y + 1])
        wall_e.append([x - 2, y + 2])
        wall_e.append([x - 2, y + 3])

    if beta is 0:
        wall_e.append([x + 3, y])
        wall_e.append([x + 4, y])
        wall_e.append([x + 5, y])
    elif beta is +1:
        wall_e.append([x + 2, y - 1])
        wall_e.append([x + 2, y - 2])
        wall_e.append([x + 2, y - 3])
    elif beta is -1:
        wall_e.append([x + 2, y + 1])
        wall_e.append([x + 2, y + 2])
        wall_e.append([x + 2, y + 3])

    for pos in wall_e:
        x, y = pos
        x, y = y, x
        if not (0 <= x < N and 0 <= y < M and board[x][y] == 0):
            return False

    return True


def create_configuration(method, count):
    nodes = list()
    progress = ShowProgress()
    for i in range(count):
        progress.show_progress(1. * len(nodes) / count)
        if method is SamplingStrategy.UNIFORM:
            x, y, alpha, beta = (
                random.randint(0, N - 1), random.randint(0, M - 1), random.randint(0, 2), random.randint(0, 2))
            node = [x, y, alpha, beta]
            while not in_range(node):
                x, y, alpha, beta = \
                    (random.randint(0, N - 1), random.randint(0, M - 1), random.randint(0, 2), random.randint(0, 2))
                node = [x, y, alpha, beta]
            if is_valid(node):
                nodes.append(node)
        elif method is SamplingStrategy.GAUSSIAN:
            loop = True
            while loop:
                loop = False
                x, y, alpha, beta = (
                    random.randint(0, N - 1), random.randint(0, M - 1), random.randint(0, 2), random.randint(0, 2))
                node = [x, y, alpha, beta]
                while not in_range(node):
                    x, y, alpha, beta = (
                        random.randint(0, N - 1), random.randint(0, M - 1), random.randint(0, 2), random.randint(0, 2))
                    node = [x, y, alpha, beta]
                dist = abs(int(math.ceil(np.random.normal(0, 3))))
                node_q = [x + random.randint(0, 2 * dist) - dist,
                          y + random.randint(0, 2 * dist) - dist,
                          alpha + random.randint(0, 2) - 1,
                          beta + random.randint(0, 2) - 1]
                while not in_range(node_q):
                    dist = abs(int(math.ceil(np.random.normal(0, 3))))
                    node_q = [x + random.randint(0, 2 * dist) - dist,
                              y + random.randint(0, 2 * dist) - dist,
                              alpha + random.randint(0, 2) - 1,
                              beta + random.randint(0, 2) - 1]
                valid_configurations = [is_valid(node), is_valid(node_q)]
                if valid_configurations[0] ^ valid_configurations[1]:
                    if valid_configurations[0]:
                        nodes.append(node)
                    else:
                        nodes.append(node_q)
                    loop = False
        elif method is SamplingStrategy.BRIDGE_TEST:
            loop = True
            while loop:
                loop = False
                x, y, alpha, beta = (
                    random.randint(0, N - 1), random.randint(0, M - 1), random.randint(0, 2), random.randint(0, 2))
                node = [x, y, alpha, beta]
                while not in_range(node):
                    x, y, alpha, beta = (
                        random.randint(0, N - 1), random.randint(0, M - 1), random.randint(0, 2), random.randint(0, 2))
                    node = [x, y, alpha, beta]
                dist = abs(int(math.ceil(np.random.normal(0, 100))))
                node_q = [x + random.randint(0, 2 * dist) - dist,
                          y + random.randint(0, 2 * dist) - dist,
                          alpha + random.randint(0, 2) - 1,
                          beta + random.randint(0, 2) - 1]
                while not in_range(node_q):
                    dist = abs(int(math.ceil(np.random.normal(0, 100))))
                    node_q = [x + random.randint(0, 2 * dist) - dist,
                              y + random.randint(0, 2 * dist) - dist,
                              alpha + random.randint(0, 2) - 1,
                              beta + random.randint(0, 2) - 1]
                valid_configurations = [is_valid(node), is_valid(node_q)]
                if not valid_configurations[0] and not valid_configurations[1]:
                    middle = []
                    for j in range(4):
                        middle.append(int((node[j] + node_q[j]) / 2 + 0.5))
                    if is_valid(middle):
                        nodes.append(middle)
                        loop = False
        elif method is SamplingStrategy.HYBRID:
            loop = True
            while loop:
                loop = False
                x, y, alpha, beta = (
                    random.randint(0, N - 1), random.randint(0, M - 1), random.randint(0, 2), random.randint(0, 2))
                node = [x, y, alpha, beta]
                while not in_range(node):
                    x, y, alpha, beta = (
                        random.randint(0, N - 1), random.randint(0, M - 1), random.randint(0, 2), random.randint(0, 2))
                    node = [x, y, alpha, beta]
                dist = abs(int(math.ceil(np.random.normal(0, 100))))
                node_q = [x + random.randint(0, 2 * dist) - dist,
                          y + random.randint(0, 2 * dist) - dist,
                          alpha + random.randint(0, 2) - 1,
                          beta + random.randint(0, 2) - 1]
                while not in_range(node_q):
                    dist = abs(int(math.ceil(np.random.normal(0, 100))))
                    node_q = [x + random.randint(0, 2 * dist) - dist,
                              y + random.randint(0, 2 * dist) - dist,
                              alpha + random.randint(0, 2) - 1,
                              beta + random.randint(0, 2) - 1]
                valid_configurations = [is_valid(node), is_valid(node_q)]
                if valid_configurations[0] and valid_configurations[1]:
                    if random.random() > 0.1:
                        selected = random.randint(0, 1)
                        if selected == 0:
                            nodes.append(node)
                        else:
                            nodes.append(node_q)
                        loop = False
                elif valid_configurations[0] ^ valid_configurations[1]:
                    if random.random() > 0.5:
                        if valid_configurations[0]:
                            nodes.append(node)
                        else:
                            nodes.append(node_q)
                        loop = False
                else:
                    middle = []
                    for j in range(4):
                        middle.append(int((node[j] + node_q[j]) / 2 + 0.5))
                    if is_valid(middle):
                        nodes.append(middle)
                        loop = False

    progress.end_progress()
    print(len(nodes), count)
    print('accepted ratio:', len(nodes) / count)
    return nodes


def exist_path(u, v):
    cur_point = copy.deepcopy(u)
    path = []
    change = True
    path.append(u)
    while cur_point != v and change:
        change = False
        for i in range(-1, +2):
            for j in range(-1, +2):
                if abs(i) + abs(j) == 1:
                    new_node = copy.deepcopy(cur_point)
                    new_node[0] += i
                    new_node[1] += j
                    if is_valid(new_node) and manhattan(new_node, v) < manhattan(cur_point, v):
                        path.append(new_node)
                        cur_point = copy.deepcopy(new_node)
                        change = True
                        break
        for i in range(-1, +2):
            for j in range(-1, +2):
                if abs(i) + abs(j) == 1:
                    new_node = copy.deepcopy(cur_point)
                    new_node[2] += i
                    new_node[3] += j
                    if is_valid(new_node) and manhattan(new_node, v) < manhattan(cur_point, v):
                        path.append(new_node)
                        cur_point = copy.deepcopy(new_node)
                        change = True
                        break
    if cur_point != v:
        return -1
    return path


def find_adjacency(nodes):
    kdt = KDTree(nodes, leaf_size=30, metric='manhattan')
    adj = kdt.query(nodes, k=knn, return_distance=False)
    exact_adj = []
    print('find adjacent')
    progress = ShowProgress()
    for u in range(len(nodes)):
        cur = []
        for v in adj[u]:
            progress.show_progress(1. * u / len(nodes))
            if v != u:  # and exist_path(nodes[u], nodes[v]) is not -1:
                cur.append(v)
        exact_adj.append(cur)
    progress.end_progress()
    return exact_adj


def manhattan(a, b):
    s = 0
    for idx in range(len(a)):
        s += abs(a[idx] - b[idx])
    return s


def find_path(positions, nodes, adj):
    """
    finding path using A* algorithm
    """
    start, end = positions
    kdt = KDTree(nodes, leaf_size=30, metric='manhattan')
    temp = kdt.query([start, end])
    total_distance = temp[0][0][0] + temp[0][1][0]
    start_node, end_node = temp[1][0][0], temp[1][1][0]
    print('start, end:', start_node, end_node)
    print(nodes[start_node], nodes[end_node])
    if exist_path(nodes[start_node], start) is -1 or exist_path(nodes[end_node], end) is -1:
        return -1, -1
    pq = queue.PriorityQueue()
    dist = [12345678901234567 for i in range(len(nodes))]
    pq.put([manhattan(nodes[start_node], nodes[end_node]), start_node, 0])
    dist[start_node] = manhattan(nodes[start_node], nodes[end_node])
    parents = dict()
    while not pq.empty():
        cur = pq.get()
        length, node, exact = cur
        if dist[node] < length:
            continue
        if node == end_node:
            break

        # print(nodes[node])
        dist[node] = length

        t = nodes[node][0:2]
        t.extend([5])
        graphics.draw_circles((N, M), [t], 666)

        for v in adj[node]:
            seq = exist_path(nodes[node], nodes[v])
            if seq is -1:
                continue
            cur_len = manhattan(nodes[node], nodes[v])
            pot = exact + cur_len
            pot2 = pot + manhattan(nodes[v], nodes[end_node])
            if pot2 < dist[v]:
                dist[v] = pot2
                pq.put([pot2, v, pot])
                parents[v] = node

    if dist[end_node] < 12345678901234567:
        total_distance += dist[end_node]
        path = [end]
        temp = end_node
        while temp != start_node:
            path.append(nodes[temp])
            temp = parents[temp]
        path.append(nodes[temp])
        path.append(start)
        path.reverse()
        return total_distance, path
    return -1, -1


def draw_workspace(nodes, adj, path=None, circles=None):
    global N, M, obstacles
    node_points = []
    for node in nodes:
        node_points.append((node[0], node[1]))
    lines = []
    for u in range(0, len(nodes)):
        for v in adj[u]:
            lines.append([nodes[u][0], nodes[u][1], nodes[v][0], nodes[v][1]])
    graphics.draw_map((N, M), obstacles, node_points, lines, path, circles)


def main():
    read_input()
    # show_board()
    print(time.clock())
    nodes = create_configuration(strategy, total_milestones)
    print(time.clock())
    adj = find_adjacency(nodes)
    draw_workspace(nodes, adj)
    print(time.clock())
    ans = find_path(([10, 10, 0, 0], [410, 850, 0, 0]), nodes, adj)
    print(time.clock())
    if ans[0] is -1:
        print("FAILURE!")
    else:
        print(ans)
        path = ans[1]
        exact_path = []
        bef_node = []
        cost = 0
        for node in path:
            if len(bef_node) is 0:
                bef_node = copy.deepcopy(node)
                exact_path.append(bef_node)
                continue
            seq = exist_path(bef_node, node)
            if seq == -1:
                t1 = bef_node[0:2]
                t1.extend([10])
                t2 = node[0:2]
                t2.extend([10])
                graphics.draw_circles((N, M), [t1, t2], 666,
                                      color=(231, 12, 23))
            else:
                cost += len(seq)
                exact_path.extend(seq[1:])
            bef_node = copy.deepcopy(node)
        print('cost:', cost)
        print('time elapsed = {0:.2f}s'.format(time.clock()))

        graphics.wait_until_click()
        path_to_show = []
        bef_node = []
        for node in exact_path:
            if len(bef_node) is 0:
                bef_node = copy.deepcopy(node)
                continue
            seq = bef_node[0:2]
            seq.extend(node[0:2])
            path_to_show.append(seq)
            bef_node = copy.deepcopy(node)
        draw_workspace(nodes, adj, path_to_show)
    print('time elapsed = {0:.2f}s'.format(time.clock()))


if __name__ == '__main__':
    main()
