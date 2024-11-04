from queue import PriorityQueue
import util

def h(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1 - x2) + abs(y1 - y2)

def reconstruct_path(came_from, current, draw):
    path = []
    while current in came_from:
        path.append(current.get_pos())
        current.make_path()
        current = came_from[current]
        draw()
    path.append(current.get_pos())
    path.reverse()
    print("Path\nStart:")
    for step in path:
        print(" ", step, end=",\n")
    print("Goal")
    print(f"Length: {len(path)} steps")
    return path

def astar(draw, grid, start, end):
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start))
    came_from = {}
    g_score = {spot: float("inf") for row in grid for spot in row}
    g_score[start] = 0
    f_score = {spot: float("inf") for row in grid for spot in row}
    f_score[start] = h(start.get_pos(), end.get_pos())
    open_set_hash = {start}

    while not open_set.empty():
        current = open_set.get()[2]
        open_set_hash.remove(current)
        if current == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            return True
        for neighbor in current.neighbors:
            temp_g_score = g_score[current] + 1
            if temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end.get_pos())
                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.make_open()
        draw()
        if current != start:
            current.make_closed()
    return False

def dfs(draw, grid, start, end):
    visited = set()
    frontier = []
    frontier.append((start, []))
    came_from = {}

    while frontier:
        current, actions = frontier.pop()
        if current == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            return True
        if current not in visited:
            visited.add(current)
            for neighbor in current.neighbors:
                if neighbor not in visited and not neighbor.is_barrier():
                    came_from[neighbor] = current
                    frontier.append((neighbor, actions + [current.get_pos()]))
                    neighbor.make_open()
        draw()
        if current != start:
            current.make_closed()
    return False

def bfs(draw, grid, start, end):
    from util import Queue
    queue = Queue()
    queue.push((start, []))
    visited = set()
    came_from = {}

    while not queue.isEmpty():
        current, path = queue.pop()
        if current == end:
            path.append(current.get_pos())
            path = reconstruct_path(came_from, end, draw)
            end.make_end()
            return True
        if current not in visited:
            visited.add(current)
            for neighbor in current.neighbors:
                if neighbor not in visited and not neighbor.is_barrier():
                    came_from[neighbor] = current
                    queue.push((neighbor, path + [current.get_pos()]))
                    neighbor.make_open()
        draw()
        if current != start:
            current.make_closed()
    return False

def uniform_cost_search(draw, grid, start, end):
    pq = PriorityQueue()
    pq.put((0, start))
    came_from = {}
    cost_so_far = {start: 0}

    while not pq.empty():
        current_cost, current = pq.get()
        if current == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            return True
        for neighbor in current.neighbors:
            new_cost = cost_so_far[current] + 1
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                came_from[neighbor] = current
                pq.put((new_cost, neighbor))
                neighbor.make_open()
        draw()
        if current != start:
            current.make_closed()
    return False

def dijkstra(draw, grid, start, end):
    pq = PriorityQueue()
    pq.put((0, start))
    came_from = {}
    cost_so_far = {start: 0}

    while not pq.empty():
        current_cost, current = pq.get()
        if current == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            return True
        for neighbor in current.neighbors:
            new_cost = cost_so_far[current] + 1
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                came_from[neighbor] = current
                pq.put((new_cost, neighbor))
                neighbor.make_open()
        draw()
        if current != start:
            current.make_closed()
    return False
