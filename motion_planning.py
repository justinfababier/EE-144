def neighbors(current):
    # Define the list of 4 neighbors: up, down, left, and right
    neighbors = [(1, 0), (-1, 0), (0, 1), (0, -1)]
    return [(current[0] + nbr[0], current[1] + nbr[1]) for nbr in neighbors]

def heuristic_distance(candidate, goal):
    # Calculate the Manhattan distance heuristic between two points
    dx = abs(candidate[0] - goal[0])
    dy = abs(candidate[1] - goal[1])
    return dx + dy

def get_path_from_A_star(start, goal, obstacles):
    # A* algorithm to find the path from start to goal on a grid with obstacles

    # Initialize path and list for open nodes
    path = []
    open_list = [(0, start)]  # List with cost and node tuple
    
    # Lists to track closed nodes, past costs, and parent nodes
    closed_list = []
    past_cost = {start: 0}
    parent = {start: None}

    while open_list:
        # Sort the open list based on cost
        open_list.sort()
        current = open_list.pop(0)[1]

        if current == goal:
            # Goal reached, reconstruct the path and return
            break

        if current in obstacles:
            # Skip nodes that are obstacles
            continue

        closed_list.append(current)  # Add current node to the closed list

        for nbr in neighbors(current):
            if nbr in obstacles:
                # Skip neighbors that are obstacles
                continue

            tentative_past_cost = past_cost[current] + 1

            if nbr not in past_cost or tentative_past_cost < past_cost[nbr]:
                # Update past cost and parent if a better path is found
                past_cost[nbr] = tentative_past_cost
                parent[nbr] = current
                new_cost = past_cost[nbr] + heuristic_distance(nbr, goal)
                open_list.append((new_cost, nbr))

    # Reconstruct the path from goal to start
    while current != start:
        path.append(current)
        current = parent[current]

    path.reverse()  # Reverse the path to start from the beginning

    return path
