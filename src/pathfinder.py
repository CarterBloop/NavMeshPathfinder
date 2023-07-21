from maze_environment import load_level, show_level, save_level_costs
from math import inf, sqrt
from heapq import heappop, heappush

def find_path(source_point, destination_point, mesh):
    """
    Searches for a path from source_point to destination_point through the mesh

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:
        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """
    source_box = find_box(source_point, mesh)
    destination_box = find_box(destination_point, mesh)

    if not source_box or not destination_box:
        print("No path possible!")
        return [],[]

    # --- A* search ---
    # path = a_star_algo(source_box, destination_box, mesh)

    # --- Bidirectional A* search ---
    path = bi_a_star_algo(source_box, destination_box, mesh)

    if path == None:
        print("No path possible!")
        return [],[]
    if len(path) == 0:
        print("No path possible!")
        return [],[]

    detail_points = get_detail_points(source_point, destination_point, path)

    return detail_points, path

def find_box(point, mesh):
    x, y = point
    for box in mesh['boxes']:
        x1, x2, y1, y2 = box
        if x1 <= x <= x2 and y1 <= y <= y2:
            return box
    return None

def get_detail_points(source_point, destination_point, path):
    point_list = []
    point_list.append(source_point)

    for i in range(0, len(path)):
        current_box = path[i]
        closest_point = calc_closest_point(source_point, current_box)
        point_list.append(closest_point)
        source_point = closest_point

    point_list.append(destination_point) 
    return point_list

def a_star_algo(source_box, destination_box, mesh):
    if source_box == destination_box:
        return [source_box]
    
    queue = []
    visited = set()
    heappush(queue, (0, source_box, [], 0))
    visited.add(source_box)

    while queue:
        priority, current_box, path, actual_cost = heappop(queue)
        if current_box == destination_box:
            return path + [current_box]

        for neighbor in mesh['adj'][current_box]:
            if neighbor not in visited:
                edge_cost = calc_edge_cost(current_box, neighbor)  
                sum_cost = actual_cost + edge_cost
                heuristic = calc_edge_cost(neighbor, destination_box)
                new_priority = sum_cost + heuristic
    
                heappush(queue, (new_priority, neighbor, path + [current_box], sum_cost))
                visited.add(neighbor)

    # No path found
    return []

def bi_a_star_algo(source_box, destination_box, mesh):
    if source_box == destination_box:
        return [source_box]

    # Distances & Prev boxes for forward / backward 
    forward_dist = {source_box: 0}
    forward_prev = {source_box: None}
    backward_dist = {destination_box: 0}
    backward_prev = {destination_box: None}

    # Bidirectional search
    queue = []
    heappush(queue, (0, source_box, destination_box))
    heappush(queue, (0, destination_box, source_box))

    while queue:
        priority, current_box, current_goal = heappop(queue) # Pop lowest priority 
        # If forward search
        if current_goal == destination_box:
            dist = forward_dist
            prev = forward_prev
            other_prev = backward_prev
        # If backward search
        else:
            dist = backward_dist
            prev = backward_prev
            other_prev = forward_prev

        # If midpoint found
        if current_box in other_prev:
            # Create path from source -> midpoint
            front_path = []
            temp_box = current_box
            while temp_box is not None:
                front_path.insert(0, temp_box)
                temp_box = prev[temp_box]

            # Create path from midpoint -> destination
            back_path = []
            temp_box = current_box
            while temp_box is not None:
                back_path.append(temp_box)
                temp_box = other_prev[temp_box]

            # Create complete path
            combined_path = front_path + back_path[1:]
            check_path(combined_path,mesh['adj']) # Check path

            if(combined_path[0] == source_box and combined_path[-1] == destination_box):
                return combined_path
            elif(combined_path[0] == destination_box and combined_path[-1] == source_box):
                return combined_path[::-1] # Flip path
            else:
                print("Error: Invalid path")
                return []

        for neighbor in mesh['adj'][current_box]:
            edge_cost = calc_edge_cost(current_box, neighbor) 
            if neighbor not in dist or dist[current_box] + edge_cost < dist[neighbor]:
                dist[neighbor] = dist[current_box] + edge_cost
                prev[neighbor] = current_box
                # Priority =  distance + heuristic
                priority = dist[neighbor] + calc_edge_cost(neighbor, current_goal)
                heappush(queue, (priority, neighbor, current_goal))

# Helper functions
def check_path(path,adj):
    for i in range(len(path) - 2):
        if path[i + 1] not in adj[path[i]]:
            print("Invalid path!")
            return False
    print("Valid path")
    return True
   

def calc_edge_cost(box1, box2):
    midpoint_box1 = midpoint(box1)
    midpoint_box2 = midpoint(box2)
    return distance(midpoint_box1, midpoint_box2)


def distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def midpoint(box):
    x1, x2, y1, y2 = box
    midpoint_x = (x1 + x2) / 2
    midpoint_y = (y1 + y2) / 2
    return midpoint_x, midpoint_y



def calc_closest_point(current_point, box):
    x1, x2, y1, y2 = box
    current_x = current_point[0]
    current_y = current_point[1]
    closest_x = max(x1, min(current_x, x2))
    closest_y = max(y1, min(current_y, y2))
    closest_point = (closest_x, closest_y)
    return closest_point