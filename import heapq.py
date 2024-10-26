import heapq
from collections import deque

class TrafficSystem:
    def __init__(self, city_map):
        self.city_map = city_map  # A graph representation of the city roads
        self.traffic_density = {}  # Dictionary to store traffic density at each node
        self.real_time_events = []  # List of roadblocks or accidents
    
    def add_event(self, node):
        self.real_time_events.append(node)
    
    def remove_event(self, node):
        if node in self.real_time_events:
            self.real_time_events.remove(node)
    
    def update_traffic_density(self, node, density):
        self.traffic_density[node] = density

    def a_star_search(self, start, goal):
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        cost_so_far = {start: 0}

        while open_list:
            current_priority, current = heapq.heappop(open_list)
            
            if current == goal:
                break
            
            for neighbor, cost in self.city_map[current]:
                new_cost = cost_so_far[current] + cost + self.traffic_density.get(neighbor, 0)
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self.heuristic(goal, neighbor)
                    heapq.heappush(open_list, (priority, neighbor))
                    came_from[neighbor] = current

        return self.reconstruct_path(came_from, start, goal)

    def bfs(self, start, goal):
        queue = deque([start])
        came_from = {start: None}
        
        while queue:
            current = queue.popleft()
            
            if current == goal:
                break
            
            for neighbor, _ in self.city_map[current]:
                if neighbor not in came_from:
                    queue.append(neighbor)
                    came_from[neighbor] = current
        
        return self.reconstruct_path(came_from, start, goal)

    def dfs(self, start, goal):
        stack = [start]
        came_from = {start: None}

        while stack:
            current = stack.pop()
            
            if current == goal:
                break
            
            for neighbor, _ in self.city_map[current]:
                if neighbor not in came_from:
                    stack.append(neighbor)
                    came_from[neighbor] = current
        
        return self.reconstruct_path(came_from, start, goal)

    def heuristic(self, node1, node2):
        # Simplified heuristic function since nodes are not coordinates
        return 0
    
    def reconstruct_path(self, came_from, start, goal):
        current = goal
        path = []
        while current != start:
            path.append(current)
            current = came_from.get(current)
            if current is None:
                return []  # No path found
        path.append(start)
        path.reverse()
        return path
# Example usage
city_map = {
    'A': [('B', 1), ('C', 4)],
    'B': [('A', 1), ('D', 2), ('E', 5)],
    'C': [('A', 4), ('F', 3)],
    'D': [('B', 2), ('G', 6)],
    'E': [('B', 5), ('H', 2)],
    'F': [('C', 3)],
    'G': [('D', 6)],
    'H': [('E', 2)]
}

traffic_system = TrafficSystem(city_map)
traffic_system.update_traffic_density('D', 2)
path_a_star = traffic_system.a_star_search('A', 'G')
path_bfs = traffic_system.bfs('A', 'G')
path_dfs = traffic_system.dfs('A', 'G')

print("Path using A*: ", path_a_star)
print("Path using BFS: ", path_bfs)
print("Path using DFS: ", path_dfs)
