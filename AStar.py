import pygame, json, heapq, pathlib, time
import numpy as np
SCREENSIZE = (400, 300)

class mobileRobot:
    def __init__(self, startPoint: tuple[int, int]):
        self.x, self.y = startPoint

    def draw(self, point: tuple[int, int]):
        pygame.draw.circle(screen, (0, 255, 0), point, 10)

class AStarPlanner:
    def __init__(self, startPoint: tuple[int, int], endPoint: tuple[int, int]):
        self.startx, self.starty = startPoint
        self.endx, self.endy = endPoint

    def heuristic(self, x, y): # Heuristic function for A*
        return abs(self.endx - x) + abs(self.endy - y)
    
    def nearWallCost(self, x, y): # Ensure a safety distance from obstacles
        scale = np.array([1, 1, 3, 3, 5, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 5, 3, 3, 1, 1])
        scale = np.outer(scale, scale)
        surround = obsMask[x-10:x+11, y-10:y+11]
        cost = np.sum(np.sum(surround * scale))
        return cost
    
    def plan(self): # Main A* algorithm
        visited = set()
        visited.add((self.startx, self.starty))
        pq = []
        route = {(self.startx, self.starty): (None, None)}
        heapq.heappush(pq, (self.heuristic(self.startx, self.starty), (self.startx, self.starty, 0, None))) # Priority queue
        while len(pq):
            _, curr = heapq.heappop(pq)
            if (curr[0] == self.endx and curr[1] == self.endy):
                return self.get_route(route)

            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                nx, ny = curr[0] + dx, curr[1] + dy
                if (                                # Check
                    0 <= nx < SCREENSIZE[0] and 0 <= ny < SCREENSIZE[1]
                    and (nx, ny) not in visited
                    and obsMask[nx, ny] == 0
                ):
                    visited.add((nx, ny))
                    route[(nx, ny)] = (curr[0], curr[1])  
                    heapq.heappush(pq, (curr[2] + 1 + self.heuristic(nx, ny) + self.nearWallCost(nx, ny), 
                                        (nx, ny, curr[2] + 1, (dx, dy))))
    
    def get_route(self, route): # Loop from back to front to form a complete path
        x, y = self.endx, self.endy
        path = []
        while (x, y) != (None, None):
            path.append((x, y))
            x, y = route[(x, y)]
        return list(reversed(path))

    
pygame.init()
obsMask = np.zeros(SCREENSIZE)
script_dir = pathlib.Path(__file__).parent.resolve() # Load map data
config_path = script_dir / "obstacles.json" 
with open(config_path, 'r') as f:
    obs = json.load(f)["obstacles"] 
    obstacles = [pygame.Rect(*ob) for ob in obs]
    for ob in obs:
        obsMask[ob[0]:ob[0]+ob[2], ob[1]:ob[1]+ob[3]] = 1

startPoint, endPoint = (25, 25), (370, 30)
screen = pygame.display.set_mode(SCREENSIZE)
pygame.display.set_caption("Robot Simulator (A* Pathfinding)")
clock = pygame.time.Clock()
robot = mobileRobot(startPoint)
astar = AStarPlanner(startPoint, endPoint)
start_time = time.time()
path = astar.plan()
end_time = time.time()
elapsed_time = end_time - start_time
stepCount = 0
done = arrived = False

print(f"From {startPoint} to {endPoint}")
print("Length of path:", len(path))
print(f"Time used to create the path: {elapsed_time:.6f} seconds")

while not done:
    for events in pygame.event.get():
        if events.type == pygame.QUIT:
            done = True
    if not arrived:
        screen.fill((255, 255, 255))
        for ob in obstacles: # Draw obstacles and start/end points
            pygame.draw.rect(screen, (255, 0, 0), ob)
        pygame.draw.circle(screen, (0, 0, 255), endPoint, 10)
        pygame.draw.circle(screen, (0, 255, 0), startPoint, 10, width=3)
        
        if path:
            for p in range(len(path)-1): # Draw path
                pygame.draw.line(screen, (0, 0, 0), path[p], path[p+1], 2)
            stepCount += 1 # Next step
            if stepCount >= len(path):
                stepCount = len(path) - 1
                arrived = True
            robot.draw(path[stepCount]) # Draw wobot
        else:
            pygame.display.set_caption("No path is found.")

        pygame.display.flip()
        clock.tick(30)

pygame.quit()