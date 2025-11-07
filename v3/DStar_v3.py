'''
D* (Dynamic A*) Path Finder - Version 3

In this version, the user-controlled mobile obstacle is replaced by a list of random generated mobile obstacles.
'''
import pygame, json, heapq, pathlib, time, sys
import numpy as np
''' 
A collection of functions, namely binary, gaussian, and inverse, that could be used for penalty for obstacle proximity,
though their actual behaviorial differences are still unclear in this project.
'''
from nearWallCost import nearWallCostFunctions
SCREENSIZE = (400, 300)
MAP = 5

class mobileRobot:
    def __init__(self, iniPoint: tuple[int, int]):
        self.x, self.y = iniPoint
        self.path = []
        self.stepCount = 0

    def draw(self, Point):
        pygame.draw.circle(screen, (0, 255, 0), Point, 10)
    
    # Generate a complete path from a list of waypoints
    def generate_path(self, waypoints):
        xpath = []
        ypath = []
        for i in range(len(waypoints)-1):
            segment_len = np.sqrt((waypoints[i+1][0]-waypoints[i][0])**2 + (waypoints[i+1][1]-waypoints[i][1])**2)
            x = np.linspace(waypoints[i][0], waypoints[i+1][0], num=int(segment_len), dtype=int)
            y = np.linspace(waypoints[i][1], waypoints[i+1][1], num=int(segment_len), dtype=int)
            if len(x) == 1:
                x = np.full((len(y),), x[0], dtype=int)
            if len(y) == 1:
                y = np.full((len(x),), y[0], dtype=int)
            xpath.extend(x.tolist())
            ypath.extend(y.tolist())
        self.path = list(zip(xpath, ypath))
    
    def next_step(self):
        if self.path:
            self.x, self.y = self.path.pop(0)
            self.stepCount += 1
        return (self.x, self.y)
       

class AStarPlanner:
    def __init__(self, startPoint: tuple[int, int], endPoint: tuple[int, int]):
        self.startx, self.starty = startPoint
        self.endx, self.endy = endPoint
        self.startTime = self.endTime = None
        self.pathLength = None
    
    # Heuristic
    def h(self, x, y):
        return abs(self.endx - x) + abs(self.endy - y)
    
    def safetyCheck(self, x, y, radius=10, mobile_range=15): #todo
        # Always check static obstacles with full safety radius
        x0, x1 = max(0, x-radius), min(SCREENSIZE[0], x+radius+1)
        y0, y1 = max(0, y-radius), min(SCREENSIZE[1], y+radius+1)
        if np.any(constMap[x0:x1, y0:y1] == 1):
            return False
            
        # Check mobile obstacles only if they're within mobile_range of current position
        if abs(x - self.startx) <= mobile_range and abs(y - self.starty) <= mobile_range:
            x0, x1 = max(0, x-radius), min(SCREENSIZE[0], x+radius+1)
            y0, y1 = max(0, y-radius), min(SCREENSIZE[1], y+radius+1)
            if np.any(varMap[x0:x1, y0:y1] == 1):
                return False
                
        return True

    # Main A* algorithm
    def plan(self):
        self.startTime = time.time()
        visited = set()
        visited.add((self.startx, self.starty))
        pq = []
        route = {(self.startx, self.starty): (None, None, None)}
        heapq.heappush(pq, (self.h(self.startx, self.starty), (self.startx, self.starty, 0, None))) # Priority queue
        while len(pq):
            _, curr = heapq.heappop(pq)
            # Arrived
            if (curr[0] == self.endx and curr[1] == self.endy):
                self.endTime = time.time()
                self.pathLength = curr[2]
                self.printInfo()
                return self.waypoints(route)

            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                nx, ny = curr[0] + dx, curr[1] + dy
                if (
                    0 <= nx < SCREENSIZE[0] and 0 <= ny < SCREENSIZE[1]
                    and (nx, ny) not in visited
                    and self.safetyCheck(nx, ny)
                ):
                    visited.add((nx, ny))
                    route[(nx, ny)] = (curr[0], curr[1], (dx, dy))
                    cost = curr[2] + 1 + self.h(nx, ny) + nearWallCostFunctions(nx, ny, 'gaussian', constMap, 10)
                    heapq.heappush(pq, (cost, (nx, ny, curr[2] + 1, (dx, dy))))
        raise Exception("No valid path found")

    def check_line_of_sight(self, p1, p2):
        # Check if a straight line between p1 and p2 is clear of obstacles
        x1, y1 = p1
        x2, y2 = p2
        dx = x2 - x1
        dy = y2 - y1
        steps = max(abs(dx), abs(dy))
        if steps == 0:
            return True
            
        x_step = dx / steps
        y_step = dy / steps
        
        # Check points along the line with 5 unit spacing on each side
        for i in range(steps + 1):
            x = int(x1 + i * x_step)
            y = int(y1 + i * y_step)

            radius = 5
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    check_x = x + dx
                    check_y = y + dy
                    if (0 <= check_x < SCREENSIZE[0] and 0 <= check_y < SCREENSIZE[1] and
                        (constMap[check_x, check_y] == 1 or not self.safetyCheck(check_x, check_y, 5))):
                        return False
        return True

    def waypoints(self, route):
        x, y = self.endx, self.endy
        d = None
        path = []
        while (x, y) != (None, None):
            if d is None or d != route[(x, y)][2]:
                path.append((x, y))
                d = route[(x, y)][2]
            x, y, d = route[(x, y)]

        if len(path) <= 2:
            return list(reversed(path))
            
        # Smooth the path by removing unnecessary waypoints
        path = list(reversed(path))
        print('original', path)
        # Iterative path smoothing
        return self.smoothPath(path)
        
    def smoothPath(self, path):
        i = 0
        smoothed_path = []
        while i < len(path) - 1:
            # Look for next point that has line of sight
            j = min(i + 2, len(path) - 1)
            if (self.check_line_of_sight(path[i], path[j])):
                print("Removing waypoints between", path[i], "and", path[j])
                smoothed_path.append(path[i])
                i = j
            
            smoothed_path.append(path[i])
            i += 1
            
        if smoothed_path[-1] != path[-1]:
            smoothed_path.append(path[-1])
        return smoothed_path
        
    
    def printInfo(self):
        print(f"From {self.startx, self.starty} to {self.endx, self.endy}")
        print("Length of path:", self.pathLength)
        print(f"Time used to create the path: {self.endTime - self.startTime:.6f} seconds")

class randomObstacle:
    def __init__(self, num:int, interval:float, speedRange):
        self.num = num
        self.xmax, self.ymax = SCREENSIZE
        self.lastUpdate = time.time()
        self.interval = interval
        self.speedRange = speedRange
        length = np.sqrt((startPoint[0] - endPoint[0])**2 + (startPoint[1] - endPoint[1])**2)
        angle = np.arctan2(endPoint[1] - startPoint[1], endPoint[0] - startPoint[0])
        rotation = np.array([[np.cos(angle), -np.sin(angle)],  # Rotation matrix
                             [np.sin(angle), np.cos(angle)]])
        # y-coordinate normally distributed, x-coordinate uniformally distributed
        y = np.random.standard_normal(num) * 20
        x = np.random.uniform(0, length, num)
        points = np.vstack([x, y])
        # Linear transformation + translation
        points = rotation @ points + np.array(startPoint).reshape((2, 1))
        self.randList = points.T.astype(np.int64)
        print("Initial positions of random obstacles", self.randList.tolist())

    # Random movement
    def randMove(self):
        self.lastUpdate = time.time()
        randd = np.random.choice(self.speedRange, size=(self.num, 2), replace=True)
        self.randList += randd
            
    def update(self):
        # Move every x seconds
        if (time.time() - self.lastUpdate > self.interval):
            self.randMove()

        varMap[:] = np.zeros((self.xmax, self.ymax))
        for nx, ny in self.randList: # Each random obstacle
            rect = pygame.Rect(nx, ny, 20, 20)
            pygame.draw.rect(screen, (255, 0, 255), rect)
            x0, x1, y0, y1 = max(0, nx), min(self.xmax, nx + 20), max(0, ny), min(self.ymax, ny + 20)
            varMap[x0:x1, y0:y1] = 1 # Modify dynamic obstacle map


pygame.init()
constMap = np.zeros(SCREENSIZE)
varMap = np.zeros(SCREENSIZE)
script_dir = pathlib.Path(__file__).parent.resolve()
config_path = script_dir / "maps.json"
with open(config_path, 'r') as f:
    maps = json.load(f)[f'map {MAP if len(sys.argv) == 1 else sys.argv[1]}'] # Use additional parameter if given
    obs = maps["obstacles"]
    obstacles = [pygame.Rect(*ob) for ob in obs]
    for ob in obs:
        constMap[ob[0]:ob[0]+ob[2], ob[1]:ob[1]+ob[3]] = 1 # Set static obstacle map

startPoint, endPoint = tuple(maps["start"]), tuple(maps["end"])
screen = pygame.display.set_mode(SCREENSIZE)
pygame.display.set_caption("Robot Simulator (Dynamic A* Pathfinding)")
clock = pygame.time.Clock()
robot = mobileRobot(startPoint)
astar = AStarPlanner(startPoint, endPoint)
randObs = randomObstacle(7, 2.0, speedRange=[-10, -5, 0, 10, 5])
waypoints = astar.plan() # Initial planning
robot.generate_path(waypoints)
done = arrived = False
last_replan_time = None
waiting_for_replan = False

while not done:
    for events in pygame.event.get():
        if events.type == pygame.QUIT:
            done = True

    if not arrived:
        screen.fill((255, 255, 255))
        
        for ob in obstacles:
            pygame.draw.rect(screen, (255, 0, 0), ob)
        if robot.path:
            for p in range(len(robot.path)-1):
                pygame.draw.line(screen, (0, 0, 0), robot.path[p], robot.path[p+1], 2)
            
            randObs.update()
            if len(robot.path) == 1:
                print("Arrived at the destination.")
                arrived = True

            prevPoint = (robot.x, robot.y)
            # If waiting for replanning, the robot should stand still
            if not waiting_for_replan:
                currPoint = robot.next_step()
            else:
                currPoint = prevPoint
            robot.draw(currPoint)
            pygame.draw.circle(screen, (0, 0, 255), endPoint, 10, width=3)
            pygame.draw.circle(screen, (0, 255, 0), startPoint, 10, width=3)

            # Only check for moving obstacles in the robot's moving direction within a range of 15
            dx = currPoint[0] - prevPoint[0]
            dy = currPoint[1] - prevPoint[1]
            vision_range = 15
            # If not waiting for a replan, check ahead in movement direction
            if not waiting_for_replan:
                blocked = False
                if dx != 0 or dy != 0:
                    for i in range(1, vision_range + 1):
                        check_x = int(currPoint[0] + (np.sign(dx) * i)) if dx != 0 else int(currPoint[0])
                        check_y = int(currPoint[1] + (np.sign(dy) * i)) if dy != 0 else int(currPoint[1])
                        if 0 <= check_x < SCREENSIZE[0] and 0 <= check_y < SCREENSIZE[1]:
                            if varMap[check_x, check_y] == 1:
                                blocked = True
                                break
                if blocked:
                    print("Obstacle detected ahead! Attempting to replan...")
                    try:
                        astar = AStarPlanner(currPoint, endPoint)
                        waypoints = astar.plan()
                        robot.generate_path(waypoints)
                        waiting_for_replan = False
                        last_replan_time = None
                    except Exception:
                        # No path found now: enter waiting mode and set timer
                        waiting_for_replan = True
                        last_replan_time = time.time()
            else:
                # Attempt replanning every 4 seconds while mobile obstacle may move
                now = time.time()
                if last_replan_time is None:
                    last_replan_time = now
                if now - last_replan_time >= 4.0:
                    print("Retrying replanning after wait...")
                    try:
                        astar = AStarPlanner((robot.x, robot.y), endPoint)
                        waypoints = astar.plan()
                        robot.generate_path(waypoints)
                        waiting_for_replan = False
                        last_replan_time = None
                    except Exception:
                        # still no path, schedule next attempt
                        last_replan_time = now
            
        else:
            pygame.display.set_caption("No Path Found")

        
        pygame.display.flip()
        clock.tick(30)

pygame.quit()