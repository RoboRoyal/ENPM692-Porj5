
#Dakota Abernathy
#ENPM692-Proj5

import math
import pygame
import time
from random import randint

HEIGHT = 300
WIDTH = 400
SCALE = 2

board = None
start = None
target = None
real_time = False

WHITE = (255, 255, 255)
BLACK = (0, 0, 0, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
CYAN = (0, 255, 255)
MAGENTA = (255, 0, 255)
YELLOW = (255, 255, 0)

nodes_visited = []
all_nodes = None
path = []
SQRT2 = math.sqrt(2)
nodes = None
MAX_SEPARATION = 40

BOT_RADIUS = 10
OBSTACLE_CLEARANCE = 5
CLEARANCE = BOT_RADIUS + OBSTACLE_CLEARANCE

THRESHOLD = 20
itt = 0
SLOW = True

def update(node):
    global itt
    draw_node(node)
    if itt % 5 == 0:
        pygame.display.update()
        pygame.event.get()
    itt += 1

# distance between two points
def distance(x1,y1,x2,y2):
    return math.sqrt(pow((x2-x1), 2)+pow((y2-y1), 2))


# class to keep track of each place visited
class Node:
    def __init__(self, x, y, parent, dist=0):
        self.x = x
        self.y = y
        self.parent = parent
        self.h = 0
        if parent:
            self.path_length = parent.path_length + dist
            self.g = parent.g + 1
        else:
            self.path_length = 0
            self.g = 0

    def length(self):
        if not self.parent:
            return 0
        return distance(self.x, self.y, self.parent.x, self.parent.y)

    def heuristic(self):  # a* heuristic
        return distance(self.x, self.y, target.x, target.y) + self.g

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __str__(self):
        return "["+str(self.x)+", "+str(self.y) + "]"

    def __lt__(self, other):
        return self.path_length < other.path_length

def draw_node(node, color=CYAN):
    pygame.draw.line(board, color, [node.x * SCALE, (HEIGHT - node.y) * SCALE],
                     [node.parent.x * SCALE, (HEIGHT - node.parent.y) * SCALE], SCALE)

# makes default board
def make_board():
    global board
    pygame.init()
    board = pygame.display.set_mode((int(WIDTH * SCALE), int(HEIGHT * SCALE)))
    pygame.display.set_caption("Path finding algorithm")
    board.fill(WHITE)

    # easy
    pygame.draw.circle(board, BLACK, [90 * SCALE, (HEIGHT - 70) * SCALE], 35 * SCALE)
    pygame.draw.ellipse(board, BLACK, [186 * SCALE, (HEIGHT - 175) * SCALE, 120 * SCALE, 60 * SCALE], 0 * SCALE)

    # Line Segment
    pygame.draw.polygon(board, BLACK,
                        [(48 * SCALE, (HEIGHT - 108) * SCALE), (37 * SCALE, (HEIGHT - 124) * SCALE),
                         (159 * SCALE, (HEIGHT - 210) * SCALE), (170 * SCALE, (HEIGHT - 194) * SCALE)])

    # C shape
    pygame.draw.polygon(board, BLACK,  # back
                        [(200 * SCALE, (HEIGHT - 270) * SCALE), (210 * SCALE, (HEIGHT - 270) * SCALE),
                         (210 * SCALE, (HEIGHT - 240) * SCALE), (200 * SCALE, (HEIGHT - 240) * SCALE)])
    pygame.draw.polygon(board, BLACK,  # top
                        [(200 * SCALE, (HEIGHT - 280) * SCALE), (230 * SCALE, (HEIGHT - 280) * SCALE),
                         (230 * SCALE, (HEIGHT - 270) * SCALE), (200 * SCALE, (HEIGHT - 270) * SCALE)])
    pygame.draw.polygon(board, BLACK,  # bottom
                        [(200 * SCALE, (HEIGHT - 240) * SCALE), (230 * SCALE, (HEIGHT - 240) * SCALE),
                         (230 * SCALE, (HEIGHT - 230) * SCALE), (200 * SCALE, (HEIGHT - 230) * SCALE)])

    # Polygon ---- whats the error allowed? lot of rounding and re-rounding
    pygame.draw.polygon(board, BLACK,  # why is this so ugly
                        [(354 * SCALE, (HEIGHT - 138) * SCALE), (380 * SCALE, (HEIGHT - 170) * SCALE),
                         (380 * SCALE, (HEIGHT - 115) * SCALE), (328 * SCALE, (HEIGHT - 63) * SCALE),
                         (286 * SCALE, (HEIGHT - 105) * SCALE), (325 * SCALE, (HEIGHT - 143) * SCALE)])


# check if point in circle
def in_circle(x, y):
    if math.pow(x - 90, 2) + math.pow(y - 70, 2) >= math.pow(35 + CLEARANCE, 2):
        return False
    return True


# check if point in ellipse
def in_ellipse(x, y):
    center_x = 246
    center_y = 146
    horizontal_axis = 60 + CLEARANCE
    vertical_axis = 30 + CLEARANCE
    if ((math.pow(x - center_x, 2) / math.pow(horizontal_axis, 2)) +
            (math.pow(y - center_y, 2) / math.pow(vertical_axis, 2))) <= 1:
        return True
    return False


# check if point in C-shape
def in_c_shape(x, y):
    if (x >= 200 - CLEARANCE and x <= 210 + CLEARANCE and y <= 280 + CLEARANCE and y >= 230 - CLEARANCE) or \
       (x >= 210 - CLEARANCE and x <= 230 + CLEARANCE and y >= 270 - CLEARANCE and y <= 280 + CLEARANCE) or \
       (y >= 230 - CLEARANCE and y <= 240 + CLEARANCE and x >= 210 - CLEARANCE and x <= 230 + CLEARANCE):
        return True
    return False


# check if point in weird polygon
def in_poly(x, y):
    if ((y - 1 * x + 181.6 - CLEARANCE) < 0 and (y + 0.3 * x - 239.9 - CLEARANCE) < 0
            and (y + 249.2 * x - 95054 - CLEARANCE) < 0 and (y - x + 265 + CLEARANCE) > 0
            and (y + x - 389.3 + CLEARANCE) > 0) or ((y - 1.13 * x + 260.75 - CLEARANCE) < 0
            and (y + 249.2 * x - 95054 - CLEARANCE) < 0 and (y + .3 * x - 240.6 + CLEARANCE) > 0):
        return True
    return False


# check if point in rotated rectangle
def in_line_segment(x, y):
    if (y + 1.4 * x - 176.5 + CLEARANCE) > 0 and (y - 0.7 * x - 74.4 + CLEARANCE) > 0 \
            and (y - 0.7 * x - 98.8 - CLEARANCE) < 0 and (y + 1.4 * x - 438.1 - CLEARANCE) < 0:
        return True
    return False



# check if point is in any obstacle
def in_obstacle(x, y):
    if in_circle(x, y) or in_ellipse(x, y) or in_c_shape(x, y) or \
            in_line_segment(x, y) or in_poly(x, y):
        return True
    return False

def is_close(x, y, x_target, y_target):
    return distance(x, y, x_target, y_target) <= THRESHOLD

# check if point inside boundaries and not in any obstacle
def point_valid(x, y, talk=True):
    if x < 0 or x >= WIDTH:
        if talk:
            print("X is outside of boundary [0,", WIDTH, "]")
        return False
    if y < 0 or y > HEIGHT:
        if talk:
            print("Y is outside of boundary [0,", HEIGHT, "]")
        return False
    if in_obstacle(x, y):
        if talk:
            print("Point is inside an obstacle")
        return False
    return True


# gets single valid point from user
def get_point_from_user(word):
    valid = False
    while not valid:
        x = int(input("Enter the X coordinate of the "+word+" point: "))
        y = int(input("Enter the Y coordinate of the " + word + " point: "))
        valid = point_valid(x, y, True)
    return x, y


# get single valid random point
def random_point():
    valid = False
    while not valid:
        x = randint(0, WIDTH)
        y = randint(0, HEIGHT)
        valid = point_valid(x, y, False)
    return x, y


# gets valid start and target point
def get_initial_conditions(human=True):
    if human:
        x1, y1 = get_point_from_user("start")
        x2, y2 = get_point_from_user("target")
    else:
        x1, y1 = random_point()
        x2, y2 = random_point()
    return Node(x1, y1, None), Node(x2, y2, None)


# returns list of nodes of all valid neighbors
def get_neighbors(parent):
    neighbors = []
    for i in range(-1, 2):
        for j in range(-1, 2):
            dist = SQRT2
            if i == j == 0:
                continue
            if point_valid(parent.x + i, parent.y + j, False):
                if i == 0 or j == 0:
                    dist = 1
                new_node = Node(parent.x + i, parent.y + j, parent, dist)
                neighbors.append(new_node)
    return neighbors



def valid_line(x1,y1,x2,y2):
    for point in get_points_in_line(x1, y1, x2, y2):
        if not point_valid(point[0], point[1], False):
            return False
    return True


def closest_point(x,y):
    min_distance = 1000000
    closest = start
    for node in nodes_visited:
        dist = distance(node.x, node.y, x, y)
        if dist <= min_distance:
            min_distance = dist
            closest = node
    return closest

def RRT(start, goal):
    found = False
    nodes_visited.append(start)
    while not found:
        x, y = random_point()
        parent = closest_point(x,y)
        if not valid_line(x, y, parent.x, parent.y) or distance(x, y, parent.x, parent.y) > MAX_SEPARATION:
            continue
        new_node = Node(x,y, parent)
        nodes_visited.append(new_node)
        update(new_node)
        if is_close(new_node.x, new_node.y, goal.x, goal.y):
            goal.parent = new_node
            return True

def reset_rrt(start, goal):
    global nodes_visited
    nodes_visited = []
    start.parent = None
    goal.parent = None

def duel_rrt(start, goal):
    global path
    RRT(start, goal)
    back_track(nodes_visited[-1])
    s_g = path
    all_nodes = nodes_visited.copy()
    reset_rrt(start, goal)
    make_board()
    add_points()
    draw_path(s_g, RED)
    print("Done first RRT: Start to Goal")
    path = []
    RRT(goal, start)
    back_track(nodes_visited[-1])
    g_s = path
    make_board()
    add_points()
    draw_path(g_s, GREEN)
    print("Done second RRT: Goal to Start")
    for n in nodes_visited:
        all_nodes.append(n)
    return s_g, g_s, all_nodes

def optimise(s_g):
    for i in range(0, len(s_g) - 1):
            for j in range(len(s_g) - 1, i + 2, -1):
                if j >= len(s_g):
                    continue
                if valid_line(s_g[i].x, s_g[i].y, s_g[j].x, s_g[j].y):
                    s_g[j].parent = s_g[i]
                    for p in range(j - i - 1):
                        s_g.pop(i + 1)
                    return s_g
    return s_g

def full_optimise(s_g):
    for i in range(0, len(s_g) - 1):
        optimise(s_g)
    for i in range(0, len(s_g) - 1):
        optimise(s_g)
    for i in range(len(s_g) - 1, 1, -1):
        s_g[i].parent = s_g[i-1]
    return s_g

def path_length(p):
    total = 0
    for n in p:
        total += n.length()
    return total

def hybrid_rrt(start, goal):
    global path
    s_g, g_s, all_nodes = duel_rrt(start, goal)
    print("Optimise S to G")
    s_g = full_optimise(s_g)
    draw_path(s_g, YELLOW)
    if SLOW:
        time.sleep(3)
    print("Optimise G to S")
    g_s = full_optimise(g_s)
    draw_path(g_s, MAGENTA)
    if SLOW:
        time.sleep(3)
    if path_length(s_g) < path_length(g_s):
        print("s_g")
        path = s_g
    else:
        print("g_s")
        path = g_s
    draw_path(path, BLACK)
    if SLOW:
        time.sleep(2.5)
    return path

# work back from target to get path to start
def back_track(end_node):
    n = end_node
    while n:
        path.append(n)
        n = n.parent
    path.reverse()

# draws a single point with a threshold area around it
def draw_point_with_threshold(point, color=GREEN):
    pygame.draw.circle(board, color, [point.x * SCALE, (HEIGHT - point.y) * SCALE], THRESHOLD * SCALE)

def draw_path(p = path, color = RED):
    for n in p:
        if n and n.parent:
            draw_node(n, color)
        pygame.display.update()
        pygame.event.get()
        if SLOW:
            time.sleep(.05)
# adds all visited nodes, the path, start and end points to board
def add_points(p = path):
    # print("Visited: ", len(nodes_visited))

    draw_point_with_threshold(start)
    draw_point_with_threshold(target, RED)
    pygame.display.update()

    draw_path(p)
    draw_point_with_threshold(start)
    draw_point_with_threshold(target, RED)
    pygame.display.update()
    # print("Path: ", len(p))

    if SLOW:
        time.sleep(1.5)


def get_points_in_line(x1, y1, x2, y2):
    points = []
    for x in range(min(x1, x2), max(x1, x2) + 1):
        for y in range(min(y1, y2), max(y1, y2) + 1):
            if x1 - x2 == 0:
                points.append([x, y])
            elif y == int(((x - x1) * (y1 - y2)) / (x1 - x2) + y1):
                points.append([x, y])
    return points

if __name__ == "__main__":
    #start, target = get_initial_conditions(False)
    start = Node(10, 10, None)
    target = Node(250, 250, None)
    print("Finding path...")
    real_time = True
    print(start, target)
    if real_time:
        make_board()
        add_points()

    # RRT(start, target)
    # duel_rrt(start, target)
    path = hybrid_rrt(start, target)
    print("found")
    make_board()
    add_points(path)
    pygame.display.update()
    print("Done")
    for i in range(501):
        time.sleep(0.1)
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.QUIT:
                raise SystemExit