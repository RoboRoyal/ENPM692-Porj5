
#Dakota Abernathy
#ENPM692-Proj5

import math
import pygame
import time
import random
from random import randint
random.seed(1234)

GRAIN = 30
HEIGHT = 10 * GRAIN
WIDTH = 10 * GRAIN
SCALE = 2
SCALAR = GRAIN * SCALE

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
MAX_SEPARATION = 30

BOT_RADIUS = 10
OBSTACLE_CLEARANCE = 10
CLEARANCE = BOT_RADIUS + OBSTACLE_CLEARANCE

THRESHOLD = 20
itt = 0
SLOW = False


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
    def __init__(self, x, y, parent = None, dist=0):
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
    pygame.draw.circle(board, BLACK, [2 * SCALAR, (HEIGHT - 2 * GRAIN) * SCALE], 1 * SCALAR)
    pygame.draw.circle(board, BLACK, [2 * SCALAR, (HEIGHT - 8 * GRAIN) * SCALE], 1 * SCALAR)
    pygame.draw.rect(board, BLACK, pygame.Rect(
        .25 * SCALAR, (HEIGHT - 5.75 * GRAIN) * SCALE, 1.5 * SCALAR, 1.5 * SCALAR))
    pygame.draw.rect(board, BLACK, pygame.Rect(
        3.75 * SCALAR, (HEIGHT - 5.75 * GRAIN) * SCALE, 2.5 * SCALAR, 1.5 * SCALAR))
    pygame.draw.rect(board, BLACK, pygame.Rect(
        7.25 * SCALAR, (HEIGHT - 4 * GRAIN) * SCALE, 1.5 * SCALAR, 2 * SCALAR))


def in_circle(x, y):  # check if point in lower circle
    if math.pow(x - 2 * GRAIN, 2) + math.pow(y - (HEIGHT - 2 * GRAIN), 2) >= math.pow(1 * GRAIN + CLEARANCE, 2):
        return False
    return True


def in_circle_2(x, y):  # check if point in upper circle
    if math.pow(x - 2 * GRAIN, 2) + math.pow(y - (HEIGHT - 8 * GRAIN) , 2) >= math.pow(1 * GRAIN + CLEARANCE, 2):
        return False
    return True


def in_rect(x, y):    # check if point in rectangle
    if .25 * GRAIN - CLEARANCE <= x <= 1.75 * GRAIN + CLEARANCE and \
            5.75 * GRAIN + CLEARANCE >= y >= 4.25 * GRAIN - CLEARANCE:
        return True
    return False


def in_rect_2(x, y):
    if 3.75 * GRAIN - CLEARANCE <= x <= 6.25 * GRAIN + CLEARANCE and \
            5.75 * GRAIN + CLEARANCE >= y >= 4.25 * GRAIN - CLEARANCE:
        return True
    return False


def in_rect_3(x, y):
    if 7.25 * GRAIN - CLEARANCE <= x <= 8.75 * GRAIN + CLEARANCE and \
            4 * GRAIN + CLEARANCE >= y >= 2 * GRAIN - CLEARANCE:
        return True
    return False


# check if point is in any obstacle
def in_obstacle(x, y):
    if in_circle(x, y) or in_circle_2(x, y) or in_rect(x, y) or in_rect_2(x, y) or in_rect_3(x, y):
        return True
    return False

def is_close(x, y, x_target, y_target):
    return distance(x, y, x_target, y_target) <= THRESHOLD

# check if point inside boundaries and not in any obstacle
def point_valid(x, y, talk=True):
    if x < CLEARANCE or x >= WIDTH - CLEARANCE:
        if talk:
            print("X is outside of boundary [0,", WIDTH, "]")
        return False
    if y < CLEARANCE or y > HEIGHT - CLEARANCE:
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
        time.sleep(1)
    print("Optimise G to S")
    g_s = full_optimise(g_s)
    draw_path(g_s, MAGENTA)
    if SLOW:
        time.sleep(1)
    if path_length(s_g) < path_length(g_s):
        print("s_g")
        path = s_g
    else:
        print("g_s")
        path = g_s
    draw_path(path, BLACK)
    if SLOW:
        time.sleep(1.5)
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

    draw_path(p, MAGENTA)
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

def sanity_check():
    for run in range(1000):
        point = random_point()
        if point_valid(point[0], point[1]):
            pygame.draw.circle(board, RED, (point[0] * SCALE, HEIGHT * SCALE - point[1] * SCALE), 1)
        else:
            pass
            #pygame.draw.circle(board, GREEN, point, 1)
        pygame.display.update(     )
        pygame.event.get()


if __name__ == "__main__":
    #start, target = get_initial_conditions(False)
    start = Node(4 * GRAIN, 1 * GRAIN, None)
    target = Node(9 * GRAIN, HEIGHT - 1 * GRAIN, None)
    print("Finding path...")
    real_time = True
    print(start, target)
    if real_time:
        make_board()
        add_points()
    #pygame.draw.circle(board, MAGENTA, (89, HEIGHT - (106 * SCALE)), 3)
    #pygame.display.update()
    # sanity_check()
    # RRT(start, target)
    # duel_rrt(start, target)
    print("Path: ")
    path = hybrid_rrt(start, target)
    for i in path:
        print(i)
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
