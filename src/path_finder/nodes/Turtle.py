#!/usr/bin/env python

import time

time.sleep(6) # wait for everything to load

import rospy
rospy.loginfo("Im Alive!")

import math
import tf
from geometry_msgs.msg import Twist, Point
# from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
try:
    from queue import PriorityQueue
except ImportError:
    from Queue import PriorityQueue

import random
from random import randint



rospy.init_node("move_robot")
pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
velocity_msg = Twist()
rate = rospy.Rate(10)
tf_listener = tf.TransformListener()

odom_frame = 'odom'
base_frame = 'base_footprint'
try:
    tf_listener.waitForTransform(odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
    base_frame = 'base_footprint'
except (tf.Exception, tf.ConnectivityException, tf.LookupException):
    try:
        tf_listener.waitForTransform(odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
        base_frame = 'base_link'
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
        rospy.signal_shutdown("tf Exception")

random.seed(1234)

# global vars not used for set-up
robot_move_speed = .25
robot_turn_speed = .4
goal_threshold = .5
angular_threshold = 0.02
POINT_THRESH = .16

GRAIN = 30
HEIGHT = 10 * GRAIN
WIDTH = 10 * GRAIN


BOT_RADIUS = 10
OBSTACLE_CLEARANCE = 5
CLEARANCE = BOT_RADIUS + OBSTACLE_CLEARANCE
THRESHOLD = 20
nodes_visited = []
path = []
SQRT2 = math.sqrt(2)
nodes = None
found_path = True
MAX_SEPARATION = 30


SLOW = False

# Returns position of bot according to odometry data
def get_odom_data():
    try:
        (trans, rot) = tf_listener.lookupTransform(odom_frame, base_frame, rospy.Time(0))
        rotation = euler_from_quaternion(rot)
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("TF Exception. (I can only assume that means 'The F@ck is this Exception')")
        return

    return Point(*trans), rotation[2]



# converts degrees to bot radians
def angle_to_dir(ang):
    if ang <= 180:
        return ang * math.pi / 180
    return -1 * (360 - ang) * math.pi / 180

# stop all motion in bot
def stop():
    velocity_msg.linear.x = 0.0
    velocity_msg.angular.z = 0.0
    pub.publish(velocity_msg)
    rate.sleep()
    pub.publish(velocity_msg)
    rate.sleep()
    time.sleep(.15)



# ----------------------------------


# distance between two points
def distance(x1, y1, x2, y2):
    return math.sqrt(pow((x2-x1), 2)+pow((y2-y1), 2))


def round_to_n(num, n=3):
    return n * round(num / n)


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

# work back from target to get path to start
def back_track(end_node):
    n = end_node
    while n:
        path.append(n)
        n = n.parent
    path.reverse()

# determines if the bot should spin left or right to turn towards specified goal
def left_spin(start, end):
    if (start >= 0) == (end >= 0):
        return start < end
    if (start >= 0) and (end <= 0):
        return abs(start - end) > math.pi
    else:
        return abs(start - end) < math.pi
    return True


# returns the difference in angle between the two given orientations
def get_diff(rot1, rot2):
    if (rot1 >= 0) == (rot2 >= 0):
        return abs(rot1) - abs(rot2)
    tmp = abs(rot1) + abs(rot2)
    if tmp > math.pi:
        return 2 * math.pi - tmp
    return tmp


# turns bot in direction of the specifies orientation
def turn(direction):
    blank, rot = get_odom_data()
    while angular_threshold < abs(get_diff(rot, direction)):
        turn_speed = robot_turn_speed
        if abs(get_diff(rot, direction)) < .2:
            turn_speed = robot_turn_speed / 4
        if abs(get_diff(rot, direction)) < .05:
            turn_speed = robot_turn_speed / 8

        if left_spin(rot, direction):
            velocity_msg.angular.z = turn_speed
        else:
            velocity_msg.angular.z = -turn_speed

        pub.publish(velocity_msg)
        rate.sleep()
        blank, rot = get_odom_data()
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)
    # stop()


def navigate(target):
    position, rot = get_odom_data()
    x = position.x + 5
    y = position.y + 5
    end_x = float(target.x) / GRAIN
    end_y = float(target.y) / GRAIN
    print(x, y, end_x, end_y)
    if distance(x, y, end_x, end_y) < POINT_THRESH / 2:
        print("Already there: ", distance(x, y, end_x, end_y))
        return
    direction = math.atan2(end_y - y, end_x - x)
    if abs(direction - rot) > angular_threshold:
        turn(direction)
    position, rot = get_odom_data()
    x = position.x + 5
    y = position.y + 5
    rospy.loginfo("Distance to next: " + str(distance(x, y, end_x, end_y)))
    last = distance(x, y, end_x, end_y)
    while distance(x, y, end_x, end_y) > POINT_THRESH and distance(x, y, end_x, end_y) <= last:
        last = distance(x, y, end_x, end_y)
        velocity_msg.linear.x = robot_move_speed
        velocity_msg.angular.z = 0
        pub.publish(velocity_msg)
        rate.sleep()
        position, rot = get_odom_data()
        x = position.x + 5
        y = position.y + 5
        rospy.loginfo("Distance to next: " + str(distance(x, y, end_x, end_y)))
    stop()


######################## HYBRID RRT ###################################


def valid_line(x1,y1,x2,y2):
    for point in get_points_in_line(x1, y1, x2, y2):
        if not point_valid(point[0], point[1], False):
            return False
    return True


def closest_point(x, y):
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
        new_node = Node(x, y, parent)
        nodes_visited.append(new_node)
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
    all_nodes = list(nodes_visited) #.copy()
    reset_rrt(start, goal)
    print("Done first RRT: Start to Goal")
    path = []
    RRT(goal, start)
    back_track(nodes_visited[-1])
    g_s = path
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
    if SLOW:
        time.sleep(1)
    print("Optimise G to S")
    g_s = full_optimise(g_s)
    if SLOW:
        time.sleep(1)
    if path_length(s_g) < path_length(g_s):
        print("s_g")
        path = s_g
    else:
        print("g_s")
        path = g_s
    if SLOW:
        time.sleep(1.5)
    return path

def get_points_in_line(x1, y1, x2, y2):
    points = []
    for x in range(min(x1, x2), max(x1, x2) + 1):
        for y in range(min(y1, y2), max(y1, y2) + 1):
            if x1 - x2 == 0:
                points.append([x, y])
            elif y == int(((x - x1) * (y1 - y2)) / (x1 - x2) + y1):
                points.append([x, y])
    return points


# get single valid random point
def random_point():
    valid = False
    while not valid:
        x = randint(0, WIDTH)
        y = randint(0, HEIGHT)
        valid = point_valid(x, y, False)
    return x, y

############################## END OF HYBRID RRT ##############################################

def conver_from_odom(x, y):
    x = (x + 5) * GRAIN
    y = (y + 5) * GRAIN
    return int(x), int(y)

# initiator of all my problems
if __name__ == "__main__":
    rospy.loginfo("Beep-Boop, here we go again")
    start_time = time.time()
    rospy.loginfo("I've got em in my sights!")
    rospy.loginfo(get_odom_data())


    start = Node(4 * GRAIN, 2 * GRAIN)
    target = Node(9 * GRAIN, HEIGHT - 1 * GRAIN)

    real_x, real_y = conver_from_odom(get_odom_data()[0].x, get_odom_data()[0].y)
    start = Node(real_x, real_y)
    if distance(start.x, start.y, real_x, real_y) > POINT_THRESH:
        print("ERROR!!!!! Bot not in right spot")
        print(distance(start.x, start.y, real_x, real_y))

    path = hybrid_rrt(start, target)
    # path = [Node(270, 270),  Node(100, 100), Node(120, 80), Node(120, 60)]
    # path is now a list of nodes
    if distance(start.x, start.y, path[0].x, path[0].y) > distance(start.x, start.y, path[-1].x, path[-1].y):
        print("reversing path")
        path.reverse()
    rospy.loginfo("Path has " + str(len(path)) + " nodes")
    num = 0
    print("Start: ")
    print(start)
    print("End: ")
    print(target)
    for node in path:
        print(node.x, node.y)

    if len(path) == 1:
        print("No go")
        SystemExit()

    for node in path:
        rospy.loginfo(str(num))
        print(node.x, node.y)
        num += 1
        # go from one node to the next
        navigate(node)

    print("Goal reached")
    stop()
    end_time = time.time()
    rospy.loginfo("Time to complete: {:.4} seconds. Not that its a race or anything....".format(end_time-start_time))

