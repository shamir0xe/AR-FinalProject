import os
import math
from enum import Enum
from random import random

import time

import graphics
from controller import *


class RobotState(Enum):
    NO_WALL = 0,
    ONE_WALL = 1,
    RIGHT_LEFT = 2.1,
    UP_RIGHT = 2.3,
    RIGHT_DOWN = 2.4,
    DOWN_LEFT = 2.5,
    LEFT_UP = 2.6,
    THREE_WALLS = 3


constants = {
    'pi': math.acos(-1),
    'time_step': 8,
    'wheel_radius': 0.0205,
    'axle_length': 0.052,
    'encoder_resolution': 159.23,
    'find_corner_timeout': 10,
    'chance_to_turn_right': 0.5,
    'normal_speed': 100,
    'epsilon_angle': 0.01,
    'eps': 1e-9,
    'wall_sensor_threshold': 100,
    'wall_centimeters_threshold': 3,
    'edge_detection_turn': 5
}

sensors = {}

min_dist = [
    [75, 105, 200, 475, 1365],
    [75, 105, 200, 475, 1365],
    [75, 105, 200, 475, 1365],
    [75, 105, 200, 475, 1365],
    [75, 105, 200, 475, 1365],
    [75, 105, 200, 475, 1365],
    [75, 105, 200, 475, 1365],
    [75, 105, 200, 475, 1365]
]

max_dist = [
    [117, 165, 265, 575, 1665],
    [117, 165, 265, 575, 1665],
    [117, 165, 265, 575, 1665],
    [117, 165, 265, 575, 1665],
    [117, 165, 265, 575, 1665],
    [117, 165, 265, 575, 1665],
    [117, 165, 265, 575, 1665],
    [117, 165, 265, 575, 1665]
]


def compute_odometry(robot):
    l, r = robot.getLeftEncoder(), robot.getRightEncoder()
    data = {
        'dl': l / constants['encoder_resolution'] * constants['wheel_radius'],
        'dr': r / constants['encoder_resolution'] * constants['wheel_radius'],
    }
    data['da'] = (data['dr'] - data['dl']) / constants['axle_length']
    return data


n, m, scale_factor = 0, 0, 0
obstacles = list()


def read_input():
    global n, m, scale_factor, obstacles
    env_file = open(os.path.join('Inputs', 'env.txt'), 'r+')
    with env_file as f:
        i = 0
        for line in f:
            if line is None or line is '\n':
                continue
            tokens = line.split(' ')
            for token in tokens:
                temp = int(token)
                if i == 0:
                    n = temp
                elif i == 1:
                    m = temp
                elif i == 2:
                    scale_factor = temp
                i += 1

    map_file = open(os.path.join('Inputs', 'map.txt'), 'r+')
    with map_file as f:
        i = -1
        for line in f:
            if line is None or line is '\n':
                continue
            i += 1
            cur_line = line.split()
            j = -1
            for temp in cur_line:
                j += 1
                if int(temp) is 1:
                    obstacles.append((i, j))


def log(text):
    """	Show @text in standart output with colors """

    blue = '\033[1;34m'
    off = '\033[1;m'

    print(''.join((blue, '[Log] ', off, str(text))))


def error(text):
    red = '\033[1;31m'
    off = '\033[1;m'

    print(''.join((red, '[Error] ', off, str(text))))


def setup(robot):
    global sensors
    robot.enableEncoders(constants['time_step'])
    distance_sensor = [None for i in range(8)]
    for i in range(8):
        device_name = 'ps{0}'.format(i)
        log('device name: {0}'.format(device_name))
        distance_sensor[i] = robot.getDistanceSensor(device_name)
        distance_sensor[i].enable(constants['time_step'] * 4)
    sensors['distance_sensor'] = distance_sensor
    while math.isnan(distance_sensor[0].getValue()):
        values = [distance_sensor[i].getValue() for i in range(8)]
        for i in range(8):
            log('sensor #{0}: {1:.0f}'.format(i, values[i]))
        do_action(robot, Action.STOP)


def get_centimeters(robot):
    values = [sensors['distance_sensor'][i].getValue() for i in range(8)]
    # for i in range(8):
    #     log('sensor #{0} value: {1}'.format(i, values[i]))
    centimeters = [10 for i in range(8)]
    n = len(min_dist[0])
    for i in range(8):
        value = values[i]
        for j in range(n):
            if cmp(value, min_dist[i][j]) >= 0 and cmp(value, max_dist[i][j]) <= 0:
                centimeters[i] = n + 1 - (j + 1)
        for j in range(n - 1):
            if cmp(value, max_dist[i][j]) >= 0 and cmp(value, min_dist[i][j + 1]) <= 0:
                centimeters[i] = n + 1 - (j + 1.5)
        if cmp(value, max_dist[i][n - 1]) > 0:
            centimeters[i] = 0
    return centimeters


def get_robot_state(robot, debug=False):
    # TODO
    state = RobotState.NO_WALL

    values = get_centimeters(robot)
    if debug:
        for i in range(8):
            log('centimeter from sensor #{0} is {1}'.format(i, values[i]))
    # three walls
    # TODO

    # two walls
    # RIGHT_UP
    if cmp(values[0], constants['wall_centimeters_threshold']) < 0 \
            and cmp(values[2], constants['wall_centimeters_threshold']) < 0:
        return RobotState.UP_RIGHT
    if cmp(values[2], constants['wall_centimeters_threshold']) < 0 \
            and cmp(values[3], constants['wall_centimeters_threshold']) < 0:
        return RobotState.RIGHT_DOWN
    if cmp(values[4], constants['wall_centimeters_threshold']) < 0 \
            and cmp(values[5], constants['wall_centimeters_threshold']) < 0:
        return RobotState.DOWN_LEFT
    if cmp(values[5], constants['wall_centimeters_threshold']) < 0 \
            and cmp(values[7], constants['wall_centimeters_threshold']) < 0:
        return RobotState.LEFT_UP
    if cmp(values[2], constants['wall_centimeters_threshold']) < 0 \
            and cmp(values[5], constants['wall_centimeters_threshold']) < 0:
        return RobotState.RIGHT_LEFT

    # one wall
    for value in values:
        if cmp(value, constants['wall_centimeters_threshold']) < 0:
            return RobotState.ONE_WALL

    return RobotState.NO_WALL


def corner_found(state):
    return not (state == RobotState.NO_WALL or state == RobotState.ONE_WALL)


# stop watch
class StopWatch:
    def __init__(self):
        self.timer = time.clock()

    def get_time_seconds(self):
        # log('timer: {0}s'.format(time.clock() - self.timer))
        return time.clock() - self.timer

    def begin(self):
        self.timer = time.clock()


# all types of action that robot can do
class Action(Enum):
    MOVE_FORWARD = 0,
    TURN_RIGHT = 1,
    TURN_LEFT = 2,
    MOVE_BACKWARD = 3,
    STOP = 4,
    EPSILON_TURN_RIGHT = 5,
    EPSILON_TURN_LEFT = 6


# status of the return values
# can be used for precising the return value
class Status:
    def __init__(self, verdict, message=None):
        self.verdict = verdict
        self.message = message

    def show_verdict(self):
        log('verdict is {0}, message is: "{1}"'.format(self.verdict, self.message))


# one step of robot operation
def robot_step(robot, speed):
    robot.setSpeed(speed[0], speed[1])
    robot.step(int(constants['time_step']))


# doing the desired action, with specific speed and specific angle
# the null action is included as STOP
def do_action(robot, action, desired_speed=None, angle=constants['pi'] / 2):
    wheel_speed = constants['normal_speed']
    if desired_speed is not None:
        wheel_speed = desired_speed
    if action == Action.MOVE_FORWARD:
        speed = [wheel_speed, wheel_speed]
        robot_step(robot, speed)
    elif action == Action.MOVE_BACKWARD:
        speed = [-wheel_speed, -wheel_speed]
        robot_step(robot, speed)
    elif action == Action.TURN_LEFT:
        cur_angle = compute_odometry(robot)['da']
        desired_angle = cur_angle + angle
        speed = [-wheel_speed, wheel_speed]
        while compute_odometry(robot)['da'] < desired_angle:
            robot_step(robot, speed)
    elif action == Action.TURN_RIGHT:
        cur_angle = compute_odometry(robot)['da']
        desired_angle = cur_angle - angle
        speed = [wheel_speed, -wheel_speed]
        while compute_odometry(robot)['da'] > desired_angle:
            robot_step(robot, speed)
    elif action == Action.STOP:
        speed = [0, 0]
        robot_step(robot, speed)
    elif action == Action.EPSILON_TURN_LEFT:
        cur_angle = compute_odometry(robot)['da']
        desired_angle = cur_angle + constants['epsilon_angle']
        speed = [-wheel_speed, wheel_speed]
        while compute_odometry(robot)['da'] < desired_angle:
            robot_step(robot, speed)
        do_action(robot, Action.STOP)
    elif action == Action.EPSILON_TURN_RIGHT:
        cur_angle = compute_odometry(robot)['da']
        desired_angle = cur_angle - constants['epsilon_angle']
        speed = [wheel_speed, -wheel_speed]
        while compute_odometry(robot)['da'] > desired_angle:
            robot_step(robot, speed)
        do_action(robot, Action.STOP)


# comparing the two double variables
def my_cmp(a, b):
    if a + constants['eps'] < b:
        return -1
    if b + constants['eps'] < a:
        return +1
    return 0


# piece of shit
# should be deleted
def compares_mod(mod, p1, p2):
    # DELETED
    if mod == 0:
        return my_cmp(p1, constants['p-value']) > 0 or my_cmp(p2, constants['p-value']) > 0
    elif mod == 1:
        return my_cmp(p1, constants['p-value']) > 0 and my_cmp(p2, constants['p-value']) > 0
    elif mod == 2:
        return my_cmp(p1, constants['p-value']) < 0 and my_cmp(constants['p-value'], p2) < 0
    elif mod == 3:
        return my_cmp(p1, constants['p-value']) > 0 and my_cmp(constants['p-value'], p2) > 0


# opposite direction of the current direction
def opposite_direction(direction):
    if direction == Action.TURN_LEFT:
        return Action.TURN_RIGHT
    if direction == Action.TURN_RIGHT:
        return Action.TURN_LEFT
    if direction == Action.EPSILON_TURN_LEFT:
        return Action.EPSILON_TURN_RIGHT
    if direction == Action.EPSILON_TURN_RIGHT:
        return Action.EPSILON_TURN_LEFT


def obstacle_avoid_forward_move(robot, properties):
    centimeters = get_centimeters(robot)
    # for i in range(8):
    #     log('centimeter from sensor#{0} is {1}'.format(i, centimeters[i]))
    if properties['turn'] >= constants['edge_detection_turn']:
        # Edge detected
        if properties['turn'] == constants['edge_detection_turn']:
            properties['turn'] += 1
            Status(True, 'Edge Detected').show_verdict()
            do_action(robot, Action.EPSILON_TURN_LEFT)
            do_action(robot, Action.EPSILON_TURN_LEFT)
            properties['counter'] = 0
            return
        if cmp(centimeters[2], constants['wall_centimeters_threshold']) > 0:
            if properties['turn'] == constants['edge_detection_turn'] + 1:
                properties['turn'] += 1
                properties['counter'] *= 2
                do_action(robot, Action.MOVE_FORWARD)
                return
            else:
                if properties['counter'] > 0:
                    properties['counter'] -= 1
                    do_action(robot, Action.MOVE_FORWARD)
                else:
                    properties['turn'] = 0
                    return
        else:
            properties['counter'] += 1
            do_action(robot, Action.MOVE_FORWARD)
            return
    if cmp(centimeters[1], 3) <= 0:
        do_action(robot, Action.EPSILON_TURN_LEFT)
        for i in range(2):
            do_action(robot, Action.MOVE_FORWARD)
        if properties['last_action'] is not Action.EPSILON_TURN_LEFT:
            properties['turn'] += 1
            properties['last_action'] = Action.EPSILON_TURN_LEFT
        else:
            properties['turn'] = 0
        return
    if cmp(centimeters[1], 5) > 0 \
            and cmp(centimeters[2], constants['wall_centimeters_threshold']) <= 0:
        do_action(robot, Action.EPSILON_TURN_RIGHT)
        for i in range(2):
            do_action(robot, Action.MOVE_FORWARD)
        if properties['last_action'] is not Action.EPSILON_TURN_RIGHT:
            properties['turn'] += 1
            properties['last_action'] = Action.EPSILON_TURN_RIGHT
        else:
            properties['turn'] = 0
        return
    if cmp(centimeters[6], 3) <= 0:
        do_action(robot, Action.EPSILON_TURN_RIGHT)
        return
    do_action(robot, Action.MOVE_FORWARD)
    properties['last_action'] = Action.MOVE_FORWARD
    properties['turn'] = 0


def right_is_the_best(robot):
    centimeters = get_centimeters(robot)
    max_value = max(centimeters)
    return centimeters[2] == max_value


# justify robot alongside the wall
# in order to keep the right side of the robot facing the wall
def justify_robot(robot, initial_direction=Action.TURN_LEFT):
    log('justify called!')
    max_value = -1234
    centimeter_min_value = -1234
    direction = initial_direction
    angle = None
    if right_is_the_best(robot):
        return
    while True:
        do_action(robot, direction, angle=constants['epsilon_angle'],
                  desired_speed=constants['normal_speed'])
        do_action(robot, Action.STOP)
        cur_value = sensors['distance_sensor'][2].getValue()
        centimeters = get_centimeters(robot)
        # log('sensor value = {0:.02f}'.format(cur_value))
        if max_value < cur_value:
            max_value = cur_value
            centimeter_min_value = centimeters[2]
            angle = compute_odometry(robot)['da']
        if max_value - cur_value > max_value * 0.95 \
                and cmp(centimeter_min_value, constants['wall_centimeters_threshold']) <= 0:
            direction = opposite_direction(direction)
            break
    # log('max_value: {0}'.format(max_value))
    cur_angle = compute_odometry(robot)['da']
    do_action(robot, direction, angle=math.fabs(cur_angle - angle),
              desired_speed=constants['normal_speed'])
    do_action(robot, Action.STOP)


# find the first corner that the robot detects
def find_corner(robot):
    # TODO
    properties = {
        'turn': 0,
        'last_action': None
    }
    state = get_robot_state(robot)
    stop_watch = StopWatch()
    while True:
        if corner_found(state):
            return Status(True, 'Corner Found!')
        init_state = state
        while init_state == state:
            obstacle_avoid_forward_move(robot, properties)
            state = get_robot_state(robot)
            if stop_watch.get_time_seconds() > constants['find_corner_timeout']:
                return Status(False, 'Timeout Reached!')
        for i in range(100):
            obstacle_avoid_forward_move(robot, properties)
        log('state = {0}'.format(state))
        if state == RobotState.NO_WALL:
            p = random()
            if p < constants['chance_to_turn_right']:
                do_action(robot, Action.TURN_RIGHT)
                # we have no wall around here yet
                continue
        # state != NO_WALL
        # face the right side of the robot to the walls
        justify_robot(robot)
        state = get_robot_state(robot, debug=True)
        log('state after justify = {0}'.format(state))


# testing moving and rotating
def test(robot):
    stop_watch = StopWatch()
    while stop_watch.get_time_seconds() < .05:
        do_action(robot, Action.MOVE_FORWARD)
    stop_watch.begin()
    while stop_watch.get_time_seconds() < .02:
        do_action(robot, Action.MOVE_BACKWARD)
    do_action(robot, Action.TURN_RIGHT)
    do_action(robot, Action.TURN_LEFT)
    do_action(robot, Action.STOP)


# testing justify alongside the wall
def test_justify(robot):
    justify_robot(robot)


# testing free rotation
def test_free_rotation(robot):
    while True:
        do_action(robot, Action.TURN_RIGHT)


def stay_still(robot):
    while True:
        values = [sensors['distance_sensor'][i].getValue() for i in range(8)]
        for i in range(8):
            log('sensor #{0}: {1:.0f}'.format(i, values[i]))
        do_action(robot, Action.STOP)


def test_robot_state(robot):
    while True:
        state = get_robot_state(robot)
        log('declared state: {0}'.format(state))
        do_action(robot, Action.STOP)


def main(robot):
    setup(robot)
    # test_robot_state(robot)
    # test_justify(robot)
    # stay_still(robot)
    # test_free_rotation(robot)
    status = find_corner(robot)
    status.show_verdict()
    # test(robot)
    return 0


if __name__ == '__main__':
    read_input()
    # graphics.draw_map((n, m), obstacles)
    main(DifferentialWheels())
