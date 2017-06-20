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
    TWO_FRONT_WALLS = 2.1,
    TWO_ADJACENT_WALLS = 2.2,
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
    'wall_sensor_threshold': 200
}

sensors = {}


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


def get_robot_state(robot):
    # TODO
    state = RobotState.NO_WALL
    log('declared state: {0}'.format(state))
    return state


def corner_found(state):
    return state == RobotState.TWO_ADJACENT_WALLS or state == RobotState.THREE_WALLS


class StopWatch:
    def __init__(self):
        self.timer = time.clock()

    def get_time_seconds(self):
        log('timer: {0}s'.format(time.clock() - self.timer))
        return time.clock() - self.timer

    def begin(self):
        self.timer = time.clock()


class Action(Enum):
    MOVE_FORWARD = 0,
    TURN_RIGHT = 1,
    TURN_LEFT = 2,
    MOVE_BACKWARD = 3,
    STOP = 4,
    EPSILON_TURN_RIGHT = 5,
    EPSILON_TURN_LEFT = 6


class Status:
    def __init__(self, verdict, message=None):
        self.verdict = verdict
        self.message = message

    def show_verdict(self):
        log('verdict is {0}, message is: "{1}"'.format(self.verdict, self.message))


def robot_step(robot, speed):
    robot.setSpeed(speed[0], speed[1])
    robot.step(int(constants['time_step']))


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


def my_cmp(a, b):
    if a + constants['eps'] < b:
        return -1
    if b + constants['eps'] < a:
        return +1
    return 0


def compares_mod(mod, p1, p2):
    if mod == 0:
        return my_cmp(p1, constants['p-value']) > 0 or my_cmp(p2, constants['p-value']) > 0
    elif mod == 1:
        return my_cmp(p1, constants['p-value']) > 0 and my_cmp(p2, constants['p-value']) > 0
    elif mod == 2:
        return my_cmp(p1, constants['p-value']) < 0 and my_cmp(constants['p-value'], p2) < 0
    elif mod == 3:
        return my_cmp(p1, constants['p-value']) > 0 and my_cmp(constants['p-value'], p2) > 0


def opposite_direction(direction):
    if direction == Action.TURN_LEFT:
        return Action.TURN_RIGHT
    return Action.TURN_LEFT


def justify_robot(robot, initial_direction=Action.TURN_LEFT):
    max_value = -1234
    direction = initial_direction
    angle = None
    while True:
        do_action(robot, direction, angle=constants['epsilon_angle'],
                  desired_speed=constants['normal_speed'] / 4)
        do_action(robot, Action.STOP)
        cur_value = sensors['distance_sensor'][2].getValue()
        log('sensor value = {0:.02f}'.format(cur_value))
        if max_value < cur_value:
            max_value = cur_value
            angle = compute_odometry(robot)['da']
        if max_value - cur_value > max_value * 0.95 and max_value > constants['wall_sensor_threshold']:
            direction = opposite_direction(direction)
            break
    log('max_value: {0}'.format(max_value))
    cur_angle = compute_odometry(robot)['da']
    do_action(robot, direction, angle=math.fabs(cur_angle - angle),
              desired_speed=constants['normal_speed'])
    do_action(robot, Action.STOP)


def find_corner(robot):
    state = get_robot_state(robot)
    stop_watch = StopWatch()
    stop_watch.begin()
    while True:
        if corner_found(state):
            return Status(True, 'Corner Found')
        init_state = state
        while init_state == state:
            do_action(robot, Action.MOVE_FORWARD)
            state = get_robot_state(robot)
            if stop_watch.get_time_seconds() > constants['find_corner_timeout']:
                return Status(False, 'Timeout Reached'.format())
        if state == RobotState.NO_WALL:
            p = random()
            if p < constants['chance_to_turn_right']:
                do_action(robot, Action.TURN_RIGHT)
                # we have no wall around here yet
                continue
        # state != NO_WALL
        # face the right side of the robot to the walls
        justify_robot(robot)
        state = get_robot_state(robot)


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


def test_justify(robot):
    justify_robot(robot)


def test_wall_detection(robot):
    pass


def test_free_rotation(robot):
    while True:
        do_action(robot, Action.TURN_RIGHT)


def stay_still(robot):
    while True:
        values = [sensors['distance_sensor'][i].getValue() for i in range(8)]
        for i in range(8):
            log('sensor #{0}: {1:.0f}'.format(i, values[i]))
        do_action(robot, Action.STOP)


def main(robot):
    setup(robot)
    test_justify(robot)
    # stay_still(robot)
    # test_free_rotation(robot)
    # find_corner(robot)
    # test(robot)
    return 0


if __name__ == '__main__':
    read_input()
    # graphics.draw_map((n, m), obstacles)
    main(DifferentialWheels())
