import os
import math
from enum import Enum
from random import random
from random import randint

import time

import copy

import numpy as np

import graphics

import sys

from math import fabs
from controller import *

sys.setrecursionlimit(5555555)


class RobotState(Enum):
    NO_WALL = 0,
    ONE_WALL = 1,
    RIGHT_LEFT = 2.1,
    UP_RIGHT = 2.3,
    RIGHT_DOWN = 2.4,
    DOWN_LEFT = 2.5,
    LEFT_UP = 2.6,
    THREE_WALLS = 3


class WorkingMode(Enum):
    SIMULATION = 0,
    REAL_WORLD = 1


working_mode = WorkingMode.REAL_WORLD
best_variance = None
best_ratio = None
LEDs = None
escape = False

constants = {
    'total_turn': 1e10,
    'pi': math.acos(-1),
    'time_step': 8,
    'wheel_radius': 0.0205,
    'axle_length': 0.052,
    'encoder_resolution': 159.23,
    'find_corner_timeout': 10,
    'chance_to_turn_right': 0.5,
    'normal_speed': 200,
    'epsilon_angle': 0.01,
    'eps': 1e-9,
    'wall_sensor_threshold': 100,
    'wall_centimeters_threshold': 3,
    'wall_centimeters_threshold1': 3,
    'wall_centimeters_threshold7': 3,
    'sensor_one_wall_min': 2.5,
    'sensor_one_wall_max': 4,
    'sensor_determination_count': 3,
    'justify_threshold': 1,
    'rotate_right': {
        '0': 1,
        '1': 1,
        '7': 1
    },
    'M': 1000,
    'epsilon_of_EG': 0.03,
    'particle_update_threshold': 10,
    'turn_left_delta_x': 0,
    'turn_right_delta_x': 12,
    'move_particle_sigma': 4,
    'transform_rate': 113.733471342,
    'delta_correction': 0.85,
    'variance_threshold': 4444,
    'sensor_one_delta': 1,
    'variance_multiplier': 4,
    'radius_threshold': 5,
    'kidnap_threshold': {
        'max': 0.5,
        'min': 0.3
    },
    'ratio_edge_right': 0.4
}

sensors = {}

dirs = [
    [+1, 0],
    [0, +1],
    [-1, 0],
    [0, -1]
]

min_dist = [
    [35, 75, 105, 200, 475, 1365],
    [35, 75, 105, 200, 475, 1365],
    [35, 75, 105, 200, 475, 1365],
    [35, 75, 105, 200, 475, 1365],
    [35, 75, 105, 200, 475, 1365],
    [35, 75, 105, 200, 475, 1365],
    [35, 75, 105, 200, 475, 1365],
    [35, 75, 105, 200, 475, 1365]
]

max_dist = [
    [68, 95, 165, 265, 575, 1665],
    [68, 95, 165, 265, 575, 1665],
    [68, 95, 165, 265, 575, 1665],
    [68, 95, 165, 265, 575, 1665],
    [68, 95, 165, 265, 575, 1665],
    [68, 95, 165, 265, 575, 1665],
    [68, 95, 165, 265, 575, 1665],
    [68, 95, 165, 265, 575, 1665]
]


def compute_odometry(robot):
    l, r = robot.getLeftEncoder(), robot.getRightEncoder()
    data = {
        'dl': l / constants['encoder_resolution'] * constants['wheel_radius'],
        'dr': r / constants['encoder_resolution'] * constants['wheel_radius']
    }
    data['da'] = (data['dr'] - data['dl']) / constants['axle_length']
    return data


n, m, scale_factor = 0, 0, 0
obstacles = list()
particles = None
board = None


def read_input():
    global n, m, scale_factor, obstacles, board
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

    board = [[0 for j in range(m)] for i in range(n)]
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
                    # log('added point: [{0}, {1}]'.format(i, j))
                    board[i][j] = 1
    color = [[0 for j in range(m)] for i in range(n)]
    t = 2
    for i in range(n):
        for j in range(m):
            if color[i][j] == 0 and board[i][j] == 0:
                color[i][j] = 1
                found = False
                for k in range(-t * scale_factor, (t + 1) * scale_factor):
                    for l in range(-t * scale_factor, (t + 1) * scale_factor):
                        xx = i + k
                        yy = j + l
                        if 0 <= xx < n and 0 <= yy < m and board[xx][yy] == 1:
                            found = True
                            break
                    if found:
                        break
                if found:
                    color[i][j] = 2
    for i in range(n):
        for j in range(m):
            if color[i][j] == 2:
                board[i][j] = 1


def log(text):
    """	Show @text in standart output with colors """

    blue = '\033[1;34m'
    off = '\033[1;m'

    print(''.join((blue, '[Log] ', off, str(text))))


def error(text):
    red = '\033[1;31m'
    off = '\033[1;m'

    print(''.join((red, '[Error] ', off, str(text))))


def calibrate_sensors(robot):
    return
    if working_mode is WorkingMode.SIMULATION:
        return
    global min_dist, max_dist
    log('calibrating sensors...\nSTAY AWAY!')
    backup = compute_odometry(robot)
    min_values = [1e10 for i in range(8)]
    max_values = [0 for i in range(8)]
    cnt = 0
    while True:
        cnt += 1
        sensor_values = [sensors['distance_sensor'][i].getValue() for i in range(8)]
        min_values = [min(min_values[i], sensor_values[i]) for i in range(8)]
        max_values = [max(max_values[i], sensor_values[i]) for i in range(8)]

        do_action(robot, Action.EPSILON_TURN_LEFT)
        cur_value = compute_odometry(robot)
        angle = fabs(cur_value['da'] - backup['da'])
        if cmp(angle, 2 * constants['pi']) >= 0:
            log('end of calibrating step, the main function will be called in 10seconds')
            break
    k_min = [min_values[i] / min_dist[i][0] for i in range(8)]
    k_max = [max_values[i] / max_dist[i][len(max_dist[i]) - 1] for i in range(8)]
    print k_min
    print k_max
    for i in range(8):
        for j in range(len(min_dist[i])):
            min_dist[i][j] *= k_max[i]
            max_dist[i][j] *= k_max[i]

    log('10 seconds waiting to adjust position')
    ringing_led(robot, seconds=5)


def ringing_led(robot, seconds=10):
    stop_watch = StopWatch()
    while stop_watch.get_alternative_seconds() < seconds:
        for i in range(8):
            LEDs[i].set(1)
            do_action(robot, Action.STOP)
            time.sleep(0.1)
        for i in range(8):
            LEDs[i].set(0)
            time.sleep(0.1)
            do_action(robot, Action.STOP)
        log('time: {0}'.format(stop_watch.get_alternative_seconds()))
    log('Gholaam, GO!')


def setup(robot):
    global constants, min_dist, max_dist, LEDs
    if working_mode is WorkingMode.REAL_WORLD:
        constants['ratio_edge_right'] = 0.4
        constants['total_turn'] = 3
        constants['wall_centimeters_threshold7'] = 2
        constants['wall_centimeters_threshold1'] = 1.5
        constants['sensor_one_delta'] = 100
        constants['normal_speed'] = 200
        constants['epsilon_angle'] = 0.01
        constants['wall_sensor_threshold'] = 100
        constants['wall_centimeters_threshold'] = 2.5
        constants['sensor_one_wall_min'] = 1
        constants['sensor_one_wall_max'] = 4
        constants['sensor_determination_count'] = 1
        constants['justify_threshold'] = 1.5
        constants['rotate_right'] = {
            '0': 1,
            '1': 1,
            '7': 1
        }
        constants['particle_update_threshold'] = 10
        constants['turn_left_delta_x'] = 0
        constants['turn_right_delta_x'] = 12
        constants['move_particle_sigma'] = 2.2
        constants['transform_rate'] = 113.733471342
        constants['delta_correction'] = 0.98
        constants['variance_threshold'] = 4444
        min_dist = [[194.11875000000001, 221.08937499999999, 236.64750000000001, 263.20468749999998, 308.34479779411765,
                     444.17216386554622, 996.9985857142857],
                    [144.63529411764708, 172.50138562091502, 195.72569696969697, 226.03820512820513, 317.47879924953094,
                     517.75081038552321, 1369.6175243810953],
                    [122.38392857142858, 146.10322478991597, 166.86430995475112, 210.32851398601397, 309.88419999999996,
                     655.42307272727271, 2343.7649411764705],
                    [220.12857142857143, 255.45619047619047, 281.10095238095238, 321.0963492063492, 409.23269841269837,
                     661.77507936507936, 1915.2774603174605],
                    [29.259144915254232, 489.90597218321227, 736.21571428571428, 782.19841269841265, 870.40444444444438,
                     1096.0180952380952, 2017.1522222222222],
                    [315.97142857142859, 360.12968253968251, 380.92253968253971, 419.96904761904761, 496.76111111111112,
                     717.69444444444446, 1805.5907936507938],
                    [175.3857142857143, 206.44380952380953, 230.25412698412697, 273.88698412698415, 365.0669841269841,
                     614.87079365079364, 1865.6471428571426],
                    [282.18600000000004, 326.32695000000001, 352.52958333333333, 394.62361111111107, 478.94222222222226,
                     698.74184456928833, 1334.5656782680185]]
        max_dist = [[218.34812500000001, 230.10249999999999, 252.95156249999999, 290.48792279411765, 384.04002100840336,
                     746.0771285714286, 1364.5940000000001],
                    [166.51560784313727, 186.88844444444445, 215.00841491841493, 276.50869293308318, 436.29954366640442,
                     977.44474118529627, 1925.2813953488373],
                    [140.96715336134454, 158.85854072398192, 192.21519230769232, 268.2848909090909, 500.56420000000003,
                     1563.2682139037433, 3411.4364705882358],
                    [249.94063492063492, 270.80380952380949, 305.87190476190472, 372.00539682539682, 551.95507936507931,
                     1333.2622222222221, 2728.1920634920639],
                    [257.79466502421303, 725.54619047619042, 764.11904761904759, 834.54793650793647, 999.188253968254,
                     1597.4033333333334, 2667.0111111111114],
                    [355.53698412698412, 372.63301587301584, 404.2373015873016, 465.54047619047617, 620.95634920634916,
                     1301.1711111111113, 2524.6920634920639],
                    [200.57206349206348, 221.06333333333333, 256.27174603174603, 327.47269841269843, 506.62126984126985,
                     1284.2417460317461, 2672.9476190476189],
                    [319.83805000000001, 342.42874999999998, 378.23750000000001, 444.22444444444443, 604.30153870162303,
                     1058.4241819676622, 1762.8439024390245]]

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
    LEDs = [robot.getLED('led{0}'.format(i)) for i in range(8)]
    # ringing_led(robot)
    calibrate_sensors(robot)


def get_centimeters(robot, debug=False, repeat=constants['sensor_determination_count']):
    ret = [dict() for i in range(8)]
    for iteration in range(repeat):
        values = [sensors['distance_sensor'][i].getValue() for i in range(8)]
        # for i in range(8):
        #     log('sensor #{0} value: {1}'.format(i, values[i]))
        # log('sensor #{0} value: {1}'.format(7, values[7]))

        centimeters = [10 for i in range(8)]
        nn = len(min_dist[0])
        for i in range(8):
            value = values[i]
            for j in range(nn):
                if cmp(value, min_dist[i][j]) >= 0 and cmp(value, max_dist[i][j]) <= 0:
                    centimeters[i] = nn + 1 - (j + 1)
            for j in range(nn - 1):
                if cmp(value, max_dist[i][j]) >= 0 and cmp(value, min_dist[i][j + 1]) <= 0:
                    centimeters[i] = nn + 1 - (j + 1.5)
            if cmp(value, max_dist[i][nn - 1]) > 0:
                centimeters[i] = 0
        if debug:
            for i in range(8):
                log('sensor #{0}: {1} centimeters'.format(i, centimeters[i]))
        for i in range(8):
            if not centimeters[i] in ret[i]:
                ret[i][centimeters[i]] = 1
            else:
                ret[i][centimeters[i]] += 1
        if working_mode is WorkingMode.SIMULATION:
            do_action(robot, Action.STOP)
            # if working_mode is WorkingMode.REAL_WORLD and gholaam_last_action == Action.MOVE_FORWARD:
            #     do_action(robot, Action.MOVE_FORWARD, desired_speed=50)
    majority = [0 for i in range(8)]
    for i in range(8):
        maxi = -1
        voted = None
        for key, value in ret[i].items():
            if maxi < value:
                maxi = value
                voted = key
        majority[i] = voted
    if debug:
        for i in range(8):
            log('mean sensor #{0}: {1} centimeters'.format(i, majority[i]))
    # log('sensor #1: {0} cm'.format(majority[1]))
    return majority


def get_robot_state(robot, debug=False, centimeters=None):
    # TODO
    if centimeters is None:
        centimeters = get_centimeters(robot)
    if debug:
        for i in range(8):
            log('centimeter from sensor #{0} is {1}'.format(i, centimeters[i]))
    # three walls
    # TODO

    # two walls
    # RIGHT_UP
    if cmp(centimeters[0], constants['wall_centimeters_threshold']) < 0 \
            and cmp(centimeters[2], constants['wall_centimeters_threshold']) < 0 \
            and cmp(centimeters[7], constants['wall_centimeters_threshold7']) < 0:
        return RobotState.UP_RIGHT
    if cmp(centimeters[2], constants['wall_centimeters_threshold']) < 0 \
            and cmp(centimeters[3], constants['wall_centimeters_threshold']) < 0:
        return RobotState.ONE_WALL  # kesaafat kaari
    if cmp(centimeters[4], constants['wall_centimeters_threshold']) < 0 \
            and cmp(centimeters[5], constants['wall_centimeters_threshold']) < 0:
        return RobotState.ONE_WALL
    if cmp(centimeters[5], constants['wall_centimeters_threshold']) < 0 \
            and cmp(centimeters[7], constants['wall_centimeters_threshold']) < 0:
        return RobotState.ONE_WALL
    if cmp(centimeters[2], constants['wall_centimeters_threshold']) < 0 \
            and cmp(centimeters[5], constants['wall_centimeters_threshold']) < 0:
        return RobotState.RIGHT_LEFT

    # one wall
    for i in range(8):
        if i != 1 and cmp(centimeters[i], constants['wall_centimeters_threshold']) < 0:
            return RobotState.ONE_WALL
        if i == 1 and cmp(centimeters[i], constants['wall_centimeters_threshold{0}'.format(i)]) < 0:
            return RobotState.ONE_WALL  # kesaafat kaari

    return RobotState.NO_WALL


# stop watch
class StopWatch:
    def __init__(self):
        self.timer = time.clock()
        self.t_time = time.time()

    def get_time_seconds(self):
        # log('timer: {0}s'.format(time.clock() - self.timer))
        if working_mode is WorkingMode.REAL_WORLD:
            return (time.clock() - self.timer) * 100 / 4
        return time.clock() - self.timer

    def get_alternative_seconds(self):
        return time.time() - self.t_time

    def begin(self):
        self.timer = time.clock()
        self.t_time = time.time()


# all types of action that robot can do
class Action(Enum):
    MOVE_FORWARD = 0,
    TURN_RIGHT = 1,
    TURN_LEFT = 2,
    MOVE_BACKWARD = 3,
    STOP = 4,
    EPSILON_TURN_RIGHT = 5,
    EPSILON_TURN_LEFT = 6,
    ROTATE_RIGHT = 7


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


gholaam_last_action = None


# doing the desired action, with specific speed and specific angle
# the null action is included as STOP
def do_action(robot, action, desired_speed=None, angle=constants['pi'] / 2):
    global gholaam_last_action
    gholaam_last_action = action
    # log('do "{0}"'.format(action))
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
            do_action(robot, Action.STOP)
    elif action == Action.TURN_RIGHT:
        cur_angle = compute_odometry(robot)['da']
        desired_angle = cur_angle - angle
        speed = [wheel_speed, -wheel_speed]
        while compute_odometry(robot)['da'] > desired_angle:
            robot_step(robot, speed)
            do_action(robot, Action.STOP)
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
    elif action == Action.ROTATE_RIGHT:
        speed = [wheel_speed, int(0.4 * wheel_speed)]
        robot_step(robot, speed)


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
        return Action.EPSILON_TURN_jLEFT


def obstacle_avoid_forward_move(robot, properties, centimeters=None):
    global escape, best_ratio
    if centimeters is None:
        centimeters = get_centimeters(robot, debug=False)
    if properties['turn_right_count'] >= constants['total_turn']:
        do_action(robot, Action.TURN_LEFT, angle=constants['pi'] / 4)
        properties['turn_right_count'] = 0
        if escape:
            do_action(robot, Action.TURN_LEFT, angle=constants['pi'] / 6)
            best_ratio = None
            centimeters = get_centimeters(robot)
            escape = False
            while cmp(centimeters[7], constants['wall_centimeters_threshold7']) > 0 or cmp(centimeters[0], constants[
                'wall_centimeters_threshold']) > 0 or cmp(centimeters[1], constants['wall_centimeters_threshold1']) > 0:
                do_action(robot, Action.MOVE_FORWARD)
                centimeters = get_centimeters(robot)
            return
        return edge_right(robot)
    if cmp(centimeters[1], constants['sensor_one_wall_min']) <= 0:
        do_action(robot, Action.EPSILON_TURN_LEFT)
        properties['turn_right_count'] = 0
        return Status(True, 'simple moves')
    if cmp(centimeters[1], constants['sensor_one_wall_max']) > 0 \
            and cmp(centimeters[1], constants['sensor_one_wall_max'] + constants['sensor_one_delta']) <= 0 \
            and cmp(centimeters[2], constants['wall_centimeters_threshold']) <= 0:
        do_action(robot, Action.EPSILON_TURN_RIGHT)
        properties['turn_right_count'] += 1
        return Status(True, 'simple moves')
    do_action(robot, Action.MOVE_FORWARD)
    return Status(True, 'simple moves')


def right_is_the_best(robot):
    centimeters = get_centimeters(robot)
    max_value = min(centimeters)
    return centimeters[2] == max_value


# justify robot alongside the wall
# in order to keep the right side of the robot facing the wall
def justify_robot(robot, initial_direction=Action.TURN_LEFT):
    log('justify called!')
    max_value = -1234
    centimeter_min_value = -1234
    direction = initial_direction
    angle = None
    while True:
        do_action(robot, direction, angle=constants['epsilon_angle'],
                  desired_speed=constants['normal_speed'])
        do_action(robot, Action.STOP)
        cur_value = sensors['distance_sensor'][2].getValue()
        centimeters = get_centimeters(robot, repeat=1)
        # log('sensor value = {0:.02f}'.format(cur_value))
        if max_value < cur_value:
            max_value = cur_value
            centimeter_min_value = centimeters[2]
            # log('centimeter_min_value = {0}'.format(centimeter_min_value))
            # log('wall_cent_threshold = {0}'.format(constants['wall_centimeters_threshold']))
            angle = compute_odometry(robot)['da']
        if max_value - cur_value > max_value * 0.3 \
                and cmp(centimeter_min_value, constants['wall_centimeters_threshold']) <= 0:
            direction = opposite_direction(direction)
            break
    # log('JUSTIFY-- max_value: {0}'.format(max_value))
    cur_angle = compute_odometry(robot)['da']
    do_action(robot, direction, angle=math.fabs(cur_angle - angle),
              desired_speed=constants['normal_speed'])
    do_action(robot, Action.STOP)


# return the particles of each components in 2d form
def get_2d_particles():
    global particles
    ret = list()
    for i in range(len(components)):
        idx = 0
        comp = components[i]
        cur_particles = particles[i]
        cur_dist = 0
        ret_cur_component = list()
        for particle in cur_particles:
            while particle >= cur_dist + get_delta(comp, idx):
                cur_dist += get_delta(comp, idx)
                idx += 1
            else:
                delta = particle - cur_dist
                if comp[idx][0] == comp[idx + 1][0]:
                    ret_cur_component.append([comp[idx][0], comp[idx][1]
                                              + sign(comp[idx + 1][1], comp[idx][1]) * delta])
                else:
                    ret_cur_component.append([comp[idx][0] + sign(comp[idx + 1][0], comp[idx][0]) * delta,
                                              comp[idx][1]])
        ret.append(ret_cur_component)
    return ret


def update_particles(edge_detected=False):
    global particles, components
    particles_2d = get_2d_particles()
    cnt = 0
    all_upgrades = list()
    for i in range(len(particles_2d)):
        cur_particles = particles_2d[i]
        comp = components[i]
        upgrades = list()
        for j in range(len(cur_particles)):
            particle_2d = cur_particles[j]
            for point in comp:
                if edge_detected and is_edge(point[0], point[1]) and cmp(
                                math.fabs(point[0] - particle_2d[0]) + math.fabs(point[1] - particle_2d[1]),
                        constants['particle_update_threshold']) < 0:
                    upgrades.append(particles[i][j])
                    break
                if not edge_detected and is_corner(point[0], point[1]) and cmp(
                                math.fabs(point[0] - particle_2d[0]) + math.fabs(point[1] - particle_2d[1]),
                        constants['particle_update_threshold']) < 0:
                    upgrades.append(particles[i][j])
                    break
        all_upgrades.append(upgrades)
        cnt += len(upgrades)

    log('cnt = {0}'.format(cnt))
    # if edge detected, increase the probability of random part
    epsilon = constants['epsilon_of_EG']
    if edge_detected:
        epsilon *= 2

    total = 0
    lengths = []
    for i in range(len(components)):
        cur = []
        s = 0
        for j in range(len(components[i]) - 1):
            s += fabs(components[i][j + 1][0] - components[i][j][0]) + fabs(
                components[i][j + 1][1] - components[i][j][1])
            cur.append(s)
        lengths.append(cur)
        total += len(cur)

    picked_particles = list()
    should_picked = int((1 - epsilon) * constants['M'])
    # should_picked = constants['M']
    random_set = [randint(0, cnt - 1) for i in range(should_picked)]
    random_set.sort()
    idx = 0
    bef_sum = 0
    for i in range(len(all_upgrades)):
        upgrades = all_upgrades[i]
        cur_picked = list()
        while idx < len(random_set) and 0 <= random_set[idx] - bef_sum < len(upgrades):
            cur_picked.append(
                int(upgrades[random_set[idx] - bef_sum] + np.random.normal(0, constants[
                    'move_particle_sigma'] / 3) + 0.5) %
                lengths[i][len(lengths[i]) - 1])
            idx += 1
        bef_sum += len(upgrades)
        picked_particles.append(cur_picked)

    # remaining picks
    random_picked = constants['M'] - should_picked
    random_set = [randint(0, total - 1) for i in range(random_picked)]
    random_set.sort()
    idx = 0
    bef_sum = 0
    for i in range(len(components)):
        while idx < len(random_set) and 0 <= random_set[idx] - bef_sum < len(lengths[i]):
            picked_particles[i].append((int(lengths[i][random_set[idx] - bef_sum] + np.random.normal(0, constants[
                'move_particle_sigma'] / 3)) + 0.5) % lengths[i][len(lengths[i]) - 1])
            idx += 1
        bef_sum += len(lengths[i])
        picked_particles[i].sort()

    # random_set = [randint(0, constants['M'] - 1) for i in range(random_picked)]
    # random_set.sort()
    # idx = 0
    # bef_sum = 0
    # for i in range(len(particles)):
    #     while idx < len(random_set) and 0 <= random_set[idx] - bef_sum < len(particles[i]):
    #         picked_particles[i].append(particles[i][random_set[idx] - bef_sum])
    #         idx += 1
    #     bef_sum += len(particles[i])
    #     picked_particles[i].sort()

    particles = copy.deepcopy(picked_particles)
    show_particles()


def edge_right(robot):
    log('edge_right called!')
    backup = compute_odometry(robot)
    while True:
        do_action(robot, Action.ROTATE_RIGHT)
        centimeters = get_centimeters(robot)
        if cmp(centimeters[0], constants['rotate_right']['0']) < 0 \
                or cmp(centimeters[1], constants['rotate_right']['1']) < 0 \
                or cmp(centimeters[7], constants['rotate_right']['7']) < 0:
            Status(True, 'wall found!').show_verdict()
            new_observation = compute_odometry(robot)
            justify_robot(robot)
            break
        cur_observation = compute_odometry(robot)
        # if fabs(cur_observation['da'] - backup['da']) > 5 * constants['pi'] / 2:
        #     do_action(robot, Action.MOVE_FORWARD)
        #     # possibly kidnapped
        #     return Status(True, 'kidnapped')
    angle = fabs(new_observation['da'] - backup['da'])
    log('angle = {0}'.format(angle))
    if cmp(angle, constants['pi'] / 3) > 0:
        Status(True, 'edge detected').show_verdict()
        update_particles(edge_detected=True)
        # move_particles(constants['turn_right_delta_x'], turn_mode=True)
        move_particles(new_observation, backup, turn_mode=True)  # ridam toosh
        return is_localized(robot)
    else:
        move_particles(new_observation, backup)
        return Status(True, 'correction part')


def move_particles(new_values, backups=None, turn_mode=False):
    global particles, components
    if turn_mode:
        if backups is None:
            delta = new_values * scale_factor
        else:
            delta = fabs(new_values['dl'] - backups['dl'] + new_values['dr'] - backups['dr']) / 2
            delta *= constants['transform_rate'] * scale_factor
            # for bad moves!
            delta *= constants['delta_correction'] * constants['ratio_edge_right']
    else:
        delta = fabs(new_values['dl'] - backups['dl'] + new_values['dr'] - backups['dr']) / 2
        delta *= constants['transform_rate'] * scale_factor
        # for bad moves!
        delta *= constants['delta_correction']
    log('turn mode = {0}'.format(turn_mode))
    log('backup = {0}'.format(backups))
    log('DELTAAA = {0} cells'.format(delta))
    lengths = list()
    for i in range(len(components)):
        s = 0
        last = None
        for point in components[i]:
            if last is not None:
                s += math.fabs(point[0] - last[0]) + math.fabs(point[1] - last[1])
            last = point
        lengths.append(s)

    for i in range(len(particles)):
        island = list()
        for particle in particles[i]:
            island.append(int((particle + np.random.normal(delta, constants['move_particle_sigma']))
                              + 0.5) % lengths[i])
        island.sort()
        particles[i] = copy.deepcopy(island)
    show_particles()


def is_kidnapped(ratio):
    global best_ratio
    if best_ratio is not None and best_ratio > constants['kidnap_threshold']['max'] and ratio < \
            constants['kidnap_threshold']['min']:
        best_ratio = None
        return True
    return False
    # global best_variance
    # if variance is None:
    #     variance = 0
    #     for i in range(len(particles)):
    #         if len(particles[i]) == 0:
    #             continue
    #         cur = np.var(particles[i])
    #         variance += cur * (len(particles[i]) - 1)
    #     variance /= constants['M']
    # if best_variance is None:
    #     best_variance = variance
    #     return False
    # if best_variance < constants['variance_multiplier'] * variance:
    #     return True
    # if best_variance > variance:
    #     best_variance = variance
    # return False


def is_localized(robot):
    global best_ratio
    best_island = None
    for i in range(len(particles)):
        if best_island is None or len(particles[i]) > len(particles[best_island]):
            best_island = i

    mid = int((len(particles[best_island]) + 1) / 2)
    counter = 0
    for k in range(len(particles)):
        for j in range(len(particles[k])):
            if fabs(particles[k][j] - particles[best_island][mid]) < constants['radius_threshold'] * scale_factor:
                counter += 1
    pot = 1. * counter / constants['M']
    if best_ratio is None or best_ratio < pot:
        best_ratio = pot

    if best_ratio > constants['kidnap_threshold']['max']:
        LED_mask(robot, '10101010', 0.1)

    if is_kidnapped(ratio=pot):
        log('============ kidnapped ============')
        for j in range(2):
            LED_mask(robot, '11111111', 1)
            LED_mask(robot, '00000000', 0.1)
        ringing_led(robot, 10)
        for j in range(2):
            LED_mask(robot, '11111111', 1)
            LED_mask(robot, '00000000', 0.1)
        return Status(False, 'kidnapped')
    log('------------------ best = {0:.3f} vs cur ratio = {1:.3f} -----------------'.format(best_ratio, pot))
    if counter >= 0.8 * constants['M']:
        return Status(True, 'localized')
    return Status(False, '')
    # variance = 0
    # for i in range(len(particles)):
    #     if len(particles[i]) == 0:
    #         continue
    #     cur = np.var(particles[i])
    #     variance += cur * (len(particles[i]) - 1)
    # variance /= constants['M']
    # log('variance is: {0}'.format(variance))
    # if is_kidnapped(variance=variance):
    #     log('========= kidnapped =========')
    # if variance < constants['variance_threshold']:
    #     return True
    # return False


def get_pos():
    for i in range(len(particles)):
        if len(particles[i]) > 0.9 * constants['M']:
            mid = int((len(particles[i]) + 1) / 2)
            return get_2d_particles()[i][mid][0] / scale_factor, get_2d_particles()[i][mid][1] / scale_factor
    return None


# find the first corner that the robot detects
def find_corner(robot):
    # TODO
    # first = True
    state = get_robot_state(robot)
    log('first state = {0}'.format(state))
    if state == RobotState.ONE_WALL and cmp(min(get_centimeters(robot)), constants['wall_centimeters_threshold']) < 0:
        test_justify(robot)
        state = get_robot_state(robot)
    properties = {
        'turn_right_count': 0
    }
    while True:
        backups = compute_odometry(robot)
        stop_watch = StopWatch()
        init_state = state
        while init_state == state:
            centimeters = None
            state = get_robot_state(robot, centimeters=centimeters)
            status = obstacle_avoid_forward_move(robot, properties, centimeters=centimeters)
            if status.message != 'simple moves':
                status.show_verdict()
                backups = compute_odometry(robot)
                if status.message == 'localized':
                    return Status(False, 'localized at position {0}'.format(get_pos()))
                continue
            if stop_watch.get_time_seconds() > 1:
                new_values = compute_odometry(robot)
                move_particles(new_values, backups)
                backups = new_values
                stop_watch.begin()
        log('state = {0}'.format(state))
        if state == RobotState.UP_RIGHT:
            new_values = compute_odometry(robot)
            move_particles(new_values, backups)

            update_particles(edge_detected=False)
            do_action(robot, Action.TURN_LEFT)

            move_particles(constants['turn_left_delta_x'], turn_mode=True)
            if is_localized(robot).verdict:
                return Status(False, 'localized at position {0}'.format(get_pos()))
            return Status(True, 'Successfully found corner!')
        if state == RobotState.ONE_WALL:
            # first = False
            # we have right wall!
            centimeters = get_centimeters(robot)
            mini = min(centimeters)
            if cmp(mini, constants['justify_threshold']) < 0 or cmp(centimeters[7], constants[
                'wall_centimeters_threshold']) < 0 and cmp(centimeters[0], constants['wall_centimeters_threshold']) < 0:
                justify_robot(robot)
            continue
        # if first:
        #     first = False
        #     continue
        # state == NO_WALL
        if init_state == RobotState.ONE_WALL:
            new_values = compute_odometry(robot)
            move_particles(new_values, backups)
            status = edge_right(robot)
            if status.message == 'localized':
                return Status(False, 'localized at position {0}'.format(get_pos()))


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


def test_centimeters(robot):
    while True:
        get_centimeters(robot, debug=True)
        do_action(robot, Action.STOP)


def get_delta(comp, idx):
    return math.fabs(comp[idx + 1][0] - comp[idx][0]) + math.fabs(comp[idx + 1][1] - comp[idx][1])


def sign(a, b):
    return math.fabs(a - b) / (a - b)


def show_particles():
    graphics.draw_map((n, m), obstacles)
    particles_2d = get_2d_particles()
    circles = list()
    for particles_ in particles_2d:
        for particle in particles_:
            circles.append([particle[0], particle[1], 1])
    graphics.draw_circles((n, m), circles, 666)
    graphics.pygame.display.update()


def LED_mask(robot, mask, seconds):
    stop_watch = StopWatch()
    while stop_watch.get_alternative_seconds() < seconds:
        for i in range(8):
            value = 0
            if mask[i] == '1':
                value = 1
            LEDs[i].set(value)
        do_action(robot, Action.STOP)


# def SLAM(robot):
#     # go to the first corner you see
#     find_corner(robot)
#     nn, mm = 333, 333
#     map = [[2 for j in range(mm)] for i in range(nn)]
#     while True:
#         init_state = state = get_robot_state(robot)
#         stop_watch =
#         while init_state == state:


# def mother_check(robot):
#     global escape
#     escape = False
#     for i in range(len(particles)):
#         if i == 0:
#             continue
#         log('ratiooooo: {0}'.format(len(particles[i]) / constants['M']))
#         if cmp(0.8 * constants['M'], len(particles[i])) < 0:
#             escape = True
#             return


def main(robot):
    # setup(robot)

    # status = find_corner(robot)
    # status.show_verdict()
    # test_centimeters(robot)
    # return
    # test_free_rotation(robot)
    # test(robot)
    # test_robot_state(robot)
    # test_justify(robot)

    global particles
    setup(robot)
    lengths = list()
    for comp in components:
        last = None
        cur = list()
        total = 0
        for point in comp:
            if last is not None:
                delta = math.fabs(point[0] - last[0]) + math.fabs(point[1] - last[1])
                total += delta
                cur.append(delta)
            last = point
        lengths.append([cur, total])
    s = 0
    for island in lengths:
        s += island[1]
    particles = list()
    for island in lengths:
        particle_count = int(island[1] / s * constants['M'])
        cur_particles = list()
        for i in range(particle_count):
            cur_particles.append(randint(0, island[1] - 1))
        cur_particles.sort()
        particles.append(cur_particles)

    show_particles()
    while True:
        status = find_corner(robot)
        status.show_verdict()
        if not status.verdict:
            # found the correct point
            do_action(robot, Action.STOP)
            for i in range(5):
                LED_mask(robot, '01010101', 1)
                LED_mask(robot, '10101010', 1)
            break
        # mother_check(robot)


def in_range(x, y):
    global board
    return 0 <= x < n and 0 <= y < m and board[x][y] == 0


def is_corner(x, y):
    if in_range(x, y) and not in_range(x + 1, y) and not in_range(x, y + 1) and not in_range(x + 1, y + 1):
        return True
    if in_range(x, y) and not in_range(x - 1, y) and not in_range(x, y + 1) and not in_range(x - 1, y + 1):
        return True
    if in_range(x, y) and not in_range(x - 1, y) and not in_range(x, y - 1) and not in_range(x - 1, y - 1):
        return True
    if in_range(x, y) and not in_range(x + 1, y) and not in_range(x, y - 1) and not in_range(x + 1, y - 1):
        return True
    return False


def is_edge(x, y):
    if in_range(x, y) and in_range(x + 1, y) and in_range(x, y + 1) and not in_range(x + 1, y + 1):
        return True
    if in_range(x, y) and in_range(x - 1, y) and in_range(x, y + 1) and not in_range(x - 1, y + 1):
        return True
    if in_range(x, y) and in_range(x - 1, y) and in_range(x, y - 1) and not in_range(x - 1, y - 1):
        return True
    if in_range(x, y) and in_range(x + 1, y) and in_range(x, y - 1) and not in_range(x + 1, y - 1):
        return True
    return False


def is_corner_or_edge(x, y):
    return is_edge(x, y) or is_corner(x, y)


def is_wall(x, y):
    if in_range(x, y):
        if not in_range(x, y + 1) or not in_range(x + 1, y) \
                or not in_range(x, y - 1) or not in_range(x - 1, y):
            return True
    return False


def DFS(x, y, color):
    color[x][y] = 1
    for direction in dirs:
        xx = x + direction[0]
        yy = y + direction[1]
        if (is_corner_or_edge(xx, yy) or is_wall(xx, yy)) and color[xx][yy] == 0:
            cur = DFS(xx, yy, color)
            if is_corner_or_edge(xx, yy):
                point = (xx, yy)
                cur.append(point)
            return cur
    return list()


def get_corner_points():
    color = [[0 for j in range(m)] for i in range(n)]
    islands = list()
    for i in range(n):
        for j in range(m):
            if color[i][j] == 0 and is_corner_or_edge(i, j):
                island = DFS(i, j, color)
                island.append((i, j))
                islands.append(island)
    return islands


if __name__ == '__main__':
    read_input()
    components = get_corner_points()
    graphics.draw_map((n, m), obstacles)
    first_comp = True
    for component in components:
        log('here we have another component:')
        if first_comp:
            component.reverse()
            first_comp = False
        for unique_iteration in range(len(component)):
            log('corner #{0}'.format(component[unique_iteration]))
            # graphics.draw_circles((n, m), [[component[i][0], component[i][1], 1]], 666)
            graphics.add_text((n, m), '{0}'.format(unique_iteration + 1), pos=component[unique_iteration])
            graphics.pygame.display.update()
        component.append(component[0])
    main(DifferentialWheels())
