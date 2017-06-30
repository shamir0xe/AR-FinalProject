import os
import math
from enum import Enum
from random import random
from random import randint

import time

import copy

import numpy as np

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


class WorkingMode(Enum):
    SIMULATION = 0,
    REAL_WORLD = 1


working_mode = WorkingMode.REAL_WORLD

constants = {
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
    'edge_detection_turn': 5,
    'sensor_one_wall_min': 2.5,
    'sensor_one_wall_max': 4,
    'sensor_determination_count': 3,
    'justify_threshold': 2,
    'rotate_right': {
        '0': 2,
        '1': 2
    },
    'M': 1000,
    'epsilon_of_EG': 0.1,
    'particle_update_threshold': 7,
    'turn_left_delta_x': 0,
    'turn_right_delta_x': 8.5,
    'move_particle_sigma': 2.2,
    'transform_rate': 15 / 0.1203,
    'delta_correction': 0.85,
    'variance_threshold': 0.01
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


def setup(robot):
    global constants, min_dist, max_dist
    if working_mode is WorkingMode.REAL_WORLD:
        constants['normal_speed'] = 185
        constants['epsilon_angle'] = 0.01
        constants['wall_sensor_threshold'] = 100
        constants['wall_centimeters_threshold'] = 4
        constants['sensor_one_wall_min'] = 1.5
        constants['sensor_one_wall_max'] = 3
        constants['sensor_determination_count'] = 1
        constants['justify_threshold'] = 1.5
        constants['rotate_right'] = {
            '0': 2,
            '1': 2
        }
        constants['particle_update_threshold'] = 7
        constants['turn_left_delta_x'] = 0
        constants['turn_right_delta_x'] = 10
        constants['move_particle_sigma'] = 2.2
        constants['transform_rate'] = 15 / 0.1203
        constants['delta_correction'] = 0.975
        constants['variance_threshold'] = 333
        min_dist = [[427.12857142857143, 486.4452380952381, 515.01253968253968, 550.8261904761905, 622.57174603174599,
                     822.68015873015872, 1724.5287301587302],
                    [172.05714285714285, 201.45857142857142, 224.95412698412699, 266.78730158730161, 353.19619047619045,
                     608.84571428571428, 2021.8746031746032],
                    [210.3857142857143, 248.68269841269841, 270.20809523809527, 306.57476190476189, 395.76857142857142,
                     653.33111111111111, 1921.1315873015874],
                    [220.12857142857143, 255.45619047619047, 281.10095238095238, 321.0963492063492, 409.23269841269837,
                     661.77507936507936, 1915.2774603174605],
                    [29.259144915254232, 489.90597218321227, 736.21571428571428, 782.19841269841265, 870.40444444444438,
                     1096.0180952380952, 2017.1522222222222],
                    [315.97142857142859, 360.12968253968251, 380.92253968253971, 419.96904761904761, 496.76111111111112,
                     717.69444444444446, 1805.5907936507938],
                    [175.3857142857143, 206.44380952380953, 230.25412698412697, 273.88698412698415, 365.0669841269841,
                     614.87079365079364, 1865.6471428571426],
                    [353.75714285714287, 814.39777777777772, 1043.2649206349206, 1092.0741269841269, 1184.3068253968254,
                     1418.7219047619046, 2490.9225396825395]]
        max_dist = [[480.42777777777775, 503.47952380952381, 538.33253968253962, 592.31714285714293, 736.03412698412706,
                     1309.550634920635, 2340.0317460317465],
                    [196.23984126984126, 215.60142857142856, 250.16507936507938, 317.53396825396828, 496.67809523809524,
                     1360.0619047619048, 2930.6444444444446],
                    [241.11095238095237, 263.01412698412696, 291.66333333333336, 357.85047619047617, 541.30380952380949,
                     1332.9477777777779, 2741.2174603174608],
                    [249.94063492063492, 270.80380952380949, 305.87190476190472, 372.00539682539682, 551.95507936507931,
                     1333.2622222222221, 2728.1920634920639],
                    [257.79466502421303, 725.54619047619042, 764.11904761904759, 834.54793650793647, 999.188253968254,
                     1597.4033333333334, 2667.0111111111114],
                    [355.53698412698412, 372.63301587301584, 404.2373015873016, 465.54047619047617, 620.95634920634916,
                     1301.1711111111113, 2524.6920634920639],
                    [200.57206349206348, 221.06333333333333, 256.27174603174603, 327.47269841269843, 506.62126984126985,
                     1284.2417460317461, 2672.9476190476189],
                    [600.58634920634927, 1032.4334920634922, 1072.6401587301589, 1147.0741269841269, 1318.103492063492,
                     1996.3790476190475, 3268.0126984126982]]

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


def get_centimeters(robot, debug=False, repeat=constants['sensor_determination_count']):
    ret = [dict() for i in range(8)]
    for iteration in range(repeat):
        values = [sensors['distance_sensor'][i].getValue() for i in range(8)]
        # log('sensor #2 value: {0}'.format(values[2]))
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
            and cmp(centimeters[2], constants['wall_centimeters_threshold']) < 0:
        return RobotState.UP_RIGHT
    if cmp(centimeters[2], constants['wall_centimeters_threshold']) < 0 \
            and cmp(centimeters[3], constants['wall_centimeters_threshold']) < 0:
        return RobotState.RIGHT_DOWN
    if cmp(centimeters[4], constants['wall_centimeters_threshold']) < 0 \
            and cmp(centimeters[5], constants['wall_centimeters_threshold']) < 0:
        return RobotState.DOWN_LEFT
    if cmp(centimeters[5], constants['wall_centimeters_threshold']) < 0 \
            and cmp(centimeters[7], constants['wall_centimeters_threshold']) < 0:
        return RobotState.LEFT_UP
    if cmp(centimeters[2], constants['wall_centimeters_threshold']) < 0 \
            and cmp(centimeters[5], constants['wall_centimeters_threshold']) < 0:
        return RobotState.RIGHT_LEFT

    # one wall
    for value in centimeters:
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
        if working_mode is WorkingMode.REAL_WORLD:
            return (time.clock() - self.timer) * 100 / 4
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


def obstacle_avoid_forward_move(robot, centimeters=None):
    if centimeters is None:
        centimeters = get_centimeters(robot, debug=False)
    if cmp(centimeters[1], constants['sensor_one_wall_min']) <= 0:
        do_action(robot, Action.EPSILON_TURN_LEFT)
        return
    if cmp(centimeters[1], constants['sensor_one_wall_max']) > 0 \
            and cmp(centimeters[1], constants['sensor_one_wall_max'] + 1) <= 0 \
            and cmp(centimeters[2], constants['wall_centimeters_threshold']) <= 0:
        do_action(robot, Action.EPSILON_TURN_RIGHT)
        return
    do_action(robot, Action.MOVE_FORWARD)


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
        if max_value - cur_value > max_value * 0.6 \
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


def update_particles():
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
                if cmp(math.fabs(point[0] - particle_2d[0]) + math.fabs(point[1] - particle_2d[1]),
                       constants['particle_update_threshold']) < 0:
                    upgrades.append(particles[i][j])
                    break
        all_upgrades.append(upgrades)
        cnt += len(upgrades)

    picked_particles = list()
    should_picked = int((1 - constants['epsilon_of_EG']) * constants['M'])
    # should_picked = constants['M']
    random_set = [randint(0, cnt - 1) for i in range(should_picked)]
    random_set.sort()
    idx = 0
    bef_sum = 0
    for i in range(len(all_upgrades)):
        upgrades = all_upgrades[i]
        cur_picked = list()
        while idx < len(random_set) and 0 <= random_set[idx] - bef_sum < len(upgrades):
            cur_picked.append(upgrades[random_set[idx] - bef_sum])
            idx += 1
        bef_sum += len(upgrades)
        picked_particles.append(cur_picked)

    # remaining picks
    random_picked = constants['M'] - should_picked
    random_set = [randint(0, constants['M'] - 1) for i in range(random_picked)]
    random_set.sort()
    idx = 0
    bef_sum = 0
    for i in range(len(particles)):
        while idx < len(random_set) and 0 <= random_set[idx] - bef_sum < len(particles[i]):
            picked_particles[i].append(particles[i][random_set[idx] - bef_sum])
            idx += 1
        bef_sum += len(particles[i])
        picked_particles[i].sort()

    particles = copy.deepcopy(picked_particles)
    show_particles()


def edge_right(robot):
    update_particles()
    while True:
        do_action(robot, Action.ROTATE_RIGHT)
        centimeters = get_centimeters(robot)
        if cmp(centimeters[0], constants['rotate_right']['0']) < 0 \
                or cmp(centimeters[1], constants['rotate_right']['1']) < 0:
            Status(True, 'wall found!').show_verdict()
            justify_robot(robot)
            return


def move_particles(new_values, backups=None, turn_mode=False):
    global particles, components
    if turn_mode:
        delta = new_values * scale_factor
    else:
        delta = (new_values['dl'] - backups['dl'] + new_values['dr'] - backups['dr']) / 2
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


def is_localized():
    variance = 0
    for i in range(len(particles)):
        cur = np.var(particles[i])
        variance += cur * (len(particles[i]) - 1)
    variance /= constants['M']
    log('variance is: {0}'.format(variance))
    if variance < constants['variance_threshold']:
        return True
    return False


def get_pos():
    for i in range(len(particles)):
        if len(particles[i]) > 0.9 * constants['M']:
            mid = int((len(particles[i]) + 1) / 2)
            return get_2d_particles()[i][mid]
    return None


# find the first corner that the robot detects
def find_corner(robot):
    # TODO
    first = True
    state = get_robot_state(robot)
    while True:
        backups = compute_odometry(robot)
        stop_watch = StopWatch()
        init_state = state
        while init_state == state:
            centimeters = None
            state = get_robot_state(robot, centimeters=centimeters)
            obstacle_avoid_forward_move(robot, centimeters=centimeters)
            if stop_watch.get_time_seconds() > 1:
                new_values = compute_odometry(robot)
                move_particles(new_values, backups)
                backups = new_values
                stop_watch.begin()
        log('state = {0}'.format(state))
        if state == RobotState.UP_RIGHT:
            new_values = compute_odometry(robot)
            move_particles(new_values, backups)

            update_particles()
            do_action(robot, Action.TURN_LEFT)

            move_particles(constants['turn_left_delta_x'], turn_mode=True)
            if is_localized():
                Status(True, 'localized at position {0}'.format(get_pos())).show_verdict()
            return Status(True, 'Successfully found corner!')
        if state == RobotState.ONE_WALL:
            first = False
            # we have right wall!
            centimeters = get_centimeters(robot)
            mini = min(centimeters)
            if cmp(mini, constants['justify_threshold']) < 0:
                justify_robot(robot)
            continue
        if first:
            first = False
            continue
        # state == NO_WALL
        if init_state == RobotState.ONE_WALL:
            new_values = compute_odometry(robot)
            move_particles(new_values, backups)
            backups = new_values

            edge_right(robot)

            move_particles(constants['turn_right_delta_x'], turn_mode=True)
            backups = compute_odometry(robot)


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

    if in_range(x, y) and in_range(x + 1, y) and in_range(x, y + 1) and not in_range(x + 1, y + 1):
        return True
    if in_range(x, y) and in_range(x - 1, y) and in_range(x, y + 1) and not in_range(x - 1, y + 1):
        return True
    if in_range(x, y) and in_range(x - 1, y) and in_range(x, y - 1) and not in_range(x - 1, y - 1):
        return True
    if in_range(x, y) and in_range(x + 1, y) and in_range(x, y - 1) and not in_range(x + 1, y - 1):
        return True
    return False


def is_edge(x, y):
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
        if (is_corner(xx, yy) or is_edge(xx, yy)) and color[xx][yy] == 0:
            cur = DFS(xx, yy, color)
            if is_corner(xx, yy):
                point = (xx, yy)
                cur.append(point)
            return cur
    return list()


def get_corner_points():
    color = [[0 for j in range(m)] for i in range(n)]
    islands = list()
    for i in range(n):
        for j in range(m):
            if color[i][j] == 0 and is_corner(i, j):
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
