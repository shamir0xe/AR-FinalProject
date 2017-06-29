import os
import math
from enum import Enum
from random import random
from random import randint

import time

import copy

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
    'particle_update_threshold': 20
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
        'dr': r / constants['encoder_resolution'] * constants['wheel_radius'],
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


def get_centimeters(robot, debug=False, repeat=constants['sensor_determination_count']):
    ret = [dict() for i in range(8)]
    for iteration in range(repeat):
        values = [sensors['distance_sensor'][i].getValue() for i in range(8)]
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
        do_action(robot, Action.STOP)
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


# doing the desired action, with specific speed and specific angle
# the null action is included as STOP
def do_action(robot, action, desired_speed=None, angle=constants['pi'] / 2):
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
        centimeters = get_centimeters(robot, debug=True)
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
            angle = compute_odometry(robot)['da']
        if max_value - cur_value > max_value * 0.95 \
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
        while idx < len(random_set) and random_set[idx] - bef_sum < len(upgrades):
            cur_picked.append(upgrades[random_set[idx] - bef_sum])
            idx += 1
        bef_sum += len(upgrades)
        picked_particles.append(cur_picked)

    # # remaining picks
    # random_picked = constants['M'] - should_picked
    # random_set = [randint(0, constants['M'] - 1) for i in range(random_picked)]
    # random_set.sort()
    # idx = 0
    # bef_sum = 0
    # for i in range(len(particles)):
    #     while idx < len(random_set) and random_set[idx] - bef_sum < len(particles[i]):
    #         picked_particles[i].append(particles[i][random_set[idx] - bef_sum])
    #         idx += 1
    #     bef_sum += len(particles[i])
    #     particles[i].sort()

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


# find the first corner that the robot detects
def find_corner(robot):
    # TODO
    state = get_robot_state(robot)
    if state == RobotState.ONE_WALL:
        justify_robot(robot)
    while True:
        init_state = state
        while init_state == state:
            centimeters = None
            state = get_robot_state(robot, centimeters=centimeters)
            obstacle_avoid_forward_move(robot, centimeters=centimeters)
        log('state = {0}'.format(state))
        if state == RobotState.UP_RIGHT:
            update_particles()
            do_action(robot, Action.TURN_LEFT)
            return Status(True, 'Successfully found corner!')
        if state == RobotState.ONE_WALL:
            # we have right wall!
            centimeters = get_centimeters(robot)
            mini = min(centimeters)
            if mini < constants['justify_threshold']:
                justify_robot(robot)
            continue
        # state == NO_WALL
        if init_state == RobotState.ONE_WALL:
            edge_right(robot)

            # state = get_robot_state(robot)
            # stop_watch = StopWatch()
            # while True:
            #     if corner_found(state):
            #         return Status(True, 'Corner Found!')
            #     init_state = state
            #     while init_state == state:
            #         obstacle_avoid_forward_move(robot, properties)
            #         state = get_robot_state(robot)
            #         # if stop_watch.get_time_seconds() > constants['find_corner_timeout']:
            #         #     return Status(False, 'Timeout Reached!')
            #     Status(True, 'state changed: {0}'.format(state)).show_verdict()
            #     for i in range(100):
            #         obstacle_avoid_forward_move(robot, properties)
            #     log('state = {0}'.format(state))
            #     if state == RobotState.NO_WALL:
            #         continue
            #         # p = random()
            #         # if p < constants['chance_to_turn_right']:
            #         #     do_action(robot, Action.TURN_RIGHT)
            #         #     # we have no wall around here yet
            #         #     continue
            #     # state != NO_WALL
            #     # face the right side of the robot to the walls
            #     justify_robot(robot)
            #     state = get_robot_state(robot, debug=False)
            #     log('state after justify = {0}'.format(state))


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

    # print lengths
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
