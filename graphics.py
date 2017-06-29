import pygame
import sys
import threading

screen = None


def draw_table(map_len, map_length):
    global screen
    map_length = (map_length, int(map_len[1] * map_length / map_len[0]))
    black = (0, 0, 0)
    screen_color = (60, 63, 65)
    screen = pygame.display.set_mode((map_length[1], map_length[0]))
    screen.fill(screen_color)

    # outers lines
    pygame.draw.lines(screen, black, True, [(0, 0), (map_length[1], 0)], 7)
    pygame.draw.lines(screen, black, True, [(0, 0), (0, map_length[0])], 7)
    pygame.draw.lines(screen, black, True, [(0, map_length[0]), (map_length[1], map_length[0])], 7)
    pygame.draw.lines(screen, black, True, [(map_length[1], 0), (map_length[1], map_length[0])], 7)

    map_len_y = map_len[1]
    map_len_x = map_len[0]
    i = 0
    while i <= map_length[1]:  # vertical lines
        pygame.draw.lines(screen, black, True, [(i, 0), (i, map_length[0])], 2)
        i += map_length[1] / map_len_y
    i = 0
    while i <= map_length[0]:  # horizontal lines
        pygame.draw.lines(screen, black, True, [(0, i), (map_length[1], i)], 2)
        i += map_length[0] / map_len_x  # 30
    return screen


def draw_obstacles(map_len, obstacles, map_length):
    global screen
    map_length = (map_length, int(map_len[1] * map_length / map_len[0]))
    obstacle_color = (249, 38, 114)
    for obstacle in obstacles:
        pygame.draw.rect(screen, obstacle_color,
                         (obstacle[1] * map_length[1] / map_len[1] + 1, obstacle[0] * map_length[0] / map_len[0] + 1,
                          map_length[1] / map_len[1], map_length[0] / map_len[0]))


def draw_points(map_len, food_pos, map_length, colour):
    global screen
    map_length = (map_length, int(map_len[1] * map_length / map_len[0]))
    food_color = colour
    for food in food_pos:
        pygame.draw.rect(screen, food_color,
                         (food[1] * map_length[1] / map_len[1] + 1, food[0] * map_length[0] / map_len[0] + 1,
                          map_length[1] / map_len[1], map_length[0] / map_len[0]))


def draw_circles(map_len, circles, map_length, color=None):
    global screen
    map_length = (map_length, int(map_len[1] * map_length / map_len[0]))
    if color is not None:
        circle_color = color
    else:
        circle_color = (92, 212, 239)
    for circle in circles:
        pygame.draw.circle(screen, circle_color,
                           (int(circle[1] * map_length[1] / map_len[1] + 1),
                            int(circle[0] * map_length[0] / map_len[0] + 1)),
                           int(circle[2] * map_length[1] / map_len[1]))


def draw_lines(map_len, lines, map_length, line_color, width):
    global screen
    map_length = (map_length, int(map_len[1] * map_length / map_len[0]))
    for line in lines:
        pygame.draw.line(screen, line_color,
                         [int(line[1] * map_length[1] / map_len[1] + 1), int(line[0] * map_length[0] / map_len[0] + 1)],
                         [int(line[3] * map_length[1] / map_len[1] + 1), int(line[2] * map_length[0] / map_len[0] + 1)],
                         width)


def add_text(map_len, text, pos=(0, 0), color=(255, 255, 0)):
    my_font = pygame.font.SysFont("monospace", 15)
    label = my_font.render(text, 1, color)
    map_length = 666
    map_length = (map_length, int(map_len[1] * map_length / map_len[0]))
    position = (int(pos[1] * map_length[1] / map_len[1] + 1), int(pos[0] * map_length[0] / map_len[0] + 1))
    screen.blit(label, position)


def draw_map(map_len, obstacles_pos, points_pos=None, lines=None, path=None, circles=None):
    global screen
    pygame.init()
    #  this is the code you need to customize
    map_length = 666
    # screen = draw_table(map_len, map_length)
    # draw_obstacles(map_len, screen, obstacles_pos, map_length)
    screen = draw_table(map_len, map_length)
    draw_obstacles(map_len, obstacles_pos, map_length)
    if lines is not None:
        draw_lines(map_len, lines, map_length, (253, 151, 32), 1)
    if points_pos is not None:
        draw_points(map_len, points_pos, map_length, (166, 226, 46))
    if path is not None:
        draw_lines(map_len, path, map_length, (102, 217, 239), 9)
    if circles is not None:
        draw_circles(map_len, circles, map_length)
    # draw_points(map_len, screen, goal_pos, map_length, (166, 226, 46))
    # draw_points(map_len, screen, start_pos, map_length, (92, 212, 239))
    update_display_forever()


def update_display_forever():
    pygame.display.update()
    threading.Timer(0.5, update_display_forever).start()


def wait_until_click():
    loop = True
    while loop:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

            # by clicking the pane, one turn will occur
            # uncomment these lines if you want it

            if event.type == pygame.MOUSEBUTTONDOWN:
                loop = False
                break
