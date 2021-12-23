import math

import numpy as np


def get_obstacle_position(d, index_mid, x, y):
    len_ranges = 666
    range_min_rad = -1.91
    range_max_rad = 1.91
    range_rad = range_max_rad - range_min_rad
    alpha_rad = (float((index_mid - 333)) / len_ranges) * range_rad  # rate
    x += d * math.cos(alpha_rad)
    y += d * math.sin(alpha_rad)
    return x, y


def get_obstacle_distance(index_start, index_end, a, x):
    range_min_rad = -1.91
    range_max_rad = 1.91
    range_rad = range_max_rad - range_min_rad
    len_ranges = 666.0
    len_indexes = float(index_end - index_start)
    alpha_rad = (len_indexes / len_ranges) * range_rad
    alpha_2_rad_tg = math.tan(alpha_rad / 2)
    radius = alpha_2_rad_tg * a
    return radius + x, radius


def same_object(c1, c2, r):
    return math.sqrt((c1[0] - c2[0]) ** 2 + (c1[1] - c2[1]) ** 2) < r


def is_obstacle_exists(c, obstacles, radius=1.2):
    is_same = False
    for i in obstacles:
        if same_object(i, (c[0], c[1]), radius):
            is_same = True
    return is_same


def get_trans_matrix_robot(rz, x, y, z):
    return np.array([
        [math.cos(rz), -math.sin(rz), 0, x],
        [math.sin(rz), math.cos(rz), 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1],
    ])
