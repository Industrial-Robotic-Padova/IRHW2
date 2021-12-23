import math

import numpy as np


def detect_obstacles_2(ranges):
    data = np.array(ranges)
    gdata = np.gradient(data)
    gradient_threshold = 0.2

    nz_gdata = np.where(abs(gdata) > gradient_threshold)[0]
    f_gdata = [nz_gdata[i] for i in range(len(nz_gdata[:-1])) if nz_gdata[i + 1] - nz_gdata[i] > 1][1:]
    nz_gdata1 = nz_gdata[::-1]
    f_gdata1 = [nz_gdata1[i] for i in range(1, len(nz_gdata1) - 1) if nz_gdata1[i] - nz_gdata1[i + 1] > 1][1:]
    p_objects = gdata[f_gdata]
    f_gdata1 = f_gdata1[::-1]
    p_objects1 = gdata[f_gdata1]

    objects = [
        (f_gdata[i], f_gdata1[i + 1]) for i in range(len(p_objects[:-1]))
        if p_objects[i] < 0 < p_objects1[i + 1]
    ]

    objects1 = []
    for obj in objects:
        if np.abs(data[obj[0]] - data[obj[1]]) < 0.2:
            objects1.append(obj)
    return objects1


def detect_obstacles(ranges):
    data = np.array(ranges)
    gradient_data = np.gradient(data)
    gradient_threshold = 0.2

    obs_indexes = []
    gradient_data_indexes = np.where(abs(gradient_data) > gradient_threshold)[0]
    for gradient_index_found in gradient_data_indexes:
        if gradient_data[gradient_index_found] < 0:
            for gradient_index_next, gradient_data_next in enumerate(gradient_data[gradient_index_found+1:]):
                if gradient_data_next > 0.1:
                    obs_indexes.append((gradient_index_found+1, gradient_index_found+gradient_index_next))
                    break
                elif gradient_data_next < -0.1:
                    break
    # print()
    # print(obs_indexes)
    # for i in obs_indexes:
    #     print(data[i[0]], data[i[1]])
    # print()
    return obs_indexes


# import json
# import matplotlib.pyplot as p
# import numpy as np
#
# with open('tiago_iaslab_simulation/src/s3.json') as f:
#     data = json.loads(f.read())
#     obss = detect_obstacles(data)
#     print(obss)
#     for i in obss:
#         print(data[i[0]], data[i[1]])
#     p.plot(data)
#     # p.show()


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
