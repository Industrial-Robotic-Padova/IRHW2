#! /usr/bin/env python
# -*- coding: utf-8 -*-

# The coordinates of objects in map
coords = {
    'base': (-6.580047, 1.369940),
    'table': (1.245143, -1.613171),
    '3': (4.007396, 1.015966),
    '2': (5.007404, 1.015966),
    '1': (6.007146, 1.015966),
}
table_size = 0.913
cylinder_size = 0.21
robot_size = 0.7


def pose_calc(obj_str):
    return coords.get(obj_str)[0] - coords.get('base')[0], coords.get(obj_str)[1] - coords.get('base')[1]


def pose_calc_cyl(obj_str):
    pose_table = list(pose_calc(obj_str))
    pose_table[1] += (cylinder_size + robot_size/2)
    pose_table.append(-1)
    return tuple(pose_table)


def pose_calc_table(angle):
    pose_table = list(pose_calc('table'))
    if angle == 1:
        pose_table[0] += table_size
        pose_table.append(3)  # pi
    elif angle == 2:
        pose_table[0] -= table_size
        pose_table.append(0)
    elif angle == 3:
        pose_table[1] += table_size
        pose_table.append(-1)
    elif angle == 4:
        pose_table[1] -= table_size
        pose_table.append(1)

    return tuple(pose_table)
