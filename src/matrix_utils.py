import sympy as sp

def ee_transformation(angles, dh_table, frame=sp.eye(4)):
    for alpha, length, offset, theta, angle in zip(dh_table['alpha'], dh_table['length'],
                                                   dh_table['offset'], dh_table['theta'],
                                                   angles):
        a = transformation_matrix(sp.rad(alpha), length, sp.rad(theta) + angle, offset)
        frame = frame * a
    return sp.Matrix(frame)


def transformation_matrix(alpha, length, theta, offset):
    sin_t, sin_a = sp.sin(theta), sp.sin(alpha)
    cos_t, cos_a = sp.cos(theta), sp.cos(alpha)
    return sp.Matrix([
        [cos_t, -cos_a * sin_t, sin_t * sin_a, length * cos_t],
        [sin_t, cos_t * cos_a, cos_t * -sin_a, length * sin_t],
        [0, sin_a, cos_a, offset],
        [0, 0, 0, 1]
    ])


def get_position(transformation):
    return transformation[:3, 3]


def get_orientation(transformation):
    rot_matrix = transformation[:3, :3]
    return sp.Matrix([
        sp.atan2(rot_matrix[2, 1], rot_matrix[2, 0]),
        sp.acos(rot_matrix[2, 2]),
        sp.atan2(rot_matrix[1, 2], -rot_matrix[0, 2])
    ])
