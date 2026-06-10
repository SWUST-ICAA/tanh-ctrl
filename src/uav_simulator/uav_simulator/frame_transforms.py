import math

import numpy as np


ENU_TO_NED = np.array(
    [
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
    ]
)
FRD_TO_FLU = np.diag([1.0, -1.0, -1.0])
FLU_TO_FRD = FRD_TO_FLU


def enu_to_ned(vector):
    return ENU_TO_NED @ np.asarray(vector, dtype=float)


def ned_to_enu(vector):
    return ENU_TO_NED @ np.asarray(vector, dtype=float)


def flu_to_frd(vector):
    return FLU_TO_FRD @ np.asarray(vector, dtype=float)


def quat_wxyz_to_matrix(quat):
    q = np.asarray(quat, dtype=float)
    norm = np.linalg.norm(q)
    if norm <= 0.0:
        return np.eye(3)
    w, x, y, z = q / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def matrix_to_quat_wxyz(matrix):
    m = np.asarray(matrix, dtype=float)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s

    quat = np.array([w, x, y, z], dtype=float)
    quat /= np.linalg.norm(quat)
    return quat


def mujoco_quat_to_px4_quat(quat_enu_flu):
    r_enu_flu = quat_wxyz_to_matrix(quat_enu_flu)
    r_ned_frd = ENU_TO_NED @ r_enu_flu @ FRD_TO_FLU
    return matrix_to_quat_wxyz(r_ned_frd)


def yaw_from_px4_quat(quat_ned_frd):
    r_ned_frd = quat_wxyz_to_matrix(quat_ned_frd)
    return math.atan2(r_ned_frd[1, 0], r_ned_frd[0, 0])
