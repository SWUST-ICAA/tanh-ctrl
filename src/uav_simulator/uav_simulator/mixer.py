from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class QuadrotorParams:
    mass_kg: float = 0.813
    gravity: float = 9.794
    max_collective_thrust_n: float = 59.2
    diagonal_wheelbase_m: float = 0.25
    moment_to_thrust_ratio_m: float = 0.3

    @property
    def max_motor_thrust_n(self):
        return self.max_collective_thrust_n / 4.0

    @property
    def arm_xy_m(self):
        return self.diagonal_wheelbase_m / (2.0 * np.sqrt(2.0))


class Px4QuadMixer:
    """PX4 motor order mixer for FRD wrench commands.

    Motor order:
    1. front_right CCW
    2. rear_left CCW
    3. front_left CW
    4. rear_right CW
    """

    def __init__(self, params):
        self.params = params
        a = params.arm_xy_m
        self.positions_frd = np.array(
            [
                [a, a, 0.0],
                [-a, -a, 0.0],
                [a, -a, 0.0],
                [-a, a, 0.0],
            ],
            dtype=float,
        )
        self.spin_sign_frd = np.array([1.0, 1.0, -1.0, -1.0], dtype=float)
        x = self.positions_frd[:, 0]
        y = self.positions_frd[:, 1]
        self.allocation = np.vstack(
            [
                np.ones(4),
                -y,
                x,
                self.spin_sign_frd * params.moment_to_thrust_ratio_m,
            ]
        )
        self._inverse_allocation = np.linalg.inv(self.allocation)

    @property
    def torque_limits(self):
        a = self.params.arm_xy_m
        f = self.params.max_motor_thrust_n
        yaw = self.params.moment_to_thrust_ratio_m * self.params.max_collective_thrust_n
        return np.array([2.0 * a * f, 2.0 * a * f, yaw], dtype=float)

    def mix(self, collective_thrust_n, torque_frd):
        wrench = np.array(
            [
                collective_thrust_n,
                torque_frd[0],
                torque_frd[1],
                torque_frd[2],
            ],
            dtype=float,
        )
        forces = self._inverse_allocation @ wrench
        return np.clip(forces, 0.0, self.params.max_motor_thrust_n)
