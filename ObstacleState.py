from enum import Enum
import numpy as np
from velocity_estimation import estimate_velocity


class Axis(Enum):
    XAXIS = 0
    YAXIS = 1


class Obstacle:
    def __init__(self, radius=15, dt=0.33, speed=60, user_mode=False):
        self.speed = speed
        self.user_mode = user_mode
        self.steps = 4
        self.Na = 4
        self.dt = dt
        self.t = [i * self.dt for i in range(self.Na)]
        self.pos = {a: [0] * self.steps for a in Axis}
        self.vx = 0
        self.vy = 0
        self.default_v = 100
        self.radius = radius
        self.last_n_pos = {a: [0] * self.Na for a in Axis}
        self.num_poses = 0

    def predict_n(self):
        if self.user_mode:
            self.pos[Axis.XAXIS] = [self.pos[Axis.XAXIS][0] + self.vx * i for i in range(self.steps)]
            self.pos[Axis.YAXIS] = [self.pos[Axis.YAXIS][0] + self.vy * i for i in range(self.steps)]
        else:
            self.pos[Axis.XAXIS] = [self.pos[Axis.XAXIS][0] + self.vx * self.dt * i for i in range(self.steps)]
            self.pos[Axis.YAXIS] = [self.pos[Axis.YAXIS][0] + self.vy * self.dt * i for i in range(self.steps)]

    def estimate_velocity(self):
        self.vx = estimate_velocity(self.t, self.last_n_pos[Axis.XAXIS][::-1])
        self.vy = estimate_velocity(self.t, self.last_n_pos[Axis.YAXIS][::-1])

    def add_pos(self, new_pos):
        for n in range(self.Na - 2, -1, -1):
            self.last_n_pos[Axis.XAXIS][n + 1] = self.last_n_pos[Axis.XAXIS][n]
            self.last_n_pos[Axis.YAXIS][n + 1] = self.last_n_pos[Axis.YAXIS][n]
        for a in Axis:
            self.last_n_pos[a][0] = new_pos[a]
        for a in Axis:
            self.pos[a][0] = new_pos[a]
        # print(self.last_n_pos[Axis.XAXIS])
        if self.num_poses > 10 and self.user_mode == False:
            self.estimate_velocity()
        self.predict_n()
        self.num_poses += 1
