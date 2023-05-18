from enum import Enum
import time

import numpy as np
from velocity_estimation import estimate_velocity


class Axis(Enum):
    XAXIS = 0
    YAXIS = 1

    @property
    def mid(self):
        return (320, 160)[self.value] #(320,240) instdead of 320.


class Averaging(Enum):
    HIGH = 0
    LOW = 1


class Target():
    """Target estimator"""

    def __init__(self, dt=0.04):

        self._dt = dt
        self._num_frames = 0
        self._N = int(10000 / dt)
        self._timer_buf = [0] * self._N
        self._target_pos_buf = {x: [0] * self._N for x in Axis}
        self.reset_state()
        self._ff_dt = 0
        self._hi_avg = int(30)  # can change these
        self._lo_avg = int(2)  # can change these
        self.set_averaging(Averaging.HIGH)
        self.Vx, self.Vy = [0] * self._N, [0] * self._N
        self.Vx_est, self.Vy_est = [0] * self._N, [0] * self._N
        self.Vex_est, self.Vey_est = [0] * self._N, [0] * self._N
        self.vel_command_delay = 80  # units of frames

    def set_averaging(self, avg: Averaging):
        self._Na = self._hi_avg if avg == Averaging.HIGH else self._lo_avg

    def reset_state(self):
        self._target_pos_buf = {x: [0] * self._N for x in Axis}
        self._timer_buf = [0] * self._N
        self._num_frames = 0

    def add_position_by_dic(self, pos):
        """position is assumed to be sent as a dictionary
        dict([Axis.XAXIS] : xvalue, [Axis.YAXIS]: yvalue)"""
        i = int(self._num_frames % self._N)
        for a in Axis:
            self._target_pos_buf[a][i] = pos[a] - a.mid
        if i == 0:
            self._timer_buf[i] = 0
        elif (i > 0):
            self._timer_buf[i] = self._timer_buf[i - 1] + self._dt
        self._num_frames += 1

    def get_position(self):
        """get position extrapolate to the current time"""
        if self._num_frames < 1:
            return None
        v = self.get_velocity()
        if v is None:
            return None
        i = self._num_frames % self._N
        ret = {a: self._target_pos_buf[a][i - 1] + self._ff_dt * v[a] for a in Axis}
        return ret

    def set_dt(self, dt):
        self._dt = dt

    def get_velocity(self):
        """Low pass filter; return the derivative of the average of N last values"""
        Na = self._Na
        dt = self._dt
        if self._num_frames < Na + 3:
            return None
        i = self._num_frames % self._N

        """polyfit method"""
        vel_x = estimate_velocity(self._timer_buf[i - 1:i - self._Na - 1:-1],
                                  self._target_pos_buf[Axis.XAXIS][i - 1:i - self._Na - 1:-1])
        vel_y = estimate_velocity(self._timer_buf[i - 1:i - self._Na - 1:-1],
                                  self._target_pos_buf[Axis.YAXIS][i - 1:i - self._Na - 1:-1])
        ret = {Axis.XAXIS: vel_x, Axis.YAXIS: vel_y}
        return ret

    def write_to_file(self, filename='pos.csv'):
        f = open(filename, 'wt')
        f.write('time[epoch seconds], X, Y, Vx, Vy, Vvx, Vvy\n')
        for i in range(self._num_frames):
            f.write('%f, %f, %f, %f, %f, %f, %f\n' % (
                self._timer_buf[i], self._target_pos_buf[Axis.XAXIS][i], self._target_pos_buf[Axis.YAXIS][i],
                self.Vx[i], self.Vy[i], self.Vx_est[i], self.Vy_est[i]))
        f.close()
        print('wrote target estimator to file ', filename)

    # in your tracker code you should do this:
    def add_pos(self, new_pos):
        # the function self_get_target_loc() returns the location in a dictionary based
        # on the kcf/csrt tracker. E.g. {Axis.XAxis: 100, Axis.YAxis : 200}
        # we then send the measurement to the target estimator
        """edit: cancel self.get_target_loc() function, just get the pos from user"""
        pos_x, pos_y = new_pos[0], new_pos[1]
        pos = {Axis.XAXIS: pos_x, Axis.YAXIS: pos_y}
        self.add_position_by_dic(pos)

    def velocity_cmd(self, Kv=0.05, Ke=0.15, h=0):  # kv=0.09 Ke=0.352 for pd controller
        """26.3.23 was Ke=0.125 and Kv=0.0"""
        """L- cm we see on camera in y axis, h-drone height
           L=0.7397*h+6.1285
           see samples and calculations in 'find_drone_angle' module """
        """for height of 800cm we get 598cm in camera. for y axis of 480 pixels we get 1.2456 cm/pixel"""
        """velocity control command"""
        tan_fov = 0.39
        L = 2 * h * tan_fov
        pix2cm_x = L / 640
        pix2cm_y = L / 480
        i = self._num_frames % self._N
        # we assumed you created a target object named self._target
        tgt_vel = self.get_velocity()
        self.Vx_est[i] = int(Kv * tgt_vel[Axis.XAXIS])  # for pd controller
        self.Vy_est[i] = int(Kv * tgt_vel[Axis.YAXIS])  # for pd controller
        # self.Vx_est[i] = round(pix2cm_x * tgt_vel[Axis.XAXIS])
        # self.Vy_est[i] = round(pix2cm_y * tgt_vel[Axis.YAXIS])
        tgt_pos = self.get_position()
        i = int(self._num_frames % self._N)
        ex, ey = self._target_pos_buf[Axis.XAXIS][i - 1], self._target_pos_buf[Axis.YAXIS][i - 1]
        ex_cm, ey_cm = ex * pix2cm_x, ey * pix2cm_y
        # get the locations from the tracker
        # velocity command based on position error
        self.Vex_est[i] = round(Ke * ex)  # 0.3 for outside
        self.Vey_est[i] = round(Ke * ey)  # 0.2 for outside 6 times, 0.11 for 740 cm , 0.5 for home
        rates = [round(self.Vx_est[i] + self.Vex_est[i]), round(self.Vy_est[i] + self.Vey_est[i])]
        """print(f"err[x]: {err[Axis.XAXIS]}   , err[y]:{err[Axis.YAXIS]}")"""
        """print(f"vel[x]:{self.Vx_est}, vel[1]:{self.Vy_est}\n")"""
        self.Vx[i] = rates[0]
        self.Vy[i] = rates[1]
        # send the command to the drone
        """drone.rate_command(rates)"""
        V_vx, V_vy, V_ex, V_ey = self.Vx_est[i], self.Vy_est[i], self.Vex_est[i], self.Vey_est[i]
        return ex_cm, ey_cm

##################################
# target = Target()
# t2 = time.perf_counter()
# t1 = time.perf_counter()
# positions = [(i, 480 - i) for i in range(320)]
# n_positions = [(320,160) for i in range(320)]
# positions+=n_positions
# Vx,Vy=0,0
# for i, pos in enumerate(positions):
#     t1 = time.perf_counter()
#     dt = 0.0001
#     if (dt > 0):
#         target.set_dt(dt)
#     target.track(pos)
#     if (i > 33):
#         Vxc,Vcy = target.velocity_cmd()
#         target.write_to_file()
#     # time.sleep(0.001)
#     t2 = time.perf_counter()
