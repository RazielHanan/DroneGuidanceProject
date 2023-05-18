import time

from ObstacleState import Obstacle, Axis
import numpy as np
import pygame
import cv2


def write_loop_timings_file(f, loop_time, filename='navigation_timings.csv'):
    if f == None:
        f = open(filename, 'wt')
        f.write('loops timing\n')
    f.write('%f,' % loop_time)
    return f


# assuming we already used predict_n on both obstacles
def dist(obs1, obs2):
    min_dist = 10000
    # print(obs2.pos[Axis.XAXIS])
    for i in range(1,obs1.steps):
        curr_dis = np.sqrt(np.power((obs1.pos[Axis.XAXIS][i] - obs2.pos[Axis.XAXIS][i]), 2) + np.power(
            (obs1.pos[Axis.YAXIS][i] - obs2.pos[Axis.YAXIS][i]), 2))
        if (curr_dis < min_dist):
            min_dist = curr_dis
    return min_dist


def check_collisions(user, obstacles):
    min = 100000
    for obs in obstacles:
        dis = dist(user, obs)
        dis = dis - obs.radius - 5 #- user.radius
        is_collision = dis < (15)
        if (is_collision):
            return 0
        if (dis < min):
            min = dis
    return min


def sign(x):
    if x > 0:
        return 1
    elif x == 0:
        return 0
    else:
        return -1


def draw_direction(window, new_user, collision, dir_x, dir_y):
    posx, posy = new_user.pos[Axis.XAXIS][0], new_user.pos[Axis.YAXIS][0]
    if (collision != 0):
        pygame.draw.line(window, (255, 120, 255), [posx, posy], [posx + dir_x, posy + dir_y], 1)
    else:
        pygame.draw.line(window, (0, 0, 0), [posx, posy], [posx + dir_x, posy + dir_y], 1)
    pygame.display.update()


def draw_chosen_dir(window, new_user, theta_rad, v_user):
    vx = np.cos(theta_rad) * v_user
    vy = np.sin(theta_rad) * v_user
    posx, posy = new_user.pos[Axis.XAXIS][0], new_user.pos[Axis.YAXIS][0]
    pygame.draw.line(window, (255, 255, 0), [posx, posy], [posx + vx, posy + vy], 1)
    pygame.display.update()


def check_angle(angle, v_user, new_user, obstacles):
    angle_rad = angle * np.pi / 180
    if new_user.user_mode:
        vx = np.cos(angle_rad) * new_user.radius
        vy = np.sin(angle_rad) * new_user.radius
        new_user.vx, new_user.vy = vx, vy
    else:
        vx = np.cos(angle_rad) * v_user
        vy = np.sin(angle_rad) * v_user
        new_user.vx, new_user.vy = vx, vy
    new_user.predict_n()
    min_dist = check_collisions(new_user, obstacles)
    # print(f"{theta}:,{round(min_dist,2)}")
    """draw_direction(window, new_user,min_dist,np.cos(theta_rad) * 40, np.sin(theta_rad)*40)"""
    return min_dist


def find_best_theta(user, obstacles, end_point, theta_step):
    v_user = user.speed
    safe_dist = 1
    pos_user = {a: user.pos[a][0] for a in Axis}
    new_user = Obstacle(speed=user.speed, user_mode=True)
    new_user.add_pos(pos_user)
    theta_base = np.arctan(
        (np.abs(end_point[Axis.YAXIS] - pos_user[Axis.YAXIS])) / (0.001 + np.abs(
            end_point[Axis.XAXIS] - pos_user[Axis.XAXIS])))  # +0.001 to avoid division by zero.
    # print(round(theta_base*180/np.pi,2))
    if (end_point[Axis.XAXIS] < pos_user[Axis.XAXIS]):
        theta_base = (np.pi - theta_base)
    if (end_point[Axis.YAXIS] < pos_user[Axis.YAXIS]):
        theta_base = -theta_base
    while (theta_base < 0):
        theta_base = theta_base + 2 * np.pi
    while (theta_base > 2 * np.pi):
        theta_base = theta_base - 2 * np.pi
    if (theta_base > 355 * np.pi / 180):
        theta_base = 0
    theta_base_deg = theta_base * 180 / np.pi
    loop_num = 0
    while (loop_num < 180):
        angle = theta_base_deg + loop_num
        while (angle >= 360 or angle < 0):
            if angle >= 360:
                angle = angle - 360
            else:
                angle = angle + 360
        # time.sleep(0.0001)
        min_dist = check_angle(angle, v_user, new_user, obstacles)
        """draw_direction(, new_user, min_dist, np.cos(angle * np.pi / 180) * 40, np.sin(angle * np.pi / 180) * 40)"""
        if min_dist > safe_dist:
            return angle * np.pi / 180
        angle = theta_base_deg - loop_num
        while (angle >= 360 or angle < 0):
            if angle >= 360:
                angle = angle - 360
            else:
                angle = angle + 360
        # time.sleep(0.0001)
        min_dist = check_angle(angle, v_user, new_user, obstacles)
        """draw_direction(window, new_user, min_dist, np.cos(angle * np.pi / 180) * 40, np.sin(angle * np.pi / 180) * 40)"""
        if min_dist > safe_dist:
            return angle * np.pi / 180
        loop_num = loop_num + theta_step
    angle = theta_base_deg + 180
    # time.sleep(0.00)
    min_dist = check_angle(angle, v_user, new_user, obstacles)
    """draw_direction(window, new_user, min_dist, np.cos(angle * np.pi / 180) * 40, np.sin(angle * np.pi / 180) * 40)"""
    if min_dist > safe_dist:
        return angle * np.pi / 180
    return 1000


def find_best_theta_2(user, obstacles, end_point, theta_step, window):
    v_user = np.sqrt(np.power(user.vx, 2) + np.power(user.vy, 2))
    v_user = user.speed
    pos_user = {a: user.pos[a][0] for a in Axis}
    new_user = Obstacle()
    new_user.add_pos(pos_user)
    theta_vs_dist = {}

    for theta in range(0, 360, theta_step):
        theta_rad = theta * np.pi / 180
        vx = np.cos(theta_rad) * v_user
        vy = np.sin(theta_rad) * v_user
        new_user.vx, new_user.vy = vx, vy
        new_user.predict_n()
        min_dist = check_collisions(new_user, obstacles, window)
        # print(f"{theta}:,{round(min_dist,2)}")
        """draw_direction(window, new_user,min_dist,np.cos(theta_rad) * 40, np.sin(theta_rad)*40)"""  #############################3
        if (min_dist > 0):
            theta_vs_dist[theta] = min_dist
    # print(f"dic: {theta_vs_dist}")
    theta_base = np.arctan(
        (np.abs(end_point[Axis.YAXIS] - pos_user[Axis.YAXIS])) / (0.001 + np.abs(
            end_point[Axis.XAXIS] - pos_user[Axis.XAXIS])))  # +0.001 to avoid division by zero.
    # print(round(theta_base*180/np.pi,2))
    if (end_point[Axis.XAXIS] < pos_user[Axis.XAXIS]):
        theta_base = (np.pi - theta_base)
    if (end_point[Axis.YAXIS] < pos_user[Axis.YAXIS]):
        theta_base = -theta_base
    while (theta_base < 0):
        theta_base = theta_base + 2 * np.pi
    while (theta_base > 2 * np.pi):
        theta_base = theta_base - 2 * np.pi
    if (theta_base > 355 * np.pi / 180):
        theta_base = 0

    """print(f"WANTED THETA: {round(theta_base*180/np.pi,2)} deg")"""
    min = 1000
    theta_min = 1000
    for theta in theta_vs_dist:
        theta_rad = theta * np.pi / 180
        if (theta_base < 15 * np.pi / 180 or theta_base > 345 * np.pi / 180):
            if (theta_rad < np.pi):
                theta_rad = theta_rad + 2 * np.pi
            if (theta_base < 15 * np.pi / 180):
                theta_base += 2 * np.pi
        # print(theta_rad)
        theta_eff = np.abs(theta_rad - theta_base)
        # print(theta_eff,theta,theta_base)
        if (theta_eff < min):
            min = theta_eff
            theta_min = theta_rad
    # print(theta_min)
    # draw_chosen_dir(window,user,theta_min,v_user)

    return theta_min


def get_best_angle(user, obstacles, end_point, file):
    t_start = time.perf_counter()
    if end_point[Axis.XAXIS] == 0 and end_point[Axis.YAXIS] == 0:
        best_theta = 1000
        v_user = user.speed
    else:
        num_obstacles = len(obstacles)
        theta_step = 5
        best_theta = find_best_theta(user, obstacles, end_point, theta_step)
        """print(f"OUR THETA: {round(best_theta*180/np.pi,2)} deg\n")"""
        v_user = user.speed
    if best_theta == 1000:
        v_user = 0
    if user.user_mode:
        user.vx = np.cos(best_theta) * user.radius
        user.vy = np.sin(best_theta) * user.radius
    else:
        user.vx = np.cos(best_theta) * v_user
        user.vy = np.sin(best_theta) * v_user
    t_end = time.perf_counter()
    loop_time = round(t_end - t_start, 4)
    file = write_loop_timings_file(file, loop_time)
    return best_theta, file
