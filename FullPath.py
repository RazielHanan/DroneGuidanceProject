import time
from ObstacleState import Obstacle, Axis
import numpy as np
import cv2
from Path import find_best_theta, check_angle, check_collisions, dist, get_best_angle
import numpy as np


def dist_between_pixels(p1, p2):
    return np.sqrt(np.power((p1[Axis.XAXIS] - p2[Axis.XAXIS]), 2) + np.power((p1[Axis.YAXIS] - p2[Axis.YAXIS]), 2))


def get_full_path(frame, user, obstacles, end_point, file):
    theta_step = 5
    best_theta, file = get_best_angle(user, obstacles, end_point, file)
    posx, posy = user.pos[Axis.XAXIS][0], user.pos[Axis.YAXIS][0]
    cv2.line(frame, (int(posx), int(posy)), (int(posx + user.vx), int(posy + user.vy)),
             (255, 120, 255), 3)
    v_user = user.speed
    full_path_best_theta = [best_theta]
    full_path_best_theta_len = 0
    user_cpy = Obstacle(user_mode=True, speed=user.speed)
    user_cpy = user
    user_cpy.predict_n()
    curr_dis = dist_between_pixels(
        {Axis.XAXIS: user_cpy.pos[Axis.XAXIS][0], Axis.YAXIS: user_cpy.pos[Axis.YAXIS][0]}, end_point)
    # if the user is still far from the target calculate another step
    while (curr_dis > 1.5 * user.radius and full_path_best_theta_len < 100):
        theta, file = get_best_angle(user_cpy, obstacles, end_point, file)
        full_path_best_theta.append(theta)
        full_path_best_theta_len = full_path_best_theta_len + 1
        user_cpy.add_pos({Axis.XAXIS: user_cpy.pos[Axis.XAXIS][1], Axis.YAXIS: user_cpy.pos[Axis.YAXIS][1]})
        user_cpy.predict_n()
        posx, posy = user_cpy.pos[Axis.XAXIS][0], user_cpy.pos[Axis.YAXIS][0]
        cv2.line(frame, (int(posx), int(posy)), (int(posx + user_cpy.vx), int(posy + user_cpy.vy)),
                 (255, 120, 255), 3)
        curr_dis = dist_between_pixels(
            {Axis.XAXIS: user_cpy.pos[Axis.XAXIS][0], Axis.YAXIS: user_cpy.pos[Axis.YAXIS][0]},
            end_point)
    """print(f"OUR THETA: {round(best_theta*180/np.pi,2)} deg\n")"""
    file.write('\n')
    return best_theta, file
