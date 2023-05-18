import time
import cv2


def past_n_sec(t_last_command, n=0.01):
    t_curr = time.perf_counter()
    past = 0
    if (t_curr - t_last_command > n):
        past = 1
        t_last_command = time.perf_counter()
    return t_last_command, past


def sign(x):
    if x > 0:
        return 1
    elif x == 0:
        return 0
    else:
        return -1


def limit_velocity(total_Vx, total_Vy):
    if (total_Vx > 16):
        total_Vx = 16
    elif total_Vx < -16:
        total_Vx = -16
    if (total_Vy > 16):
        total_Vy = 16
    elif total_Vy < -16:
        total_Vy = -16
    if (abs(total_Vy) < 10):
        total_Vy = 0
    if (abs(total_Vx) < 10):
        total_Vx = 0
    return total_Vx, total_Vy


def limit_velocity_2(v_x, v_y):
    if v_x > 10:
        v_x = 15
    if v_x < -10:
        v_x = -15
    if v_y > 10:
        v_y = 15
    if v_y < -10:
        v_y = -15
    return v_x, v_y


def fix_height(me, speed, HOME_MODE, drone_height):
    z_speed = 0
    high = 800 - (drone_height)
    sign_high = sign(high)
    if abs(high) > 100: z_speed = sign_high * speed
    if HOME_MODE == 1: z_speed = 0
    return z_speed


def put_text_on_frame(frame, data, drone_height):
    font = cv2.FONT_HERSHEY_SIMPLEX
    # org
    org = (30, 30)
    # fontScale
    fontScale = 1
    # Blue color in BGR
    color = (255, 0, 0)
    # Line thickness of 2 px
    thickness = 2
    # Using cv2.putText() method
    image = cv2.putText(frame,
                        f'Vx:{round(data[0], 3)}, Vy:{round(data[1], 3)}, Height:{round(drone_height, 2)}',
                        org, font, fontScale, color, thickness, cv2.LINE_AA)


def draw_center_rect(center_bbox, me, frame):
    cv2.circle(frame, center=(int(center_bbox[0]), int(center_bbox[1])), radius=2, color=(0, 200, 0),
               thickness=1)
    down_left = (int(frame.shape[1] * 0.4), int(frame.shape[0] * 0.6))
    up_right = (int(frame.shape[1] * 0.6), int(frame.shape[0] * 0.4))
    cv2.rectangle(frame, down_left, up_right, (255, 255, 0), 2)
    ############ get center rectangle relatively to drones pitch ######################
    theta = me.get_pitch()
    n_down_left = [down_left[0], down_left[1]]
    n_up_right = [up_right[0], up_right[1]]
    n_down_left[1] = down_left[1] - 10 * int(theta)
    n_up_right[1] = up_right[1] - 10 * int(theta)
    cv2.rectangle(frame, n_down_left, n_up_right, (0, 255, 0), 2)
    center_bbox_with_pitch = [(n_down_left[0] + n_up_right[0]) / 2, (n_down_left[1] + n_up_right[1]) / 2]
    #################################### END #########################################
    return center_bbox_with_pitch


def folllowUser(me, frame, user_bbox, HOME_MODE, t2, user_vel_target, t_last_command, last_command, drone_height,
                t1_keepalive):
    center_bbox = [user_bbox[0] + user_bbox[2] / 2, user_bbox[1] + user_bbox[3] / 2]
    speed = 15
    V_x, V_y = 0, 0
    t1 = time.perf_counter()
    dt = t1 - t2  # time between frames
    t2 = time.perf_counter()
    if (dt > 0):
        user_vel_target.set_dt(round(dt, 5))
    ex, ey = 0, 0
    user_vel_target.add_pos((center_bbox[0], center_bbox[1]))
    t_last_command, send_command = past_n_sec(t_last_command, n=0.01)
    if send_command:
        if (user_vel_target._num_frames > user_vel_target._Na + 3):
            ex, ey = user_vel_target.velocity_cmd(h=drone_height)
            i = user_vel_target._num_frames % user_vel_target._N
            V_vx, V_vy, V_ex, V_ey = user_vel_target.Vx_est[i], user_vel_target.Vy_est[i], user_vel_target.Vex_est[i], \
                                     user_vel_target.Vey_est[i]
            V_x, V_y = V_vx + V_ex, V_vy + V_ey
            user_vel_target.write_to_file()
        # V_x, V_y = limit_velocity(V_x, V_y)
        me.left_right_velocity, me.forward_backward_velocity = limit_velocity(V_x, V_y)
        # me.left_right_velocity, me.forward_backward_velocity = limit_velocity_2(V_x, V_y)
        #########################################################################
        put_text_on_frame(frame, [me.left_right_velocity, me.forward_backward_velocity], drone_height)
        #########################################################################
        z_speed = 0
        if center_bbox[0] != 0 and center_bbox[1] != 0:
            # center_bbox_with_pitch = draw_center_rect(center_bbox, me, frame)
            z_speed = fix_height(me, speed, HOME_MODE, drone_height)
            if (me.left_right_velocity != last_command[0] or me.forward_backward_velocity != last_command[
                1]):  # and send_zero==True):
                me.send_rc_control(me.left_right_velocity, me.forward_backward_velocity, 0, 0)
                last_command = [me.left_right_velocity, me.forward_backward_velocity]
                t1_keepalive = time.perf_counter()
    return me, t2, last_command, user_vel_target, t_last_command, t1_keepalive
# fix_height,put text
