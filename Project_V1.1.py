import cv2
from write import Writer as wr
import time
from djitellopy import tello
from defines import DEBUG, DATA_DIR
from velocity_controller import Target
from UserTracker import folllowUser, put_text_on_frame
from UserRecognition import detect_user, UserData
from ObstacleState import Obstacle, Axis
# from Path import get_best_angle
from FullPath import get_full_path
from ObstacleDetector import ObstacleData, detect_obstacles
from DestinationRecognition import TargetData, detect_target

me = tello.Tello()
me.connect()
print(me.get_battery())
me.streamon()
w = wr()
time.sleep(1)
w2 = wr()
#######################global variables##############
frame = me.get_frame_read().frame
frame = cv2.resize(frame, (640, 480))
HOME_MODE = 1
drone_height = 0
t_keepalive = time.perf_counter()


#####################################################
def drone_stay_alive(t_last_move):
    global me
    t_curr = time.perf_counter()
    if (t_curr - t_last_move > 6):
        me.rotate_clockwise(1)
        print("clokwise 11111111111111111111111111111\n")
        t_last_move = time.perf_counter()
    return t_last_move


def draw_guideline(frame_1, user_data):
    posx, posy = user_data.user_obs.pos[Axis.XAXIS][0], user_data.user_obs.pos[Axis.YAXIS][0]
    cv2.line(frame_1, (int(posx), int(posy)), (int(posx + user_data.user_obs.vx), int(posy + user_data.user_obs.vy)),
             (255, 120, 255), 3)


def wait_for_start():
    global t_keepalive, drone_height
    start_fly = 0
    while True:
        frame = me.get_frame_read().frame
        frame = cv2.resize(frame, (640, 480))
        w.add_image(frame)
        put_text_on_frame(frame, [0, 0], drone_height)
        if start_fly:
            t_keepalive = drone_stay_alive(t_keepalive)
        cv2.imshow("Tracking", frame)
        k = cv2.waitKey(1) & 0xff
        start_fly = GetKeyboardInput(k)
        if k == ord('l'):
            return 0, start_fly
        if k == ord('h'):
            return 1, start_fly


def main_func(user_performence_file=None, obstacles_performence_file=None, target_performence_file=None,
              navigation_timing_file=None):
    global me, t_keepalive, frame, drone_height
    ###### User tracking vars ###########
    user_data = UserData()

    ###### Obstacles tracking vars ######
    obstacle_data = ObstacleData()

    index = 0
    ###### Destination tracking vars ####
    target_data = TargetData()

    ###### drone follow user vars #######
    user_vel_target = Target()
    dt_between_frames = time.perf_counter()
    last_command = [0, 0]
    t_last_command = time.perf_counter()
    me.left_right_velocity, me.forward_backward_velocity = 0, 0
    ###################################
    while True:
        frame = me.get_frame_read().frame
        frame = cv2.resize(frame, (640, 480))
        w.add_image(frame)
        user_data, user_performence_file = detect_user(frame, user_data, user_performence_file, draw_circle=True)

        obstacle_data, obstacles_performence_file = detect_obstacles(frame, obstacle_data, index,
                                                                     obstacles_performence_file, draw_circles=True)
        target_data, target_performence_file = detect_target(frame, target_data, target_performence_file)

        theta, navigation_timing_file = get_full_path(frame, user_data.user_obs, obstacle_data.obstacles,
                                                         target_data.end_point,
                                                         navigation_timing_file)

        t_keepalive = drone_stay_alive(t_keepalive)

        me, dt_between_frames, last_command, user_vel_target, t_last_command, t_keepalive = folllowUser(me, frame,
                                                                                                        user_data.bbox,
                                                                                                        HOME_MODE,
                                                                                                        dt_between_frames,
                                                                                                        user_vel_target,
                                                                                                        t_last_command,
                                                                                                        last_command,
                                                                                                        drone_height,
                                                                                                        t_keepalive)
        w2.add_image(frame)
        cv2.imshow("Tracking", frame)
        k = cv2.waitKey(1) & 0xff
        start_fly = GetKeyboardInput(k)
        index = index + 1
        if k == ord('r'):
            me.send_rc_control(0, 0, 0, 0)
            return user_performence_file, obstacles_performence_file, target_performence_file, navigation_timing_file


def after_function(user_data, obstacle_data, f_user, f_obstacle):
    user_data.close_file(f_user)
    obstacle_data.close_file(f_obstacle)


def GetKeyboardInput(k, start_fly=True):
    global drone_height
    step = 100
    if k == ord('h'):
        me.move_right(step)
    elif k == ord('f'):
        me.move_left(step)
    if k == ord('t'):
        me.move_forward(step)
    elif k == ord('v'):
        me.move_back(step)
    if k == ord('w'):
        me.move_up(step)
        drone_height = drone_height + step
    elif k == ord('s'):
        me.move_down(step)
        drone_height = drone_height - step
    if k == ord('a'):
        me.send_rc_control(0, 0, 10, 0)
    elif k == ord('d'):
        me.send_rc_control(0, 0, 0, 0)
    if k == ord('q'):
        me.land()
        time.sleep(3)
        drone_height = 0
    if k == ord('p'):
        me.takeoff()
        drone_height = 40
        return True
    return start_fly


def project_v1_1():
    user_performence_file = None
    obstacles_performence_file = None
    target_performence_file = None
    navigation_timing_file = None
    while True:
        out, _ = wait_for_start()
        if out:
            break
        user_performence_file, obstacles_performence_file, target_performence_file, navigation_timing_file = main_func(
            user_performence_file, obstacles_performence_file, target_performence_file, navigation_timing_file)


project_v1_1()
