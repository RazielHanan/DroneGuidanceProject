import cv2
import numpy as np
from ObstacleState import Obstacle, Axis
import time


class UserData:
    def __init__(self, bbox=[0, 0, 0, 0], user_obs=Obstacle(speed=80, user_mode=True), tracker=cv2.TrackerKCF_create()):
        self.bbox = bbox
        self.user_obs = user_obs
        self.tracker = tracker
        self.valid = False
        self.new = True  # debug
        self.loop_time = 0

    def draw_bbox(self, frame, draw_circle=False):
        cv2.rectangle(frame, (self.bbox[0], self.bbox[1]), (self.bbox[0] + self.bbox[2], self.bbox[1] + self.bbox[3]),
                      (0, 255, 0), 3)
        if draw_circle:
            center_x, center_y = round(self.bbox[0] + self.bbox[2] / 2), round(self.bbox[1] + self.bbox[3] / 2)
            cv2.circle(frame, center=(center_x, center_y), radius=round(self.user_obs.radius),
                       color=(0, 255, 0),
                       thickness=3)

    def write_to_file(self, f, filename='loop_times_user_tracking.csv'):
        if f == None:
            f = open(filename, 'wt')
            f.write('bbox_x,bbox_y,bbox_w,bbox_h,new,valid\n')
        f.write('%f, %f, %f, %f, %f, %f, %f\n' % (
            self.loop_time, self.bbox[0], self.bbox[1], self.bbox[2], self.bbox[3], self.new, self.valid))
        return f

    def close_file(self, f, filename='loop_times_user_tracking.csv'):
        f.close()
        # print('wrote target estimator to file ', filename)


def user_reco(frame, target_color_lab=(190, 94, 132), threshold=20):
    frame = cv2.GaussianBlur(frame, (11, 11), 0)
    # Convert the frame to LAB color space
    frame_lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

    # Compute the Euclidean distance between each pixel in the frame and the target color
    distance = np.linalg.norm(frame_lab - target_color_lab, axis=-1)
    # Find all pixels that are within the threshold distance from the target color
    mask = distance <= threshold
    mask = mask.astype(np.uint8)
    mask = mask * 255
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 4, cv2.CV_32S)

    # Find the bounding boxes of all connected components (except the background label 0)
    max_s = 0
    bbox = [0, 0, 0, 0]
    for i in range(1, num_labels):
        x = stats[i, cv2.CC_STAT_LEFT]
        y = stats[i, cv2.CC_STAT_TOP]
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]
        s = w * h
        # for j in range(1, len(label_ids)):
        obstacle_bbox = [round(x), round(y), round(w), round(h)]
        if s >= max_s:
            bbox = obstacle_bbox
            max_s = s

    # Return the largest bounding box
    if max_s != 0:
        valid = True
    else:
        valid = False
    return valid, bbox


def user_reco_2(frame):
    frame = cv2.GaussianBlur(frame, (7, 7), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    l_b = np.array([23, 53, 0])
    u_b = np.array([90, 118, 251])
    mask = cv2.inRange(hsv, l_b, u_b)
    res = cv2.bitwise_and(frame, frame, mask=mask)
    # mask=cv2.cvtColor(mask,cv2.COLOR_BGR2GRAY)
    analysis = cv2.connectedComponentsWithStats(mask, 4, cv2.CV_32S)
    (totalLabels, label_ids, values, centroid) = analysis
    # Initialize a new image to store
    # all the output components
    output = np.zeros(mask.shape, dtype="uint8")
    max_area = 0
    max_area_compnent = 0
    x, y, w, h = 0, 0, 0, 0
    delta = 0
    # Loop through each component
    for i in range(1, totalLabels):
        # Area of the component
        area = values[i, cv2.CC_STAT_AREA]
        if area > max_area:
            x = values[i, cv2.CC_STAT_LEFT]
            y = values[i, cv2.CC_STAT_TOP]
            w = values[i, cv2.CC_STAT_WIDTH]
            h = values[i, cv2.CC_STAT_HEIGHT]
            max_area_compnent = i
            max_area = area
    bbox = (x - delta, y - delta, w + delta, h + delta)
    not_ok = (bbox[0] == -delta) and (bbox[1] == -delta) and (bbox[2] == delta) and (bbox[3] == delta)
    return (not not_ok), bbox


def detect_user(frame, userdata, f, draw_circle=False):
    t1 = time.perf_counter()
    if userdata.valid:
        # (userdata.bbox)
        ok, bbox = userdata.tracker.update(frame)
        if ok:
            # print("continue\n")
            center_bbox_x, center_bbox_y = bbox[0] + bbox[2] / 2, bbox[1] + bbox[3] / 2
            userdata.user_obs.add_pos({Axis.XAXIS: center_bbox_x, Axis.YAXIS: center_bbox_y})
            userdata.bbox = bbox
            userdata.new = False
            userdata.valid = True
        else:
            valid, bbox = user_reco(frame)
            if valid:
                # print("helped by reco\n")
                center_bbox_x, center_bbox_y = bbox[0] + bbox[2] / 2, bbox[1] + bbox[3] / 2
                userdata.tracker = cv2.TrackerKCF_create()
                userdata.tracker.init(frame, bbox)
                radius = np.sqrt(bbox[2] * bbox[2] + bbox[3] * bbox[3]) / 2
                userdata.user_obs.radius = radius
                userdata.user_obs.add_pos({Axis.XAXIS: center_bbox_x, Axis.YAXIS: center_bbox_y})
                userdata.bbox = bbox
                userdata.new = False
                userdata.valid = True
            else:
                # print("was fine, lost\n")
                userdata.valid = False
    else:
        valid, bbox = user_reco(frame)
        if valid:
            # print("wasnt fine, we init\n")
            center_bbox_x, center_bbox_y = bbox[0] + bbox[2] / 2, bbox[1] + bbox[3] / 2
            userdata.bbox = bbox
            radius = np.sqrt(bbox[2] * bbox[2] + bbox[3] * bbox[3]) / 2
            userdata.user_obs.radius = radius
            userdata.user_obs.add_pos({Axis.XAXIS: center_bbox_x, Axis.YAXIS: center_bbox_y})
            userdata.tracker = cv2.TrackerKCF_create()
            userdata.tracker.init(frame, bbox)
            userdata.valid = True
            userdata.new = True
        else:
            # print("wasnt good, still bad\n")
            userdata.valid = False
    t2 = time.perf_counter()
    userdata.loop_time = round(t2 - t1, 4)
    # print(userdata.loop_time)
    userdata.draw_bbox(frame, draw_circle=draw_circle)
    f = userdata.write_to_file(f=f, filename='user_tracking.csv')
    return userdata, f
