import cv2
import numpy as np
from write import Writer as wr
from ObstacleState import Obstacle, Axis
import time


class ObstacleData:
    def __init__(self, bboxs=[], obstacles=[], trackers=[], valid=[]):
        self.bboxs = bboxs
        self.obstacles = obstacles
        self.trackers = trackers
        self.valid = valid
        self.new = []  # debug
        self.loop_time = 0

    def add_position(self, index):
        center_bbox_x, center_bbox_y = self.bboxs[index][0] + self.bboxs[index][2] / 2, self.bboxs[index][1] + \
                                       self.bboxs[index][3] / 2
        self.obstacles[index].add_pos({Axis.XAXIS: center_bbox_x, Axis.YAXIS: center_bbox_y})

    def start_tracking(self, frame):
        for j, bbox in enumerate(self.bboxs):
            center_bbox_x, center_bbox_y = bbox[0] + bbox[2] / 2, bbox[1] + bbox[3] / 2
            radius = np.sqrt(bbox[2] * bbox[2] + bbox[3] * bbox[3]) / 2
            self.trackers.append(cv2.TrackerKCF_create())
            self.trackers[j].init(frame, self.bboxs[j])
            self.obstacles.append(Obstacle(radius=radius))
            self.obstacles[j].add_pos({Axis.XAXIS: center_bbox_x, Axis.YAXIS: center_bbox_y})
            self.valid.append(True)
            self.new.append(True)

    def update_tracker(self, frame, index):
        ok, bbox = self.trackers[index].update(frame)
        if ok:
            radius = np.sqrt(bbox[2] * bbox[2] + bbox[3] * bbox[3]) / 2
            self.obstacles[index].radius = radius
            self.bboxs[index] = bbox
            self.add_position(index)
            self.new[index] = False
        else:
            self.valid[index] = False

    def reset_lsts(self):
        self.bboxs, self.obstacles, self.trackers, self.valid, self.new = [], [], [], [], []

    def draw_bboxs(self, frame, with_circles=False):
        for j, bbox in enumerate(self.bboxs):
            if self.valid[j]:
                cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[0] + bbox[2], bbox[1] + bbox[3]), (0, 0, 255), 3)
                if with_circles:
                    center_x, center_y = round(bbox[0] + bbox[2] / 2), round(bbox[1] + bbox[3] / 2)
                    #circle radius- need to change here and in Path.py line 34 !!!
                    cv2.circle(frame, center=(center_x, center_y), radius=round(self.obstacles[j].radius) + 15,
                               color=(0, 0, 255),
                               thickness=3)

    def match_rects(self, frame, bboxes):
        new_bboxes_lst = []
        new_obstacle_lst = []
        new_trackers_lst = []
        new_valid = []
        new_new_lst = []
        for i, new_bbox in enumerate(bboxes):
            match = 0
            for j, old_bbox in enumerate(self.bboxs):
                epsilon = 20
                if abs(new_bbox[0] - old_bbox[0]) <= epsilon and abs(new_bbox[1] - old_bbox[1]) <= epsilon and abs(
                        new_bbox[2] - old_bbox[2]) <= epsilon and abs(new_bbox[3] - old_bbox[3]) <= epsilon:
                    if (self.valid[j]):
                        new_bboxes_lst.append(old_bbox)
                        new_obstacle_lst.append(self.obstacles[j])
                        new_trackers_lst.append(self.trackers[j])
                        new_valid.append(True)
                        new_new_lst.append(False)
                        # print("found match")
                    else:
                        new_tracker = cv2.TrackerKCF_create()
                        new_tracker.init(frame, new_bbox)
                        center_bbox_x, center_bbox_y = new_bbox[0] + new_bbox[2] / 2, new_bbox[1] + new_bbox[3] / 2
                        self.obstacles[j].add_pos({Axis.XAXIS: center_bbox_x, Axis.YAXIS: center_bbox_y})
                        new_bboxes_lst.append(new_bbox)
                        new_obstacle_lst.append(self.obstacles[j])
                        new_trackers_lst.append(self.trackers[j])
                        new_valid.append(True)
                        new_new_lst.append(False)
                        # print("MAKE match")
                    match = 1
                    break
            if match == 0:
                center_bbox_x, center_bbox_y = new_bbox[0] + new_bbox[2] / 2, new_bbox[1] + new_bbox[3] / 2
                radius = np.sqrt(new_bbox[2] * new_bbox[2] + new_bbox[3] * new_bbox[3]) / 2
                new_obs = Obstacle(radius=radius)
                new_obs.add_pos({Axis.XAXIS: center_bbox_x, Axis.YAXIS: center_bbox_y})
                new_tracker = cv2.TrackerKCF_create()
                new_tracker.init(frame, new_bbox)
                new_bboxes_lst.append(new_bbox)
                new_obstacle_lst.append(new_obs)
                new_trackers_lst.append(new_tracker)
                new_valid.append(True)
                new_new_lst.append(True)
                # print("Lost track")
        self.bboxs = new_bboxes_lst
        self.obstacles = new_obstacle_lst
        self.trackers = new_trackers_lst
        self.valid = new_valid
        self.new = new_new_lst

    def write_to_file(self, f=None, filename='obstacle_tracking.csv'):
        if f == None:
            f = open(filename, 'wt')
            f.write('title,bbox_x,bbox_y,bbox_w,bbox_h,new,vx,vy,posx,posy,radius,num_poses\n')
        f.write('loop time:,%f\n' % self.loop_time)
        for i in range(len(self.bboxs)):
            f.write('bbox %d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n' % (
                i, self.bboxs[i][0], self.bboxs[i][1], self.bboxs[i][2], self.bboxs[i][3], self.new[i],
                self.obstacles[i].vx, self.obstacles[i].vy, self.obstacles[i].pos[Axis.XAXIS][0],
                self.obstacles[i].pos[Axis.YAXIS][0], self.obstacles[i].radius, self.obstacles[i].num_poses))
        f.write('\n')
        return f

    def close_file(self, f, filename='loop_times_obstacle_tracking.csv'):
        f.close()
        # print('wrote target estimator to file ', filename)


def initial_obstacle_detection_2(frame, target_color_lab=(99, 177, 158), threshold=20):
    frame = cv2.GaussianBlur(frame, (11, 11), 0)
    # Convert the frame to LAB color space
    frame_lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

    # Compute the Euclidean distance between each pixel in the frame and the target color
    distance = np.linalg.norm(frame_lab - target_color_lab, axis=-1)
    # Find all pixels that are within the threshold distance from the target color
    mask = distance <= threshold
    mask = mask.astype(np.uint8)
    mask = mask * 255
    analysis = cv2.connectedComponentsWithStats(mask, 4, cv2.CV_32S)
    (totalLabels, label_ids, values, centroid) = analysis

    x, y, w, h = 0, 0, 0, 0
    bbox_list = []
    for i in range(1, totalLabels):
        x = values[i, cv2.CC_STAT_LEFT]
        y = values[i, cv2.CC_STAT_TOP]
        w = values[i, cv2.CC_STAT_WIDTH]
        h = values[i, cv2.CC_STAT_HEIGHT]
        # for j in range(1, len(label_ids)):
        delta = 0
        obstacle_bbox = [x - delta, y - delta, w + delta, h + delta]
        bbox_list.append(obstacle_bbox)

    return bbox_list


def initial_obstacle_detection(frame):
    frame = cv2.resize(frame, (640, 480))
    frame = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Two possible ranges for the color RED in HSV, these are their bounds
    lower1 = np.array([0, 100, 20])
    upper1 = np.array([10, 255, 255])
    lower2 = np.array([160, 100, 20])
    upper2 = np.array([179, 255, 255])

    lower_mask = cv2.inRange(hsv_frame, lower1, upper1)
    upper_mask = cv2.inRange(hsv_frame, lower2, upper2)
    mask = lower_mask + upper_mask

    cv2.bitwise_and(hsv_frame, hsv_frame, mask=mask)
    analysis = cv2.connectedComponentsWithStats(mask, 4, cv2.CV_32S)
    (totalLabels, label_ids, values, centroid) = analysis

    x, y, w, h = 0, 0, 0, 0
    bbox_list = []
    for i in range(1, totalLabels):
        area = values[i, cv2.CC_STAT_AREA]
        w = values[i, cv2.CC_STAT_WIDTH]
        h = values[i, cv2.CC_STAT_HEIGHT]
        if area > 10 and area<8000 and w<210 and h<210:
            x = values[i, cv2.CC_STAT_LEFT]
            y = values[i, cv2.CC_STAT_TOP]
            # for j in range(1, len(label_ids)):
            delta = 0
            obstacle_bbox = [x - delta, y - delta, w + delta, h + delta]
            bbox_list.append(obstacle_bbox)

    return bbox_list


def detect_obstacles(frame, obstacle_data, index, f, draw_circles=False):
    t1 = time.perf_counter()
    if (index == 0):
        obstacle_data.reset_lsts()
        obstacle_data.bboxs = initial_obstacle_detection(frame)
        obstacle_data.start_tracking(frame)
        t2 = time.perf_counter()
        obstacle_data.loop_time = round(t2 - t1, 4)
        f = obstacle_data.write_to_file()
        index += 1
        return obstacle_data, f
    for i in range(len(obstacle_data.bboxs)):
        obstacle_data.update_tracker(frame, i)
    bboxes = initial_obstacle_detection(frame)
    obstacle_data.match_rects(frame, bboxes)
    obstacle_data.draw_bboxs(frame, draw_circles)
    t2 = time.perf_counter()
    # print(f"loop time: {t2 - t1}")
    obstacle_data.loop_time = round(t2 - t1, 4)
    f = obstacle_data.write_to_file(f=f)
    return obstacle_data, f
