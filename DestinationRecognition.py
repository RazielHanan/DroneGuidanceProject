import cv2
import numpy as np
from ObstacleState import Obstacle, Axis
import time


class TargetData:
    def __init__(self, bbox=(0, 0, 0, 0), tracker=cv2.TrackerKCF_create()):
        self.bbox = bbox
        self.tracker = tracker
        self.valid = False
        self.new = True  # debug
        self.loop_time = 0
        self.calculate_end_point()

    def calculate_end_point(self):
        center_x, center_y = self.bbox[0] + self.bbox[2] / 2, self.bbox[1] + self.bbox[3] / 2
        self.end_point = {Axis.XAXIS: center_x, Axis.YAXIS: center_y}

    def draw_bbox(self, frame):
        cv2.rectangle(frame, (self.bbox[0], self.bbox[1]), (self.bbox[0] + self.bbox[2], self.bbox[1] + self.bbox[3]),
                      (255, 0, 0), 3)

    def write_to_file(self, f, filename='loop_times_user_tracking.csv'):
        if f == None:
            f = open(filename, 'wt')
            f.write('bbox_x,bbox_y,bbox_w,bbox_h,new,valid\n')
        f.write('%f, %f, %f, %f, %f, %f, %f\n' % (
            self.loop_time, self.bbox[0], self.bbox[1], self.bbox[2], self.bbox[3], self.new, self.valid))
        return f

    def close_file(self, f, filename='loop_times_user_tracking.csv'):
        f.close()
        #print('wrote target estimator to file ', filename)


def destination_reco(frame, target_color_lab=(129, 145, 77), threshold=25):
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
    max_s = 10
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
    if max_s != 10:
        valid = True
    else:
        valid = False
    return valid, bbox


def detect_target(frame, targetdata, f):
    t1 = time.perf_counter()
    if targetdata.valid:
        #print(targetdata.bbox)
        ok, bbox = targetdata.tracker.update(frame)
        if ok:
            #print("continue\n")
            center_bbox_x, center_bbox_y = bbox[0] + bbox[2] / 2, bbox[1] + bbox[3] / 2
            targetdata.bbox = bbox
            targetdata.new = False
            targetdata.valid = True
        else:
            valid, bbox = destination_reco(frame)
            if valid:
                #print("helped by reco\n")
                center_bbox_x, center_bbox_y = bbox[0] + bbox[2] / 2, bbox[1] + bbox[3] / 2
                targetdata.tracker = cv2.TrackerKCF_create()
                targetdata.tracker.init(frame, bbox)
                targetdata.bbox = bbox
                targetdata.new = False
                targetdata.valid = True
            else:
                #print("was fine, lost\n")
                targetdata.valid = False
    else:
        valid, bbox = destination_reco(frame)
        if valid:
            #print("wasnt fine, we init\n")
            center_bbox_x, center_bbox_y = bbox[0] + bbox[2] / 2, bbox[1] + bbox[3] / 2
            targetdata.bbox = bbox
            targetdata.tracker = cv2.TrackerKCF_create()
            targetdata.tracker.init(frame, bbox)
            targetdata.valid = True
            targetdata.new = True
        else:
            #print("wasnt good, still bad\n")
            targetdata.valid = False
    t2 = time.perf_counter()
    targetdata.loop_time = round(t2 - t1, 4)
    #print(targetdata.loop_time)
    targetdata.draw_bbox(frame)
    f = targetdata.write_to_file(f=f, filename='target.csv')
    if targetdata.valid:
        targetdata.calculate_end_point()
    return targetdata, f
