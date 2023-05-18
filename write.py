# write images to folders

import time
import os
import cv2
from defines import DEBUG, DATA_DIR

class Writer:
    """Stores drone images to data folders, automatically"""
    
    def __init__(self):
        self.time = time.localtime()
        self.image_index = 0
        self.folder = None
        self.is_ok = True
        self.create_folder()
    
    def _print(self, *s):
        if DEBUG:
            print('writer:', *s)

    def create_folder(self):
        t = self.time
        folder = f'{t.tm_year:04d}-{t.tm_mon:02d}-{t.tm_mday:02d}T{t.tm_hour:02d}_{t.tm_min:02d}_{t.tm_sec:02d}'
        self.folder = os.path.join(DATA_DIR, folder)
        try:
            os.mkdir(self.folder)
            self._print('succesfully created folder', self.folder)
        except FileNotFoundError:
            self._print(f'Unable to create direcotry {self.folder}')
            self.is_ok = False

    def add_image(self, img):
        # write the image to the folder
        image_name = f'image{self.image_index:04d}.png'
        image_path = os.path.join(self.folder, image_name)
        # write image to file
        cv2.imwrite(image_path, img)
        self.image_index += 1
        
if __name__ == '__main__':
    # test unit creates a folder with today's time and fills it with 255 images with a varying green color
    import numpy as np
    m = np.zeros([100, 100, 3], dtype=np.uint8)
    w = Writer()
    if w.is_ok:
        for i in range(255):
            print(i)
            m[0:50, 50:100, 1] = i
            w.add_image(m)
    else:
        print('unit testing writer() failed')