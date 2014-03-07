#!/usr/bin/env python

'''
Lucas-Kanade tracker
====================

Lucas-Kanade sparse optical flow demo. Uses goodFeaturesToTrack
for track initialization and back-tracking for match verification
between frames.

Usage
-----
lk_track.py [<video_source>]


Keys
----
ESC - exit
'''

import numpy as np
import cv2
#import video
from common import anorm2, draw_str
from time import clock
import math
import colorsys

from __future__ import division

import libardrone

lk_params = dict( winSize  = (15, 15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

feature_params = dict( maxCorners = 500,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )

BLUE = [255,0,0]        # rectangle color
RED = [0,0,255]         # PR BG
GREEN = [0,255,0]       # PR FG
BLACK = [0,0,0]         # sure BG
WHITE = (255,255,255)   # sure FG

DRAW_BG = {'color' : BLACK, 'val' : 0}
DRAW_FG = {'color' : WHITE, 'val' : 1}
DRAW_PR_FG = {'color' : GREEN, 'val' : 3}
DRAW_PR_BG = {'color' : RED, 'val' : 2}

class App:
    def __init__(self, video_src):
        self.track_len = 10
        self.detect_interval = 5
        self.tracks = []
        #self.cam = video.create_capture(video_src)
        self.frame_idx = 0
        self.drone = libardrone.ARDrone(is_ar_drone_2=True)
        self.time = clock()

    def run(self):
        try:
            while True:
                #ret, frame = self.cam.read()
                frame = self.drone.image
                frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                vis = frame.copy()
                output = frame.copy()
                vis = cv2.cvtColor(vis, cv2.COLOR_BGR2RGB)
                t_rex = frame.copy()
                cv2.rectangle(t_rex, (0,0), (vis.shape[1], vis.shape[0]), BLACK, thickness=-1)
                gc_mask = np.zeros(vis.shape[:2],dtype = np.uint8) # mask initialized to PR_BG
                cluster_plot = np.zeros(vis.shape[:2],dtype = np.uint8)

                if len(self.tracks) > 0:
                    img0, img1 = self.prev_gray, frame_gray
                    p0 = np.float32([tr[-1] for tr in self.tracks]).reshape(-1, 1, 2)
                    p1, st, err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **lk_params)
                    p0r, st, err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **lk_params)
                    d = abs(p0-p0r).reshape(-1, 2).max(-1)
                    good = d < 1
                    new_tracks = []
                    for tr, (x, y), good_flag in zip(self.tracks, p1.reshape(-1, 2), good):
                        if not good_flag:
                            continue
                        tr.append((x, y))
                        if len(tr) > self.track_len:
                            del tr[0]
                        new_tracks.append(tr)
                        #if len(tr) > 1:
                        #    (x_old, y_old) = tr[0]
                        #    if x > x_old:
                        #        color = (0, 255, 0)
                        #    else:
                        #        color = (255, 0, 0)
                        #else:
                        #    color = (0, 255, 0)
                        #cv2.circle(vis, (x, y), 2, color, -1)
                        #cv2.polylines(vis, [np.int32(dummy) for dummy in [tr]], False, (color))
                    self.tracks = new_tracks
                    sum_x = 0
                    sum_y = 0
                    for tr in self.tracks:
                        if len(tr) > 1:
                            sum_x += tr[-1][0] - tr[0][0]
                            sum_y += tr[-1][1] - tr[0][1]
                    if len(self.tracks) > 0:
                        mean_x = sum_x * 1.0 / len(self.tracks)
                        mean_y = sum_y * 1.0 / len(self.tracks)
                    else:
                        mean_x = 0
                        mean_y = 0
                    d = []
                    v = []
                    a = []
                    for tr in self.tracks:
                        if len(tr) > 1:
                            (x, y) = tr[-1]
                            v_x = (tr[-1][0] - tr[0][0]) / 1#len(tr)
                            v_y = (tr[-1][1] - tr[0][1]) / 1#len(tr)
                            angle = math.atan2(v_y, v_x)
                            a.append((np.pi + angle) / (2 * np.pi))
                            v.append((v_x, v_y))
                            d.append(np.abs(v_x - mean_x) + np.abs(v_y - mean_y))
                        else:
                            d.append(0)
                    try:
                        max_d = max(d)
                    except:
                        max_d = 1
                    for (i, tr) in enumerate(self.tracks):
                        intensity = np.floor(255.0 * d[i] / max_d)
                        color = (255 - intensity, 255 - intensity, 255 - intensity)
                        #angle_color = [temp * 255 for temp in colorsys.hsv_to_rgb(a[i],intensity/255,intensity/255)]
                        angle_color = [temp * 255 for temp in colorsys.hsv_to_rgb(a[i],1,1)]
                        (x, y) = tr[-1]
                        cv2.circle(vis, (x, y), 10, angle_color, -1)
                        cv2.circle(t_rex, (x, y), 10, angle_color, -1)
                        cv2.circle(cluster_plot, (320+int(np.floor(1*v[i][0])), 180+int(np.floor(1*v[i][1]))), 5, WHITE, -1)
                        #if intensity > 200:
                        #    cv2.circle(gc_mask, (x, y), 10, DRAW_PR_FG['val'], -1)
                        #else:
                        #    cv2.circle(gc_mask, (x, y), 10, DRAW_PR_BG['val'], -1)

                    #bgdmodel = np.zeros((1,65),np.float64)
                    #fgdmodel = np.zeros((1,65),np.float64) 
                    #rect = None
                    #cv2.grabCut(frame,gc_mask,rect,bgdmodel,fgdmodel,1,cv2.GC_INIT_WITH_MASK)
                    #mask2 = np.where((gc_mask==1) + (gc_mask==3),255,0).astype('uint8')
                    #output = cv2.bitwise_and(frame,frame,mask=mask2)   

                    #cv2.polylines(vis, [np.int32(tr) for tr in self.tracks], False, (0, 255, 0))
                    draw_str(vis, (20, 20), 'track count: %d' % len(self.tracks))

                if self.frame_idx % self.detect_interval == 0:
                    mask = np.zeros_like(frame_gray)
                    mask[:] = 255
                    for x, y in [np.int32(tr[-1]) for tr in self.tracks]:
                        cv2.circle(mask, (x, y), 5, 0, -1)
                    p = cv2.goodFeaturesToTrack(frame_gray, mask = mask, **feature_params)
                    if p is not None:
                        for x, y in np.float32(p).reshape(-1, 2):
                            self.tracks.append([(x, y)])


                self.frame_idx += 1
                self.prev_gray = frame_gray

                loop_time = (clock() - self.time) * 1000
                self.time = clock()
                draw_str(vis, (20, 40), 'Time : %dms' % np.floor(loop_time))

                cv2.imshow('lk_track', vis)
                cv2.imshow('t_rex', t_rex)
                #cv2.imshow('grab_cut', output)
                cv2.imshow('cluster', cluster_plot)

                ch = 0xFF & cv2.waitKey(1)
                if ch == 27:
                    break
        finally:
            cv2.destroyAllWindows()
            self.drone.emergency()
            self.drone.reset()
            self.drone.halt()

def main():
    import sys
    try: video_src = sys.argv[1]
    except: video_src = 0

    print __doc__
    App(video_src).run()

if __name__ == '__main__':
    main()
