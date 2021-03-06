#!/usr/bin/env python

'''
Leap-copter
================

Controlling an ARDrone with a leap motion

Usage:
------
    leap-copter.py

    ...to be updated...

Keys:
-----
    ESC   - exit
    ...to be updated...
'''

DRONE = True

import sys
sys.path.append('../libleap/')
sys.path.append('../libardrone/')

import numpy as np
import cv2

if DRONE:
    import libardrone

import Leap
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture

class LeapToQuadListener(Leap.Listener):
    def on_init(self, controller):
        print "Leap listener initialised"

    def on_connect(self, controller):
        print "Connected to leap motion"

    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        print "Disconnected from leap motion"

    def on_exit(self, controller):
        print "Exited leap motion listener"

    def on_frame(self, controller):
        if self.parent.leap_control and (not self.parent.ai_control):

            # Get the most recent frame and report some basic information
            frame = controller.frame()

            if not frame.hands.is_empty:
                hand = frame.hands[0]
                normal = hand.palm_normal
                direction = hand.direction
                roll = normal.roll * Leap.RAD_TO_DEG
                pitch = direction.pitch * Leap.RAD_TO_DEG
                yaw = direction.yaw * Leap.RAD_TO_DEG
                up_down = hand.palm_position[1] - 300
                left_right = hand.palm_position[0]

                threshold = 20 # Number of degress before command is registered

                # if roll < -threshold:
                #     self.parent.drone.move_right()
                # elif roll > threshold:
                #     self.parent.drone.move_left()
                # elif pitch < -threshold:
                #     self.parent.drone.move_forward()
                # elif pitch > threshold:
                #     self.parent.drone.move_backward()
                # else:
                #     self.parent.drone.hover()

                scale_factor = self.parent.drone.speed / threshold
                up_down_scale_factor = self.parent.drone.speed / 50
                yaw_scale_factor = self.parent.drone.speed / 50

                # if np.abs(roll) < 10:
                #     roll = 0
                # if np.abs(pitch) < 10:
                #     pitch = 0
                # if np.abs(up_down) < 25:
                #     up_down = 0
                # if np.abs(left_right) < 10:
                #     left_right = 0

                self.parent.drone.set_velocity(-roll * scale_factor, pitch * scale_factor, \
                                                up_down * up_down_scale_factor, left_right * yaw_scale_factor)
                print hand.palm_position
                print yaw

            else:
                # Probably want to hover or something else neutral
                self.parent.drone.hover()

    def state_string(self, state):
        if state == Leap.Gesture.STATE_START:
            return "STATE_START"

        if state == Leap.Gesture.STATE_UPDATE:
            return "STATE_UPDATE"

        if state == Leap.Gesture.STATE_STOP:
            return "STATE_STOP"

        if state == Leap.Gesture.STATE_INVALID:
            return "STATE_INVALID"

class App(object):
    def reset_speed(self):
        self.x_speed = 0
        self.y_speed = 0
        self.z_speed = 0

    def __init__(self):
        if DRONE:
            self.drone = libardrone.ARDrone(is_ar_drone_2=True)
            self.frame = cv2.cvtColor(self.drone.get_image(), cv2.COLOR_BGR2RGB)
        else:
            self.camera = cv2.VideoCapture(0)
            rval, self.frame = self.camera.read()

        cv2.namedWindow('camshift')
        #cv2.namedWindow('mask')
        #cv2.namedWindow('prob')
        cv2.setMouseCallback('camshift', self.onmouse)

        self.selection = None
        self.drag_start = None
        self.tracking_state = 0
        self.show_backproj = False
        
        self.ai_control = False
        self.leap_control = False

        # Set up leap motion listener

        self.listener = LeapToQuadListener()
        self.controller = Leap.Controller()

        self.listener.parent = self # So it can send messages back if necessary

        # Have the listener receive events from the controller
        self.controller.add_listener(self.listener)

        # Controller stuff
        self.reset_speed()
        self.I_gain = 0.05
        self.I_decay = 0.90
        self.tracking_status_changed = False
        self.z_set_height = 0.4

    def onmouse(self, event, x, y, flags, param):
        x, y = np.int16([x, y]) # BUG
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drag_start = (x, y)
            self.tracking_state = 0
        if self.drag_start:
            if flags & cv2.EVENT_FLAG_LBUTTON:
                h, w = self.frame.shape[:2]
                xo, yo = self.drag_start
                x0, y0 = np.maximum(0, np.minimum([xo, yo], [x, y]))
                x1, y1 = np.minimum([w, h], np.maximum([xo, yo], [x, y]))
                self.selection = None
                if x1-x0 > 0 and y1-y0 > 0:
                    self.selection = (x0, y0, x1, y1)
            else:
                self.drag_start = None
                if self.selection is not None:
                    self.tracking_state = 1
                    self.reset_speed()

    def run(self):
        try:
            while True:
                if DRONE:
                    self.frame = cv2.cvtColor(self.drone.get_image(), cv2.COLOR_BGR2RGB)
                else:
                    rval, self.frame = self.camera.read()
                vis = self.frame.copy()
                hsv_image = cv2.cvtColor(self.frame, cv2.COLOR_RGB2HSV)
                mask = cv2.inRange(hsv_image, np.array((0., 60., 32.)), np.array((180., 255., 255.)))
                #mask = cv2.inRange(hsv_image, np.array((0., 0., 0.)), np.array((180., 255., 255.)))

                if self.selection:
                    x0, y0, x1, y1 = self.selection
                    self.track_window = (x0, y0, x1-x0, y1-y0)
                    hsv_roi = hsv_image[y0:y1, x0:x1]
                    mask_roi = mask[y0:y1, x0:x1]
                    hist = cv2.calcHist( [hsv_roi], [0], mask_roi, [16], [0, 180] )
                    cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX);
                    self.hist = hist.reshape(-1)

                    vis_roi = vis[y0:y1, x0:x1]
                    cv2.bitwise_not(vis_roi, vis_roi)
                    vis[mask == 0] = 0

                if self.tracking_state == 1:
                    try:
                        self.selection = None
                        prob = cv2.calcBackProject([hsv_image], [0], self.hist, [0,180], 1)
                        prob &= mask
                        term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 100, 1 )
                        track_box, self.track_window = cv2.CamShift(prob, self.track_window, term_crit)

                        #cv2.imshow('prob', prob[...,np.newaxis])
                        try: cv2.ellipse(vis, track_box, (0, 0, 255), 2)
                        except: print track_box
                        x_distance = 2*(track_box[0][0] / hsv_image.shape[1]) - 1
                        x_error = -0.5 * x_distance
                        y_distance = 2*(track_box[0][1] / hsv_image.shape[0]) - 1
                        y_error = 0.2 * y_distance
                        #z_distance = np.min([(track_box[1][0] / hsv_image.shape[0]), (track_box[1][1] / hsv_image.shape[0])])
                        # This is an approximation to the correct solution
                        z_height = (track_box[1][1] / hsv_image.shape[0])
                        if self.tracking_status_changed:
                            self.z_set_height = z_height
                            self.tracking_status_changed = False
                        z_error = -0.3 * ((z_height / self.z_set_height) - 1)
                        #print (x_speed, y_speed, z_speed)
                        #print track_box

                        # P controller
                        #self.x_speed = -x_error * self.drone.speed
                        #self.y_speed = -y_error * self.drone.speed
                        #self.z_speed = -z_error * self.drone.speed

                        # I controller
                        self.x_speed = self.I_decay * self.x_speed - x_error * self.I_gain * self.drone.speed
                        self.y_speed = self.I_decay * self.y_speed - y_error * self.I_gain * self.drone.speed
                        self.z_speed = self.I_decay * self.z_speed - z_error * self.I_gain * self.drone.speed

                        print 'Error: %0.4f %0.4f %0.4f' % (x_error, y_error, z_error)
                        print 'Speed: %0.4f %0.4f %0.4f' % (self.x_speed, self.y_speed, self.z_speed)
                        print 'I_gain: %0.4f' % self.I_gain
                        print 'I_decay: %0.4f' % self.I_decay

                        max_speed = 0.5 * self.drone.speed

                        if self.x_speed > max_speed:
                            self.x_speed = max_speed
                        elif self.x_speed < -max_speed:
                            self.x_speed = -max_speed  

                        if self.y_speed > max_speed:
                            self.y_speed = max_speed
                        elif self.y_speed < -max_speed:
                            self.y_speed = -max_speed  

                        if self.z_speed > max_speed:
                            self.z_speed = max_speed
                        elif self.z_speed < -max_speed:
                            self.z_speed = -max_speed                     
                        
                        if self.ai_control:
                            self.drone.at(libardrone.at_pcmd, True, 0, self.z_speed, self.y_speed, self.x_speed)
                    except:
                        e = sys.exc_info()[0]
                        print 'Tracking failed'
                        #print e
                        self.ai_control = False
                        if DRONE:
                            self.drone.hover()

                cv2.imshow('camshift', vis)
                #cv2.imshow('mask', mask)

                #ch = 0xFF & cv2.waitKey(5)
                ch = cv2.waitKey(5)
                if ch == 27:
                    self.drone.emergency()
                if ch == ord('q'):
                    break
                if ch == ord('b'):
                    self.show_backproj = not self.show_backproj
                if ch == ord('\r'):
                    self.drone.takeoff()
                if ch == ord(' '):
                    self.drone.land()                
                if ch == ord('a'):
                    self.drone.move_left()
                    self.ai_control = False
                if ch == ord('d'):
                    self.drone.move_right()
                    self.ai_control = False
                if ch == ord('w'):
                    self.drone.move_forward()
                    self.ai_control = False
                if ch == ord('s'):
                    self.drone.move_backward()
                    self.ai_control = False
                if ch == ord('5'):
                    self.I_decay = self.I_decay + 0.01
                if ch == ord('4'):
                    self.I_decay = self.I_decay - 0.01
                if ch == ord('7'):
                    self.I_gain = self.I_gain * 1.5
                if ch == ord('6'):
                    self.I_gain = self.I_gain * 0.75
                if ch == ord('9'):
                    self.drone.speed = self.drone.speed * 1.1
                if ch == ord('8'):
                    self.drone.speed = self.drone.speed * 0.9
                if ch == ord('1'):
                    self.leap_control = not self.leap_control
                    if self.leap_control:
                        print 'Leap is in control'
                    else:
                        print 'Leap is NOT in control'
                if ch == 63234:
                    self.drone.turn_left(mult=2.5)
                    self.ai_control = False
                if ch == 63235:
                    self.drone.turn_right(mult=2.5)
                    self.ai_control = False
                if ch == 63232:
                    self.drone.move_up()
                    self.ai_control = False
                if ch == 63233:
                    self.drone.move_down()
                    self.ai_control = False
                if (ch == ord('z')) or (ch == ord('x')) or (ch == ord('c')) or (ch == ord('v')): # i.e. can mash keyboard
                    self.drone.hover()
                    self.ai_control = False
                if ch == ord('y'):
                    self.drone.trim()
                if ch == ord('p'):
                    self.drone.reset()
                if ch == ord('t'):
                    self.reset_speed()
                    self.ai_control = not self.ai_control
                    self.tracking_status_changed = True
                    if self.ai_control:
                        print 'AI in control'
                    else:
                        if DRONE:
                            self.drone.hover()
                        print 'No AI control'
        finally:
            # Remove the sample listener when done
            self.controller.remove_listener(self.listener)
            # Close OpenCV
            cv2.destroyAllWindows()
            # Turn off drone
            if DRONE:
                self.drone.emergency()
                self.drone.reset()
                self.drone.halt()


if __name__ == '__main__':
    print __doc__
    App().run()