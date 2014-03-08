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

import sys
sys.path.append('../libardrone/')
sys.path.append('../libleap/')

import numpy as np
import cv2

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
        # Get the most recent frame and report some basic information
        frame = controller.frame()

        if not frame.hands.is_empty:
            hand = frame.hands[0]
            normal = hand.palm_normal
            roll = normal.roll * Leap.RAD_TO_DEG

            if roll < -20:
                print 'Right'
            elif roll > 20:
                print 'Left'

        else:
            # Probably want to hover or something else neutral
            pass

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
    def __init__(self):
        cv2.namedWindow('camshift')
        self.vc = cv2.VideoCapture(0)
        cv2.setMouseCallback('camshift', self.onmouse)

        self.listener = LeapToQuadListener()
        self.controller = Leap.Controller()

        self.listener.parent = self

        # Have the listener receive events from the controller
        self.controller.add_listener(self.listener)

    def onmouse(self, event, x, y, flags, param):
        print 'Mouse'

    def run(self):
        try:
            while True:
                rval, vis = self.vc.read()
                cv2.imshow('camshift', vis)

                ch = cv2.waitKey(5)
                if ch == ord('q'):
                    break
                if ch == ord(' '):
                    print 'Spacebar'
        finally:
            del(self.vc)
            cv2.destroyAllWindows()
            # Remove the sample listener when done
            self.controller.remove_listener(self.listener)


if __name__ == '__main__':
    print __doc__
    App().run()