# -*- coding: utf-8 -*-
"""
Created on 10/04/2024

Functions to use an IntelRealSense and MediaPipe to detect skeleton,
compute arm pose and associated deweighting force for EMU device and
various visualisations functions.

@author: vcrocher - Unimelb
"""

import time
import logging
import os, sys, subprocess, csv, math
import datetime
import cv2
import mediapipe as mp
import pyrealsense2 as rs2
#import pupil_apriltags as apriltag
import keyboard
from enum import IntEnum
from numpy.linalg import norm, inv
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import warnings
warnings.filterwarnings('ignore') # to remove deprecation warning

from realsense_camera import *
from FLNL import *
from threading import Thread
from visual_lib_deweight import *
# from isbulmodel.Deweighting import Deweighting


#TODO: to include FLNL client for communication with CORC app and manage logic for deweighting

def main():
    #Process parameters
    sys.argv = ["0","1","1","1"]
    if len(sys.argv)>1:
        if sys.argv[1] == "1":
            drawing=True
            print("Visualisation ON")
        else:
            drawing=False
            print("Visualisation OFF")
    else:
        drawing=False
        print("Visualisation OFF")

    recording=False
    if len(sys.argv)>2:
        if sys.argv[2] == "1":
            recording=True
            video=cv2.VideoWriter(datetime.now().strftime("../%d_%m_%Y-%H_%M_%S")+'.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 25, (640,360))
            print(video.isOpened())
            print("Recording ON")
        else:
            video = False
            recording=False
            print("Recording OFF")
    else:
        recording=False
        print("Recording OFF")

    noClientTesting=False
    if len(sys.argv)>3:
        if sys.argv[3] == "1":
            noClientTesting=True
            print("TESTING NO FLNL ON")
        else:
            noClientTesting=False
            print("Testing no FLNL OFF")
    else:
        noClientTesting=False
        print("Testing no FLNL OFF")

    ## Check camera acq
    #Try to open and create rs camera
    rs = RealsenseCamera(recording)
    if(not rs.init):
        #Fallback to webcam capture
        cap = cv2.VideoCapture(0)
        print("Use default webcam.")
    else:
        cap = 0
        print("Use RealSense.")

    ### Init detection processes: arms poses and Tag
    #tag = AprilTag()

    ## Start FLNL and wait for connection
    server = FLNLServer()
    if(noClientTesting):
        # streaming=False
        streaming=True
        #if(rs.init and recording):
        #    rs.recorder.resume()
    else:
        streaming=False
        server.WaitForClient() #on default address 127.0.0.1 and port 2042
        print("FLNL connected")

    # Define an arm model
    arm_model_params_d = {'ua_l': 0.3,
                          'fa_l': 0.25,
                          'ha_l': 0.1,
                          'm_ua': 2.0,
                          'm_fa': 1.1+0.23+0.6}
    # ISB_UL = EMUDeweighting(arm_model_params_d)

    if(server.IsConnected() or noClientTesting):
        app = QApplication(sys.argv)
        a = DeweightApp([rs,cap],[drawing,recording,noClientTesting,streaming],server,video)
        a.show()
        sys.exit(app.exec())

if __name__ == '__main__':
    main()

