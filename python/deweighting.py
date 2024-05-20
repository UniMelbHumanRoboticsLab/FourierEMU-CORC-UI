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
from collections import deque

from realsense_camera import *
from FLNL import *
from isbulmodel.Deweighting import Deweighting

## Joints points names to IDs:
class J(IntEnum):
    L_Y = 2 #Left eye
    R_Y = 5 #Right eye
    L_H = 23 #Left Hip
    R_H = 24 #Right Hip
    L_S = 11 #Left Shoulder
    R_S = 12 #Right Shoulder
    L_E = 13 #Left Elbow
    R_E = 14 #Right Elbow
    L_W = 15 #Left Wrist
    R_W = 16 #Right Wrist


class PoseDetector:

    def __init__(self):
        self.mpDraw = mp.solutions.drawing_utils
        self.mpPose = mp.solutions.pose
        self.pose = self.mpPose.Pose(model_complexity=0, #Use lighter model: faster w/ sufficient tracking
                                     min_detection_confidence=0.5,
                                     min_tracking_confidence=0.5,
                                     smooth_landmarks=True #better result given mvt continuity required
                                     )

    def findPose(self, img, arm_side, objects_to_track, draw=False):
        self.results = self.pose.process(img)

        if draw:
            img.setflags(write=True)
            if arm_side=='r':
                POSE_CONNECTIONS = frozenset([(5, 2), (11, 12), (12, 14), (14, 16), (11, 23), (12, 24),(23,24)])
            else:
                POSE_CONNECTIONS = frozenset([(5, 2), (12, 11), (11, 13), (13, 15), (11, 23), (12, 24),(23,24)])
            if self.results.pose_landmarks:
                    self.mpDraw.draw_landmarks(img, self.results.pose_landmarks, POSE_CONNECTIONS)

        #Image positions of landmarks:
        jointsImgPos = {}
        if self.results.pose_landmarks:
            h, w, c = img.shape
            for joint_id in objects_to_track:
                joint=self.results.pose_landmarks.landmark[joint_id]
                cx, cy = int(joint.x * w), int(joint.y * h)
                if draw:
                    img = cv2.circle(img, (cx, cy), 8, (255, 0, 0))#, cv2.FILLED)
                if cx<w and cy<h:
                    jointsImgPos[joint_id] = [cx, cy, joint.z]
                else:
                    jointsImgPos[joint_id] = [np.NAN, np.NAN, np.NAN]
        if draw:
            cv2.imshow('Frame', img)

        return jointsImgPos


#Compute distance between two joints
def dist3D(p1, p2):
    return np.sqrt((p1[0]-p2[0])*(p1[0]-p2[0]) + (p1[1]-p2[1])*(p1[1]-p2[1]) + (p1[2]-p2[2])*(p1[2]-p2[2]))



##TODO:
    # - detect when joints occluded: continuous x/y but jump z
    # - Get MP estimated joint z value (depth)
    # - plot resulting 3d poses in // of other one. Plot jointangles in // of other ones and compare

    # ((- Convert it back to real unit by averaging camera depth of trunk
    # and scale by arm length or Shoulder-Shoulder))
    # - based on last 10 pts or so, for each joint, get an estimated MP.Z <=> realsense.Z ratio
    # - when not avaialble/incorrect, interpolate z value based on MP based estimate: change rate only w/ estimated ratio
    # OR
    # - at given pose, for joints w/ valid depth, compute MP.Z <=> realsense.Z ratio
    # - apply ratio for joint w/ not avaialble/incorrect depth


#Show a 3D view/split 2D views of skeleton
class skeletonView():
    def __init__(self, nb_time_plots=1):
        plt.interactive(True)
        self.fig = plt.figure()
        # Create 2x2 sub plots
        gs = gridspec.GridSpec(1+nb_time_plots, 2)
        self.ax=[]
        self.ax.append(plt.subplot(gs[0, 0], projection='3d'))
        self.ax[0].view_init(elev=0, azim=-90, roll=180) #Top view
        self.ax.append(plt.subplot(gs[0, 1], projection='3d'))
        self.ax[1].view_init(elev=0, azim=0, roll=-90) #Side view
        for ax in self.ax:
            ax.axes.set_xlim3d(left=-0.5, right=0.5)
            ax.axes.set_ylim3d(bottom=-1.0, top=1.0)
            ax.axes.set_zlim3d(bottom=0.5, top=1.5)
        self.lines = [-1, -1]
        self.prevPlotted = []
        for i in range(nb_time_plots):
            self.prevPlotted.append(-1)
            self.ax.append(plt.subplot(gs[1+i, :]))
        self.plotValues = [[] for i in range(nb_time_plots)]
        self.fig.tight_layout()


    def update(self, jointsPos, armSide='l'):
        if(armSide=='l'):
            xx=[x/1000. for x in [jointsPos[J.L_Y][0], jointsPos[J.R_Y][0], jointsPos[J.L_Y][0] + (jointsPos[J.R_Y][0]-jointsPos[J.L_Y][0])/2., jointsPos[J.L_S][0] + (jointsPos[J.R_S][0]-jointsPos[J.L_S][0])/2., jointsPos[J.R_S][0], jointsPos[J.L_S][0], jointsPos[J.L_E][0], jointsPos[J.L_W][0]]]
            yy=[x/1000. for x in [jointsPos[J.L_Y][1], jointsPos[J.R_Y][1], jointsPos[J.L_Y][1] + (jointsPos[J.R_Y][1]-jointsPos[J.L_Y][1])/2., jointsPos[J.L_S][1] + (jointsPos[J.R_S][1]-jointsPos[J.L_S][1])/2., jointsPos[J.R_S][1], jointsPos[J.L_S][1], jointsPos[J.L_E][1], jointsPos[J.L_W][1]]]
            zz=[x/1000. for x in [jointsPos[J.L_Y][2], jointsPos[J.R_Y][2], jointsPos[J.L_Y][2] + (jointsPos[J.R_Y][2]-jointsPos[J.L_Y][2])/2., jointsPos[J.L_S][2] + (jointsPos[J.R_S][2]-jointsPos[J.L_S][2])/2., jointsPos[J.R_S][2], jointsPos[J.L_S][2], jointsPos[J.L_E][2], jointsPos[J.L_W][2]]]
        else:
            xx=[x/1000. for x in [jointsPos[J.L_Y][0], jointsPos[J.R_Y][0], jointsPos[J.L_Y][0] + (jointsPos[J.R_Y][0]-jointsPos[J.L_Y][0])/2., jointsPos[J.L_S][0] + (jointsPos[J.R_S][0]-jointsPos[J.L_S][0])/2., jointsPos[J.L_S][0], jointsPos[J.R_S][0], jointsPos[J.R_E][0], jointsPos[J.R_W][0]]]
            yy=[x/1000. for x in [jointsPos[J.L_Y][1], jointsPos[J.R_Y][1], jointsPos[J.L_Y][1] + (jointsPos[J.R_Y][1]-jointsPos[J.L_Y][1])/2., jointsPos[J.L_S][1] + (jointsPos[J.R_S][1]-jointsPos[J.L_S][1])/2., jointsPos[J.L_S][1], jointsPos[J.R_S][1], jointsPos[J.R_E][1], jointsPos[J.R_W][1]]]
            zz=[x/1000. for x in [jointsPos[J.L_Y][2], jointsPos[J.R_Y][2], jointsPos[J.L_Y][2] + (jointsPos[J.R_Y][2]-jointsPos[J.L_Y][2])/2., jointsPos[J.L_S][2] + (jointsPos[J.R_S][2]-jointsPos[J.L_S][2])/2., jointsPos[J.L_S][2], jointsPos[J.R_S][2], jointsPos[J.R_E][2], jointsPos[J.R_W][2]]]

        if(self.lines[0]!=-1):
            for i in range(len(self.lines)):
                for l in self.lines[i]:
                    l.remove()
        for i in range(len(self.lines)):
            self.lines[i]=self.ax[i].plot3D(xx, yy, zz, linewidth=4)

        plt.draw()
        self.fig.canvas.flush_events()


    def updateTimePlot(self, n, vals, leg=[], max_val=100):
        mycolors=['r', 'g', 'b', 'c', 'm', 'y', 'k', 'r', 'g', 'b', 'c', 'm', 'y', 'k']
        self.plotValues[n].append(vals)
        if(self.prevPlotted[n]!=-1):
            for l in self.prevPlotted[n]:
                l.remove()
        self.prevPlotted[n]=self.ax[2+n].plot(self.plotValues[n], '-')
        for l,c in zip(self.prevPlotted[n], mycolors):
            l.set_color(c)
        self.ax[2+n].legend(leg)
        self.fig.canvas.flush_events()
        if(len(self.plotValues[n])>max_val):
            self.plotValues[n].pop(0)



#%% Arm IK functions from joints positions

#convenience function returning unit vector from a to b
def unit(from_a, to_b):
    return (to_b-from_a)/np.linalg.norm(to_b-from_a)

#convenience function returning unit vector of vec
def unitV(vec):
    return vec/np.linalg.norm(vec)


def FramesToq(xs, ys, zs, xua, yua, zua, zm, yfa):
    """Performed ISB IK from specific shoulder and UA and forearm frame/vectors
    and return joint angles. Outcome joint angles same as ISB_UL arm model (and
    so also the OpenSIM MoBL-ARMS)

    ## Trunk/Shoulder frame:
    # Zs: contralateral to shoulder
    # Ys: orthogonal, through Eyes point (=> ~upward)
    # Xs: resulting, forward

    ## UA (humerus) frame:
    # Yua: along humerus, up: elbow->shoulder
    # Zua: external, out of shoulder-elbow-wrist plane
    # Xua: front forward (i.e. in arm plane)
    # Zm: modified by first rotation: projection of Yua in horizontal shoulder plane (formed by Xs/Zs, with Ys as normal)

    ## Forearm vector
    # Yfa: along radius/cubitus towards wrist
    """

    ## Elbow angle: 0 fully extended, +180 fully flexed
    q_elb = np.pi-np.arccos(np.dot(yua,yfa))

    ## Shoulder angles (almost ISB, different ref and range. ISB sucks):
    q_ele = np.arccos(np.dot(yua,ys)) #Elevation: around Ztransformed, 0 along body, +90 full extended forward (at 0 pel)

    epsilon=np.pi/50 #~<4degrees
    if(q_ele>epsilon):
        q_pel = np.arccos(np.dot(-zs, zm)) #Planele: around Ytrunk: 0 full external, +90 elbow forward, 180 full internal. Computed as angle between Zs(external) and projection of Yua in horizontal shoulder plane (formed by Xs/Zs, with Ys as normal)
    else:
        q_pel = np.NAN #q_pel undefined at 0 elevation

    # Undefined if q_pel is undefined
    if(q_ele>epsilon):
        zm = unitV(zm-np.dot(zm, yua)*yua) #first project zm into Xua/Zua plane
        q_rot = -np.arccos(np.dot(zm, zua))+np.pi/2 #Int/ext rotation: around Ytransformed, angle in plane of normal Yua, between Zua and Zs rotated by pel (aka Zm)
    else:
        #q_rot purely defined based on shoulder-elbow-wrist plane when at 0 elevation: i.e. q_pele is 0
        if(q_elb>epsilon):
            q_rot = np.arccos(np.dot(zs, zua))
        else:
            q_rot = np.NAN #Trully undefined when arm in full extension

    return  q_pel, q_ele, q_rot, q_elb


def IK(jointsPos, arm_side, true_vertical=np.array([]), debug=False):
    """Compute ISB joint angles from MediaPipe joints positions (in mm)
    making some assumptions on true vertical for trunk pose if desired."""

    # Which arm?
    if(arm_side=='l'):
        contra_shoulder=J.R_S
        shoulder=J.L_S
        elbow=J.L_E
        wrist=J.L_W
    else:
        contra_shoulder=J.L_S
        shoulder=J.R_S
        elbow=J.R_E
        wrist=J.R_W

    # Contralateral Shoulder position arrays list
    j=contra_shoulder
    CShoulder=np.array([jointsPos[j][0]/1000., jointsPos[j][1]/1000., jointsPos[j][2]/1000.]).transpose()

    # Shoulder position arrays list (convert to meter)
    j=shoulder
    Shoulder=np.array([jointsPos[j][0]/1000., jointsPos[j][1]/1000., jointsPos[j][2]/1000.]).transpose()

    # Elbow position arrays list
    j=elbow
    Elbow=np.array([jointsPos[j][0]/1000., jointsPos[j][1]/1000., jointsPos[j][2]/1000.]).transpose()

    # Wrist position arrays list
    j=wrist
    Wrist=np.array([jointsPos[j][0]/1000., jointsPos[j][1]/1000., jointsPos[j][2]/1000.]).transpose()

    # Center eyes arrays list
    Eyes=np.array([(jointsPos[J.L_Y][0] + (jointsPos[J.R_Y][0]-jointsPos[J.L_Y][0])/2.)/1000.,
                    (jointsPos[J.L_Y][1] + (jointsPos[J.R_Y][1]-jointsPos[J.L_Y][1])/2.)/1000.,
                    (jointsPos[J.L_Y][2] + (jointsPos[J.R_Y][2]-jointsPos[J.L_Y][2])/2.)/1000.]).transpose()

    ## Trunk frame:
    if(len(true_vertical)==3):
        ## Trunk frame using true vertical:
        # Z: contralateral to shoulder in tranverse plane only (no Y)
        # Y: true vertical: -Y in camera space
        # X: resulting, forward
        zs = unit(CShoulder, Shoulder)
        ys = unitV(true_vertical) #ys is true vertical
        zs = unitV(zs-np.dot(zs,ys)*ys) #no zs component on true vertical
        if(arm_side=='r'):
            xs = np.cross(ys, zs)
        else:
            xs = np.cross(zs, ys) # ISB sucks and doesn't use right hand coordinates on left side
    else:
        # Z: contralateral to shoulder in tranverse
        # Y: through eyes center
        # X: resulting, forward
        zs = unit(CShoulder, Shoulder)
        if(arm_side=='r'):
            xs = unitV(np.cross(unit(Shoulder, Eyes), zs))
            ys = unitV(np.cross(zs, xs))
        else:
            xs = unitV(np.cross(zs, unit(Shoulder, Eyes))) # ISB sucks and doesn't use right hand coordinates on left side
            ys = unitV(np.cross(zs, -xs))

    ## Forearm vector
    # Yfa: along radius/cubitus towards wrist
    yfa = unit(Elbow, Wrist)

    ## UA (humerus) frame:
    # Y: along humerus, up: elbow->shoulder
    # Z: external, out of shoulder-elbow-wrist plane
    # X: forward in elbow-wrist-shoulder plane
    # Zm: transformed by first rotation: projection of yua into Xs/Zs plane
    yua = unit(Elbow, Shoulder)
    if(arm_side=='r'):
        zua = unitV(np.cross(yfa, yua))
        xua = unitV(np.cross(yua, zua))
        zm = unitV(yua-np.dot(yua, ys)*ys)
    else:
        zua = unitV(np.cross(yua, yfa))
        xua = unitV(np.cross(zua, yua)) # ISB sucks and doesn't use right hand coordinates on left side
        zm = unitV(yua-np.dot(yua, ys)*ys)

    if(debug):
        ax = plt.subplots(1,1)
        from spatialmath import SO3, SE3, Plane3
        Ts=SE3([0, 0, 0])*SE3(SO3(np.array([xs, ys, zs]).transpose()))
        ax=Ts.plot(frame='S')
        Tua=SE3([0, 0, 0])*SE3(SO3(np.array([xua, yua, zua]).transpose()))
        Tua.plot(frame='UA', color='green', length=0.5, ax=ax)
        Tm=SE3(Elbow)*SE3(SO3(np.array([unitV(np.cross(ys, zm)), ys, zm]).transpose()))
        Tm.plot(frame='m', color='black', length=0.5, ax=ax)
        Tw=SE3(Wrist)*SE3(SO3(np.array([unitV(np.cross(yfa, zua)), yfa, zua]).transpose()))
        Tw.plot(frame='w', color='black', length=0.5)
        plt.show()

    ## Compute ISB joint angles from frames
    q_pel, q_ele, q_rot, q_elb = FramesToq(xs, ys, zs, xua, yua, zua, zm, yfa)

    return [q_pel, q_ele, q_rot, q_elb]


#%%

#Hold previous times of signals value (i.e. circular buffer) and apply various
#filtering or computation on it
class Signal():
    def __init__(self, first_element, t_first, memory_length):
        self.d = deque([first_element], maxlen=memory_length)
        self.t = deque([t_first], maxlen=memory_length)
        self.dt = -1

    def update(self, element, t):
        self.d.append(element)
        self.t.append(t)
        return self.d

    def setPeriod(self, dt):
        self.dt = dt

    def predictTwoSteps(self, at_t):
        if(len(self.d)>2):
            return np.array(self.d[-1]) + (np.array(self.d[-1])-np.array(self.d[-3]))/(self.t[-1]-self.t[-3])*(at_t-self.t[-1])
        else:
            return self.d[-1]

    def predictOneStep(self, at_t):
        if(len(self.d)>1):
            return np.array(self.d[-1]) + (np.array(self.d[-1])-np.array(self.d[-2]))/(self.t[-1]-self.t[-2])*(at_t-self.t[-1])
        else:
            return self.d[-1]


##TODO: to include FLNL client for communication with CORC app and manage logic for deweighting

def main():
    #Process parameters
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
            video=cv2.VideoWriter(datetime.now().strftime("../%d_%m_%Y-%H_%M_%S")+'.mp4', cv2.VideoWriter_fourcc(*'MP4V'), 25, (640,480))
            print("Recording ON")
        else:
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

    #Arm side to track
    armSide='l';
    # Left eye, Right eye, Left Hip, Right Hip, Left Shoulder, Right Shoulder, Left Elbow, Left Wrist
    objects_to_track = [J.L_Y, J.R_Y, J.L_H, J.R_H, J.L_S, J.R_S, J.L_E, J.L_W]

    ## Check camera acq
    #Try to open and create rs camera
    rs = RealsenseCamera(recording)
    if(not rs.init):
        #Fallback to webcam capture
        cap = cv2.VideoCapture(0)
        print("Use default webcam.")
    else:
        print("Use RealSense.")

    ### Init detection processes: arms poses and Tag
    #tag = AprilTag()

    ## Start FLNL and wait for connection
    server = FLNLServer()
    if(noClientTesting):
        streaming=False
        #streaming=True
        #if(rs.init and recording):
        #    rs.recorder.resume()
    else:
        streaming=False
        server.WaitForClient() #on default address 127.0.0.1 and port 2042
        print("FLNL connected")


    ## Init pose detector
    detector = PoseDetector()


    # Define an arm model
    arm_model_params_d = {'ua_l': 0.3,
                          'fa_l': 0.25,
                          'ha_l': 0.1,
                          'm_ua': 2.0,
                          'm_fa': 1.1+0.23+0.6}
    ISB_UL = EMUDeweighting(arm_model_params_d)

    if(drawing):
        view3d = skeletonView(2)

    depthS = Signal(np.array([0 for j in objects_to_track]), 0, 3)

    cTime = 0
    pTime = 0
    fps = 0
    while(server.IsConnected() or noClientTesting):

        ## When connected, continue detection
        #Use realsense camera:
        if(rs.init):
            # Capture Camera Frame
            ret, color_image, depth_image = rs.get_frame_stream()
            # copy
            frame = color_image.copy()

            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        else:
            # Webcam otherwise
            sucess, frame = cap.read()
        frame.flags.writeable = False

        #iTime0 = time.time()
        #print('Int. time 0:', (iTime0 - cTime))

        ## Get landmarks image position
        jointsPosImg = detector.findPose(frame, armSide, objects_to_track, drawing)

        #iTime1 = time.time()
        #print('Int. time 1:', (iTime1 - iTime0))

        ## Convert to 3D
        if(rs.init and len(jointsPosImg)>0):
            jointsPos = rs.imgTo3D(jointsPosImg, depth_image)
            #Joint angles
            print(jointsPos)
            q=IK(jointsPos, armSide)
            #qMP=IK(jointsPosImg, armSide)

            if(drawing):
                view3d.update(jointsPos, armSide)
                view3d.updateTimePlot(0, np.array(q)*180/np.pi, ['Planeele', 'Ele', 'Rot', 'Elbow'])
                #view3d.updateTimePlot(1, np.array(qMP)*180/np.pi)
                depth=[jointsPos[j][2]  for j in objects_to_track]
                depthMP=[jointsPosImg[j][2] for j in objects_to_track]
                pred_de=depthS.predictOneStep(cTime)
                #print(pred_de, "= ? =", depth, '\n')
                depthS.update(depth, cTime)
                #view3d.updateTimePlot(1, np.array(depth))
                #view3d.updateTimePlot(2, np.array([depth[-1]/1000., depthMP[-1]]))
                depthMP=[jointsPosImg[j][2] for j in objects_to_track]
                #view3d.updateTimePlot(2, np.array(depthMP))
                print((np.array(q)*180/np.pi).round())

                #ISB_UL.ISB_UL.plot(q+[0,0,0], backend='pyplot', block=False)


        #iTime2 = time.time()
        #print('Int. time 2:', (iTime2 - iTime1))


        ## Send joint positions when valid values are available
        if(streaming):
            if(recording):
                video.write(color_image)

            if len(jointsPosImg)>0:
                ## Build values to send: ArmSide (0:l,1:r), joints img pos, 3d img pos
                val = []
                if armSide=='l':
                    val.append(0.0)
                else:
                    val.append(1.0)
                # #First image positions
                # for id in objects_to_track:
                    # val.append(jointsPosImg[id][0])
                    # val.append(jointsPosImg[id][1])
                #Then 3d positions
                for id in objects_to_track:
                    val.append(jointsPos[id][0])
                    val.append(jointsPos[id][1])
                    val.append(jointsPos[id][2])
                if(noClientTesting):
                    np.set_printoptions(precision=2)
                    np.set_printoptions(suppress=True)
                    print(fps, 'fps')
                    #Distances
                    #print(str(dist3D(jointsPos[J.R_S],jointsPos[J.L_S])/1000)+', '+str(dist3D(jointsPos[J.L_S],jointsPos[J.L_E])/1000)+', '+str(dist3D(jointsPos[J.L_E],jointsPos[J.L_W])/1000))
                    #Points
                    #for id in objects_to_track:
                    #    print(jointsPos[id][0], jointsPos[id][1], jointsPos[id][2], sep=',', end=',')
                    #print('')
                    #Wrist position only
                    #print(jointsPos[J.L_W][0], jointsPos[J.L_W][1], jointsPos[J.L_W][2], sep='\t')
                else:
                    server.SendValues(val)
                    print(fps)

        #iTime3 = time.time()
        #print('Int. time 3:', (iTime3 - iTime2))

        ##Process incoming commands if we are connected
        if(not noClientTesting):
            ## Start streaming?
            if(server.IsCmd("STA")):
                streaming=True
                print("Start streaming values")
                if(rs.init and recording):
                    rs.recorder.resume()
                    print("Start recording stream")

            ## Stop streaming?
            if(server.IsCmd("STO")):
                streaming=False
                print("Stop streaming values")
                if(rs.init and recording):
                    rs.recorder.pause()
                    print("Stop recording stream")

            ## Set to left arm
            if(server.IsCmd("ARL")):
                armSide='l'
                # Left eye, Right eye, Left Hip, Right Hip, Left Shoulder, Right Shoulder, Left Elbow, Left Wrist
                objects_to_track = [J.L_Y, J.R_Y, J.L_H, J.R_H, J.L_S, J.R_S, J.L_E, J.L_W]
                print("Tracking LEFT arm")

            ## Set to right arm
            if(server.IsCmd("ARR")):
                armSide='r'
                # Left eye, Right eye, Left Hip, Right Hip, Left Shoulder, Right Shoulder, Right Elbow, Right Wrist
                objects_to_track = [J.L_Y, J.R_Y, J.L_H, J.R_H, J.L_S, J.R_S, J.R_E, J.R_W]
                print("Tracking RIGHT arm")

            ## Disconnect command
            if(server.IsCmd("DIS")):
                break

        else:## For testing when not connected
            k = cv2.waitKey(1)
            ## Start streaming?
            if(k==ord('t')) or keyboard.is_pressed('t'):
                streaming=True
                print("Start streaming values")
                if(rs.init and recording):
                    rs.recorder.resume()
                    print("Start recording stream")

            ## Stop streaming?
            if(k==ord('s') or keyboard.is_pressed('s')):
                streaming=False
                print("Stop streaming values")
                if(rs.init and recording):
                    rs.recorder.pause()
                    print("Stop recording stream")

            ## Set to left arm
            if(k==ord('l') or keyboard.is_pressed('l')):
                armSide='l'
                # Left eye, Right eye, Left Hip, Right Hip, Left Shoulder, Right Shoulder, Left Elbow, Left Wrist
                objects_to_track = [J.L_Y, J.R_Y, J.L_H, J.R_H, J.L_S, J.R_S, J.L_E, J.L_W]
                print("Tracking LEFT arm")

            ## Set to right arm
            if(k==ord('r') or keyboard.is_pressed('r')):
                armSide='r'
                # Left eye, Right eye, Left Hip, Right Hip, Left Shoulder, Right Shoulder, Right Elbow, Right Wrist
                objects_to_track = [J.L_Y, J.R_Y, J.L_H, J.R_H, J.L_S, J.R_S, J.R_E, J.R_W]
                print("Tracking RIGHT arm")

            #Turn drawing on/off
            if(k==ord('d')) or keyboard.is_pressed('d'):
                drawing = not drawing

        ## Can also close with 'q'
        if drawing:
            k = cv2.waitKey(1)
            if k == ord('q') or keyboard.is_pressed('q'):
                break

        ## Calculate FPS
        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime

    ## Wait for disconnection
    print("Disconnected. Exiting...")

    ## Exit
    streaming=False
    if(not noClientTesting):
        if(server.IsConnected()):
            server.Close()
    if(rs.init):
        if(recording):
            rs.recorder.pause()
            video.release()
        rs.release()
    else:
        cap.release()

    # Clean up
    cv2.destroyAllWindows()
    plt.close('all')


if __name__ == '__main__':
    main()

