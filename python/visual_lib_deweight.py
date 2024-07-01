import numpy as np
import time
import serial
import serial.tools.list_ports
from FLNL import * 
import sys
import signal
from time import sleep
import math

from enum import IntEnum
import mediapipe as mp
import cv2
import matplotlib.gridspec as gridspec
from collections import deque
import matplotlib.pyplot as plt
import keyboard

from PyQt6 import QtGui,QtWidgets
from PyQt6.QtGui import QPixmap,QImage
from PyQt6.QtWidgets import QApplication,QWidget,QLabel,QVBoxLayout,QHBoxLayout
from PyQt6.QtCore import QThread,pyqtSignal,pyqtSlot,Qt 
import pyqtgraph.opengl as gl
import pyqtgraph as pg
from pglive.kwargs import Axis
from pglive.sources.data_connector import DataConnector
from pglive.sources.live_axis import LiveAxis
from pglive.sources.live_axis_range import LiveAxisRange
from pglive.sources.live_plot import LiveLinePlot
from pglive.sources.live_plot_widget import LivePlotWidget
# from isbulmodel.Deweighting import Deweighting

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

#%% to find pose and plot pose in 3D
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

        # Image positions of landmarks:
        jointsImgPos = {}
        if self.results.pose_landmarks:
            h, w, c = img.shape
            for joint_id in objects_to_track:
                joint=self.results.pose_landmarks.landmark[joint_id]
                cx, cy = int(joint.x * w), int(joint.y * h)
                # if draw:
                #     img = cv2.circle(img, (cx, cy), 8, (255, 0, 0))#, cv2.FILLED)
                if cx<w and cy<h:
                    jointsImgPos[joint_id] = [cx, cy, joint.z]
                else:
                    jointsImgPos[joint_id] = [np.NAN, np.NAN, np.NAN]

        return jointsImgPos,img
    
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

    def skeleton_lines(self, jointsPos, armSide='l'):
        if(armSide=='l'):
            xx=[x/1000. for x in [jointsPos[J.L_Y][0], jointsPos[J.R_Y][0], jointsPos[J.L_Y][0] + (jointsPos[J.R_Y][0]-jointsPos[J.L_Y][0])/2., jointsPos[J.L_S][0] + (jointsPos[J.R_S][0]-jointsPos[J.L_S][0])/2., jointsPos[J.R_S][0], jointsPos[J.L_S][0], jointsPos[J.L_E][0], jointsPos[J.L_W][0]]]
            yy=[x/1000. for x in [jointsPos[J.L_Y][1], jointsPos[J.R_Y][1], jointsPos[J.L_Y][1] + (jointsPos[J.R_Y][1]-jointsPos[J.L_Y][1])/2., jointsPos[J.L_S][1] + (jointsPos[J.R_S][1]-jointsPos[J.L_S][1])/2., jointsPos[J.R_S][1], jointsPos[J.L_S][1], jointsPos[J.L_E][1], jointsPos[J.L_W][1]]]
            zz=[x/1000. for x in [jointsPos[J.L_Y][2], jointsPos[J.R_Y][2], jointsPos[J.L_Y][2] + (jointsPos[J.R_Y][2]-jointsPos[J.L_Y][2])/2., jointsPos[J.L_S][2] + (jointsPos[J.R_S][2]-jointsPos[J.L_S][2])/2., jointsPos[J.R_S][2], jointsPos[J.L_S][2], jointsPos[J.L_E][2], jointsPos[J.L_W][2]]]
        else:
            xx=[x/1000. for x in [jointsPos[J.L_Y][0], jointsPos[J.R_Y][0], jointsPos[J.L_Y][0] + (jointsPos[J.R_Y][0]-jointsPos[J.L_Y][0])/2., jointsPos[J.L_S][0] + (jointsPos[J.R_S][0]-jointsPos[J.L_S][0])/2., jointsPos[J.L_S][0], jointsPos[J.R_S][0], jointsPos[J.R_E][0], jointsPos[J.R_W][0]]]
            yy=[x/1000. for x in [jointsPos[J.L_Y][1], jointsPos[J.R_Y][1], jointsPos[J.L_Y][1] + (jointsPos[J.R_Y][1]-jointsPos[J.L_Y][1])/2., jointsPos[J.L_S][1] + (jointsPos[J.R_S][1]-jointsPos[J.L_S][1])/2., jointsPos[J.L_S][1], jointsPos[J.R_S][1], jointsPos[J.R_E][1], jointsPos[J.R_W][1]]]
            zz=[x/1000. for x in [jointsPos[J.L_Y][2], jointsPos[J.R_Y][2], jointsPos[J.L_Y][2] + (jointsPos[J.R_Y][2]-jointsPos[J.L_Y][2])/2., jointsPos[J.L_S][2] + (jointsPos[J.R_S][2]-jointsPos[J.L_S][2])/2., jointsPos[J.L_S][2], jointsPos[J.R_S][2], jointsPos[J.R_E][2], jointsPos[J.R_W][2]]]

        xx = np.array(xx)
        yy = np.array(yy)
        zz = np.array(zz)

        return np.transpose(np.vstack((xx,yy,zz)))


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

#%% for signal filtering
# Hold previous times of signals value (i.e. circular buffer) and apply various
# filtering or computation on it
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

#%% for Deweighting GUI
class DeweightThread(QThread):
    ImageUpdate = pyqtSignal(QImage)
    SkeletonUpdate = pyqtSignal(np.ndarray)
    def __init__(self,cameras,flags,data_connectors,server,video):
        super().__init__()
        self._run_flag = True
        self.cameras = cameras
        self.flags = flags
        self.data_connectors = data_connectors
        self.server = server
        self.video = video
        
    def run(self):

        #Arm side to track
        armSide='l'
        # Left eye, Right eye, Left Hip, Right Hip, Left Shoulder, Right Shoulder, Left Elbow, Left Wrist
        objects_to_track = [J.L_Y, J.R_Y, J.L_H, J.R_H, J.L_S, J.R_S, J.L_E, J.L_W]

        pTime = 0
        fps = 0
        final_fps = 30
        detector = PoseDetector()
        depthS = Signal(np.array([0 for j in objects_to_track]), 0, 3)
        while self._run_flag:
            fps = fps + 1
            cTime = time.time()

            if (cTime-pTime > 1.0):
                final_fps = fps
                fps = 0
                pTime = cTime

            """
            Run Pose Estimation
            """
            if(self.cameras[0].init):
                # Capture Camera Frame
                ret, color_image, depth_image = self.cameras[0].get_frame_stream()
                # copy
                frame = color_image.copy()
                color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            else:
                # Webcam otherwise
                success, frame = self.cameras[1].read()
                color_image = frame
            # frame is read-only
            frame.flags.writeable = False
            ## Get landmarks image position
            jointsPosImg,self.img = detector.findPose(frame, armSide, objects_to_track, self.flags[0])
            self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB)
            self.img = cv2.putText(self.img,f"{final_fps}",(50,50),cv2.FONT_HERSHEY_COMPLEX,1,(255,0,0),2,cv2.LINE_AA)
            self.img = QImage(self.img.data, self.img.shape[1], self.img.shape[0], QImage.Format.Format_RGB888)
            self.img = self.img.scaled(640, 360, Qt.AspectRatioMode.KeepAspectRatio)
            self.ImageUpdate.emit(self.img)

            ## Convert to 3D and do real time plotting
            if(self.cameras[0].init and len(jointsPosImg)>0):
                jointsPos = self.cameras[0].imgTo3D(jointsPosImg, depth_image)
                q=IK(jointsPos, armSide)

                if(self.flags[0]):
                    # plot for 
                    if np.sum(np.isnan(q)) < 1: # only attach "valid" joint angles
                        timestamp = time.time()
                        for index, data_connector in enumerate(self.data_connectors):
                            # print(data_connector)
                            data_connector.cb_append_data_point(q[index]*180/np.pi, timestamp)
                        # Update skeleton view
                        self.SkeletonUpdate.emit(detector.skeleton_lines(jointsPos,armSide=armSide))

                        # perform signal filtering here
                        depth=[jointsPos[j][2]  for j in objects_to_track]
                        depthMP=[jointsPosImg[j][2] for j in objects_to_track]
                        pred_de=depthS.predictOneStep(cTime)
                        depthS.update(depth, cTime)
                        depthMP=[jointsPosImg[j][2] for j in objects_to_track]

            """
            Run Streaming to FLNL or Serial
            """
            if(self.flags[3]):
                if(self.flags[1]):
                    self.video.write(color_image)

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
                    if(self.flags[2]):
                        np.set_printoptions(precision=2)
                        np.set_printoptions(suppress=True)
                        print("Streaming",final_fps)
                        #Distances
                        #print(str(dist3D(jointsPos[J.R_S],jointsPos[J.L_S])/1000)+', '+str(dist3D(jointsPos[J.L_S],jointsPos[J.L_E])/1000)+', '+str(dist3D(jointsPos[J.L_E],jointsPos[J.L_W])/1000))
                        #Points
                        #for id in objects_to_track:
                        #    print(jointsPos[id][0], jointsPos[id][1], jointsPos[id][2], sep=',', end=',')
                        #print('')
                        #Wrist position only
                        #print(jointsPos[J.L_W][0], jointsPos[J.L_W][1], jointsPos[J.L_W][2], sep='\t')
                    else:
                        self.server.SendValues(val)
                        print("Sending",final_fps)


            """
            Process Incoming Commands
            """
            if(not self.flags[2]):
                """
                connected
                """
                ## Start streaming?
                if(self.server.IsCmd("STA")):
                    self.flags[3]=True
                    print("Start streaming values")
                    if(self.cameras[0] and self.flags[1]):
                        self.cameras[0].recorder.resume()
                        print("Start recording stream")

                ## Stop streaming?
                if(self.server.IsCmd("STO")):
                    self.flags[3]=False
                    print("Stop streaming values")
                    if(self.cameras[0].init and self.flags[1]):
                        self.cameras[0].recorder.pause()
                        print("Stop recording stream")

                ## Set to left arm
                if(self.server.IsCmd("ARL")):
                    armSide='l'
                    # Left eye, Right eye, Left Hip, Right Hip, Left Shoulder, Right Shoulder, Left Elbow, Left Wrist
                    objects_to_track = [J.L_Y, J.R_Y, J.L_H, J.R_H, J.L_S, J.R_S, J.L_E, J.L_W]
                    print("Tracking LEFT arm")

                ## Set to right arm
                if(self.server.IsCmd("ARR")):
                    armSide='r'
                    # Left eye, Right eye, Left Hip, Right Hip, Left Shoulder, Right Shoulder, Right Elbow, Right Wrist
                    objects_to_track = [J.L_Y, J.R_Y, J.L_H, J.R_H, J.L_S, J.R_S, J.R_E, J.R_W]
                    print("Tracking RIGHT arm")
                ## Disconnect command
                if(self.server.IsCmd("DIS")):
                    self._run_flag = False
            else:
                """
                Process Incoming Commands if not connected
                """
                ## For testing when not connected
                ## Start streaming? 
                if keyboard.is_pressed('t'):
                    self.flags[3]=True
                    print("Start streaming values")
                    if(self.cameras[0] and self.flags[1]):
                        self.cameras[0].recorder.resume()
                        print("Start recording stream")

                ## Stop streaming?
                if keyboard.is_pressed('s'):
                    self.flags[3]=False
                    print("Stop streaming values")
                    if(self.cameras[0].init and self.flags[1]):
                        self.cameras[0].recorder.pause()
                        print("Stop recording stream")

                ## Set to left arm
                if keyboard.is_pressed('l'):
                    armSide='l'
                    # Left eye, Right eye, Left Hip, Right Hip, Left Shoulder, Right Shoulder, Left Elbow, Left Wrist
                    objects_to_track = [J.L_Y, J.R_Y, J.L_H, J.R_H, J.L_S, J.R_S, J.L_E, J.L_W]
                    print("Tracking LEFT arm")

                ## Set to right arm
                if keyboard.is_pressed('r'):
                    armSide='r'
                    # Left eye, Right eye, Left Hip, Right Hip, Left Shoulder, Right Shoulder, Right Elbow, Right Wrist
                    objects_to_track = [J.L_Y, J.R_Y, J.L_H, J.R_H, J.L_S, J.R_S, J.R_E, J.R_W]
                    print("Tracking RIGHT arm")

                #Turn drawing on/off
                if keyboard.is_pressed('d'):
                    self.flags[0] = not self.flags[0]

            """
            Close
            """
            ## Can also close with 'q'
            if self.flags[0]:
                if  keyboard.is_pressed('q'):
                    # shut down capture system
                    self._run_flag = False
    def stop(self):
        """Sets run flag to False and waits for thread to finish"""
        self._run_flag = False
        ## Wait for disconnection
        print("Disconnected. Exiting...")

        ## Exit
        self.flags[3]=False
        if(not self.flags[2]):
            if(self.server.IsConnected()):
                self.server.Close()
        if(self.cameras[0].init):
            if(self.flags[1]):
                self.cameras[0].recorder.pause()
                self.video.release()
            self.cameras[0].release()
        else:
            self.cameras[1].release()
        return 0


class DeweightApp(QWidget):
    def __init__(self,cameras,flags,server,video):
        super().__init__()
        self.setWindowTitle("Deweighting GUI")
        DWGUI = QtWidgets.QHBoxLayout()

        """
        UA kinematic Widget
        """
        joint_angle_widget = pg.LayoutWidget()
        self.data_connectors = []
        poe_widget = LivePlotWidget(title=f"UA Plane of Elevation",
                                x_range_controller=LiveAxisRange(roll_on_tick=1000),
                                # y_range_controller=LiveAxisRange(fixed_range=[-1, 370])
                                )
        poe = LiveLinePlot(pen="yellow")
        poe_widget.addItem(poe)
        self.data_connectors.append(DataConnector(poe, max_points=1000))
        joint_angle_widget.addWidget(poe_widget, row=0, col=0,rowspan=1,colspan=1)

        elevation_widget = LivePlotWidget(title=f"Elevation",
                                x_range_controller=LiveAxisRange(roll_on_tick=1000),
                                y_range_controller=LiveAxisRange(fixed_range=[-100, 100]))
        elevation = LiveLinePlot(pen="yellow")
        elevation_widget.addItem(elevation)
        self.data_connectors.append(DataConnector(elevation, max_points=1000))
        joint_angle_widget.addWidget(elevation_widget, row=1, col=0,rowspan=1,colspan=1)

        ie_rot_widget = LivePlotWidget(title="I/E Rotation",
                                            x_range_controller=LiveAxisRange(roll_on_tick=1000),
                                            y_range_controller=LiveAxisRange(fixed_range=[-190, 190]))
        ie_rot = LiveLinePlot(pen="yellow")
        ie_rot_widget.addItem(ie_rot)
        self.data_connectors.append(DataConnector(ie_rot, max_points=1000))
        joint_angle_widget.addWidget(ie_rot_widget, row=2, col=0,rowspan=1,colspan=1)

        ext_flex_left_axis = LiveAxis("left", axisPen="red", textPen="red")
        ext_flex_bottom_axis = LiveAxis("bottom", axisPen="green", textPen="green", **{Axis.TICK_FORMAT: Axis.TIME})
        ext_flex_widget = LivePlotWidget(title="Ext/Flex",
                                            axisItems={'bottom': ext_flex_bottom_axis, 'left': ext_flex_left_axis},
                                            x_range_controller=LiveAxisRange(roll_on_tick=1000),
                                            y_range_controller=LiveAxisRange(fixed_range=[-190, 190]))
        ext_flex = LiveLinePlot(pen="yellow")
        ext_flex_widget.addItem(ext_flex)
        self.data_connectors.append(DataConnector(ext_flex, max_points=1000))
        joint_angle_widget.addWidget(ext_flex_widget, row=3, col=0,rowspan=1,colspan=1)
        joint_angle_widget.setFixedSize(640,720)

        """
        RealSense Video Widget
        """
        self.disply_width = 640
        self.display_height = 360
        self.CameraFeed = QLabel('label1')
        self.CameraFeed.resize(self.disply_width, self.display_height)

        """
        skeleton_top Widget
        """
        skeleton_widget = QtWidgets.QVBoxLayout()

        self.skeleton_top_view = gl.GLViewWidget()
        self.skeleton_top_view.setCameraPosition(rotation= QtGui.QQuaternion().fromEulerAngles(0,180,-90))
        self.skeleton_top_view.opts['elevation'] = 0
        grid = gl.GLGridItem(size = QtGui.QVector3D(5,5,5), color=(255, 255, 255, 76.5))
        self.skeleton_top = gl.GLLinePlotItem(pos=np.zeros((8, 3)),width=2,antialias=False)
        self.skeleton_top_view.addItem(self.skeleton_top)
        self.skeleton_top_view.setFixedSize(320,320)
        self.skeleton_top_view.addItem(grid)

        self.skeleton_side_view = gl.GLViewWidget()
        self.skeleton_side_view.setCameraPosition(rotation= QtGui.QQuaternion().fromEulerAngles(0,90,0))
        self.skeleton_side_view.opts['elevation'] = 0
        self.skeleton_side = gl.GLLinePlotItem(pos=np.zeros((8, 3)),width=2,antialias=False)
        self.skeleton_side_view.addItem(self.skeleton_side)
        self.skeleton_side_view.setFixedSize(320,320)
        self.skeleton_side_view.addItem(grid)
        top = QLabel('Top')
        top.resize(100,100)
        side = QLabel('Side')
        side.resize(100,100)
        # TODO rotate to side and top
        skeleton_widget.addWidget(top)
        skeleton_widget.addWidget(self.skeleton_side_view)
        skeleton_widget.addWidget(side)
        skeleton_widget.addWidget(self.skeleton_top_view)

        DWGUI.addWidget(joint_angle_widget)
        DWGUI.addLayout(skeleton_widget)
        DWGUI.addWidget(self.CameraFeed)
        # self.setGeometry(50,100,2000,800)
        self.setLayout(DWGUI)

        # create the video capture thread and sampling thread
        self.dw_thread = DeweightThread(cameras,flags,self.data_connectors,server,video)
        # connect its signal to the update_image slot
        self.dw_thread.ImageUpdate.connect(self.ImageUpdateSlot)
        self.dw_thread.SkeletonUpdate.connect(self.SkeletonUpdateSlot)
        # start the thread
        self.dw_thread.start()

    def ImageUpdateSlot(self, Image):
        self.CameraFeed.setPixmap(QPixmap.fromImage(Image))

    def SkeletonUpdateSlot(self, joints):
        self.skeleton_top.setData(pos=joints,width=2,antialias=False)
        self.skeleton_side.setData(pos=joints,width=2,antialias=False)

    def closeEvent(self, event):
        print(self.dw_thread.stop())
        event.accept()
