#import pkg_resources
import time
import logging
import os, sys, subprocess, csv, math
import datetime
import cv2
import mediapipe as mp
from realsense_camera import *
import pyrealsense2 as rs2
#import pupil_apriltags as apriltag
from FLNL import *
import keyboard
from enum import IntEnum

from numpy.linalg import norm, inv


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
        self.pose = self.mpPose.Pose(model_complexity=0,
                                     min_detection_confidence=0.5,
                                     min_tracking_confidence=0.5,
                                     smooth_landmarks=True)

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
                    jointsImgPos[joint_id] = [cx, cy]
                else:
                    jointsImgPos[joint_id] = [np.NAN, np.NAN]
        if draw:
            cv2.imshow('Frame', img)

        # ## TODO: use self.results.pose_world_landmarks instead???
        # if self.results.pose_world_landmarks:
            # x1 = self.results.pose_world_landmarks.landmark[11].x;
            # y1 = self.results.pose_world_landmarks.landmark[11].y;
            # z1 = self.results.pose_world_landmarks.landmark[11].z;
            # x2 = self.results.pose_world_landmarks.landmark[12].x;
            # y2 = self.results.pose_world_landmarks.landmark[12].y;
            # z2 = self.results.pose_world_landmarks.landmark[12].z;
            # s_2_s=np.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)*(z1-z2)*(z1-z2))
            # x1 = self.results.pose_world_landmarks.landmark[13].x;
            # y1 = self.results.pose_world_landmarks.landmark[13].y;
            # z1 = self.results.pose_world_landmarks.landmark[13].z;
            # x2 = self.results.pose_world_landmarks.landmark[11].x;
            # y2 = self.results.pose_world_landmarks.landmark[11].y;
            # z2 = self.results.pose_world_landmarks.landmark[11].z;
            # ua_l=np.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)*(z1-z2)*(z1-z2))
            # x1 = self.results.pose_world_landmarks.landmark[13].x;
            # y1 = self.results.pose_world_landmarks.landmark[13].y;
            # z1 = self.results.pose_world_landmarks.landmark[13].z;
            # x2 = self.results.pose_world_landmarks.landmark[15].x;
            # y2 = self.results.pose_world_landmarks.landmark[15].y;
            # z2 = self.results.pose_world_landmarks.landmark[15].z;
            # fa_l=np.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)*(z1-z2)*(z1-z2))
            # print([s_2_s, ua_l, fa_l])

        return jointsImgPos


filt0_1,filt0_2,filt0_3,filt0_4 = 0,0,0,0
ang0_1,ang0_2,ang0_3,ang0_4 = 0,0,0,0
def lowpass(ang1,ang2,ang3,ang4):
    global ang0_1,ang0_2,ang0_3,ang0_4
    global filt0_1,filt0_2,filt0_3,filt0_4
    filt1 = 0.683 * filt0_1 + 0.159 * ang1 + 0.159 * ang0_1
    filt2 = 0.683 * filt0_2 + 0.159 * ang2 + 0.159 * ang0_2
    filt3 = 0.683 * filt0_3 + 0.159 * ang3 + 0.159 * ang0_3
    filt4 = 0.683 * filt0_4 + 0.159 * ang4 + 0.159 * ang0_4
    [ang0_1, ang0_2, ang0_3, ang0_4] = [ang1,ang2,ang3,ang4]
    [filt0_1,filt0_2, filt0_3, filt0_4]  = [filt1, filt2, filt3, filt4]
    return filt1, filt2, filt3, filt4

# class AprilTag:
    # def __init__(self):
        # self.at_detector = apriltag.Detector(families='tag36h11',
                                    # nthreads=1,
                                    # quad_decimate=1.0,
                                    # quad_sigma=0.0,
                                    # refine_edges=1,
                                    # decode_sharpening=0.25,
                                    # debug=0)

    # def getPose(self, frame, color_intrin, depth_intrin, draw=False):
        # tagSize = 0.05  # Meter
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # tags = self.at_detector.detect(gray, estimate_tag_pose=True, camera_params=[color_intrin.fx, color_intrin.fy, color_intrin.ppx, color_intrin.ppy],
                                          # tag_size=tagSize)

        # if (len(tags) != 0):
            # transformation = np.append(tags[0].pose_R, tags[0].pose_t, axis=1)
            # transformation = np.append(transformation, np.array([[0,0,0,1]]), axis=0)
            # transformation = np.matmul(np.linalg.inv(transformation),
                                       # np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]]))
            # transformation = np.matmul(np.array([[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]]), transformation)
            # # rbtag = tags[0].corners[1]
            # # depth_to_object = depth_image[int(rbtag[1]), int(rbtag[0])]
            # # depth_point = rs2.rs2_deproject_pixel_to_point(depth_intrin, rbtag, depth_to_object / 1000)
            # # rbtag = [depth_point[0],depth_point[2],-depth_point[1]]
            # # rttag = tags[0].corners[2]
            # # depth_to_object = depth_image[int(rttag[1]), int(rttag[0])]
            # # depth_point = rs2.rs2_deproject_pixel_to_point(depth_intrin, rttag, depth_to_object / 1000)
            # # rttag = [depth_point[0],depth_point[2],-depth_point[1]]
            # # lttag = tags[0].corners[3]
            # # depth_to_object = depth_image[int(lttag[1]), int(lttag[0])]
            # # depth_point = rs2.rs2_deproject_pixel_to_point(depth_intrin, lttag, depth_to_object / 1000)
            # # lttag = [depth_point[0],depth_point[2],-depth_point[1]]
            # if draw:
                # for tag in tags:
                    # cv2.circle(frame, tuple(tag.corners[0].astype(int)), 4, (255, 0, 0), 2)  # left-top
                    # cv2.circle(frame, tuple(tag.corners[1].astype(int)), 4, (255, 0, 0), 2)  # right-top
                    # cv2.circle(frame, tuple(tag.corners[2].astype(int)), 4, (255, 0, 0), 2)  # right-bottom
                    # cv2.circle(frame, tuple(tag.corners[3].astype(int)), 4, (255, 0, 0), 2)  # left-bottom
                # cv2.imshow('Tag', frame)
        # else:
            # rbtag = [np.NAN,np.NAN,np.NAN]
            # rttag = [np.NAN,np.NAN,np.NAN]
            # lttag = [np.NAN,np.NAN,np.NAN]
            # if draw:
                # cv2.imshow('Tag', frame)

        # return tags #TODO


#Transform joints coordinates into local shoulder frame
#the origin frame is defined as center at shoulder point, X Left->Right shoulders, and Z vertical up crossing the in-between eyes point
def transform3DPoints(pts):
    #pts=np.array(pts)
    #print(pts)
    if(pts[J.L_W]):
        ##shoulder pt
        S = np.array(pts[J.L_S])
        ## elbow
        E = np.array(pts[J.L_E])
        ## wrist
        W = np.array(pts[J.L_W])
    else:
        ##shoulder pt
        S = np.array(pts[J.R_S])
        ## elbow
        E = np.array(pts[J.R_E])
        ## wrist
        W = np.array(pts[J.R_W])

    Tc0=np.eye(4)
    Tc0[0:3,3]=S
    #shoulder-shoulder vector as X
    Tc0[0:3,0] = (np.array(pts[J.R_S])-np.array(pts[J.L_S]))/norm(np.array(pts[J.R_S])-np.array(pts[J.L_S]))
    #eyes mid-point
    Head = np.array(pts[J.L_Y])+0.5*(np.array(pts[J.R_Y])-np.array(pts[J.L_Y]))
    #Z vector, head up. Y forward orth to X and Shoulder/head plane
    Tc0[0:3,1] = np.cross(S-Head, Tc0[0:3,0])
    Tc0[0:3,1] = Tc0[0:3,1]/norm(Tc0[0:3,1])
    Tc0[0:3,2] = np.cross(Tc0[0:3,0],Tc0[0:3,1])

    T0c=inv(Tc0)

    ## Apply transformation to remaining joints
    S = np.matmul(T0c, np.append(S,1))[0:3]
    E = np.matmul(T0c, np.append(E,1))[0:3]
    W = np.matmul(T0c, np.append(W,1))[0:3]


    np.set_printoptions(precision=2)
    np.set_printoptions(suppress=True)
    print(Tc0)
    print(E)
    print(W)

    return 0

#Compute distance between two joints
def dist3D(p1, p2):
    return np.sqrt((p1[0]-p2[0])*(p1[0]-p2[0]) + (p1[1]-p2[1])*(p1[1]-p2[1]) + (p1[2]-p2[2])*(p1[2]-p2[2]))

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

        ## Get landmarks image position
        jointsPosImg = detector.findPose(frame, armSide, objects_to_track, drawing)

        ## Convert to 3D
        if(rs.init and len(jointsPosImg)>0):
            jointsPos = rs.imgTo3D(jointsPosImg, depth_image)
            # Transform to shoulder frame
            #transform3DPoints(jointsPos)

        ## Get April tags
        #if(rs.init):
        #    tags = tag.getPose(frame, color_intrin, depth_intrin, drawing)

        ## Send joint positions and tag frame when valid values
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
                    #Distances
                    #print(str(dist3D(jointsPos[J.R_S],jointsPos[J.L_S])/1000)+', '+str(dist3D(jointsPos[J.L_S],jointsPos[J.L_E])/1000)+', '+str(dist3D(jointsPos[J.L_E],jointsPos[J.L_W])/1000))
                    #Points
                    #for id in objects_to_track:
                    #    print(jointsPos[id][0], jointsPos[id][1], jointsPos[id][2], sep=',', end=',')
                    #print('')
                    #Wrist position only
                    print(fps, 'fps: ', jointsPos[J.L_W][0], jointsPos[J.L_W][1], jointsPos[J.L_W][2], sep='\t')
                else:
                    server.SendValues(val)
                    print(fps)


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
            if(k==ord('r')):
                streaming=True
                print("Start streaming values")
                if(rs.init and recording):
                    rs.recorder.resume()
                    print("Start recording stream")

            ## Stop streaming?
            if(k==ord('s')):
                streaming=False
                print("Stop streaming values")
                if(rs.init and recording):
                    rs.recorder.pause()
                    print("Stop recording stream")

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

if __name__ == '__main__':
    main()
