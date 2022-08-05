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

class PoseDetector:

    def __init__(self):
        self.mpDraw = mp.solutions.drawing_utils
        self.mpPose = mp.solutions.pose
        self.pose = self.mpPose.Pose(model_complexity=0, min_detection_confidence=0.5, min_tracking_confidence=0.5)

    def findPose(self, img, arm_side, objects_to_track, draw=False):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.pose.process(imgRGB)

        if draw:
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
                    cv2.circle(img, (cx, cy), 8, (255, 0, 0))#, cv2.FILLED)
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
            print("Recording ON")
        else:
            recording=False
            print("Recording OFF")
    else:
        recording=False
        print("Recording OFF")
        
    #Arm side to track
    armSide='l';
    # Left eye, Right eye, Left Hip, Right Hip, Left Shoulder, Right Shoulder, Left Elbow, Left Wrist
    objects_to_track = [2, 5, 23, 24, 11, 12, 13, 15]

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
    streaming=True
    server = FLNLServer()
    server.WaitForClient() #on default address 127.0.0.1 and port 2042
    print("FLNL connected")
    
    ## Init pose detector
    detector = PoseDetector()
    
    cTime = 0
    pTime = 0
    fps = 0
    while(server.IsConnected() or True):
        
        ## When connected, continue detection
        #Use realsense camera:
        if(rs.init):
            # Capture Camera Frame
            ret, color_image, depth_image = rs.get_frame_stream()
            # copy
            frame = color_image
        else:
            # Webcam otherwise
            sucess, frame = cap.read()
        frame.flags.writeable = False
            
        ## Get landmarks image position
        jointsPosImg = detector.findPose(frame, armSide, objects_to_track, drawing)
        
        ## Convert to 3D
        if(rs.init):
            jointsPos = rs.imgTo3D(jointsPosImg, depth_image)
        
        ## Get April tags
        #if(rs.init):
        #    tags = tag.getPose(frame, color_intrin, depth_intrin, drawing)

        ## Send joint positions and tag frame when valid values
        if(streaming):
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
                server.SendValues(val)
                print(fps)
        
        ## Start streming?
        if(server.IsCmd("STA")):
            streaming=True
            print("Start streaming values")
            if(recording):
                rs.recorder.resume()
                print("Start recording stream")

        ## Stop?    
        if(server.IsCmd("STO")):
            streaming=False
            print("Stop streaming values")
            if(recording):
                rs.recorder.pause()
                print("Stop recording stream")
            
        ## Set to left arm
        if(server.IsCmd("ARL")):
            armSide='l'
            # Left eye, Right eye, Left Hip, Right Hip, Left Shoulder, Right Shoulder, Left Elbow, Left Wrist
            objects_to_track = [2, 5, 23, 24, 11, 12, 13, 15]
            print("Tracking LEFT arm")
            
        ## Set to right arm
        if(server.IsCmd("ARR")):
            armSide='r'
            # Left eye, Right eye, Left Hip, Right Hip, Left Shoulder, Right Shoulder, Right Elbow, Right Wrist
            objects_to_track = [2, 5, 23, 24, 11, 12, 14, 16]
            print("Tracking Right arm")
            
        ## Disconnect command
        if(server.IsCmd("DIS")):
            break
            
        ## Can also close with 'q' if display
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
    if(server.IsConnected()):
        server.Close()
    if(rs.init):
        rs.release()
    else:
        cap.release()

    # Clean up
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

