import numpy as np
import time
import serial.tools.list_ports
from FLNL import * 
import matplotlib.pyplot as plt
import matplotlib
import sys
import time
from random import randint
from threading import Thread
from time import sleep
from typing import Union

ports = serial.tools.list_ports.comports()
serialInst = serial.Serial()
portLists = []
print("Hello")
for onePort in ports:
    print(onePort)

serialInst.baudrate = 38400
serialInst.port = "/dev/ttyACM0"
cur_runtime = time.time()

# #Show a view for one BNO055 data
class imuView():
    def __init__(self, target_val="Orient",num_dim=3):
        plt.interactive(True)
        self.orient_vec = np.array([[0,0,0]])
        self.lin_acc_vec = np.array([[0,0,0]])
        self.gravity_vec = np.array([[0,0,0]])
        self.quater_vec = np.array([[0,0,0,0]])
        self.num_dim = num_dim
        
    def update_vec(self, val_type, val):
        cmd_type = ""
        if val_type == 'Orient:':
            cmd_type = "ORT"
            self.orient_vec = np.vstack((self.orient_vec ,val))
        elif val_type == 'Linear:':
            cmd_type = "ACC"
            self.lin_acc_vec = np.vstack((self.lin_acc_vec ,val))
        elif val_type == 'Gravit:':
            cmd_type = "GRA"
            self.gravity_vec = np.vstack((self.gravity_vec ,val))
        elif val_type == 'Quater:':
            cmd_type = "QUA"
            self.quater_vec = np.vstack((self.quater_vec ,val))
        return cmd_type

target_val = "Orient:"
num_dim = 3
imuViewObj = imuView(target_val,num_dim)
try:
    serialInst.open()
except Exception as error:
    print(error)
    print("COM busy or accelerometer not connected")
else:
    try: 
        p = 0
        client = FLNLClient()
    except Exception as error:
        print(error)
        print("Can't connect to Server")
    else:
        # client.Connect(ip="127.0.0.1",port=2048)
        while True: #client.Connected:
            try: 
                packet = serialInst.readline()
            except:
                if time.time() > cur_runtime+1:
                    print("Arduino Disconnected\n")
                    break
            else:
                try:
                    cur_runtime = time.time()
                    str = packet.decode('utf')[:-2]
                    tokens = str.split("\t")
                    if (tokens[0] == target_val):
                        print(str)
                    val_str = [word[2:-2] for word in tokens[1:]]
                    val = np.array([list(map(float,val_str))])
                    flnl_Cmd = imuViewObj.update_vec(tokens[0],val)
                    # client.SendCmd(flnl_Cmd,val.tolist())
                    
                except Exception as error:
                    print("Aligning Serial\n")
                    print(error)
                    continue

