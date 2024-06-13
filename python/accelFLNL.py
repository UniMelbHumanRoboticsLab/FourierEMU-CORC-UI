import numpy as np
import time
import serial.tools.list_ports
from FLNL import * 
import matplotlib.pyplot as plt
import matplotlib

ports = serial.tools.list_ports.comports()
serialInst = serial.Serial()
portLists = []
print("Hello")
for onePort in ports:
    print(onePort)

serialInst.baudrate = 38400
serialInst.port = "/dev/ttyACM0"
cur_runtime = time.time()

# #Show a view for BNO055 data
class imuView():
    def __init__(self, target_val="Orient",num_dim=3):
        plt.interactive(True)
        self.orient_vec = np.array([[0,0,0]])
        self.lin_acc_vec = np.array([[0,0,0]])
        self.gravity_vec = np.array([[0,0,0]])
        self.quater_vec = np.array([[0,0,0,0]])
        self.total_vec = []
        self.num_dim = num_dim
        self.fig,self.ax = plt.subplots(num_dim)
        self.val_types = ["Orient:","Lin_Acc:","Gravity:","Quater:"]
        if num_dim == 3:
            self.val_dims = ["x","y","z"]
        elif num_dim == 4:
            self.val_dims = ["w","x","y","z"]
        # plt.gcf().canvas.set_window_title(target_val)
        self.fig.tight_layout()
        
    def update_vec(self, val_type, val):
        max_val = 10
        cmd_type = ""
        if val_type == 'Orient:':
            cmd_type = "ORT"
            self.orient_vec = np.vstack((self.orient_vec ,val))
            if(self.orient_vec.shape[0]>max_val):
                self.orient_vec = np.delete(self.orient_vec, 0, 0)
        elif val_type == 'Linear:':
            cmd_type = "LACC"
            self.lin_acc_vec = np.vstack((self.lin_acc_vec ,val))
            if(self.lin_acc_vec.shape[0]>max_val):
                self.lin_acc_vec = np.delete(self.lin_acc_vec, 0, 0)
        elif val_type == 'Gravit:':
            cmd_type = "GRA"
            self.gravity_vec = np.vstack((self.gravity_vec ,val))
            if(self.gravity_vec.shape[0]>max_val):
                self.gravity_vec = np.delete(self.gravity_vec, 0, 0)
        elif val_type == 'Quater:':
            cmd_type = "QUA"
            self.quater_vec = np.vstack((self.quater_vec ,val))
            if(self.quater_vec.shape[0]>max_val):
                self.quater_vec = np.delete(self.quater_vec, 0, 0)

        self.total_vec = [self.orient_vec,self.lin_acc_vec,self.gravity_vec,self.quater_vec]
        
        return cmd_type

    def updateTimePlot(self,val_type):
        
        index = self.val_types.index(val_type)
        cur_vec = self.total_vec[index]
        
        for i in range(self.num_dim):
            self.ax[i].cla()
            self.ax[i].set_title(self.val_dims[i])
            self.ax[i].plot(range(cur_vec[:,i].shape[0]),cur_vec[:,i])
        plt.pause(0.01)

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
                    imuViewObj.updateTimePlot(target_val)
                    # client.SendCmd(flnl_Cmd,val.tolist())
                    
                except Exception as error:
                    print("Aligning Serial\n")
                    print(error)
                    continue

