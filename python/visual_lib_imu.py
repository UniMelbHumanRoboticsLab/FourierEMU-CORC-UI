import numpy as np
import time
import serial.tools.list_ports
from FLNL import * 
import sys
import signal
from time import sleep
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import QApplication

# variables to start Serial
ports = serial.tools.list_ports.comports()
serialInst = serial.Serial()
portLists = []
print("Hello")
for onePort in ports:
    print(onePort)
serialInst.baudrate = 38400
print(serialInst.baudrate )
serialInst.port = "/dev/ttyACM0"
cur_runtime = time.time()

# start FLNL Server here
try: 
    client = FLNLClient()
except Exception as error:
    print(error)
    print("Can't connect to Server")
else:
    client.Connect(ip="127.0.0.1",port=2048)

# variables to start app
running = True
app = QApplication(sys.argv)

def stop():
    """Stop current QApplication"""
    global running
    running = False
    app.exit(0)

def sample(*data_connectors, visual_val="Orient:"):
    x = 0
    while running:
        try: 
            packet = serialInst.readline()
        except:
            if time.time() > cur_runtime+1:
                print("Arduino Disconnected\n")
                stop()
        else:
            try:
                cur_runtime = time.time()
                str = packet.decode('utf')[:-2]
                tokens = str.split("\t")
                val_str = [word[2:-2] for word in tokens[1:]]
                
                
                if tokens[0] == 'Orient:':
                    cmd_type = "ORT"
                elif tokens[0] == 'Linear:':
                    cmd_type = "ACC"
                elif tokens[0] == 'Gravit:':
                    cmd_type = "GRA"
                elif tokens[0] == 'Quater:':
                    cmd_type = "QUA"
                else: 
                    cmd_type = -1

                if cmd_type != -1:
                    val = np.array(list(map(float,val_str)))
                    # client.SendCmd(cmd_type,val.tolist())
                    # print(f"Sending {cmd_type} to Server\n")
                else:
                    continue
                
                # Send 1 and visualize
                if (tokens[0] == visual_val):
                    timestamp = time.time()
                    print(f"epoch: {timestamp}, {str}")
                    client.SendCmd(cmd_type,val.tolist())
                    print(f"Sending {cmd_type} to Server\n")
                    for index, data_connector in enumerate(data_connectors):
                        data_connector.cb_append_data_point(val[index], timestamp)

            except Exception as error:
                print("Aligning Serial\n")
                print(error)
                continue
            
# Connect SIGINT with stop function
signal.signal(signal.SIGINT, lambda sig, frame: stop())
