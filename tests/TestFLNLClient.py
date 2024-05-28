import time
import sys
sys.path.append('../python')
from FLNL import *


c = FLNLClient()

if(c.Connect()):
    vals = [42.0, 2048, 1./3.];

    for i in range(1,100):
        c.SendValues(vals)
        time.sleep(0.5)
        if c.IsCmd("STA"):
            c.SendCmd("OKS")
            print("received start")
        if c.IsCmd("STO"):
            c.SendCmd("OKO")
            print("received stop")
        if(c.IsValues()):
            print(c.GetValues())
        c.SendCmd("RRR")
    c.Close()
