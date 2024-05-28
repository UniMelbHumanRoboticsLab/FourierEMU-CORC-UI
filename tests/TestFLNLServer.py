import time
import sys
sys.path.append('../python')
from FLNL import *


c = FLNLServer()

c.WaitForClient()

vals = [2.365, 10.23, 0.042];

c.SendCmd("STA")
for i in range(1,1000):
	c.SendValues(vals)
	time.sleep(0.5)
	if c.IsCmd("STO"):
		c.SendCmd("OK")
		print("received start")
	elif c.IsCmd("OKO"):
		c.SendCmd("OK")
		print("received OKO")
	elif c.IsAnyCmd():
	    print("received", c.GetCmd())
	c.SendCmd("STO")
	if(c.IsValues()):
	    print(c.GetValues())
	if(not c.IsConnected()):
		c.WaitForClient()

c.Close()
