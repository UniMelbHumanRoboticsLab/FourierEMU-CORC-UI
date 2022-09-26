import time
from FLNL import *


c = FLNLServer()

c.WaitForClient()

vals = [2.365, 10.23, 0.042];

for i in range(1,1000):
	c.SendValues(vals)
	time.sleep(0.5)
	if c.IsCmd("STA"):
		c.SendCmd("OK")
		print("received start")
	if c.IsCmd("STO"):
		c.SendCmd("OK")
		print("received stop")

c.Close()