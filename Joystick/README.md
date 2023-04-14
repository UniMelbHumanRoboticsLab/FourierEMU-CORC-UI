# FourierEMU-CORC-UI
## Joystick / Pendant

CAD files and firmware (Arduino project) to manage a minimal pendant equipped with two push-buttons and a small display allowing to control some parts of the Unity UI conveniently (from the robot handle/patient's wrist).

The pendant communicate with main PC (running Unity UI) using a serial communication protocol (115200 bps). Communication as follows:

From PC to Pendant:
 - S[D/A/M] to switch to Deweight, Assist or Mob mode
 - P[val] to update progress value (0-100) in Assist mode

From Pendant to PC:
 - B[0/1][0/1] when a button has been pressed (rising transition) with status of each button
