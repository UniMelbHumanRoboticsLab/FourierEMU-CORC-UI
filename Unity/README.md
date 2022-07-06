# FourierEMU-CORC-UI
## Unity UI

Single scene 2D Unity interface for control of Fourier EMU device with CORC. 

It is intended for simple touchscreen use and to allow recording of activities and movements in dedicated log files (see `SceneManager.cs` and `SessionsData.cs` scripts for loging). 

A module is used to synchronously record arm movements using NDI trakSTAR sensors as well as Intel RealSense depth camera.

The interface interacts with CORC using FLNL (see main README for commands and `CORCM3_FOURIER.cs` script).


