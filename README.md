# FourierEMU-CORC-UI

UI (Unity) and [CORC](https://github.com/UniMelbHumanRoboticsLab/CANOpenRobotController) stateMachine for Fourier EMU. The associated publication can be found [here](https://doi.org/10.1080/09638288.2024.2394175).

The repository includes:

 - A Unity project within [Unity folder](./Unity) providing the user interface intended for a touch screen and interface for trakSTAR magnetic sensors
 - CORC stateMachine in [EMUFourierMachine](./EMUFourierMachine) : CORC CMakeFile required to point to this folder for compilation
 - A Python program using MediaPipe for arm pose tracking (see [MediaPipe folder](./MediaPipe))
 - All communication between the different processes relies on custom [FLNL library](https://github.com/vcrocher/libFLNL) included in CORC.
 - A simple pendant/joystick firmware and hardware description used to control the Unity interface via USB in [Joystick](./Joystick)



