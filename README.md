# FourierEMU-CORC-UI
UI (Unity) and [CORC](https://github.com/UniMelbHumanRoboticsLab/CANOpenRobotController) stateMachine for Fourier EMU.

 - Unity project within [Unity folder](./Unity)
 - CORC stateMachine in EMUFourierMachine: CORC CMakeFile required to point to this folder for compilation
 - A standalone Python program using MediaPipe for arm pose tracking (see [MediaPipe folder](./MediaPipe)
 - Communication relies on custom [FLNL library](https://github.com/vcrocher/libFLNL) included in CORC

## List of commands
Communication commands from the UI side to the CORC state machine:
|   Command|  Parameters                          | Response(s)   |   Action                                                                           |
|----------|--------------------------------------|---------------|------------------------------------------------------------------------------------|
|      GOCA|    /                                 |   OKCA        |  Go from Nothing state to Calibration state.                                       |
|      GOLO|       /                              |   OKLO        |  Go to Lock state (from any other but Nothing or Reset states).                    |
|      GOUN|    /                                 |  OKUN         |  Unlock (go to Deweighting from Lock state)                                        |
|      GORE| /                                    | OKRE          |  Go to Reset state: reset torque control and apply 0 torque.                       |
|      GOJE| [nbpts,x0,y0,z0,T0,T0pause, ..., xn,yn,zn,Tn,Tnpause]| OKJE/ERJE* |  Go to Min Jerk (passive mob.) with n points (wait Tpause s at pt).   |
|      GOPA| [nbpts,x0,y0,z0,T0,T0pause, ..., xn,yn,zn,Tn,Tnpause]| OKPA/ERPA* |  Go to Path state (guidance) with n points (T param not used)         |
|      GOGR|    [mass]                            | OKGR/ERGR*    |  Go to Deweighting state with specified mass compensation                          |
|      UDPA| [assistance]                         | OKUP/ERUD*    |  Update Path (guidance) assistance viscosity. In path state only.                  |
|      UDMA|    [mass]                            | OKUM/ERUD*    |  Update mass compensation. In Deweigthing state only.                              |
|      QUIT| /                                    | OKQU          |  Quit the CORC app properly.                                                       |

*Commands return ER__ only if wrong/inconsistent number of parameters are provided but OK__ if command is applied but parameter value is out of bound (and so saturated or not applied). TL;DR: ER represents only communication errors, nothing else.

See EMUFourierMachine constructor for allowed transitions.



