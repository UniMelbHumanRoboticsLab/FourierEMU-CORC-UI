# FourierEMU-CORC-UI
UI (Unity) and [CORC](https://github.com/UniMelbHumanRoboticsLab/CANOpenRobotController) stateMachine for Fourier EMU.

 - Unity project within Unity folder (!)
 - CORC stateMachine in EMUFourierMachine: CORC CMakeFile required to point to this folder for compilation
 - Communication relies on custom [FLNL library](https://github.com/vcrocher/libFLNL) included in CORC

## List of commands
Communication commands from the UI side to the CORC state machine:
|   Command|  Parameters                          | Response(s)|   Action                                                     |
|----------|--------------------------------------|------------|--------------------------------------------------------------|
|      GOCA|    /                                 |   OKCA     |  Go from Nothing state to Calbration                         |
|      GOLO|       /                              |   OKLO     |  Go to Lock state                                            |
|      GOUN|    /                                 |  OKUN      |  Unlock (go to Deweighting from Lock state)                  |
|      GOJE| [nbpts,x0,y0,z0,T0, ..., xn,yn,zn,Tn]| OKJE/ERJE* |  Go to min Jerk (passive mob.) with n points                 |
|      GOPA| [nbpts,x0,y0,z0,T0, ..., xn,yn,zn,Tn]| OKPA/ERPA* |  Go to Path state (guidance) with n points (T param not used)|
|      GOGR|    [mass]                            | OKGR/ERGR* |  Go to deweighting with specified mass compensation          |
|      UDPA| [assistance]                         | OKUP/ERUD* |  Update Path (guidance) assistance viscosity. In path only.  |
|      UDMA|    [mass]                            | OKUM/ERUD* |  Update mass compensation in Deweigthing state               |

*Commands return ER__ only if wrong/inconsistent number of parameters are provided but OK__ if command is applied but parameter value is out of bound (and so saturated or not applied).

See EMUFourierMachine constructor for allowed transitions.
