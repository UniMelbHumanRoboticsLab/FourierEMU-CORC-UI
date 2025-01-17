#include "EMUDeweighting.h"

using namespace std;

bool goToCalib(StateMachine & SM) {
    EMUDeweighting & sm = static_cast<EMUDeweighting &>(SM); //Cast to specific StateMachine type

    //keyboard or joystick press
    if ( sm.robot()->keyboard->getKeyUC()=='C' )
        return true;

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd("GOCA") ) {
        sm.UIserver->sendCmd(string("OKCA"));
        spdlog::debug("goToCalib");
        return true;
    }

    //Otherwise false
    return false;
}

bool endCalib(StateMachine & sm) {
    return (sm.state<M3CalibState>("CalibState"))->isCalibDone();
}

bool goToLock(StateMachine & SM) {
    EMUDeweighting & sm = static_cast<EMUDeweighting &>(SM); //Cast to specific StateMachine type

    //keyboard press
    if ( sm.robot()->joystick->isButtonTransition(3)>0 || sm.robot()->keyboard->getKeyUC()=='L' )
        return true;

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd("GOLO") ) {
        sm.UIserver->sendCmd(string("OKLO"));
        spdlog::debug("goToLock");
        return true;
    }

    //Otherwise false
    return false;
}

bool goToUnlock(StateMachine & SM) {
    EMUDeweighting & sm = static_cast<EMUDeweighting &>(SM); //Cast to specific StateMachine type

    //keyboard press
    if ( sm.robot()->keyboard->getKeyUC()=='U' )
        return true;

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd("GOUN") ) {
        sm.UIserver->sendCmd(string("OKUN"));
        spdlog::debug("goToUnlock");
        return true;
    }

    //Otherwise false
    return false;
}

bool goToReset(StateMachine & SM) {
    EMUDeweighting & sm = static_cast<EMUDeweighting &>(SM); //Cast to specific StateMachine type

    //keyboard press
    if ( sm.robot()->keyboard->getKeyUC()=='R' )
        return true;

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd("GORE") ) {
        sm.UIserver->sendCmd(string("OKRE"));
        spdlog::debug("goToReset");
        return true;
    }

    //Otherwise false
    return false;
}

bool goToJerk(StateMachine & SM) {
    EMUDeweighting & sm = static_cast<EMUDeweighting &>(SM); //Cast to specific StateMachine type

    std::vector<double> params;
    if ( sm.UIserver->isCmd("GOJE", params) ) {
        if(params.size()>0) {
            //Parse parameters: first is nb of pts followed by pts: (x,y,z,tf)
            unsigned int nbpts= (int)params[0];
            if(params.size() == 1+nbpts*5) {
                std::shared_ptr<M3MinJerkPosition> s = sm.state<M3MinJerkPosition>("MinJerkState");
                s->clearPts();
                int idx = 1;
                //Fill in trajectory pts
                for(unsigned int i=0; i<nbpts; i++) {
                    s->addPt(params[idx], params[idx+1], params[idx+2], params[idx+3], params[idx+4]);
                    spdlog::debug("goToJerk: Added point ({},{},{}:{}:{}).", params[idx], params[idx+1], params[idx+2], params[idx+3], params[idx+4]);
                    idx+=5;
                }

                spdlog::debug("goToJerk: Fed {} trajectory points.", nbpts);
                sm.UIserver->sendCmd(string("OKJE"));
                return true;
            }
            else {
                spdlog::warn("goToJerk: Error: number of command parameters.");
                sm.UIserver->sendCmd(string("ERJE"));
                return false;
            }
        }
        else {
            spdlog::warn("goToJerk: Error: number of command parameters.");
            sm.UIserver->sendCmd(string("ERJE"));
            return false;
        }
    }
    else {
        if ( sm.robot()->keyboard->getKeyUC()=='J' ) {
            //Test points added automatically when keyboard interaction is used
            std::shared_ptr<M3MinJerkPosition> s = sm.state<M3MinJerkPosition>("MinJerkState");
            s->clearPts();
            s->addPt(-0.688, -0.376, -0.323, 2.0, .0);
            s->addPt(-0.681, -0.138, -0.289, 2.0, 3.0);
            s->addPt(-0.907, -0.047, -0.251, 2.0, 1.0);

            return true;
        }
    }

    return false;
}

bool goToPath(StateMachine & SM) {
    EMUDeweighting & sm = static_cast<EMUDeweighting &>(SM); //Cast to specific StateMachine type

    std::vector<double> params;
    if ( sm.UIserver->isCmd("GOPA", params) ) {
        if(params.size() >0) {
            //Parse parameters: first is nb of pts followed by pts: (x,y,z,tf)
            unsigned int nbpts = (int)params[0];
            if(params.size() == 1+nbpts*5) {
                std::shared_ptr<M3PathState> s = sm.state<M3PathState>("PathState");
                s->clearPts();
                int idx = 1;
                //Fill in trajectory pts
                for(unsigned int i=0; i<nbpts; i++) {
                    s->addPt(params[idx], params[idx+1], params[idx+2], params[idx+3], params[idx+4]);
                    spdlog::debug("goToPath: Added point ({},{},{}:{}).", params[idx], params[idx+1], params[idx+2], params[idx+3], params[idx+4]);
                    idx+=5;
                }

                spdlog::debug("goToPath: Fed {} trajectory points.", nbpts);
                sm.UIserver->sendCmd(string("OKPA"));
                return true;
            }
            else {
                spdlog::warn("goToPath: Error: number of command parameters.");
                sm.UIserver->sendCmd(string("ERPA"));
                return false;
            }
        }
        else {
            spdlog::warn("goToPath: Error: number of command parameters.");
            sm.UIserver->sendCmd(string("ERPA"));
            return false;
        }
    }
    else {
        if ( sm.robot()->keyboard->getKeyUC()=='P' ) {
            //Test points added automatically when keyboard interaction is used
            std::shared_ptr<M3MinJerkPosition> s = sm.state<M3MinJerkPosition>("PathState");
            s->clearPts();
            s->addPt(-0.688, -0.376, -0.323, 2.0, .0);
            s->addPt(-0.681, -0.138, -0.289, 2.0, 3.0);
            s->addPt(-0.907, -0.047, -0.251, 2.0, 1.0);
            return true;
        }
    }

    return false;
}

bool goToGravity(StateMachine & SM) {
    EMUDeweighting & sm = static_cast<EMUDeweighting &>(SM); //Cast to specific StateMachine type

    //TODO: differentiate (for logging) from standby state
    std::vector<double> params;
    if ( sm.UIserver->isCmd("GOGR", params) ) {
        if(params.size() == 1) {
            std::shared_ptr<M3MassCompensation> s = sm.state<M3MassCompensation>("StandbyState");
            s->setMass(params[0]);
            sm.UIserver->sendCmd(string("OKGR"));
            spdlog::debug("goToGravity");
            return true;
        }
        else {
            spdlog::warn("goToGravity: Error: number of command parameters.");
            sm.UIserver->sendCmd(string("ERGR"));
            return false;
        }
    }
    else {
        if ( sm.robot()->keyboard->getKeyUC()=='D' )
            return true;
    }

    return false;
}

//Exit CORC app properly
bool quit(StateMachine & SM) {
    EMUDeweighting & sm = static_cast<EMUDeweighting &>(SM); //Cast to specific StateMachine type

    //keyboard press
    if ( sm.robot()->keyboard->getKeyUC()=='Q' ) {
        std::raise(SIGTERM); //Clean exit
        return true;
    }

    //Check incoming command requesting state change
    if ( sm.UIserver->isCmd("QUIT") ) {
        sm.UIserver->sendCmd(string("OKQU"));
        spdlog::debug("goToQuit");
        std::raise(SIGTERM); //Clean exit
        return true;
    }

    return false;
}

//Fake transition (return false all the time) used to update the Path state parameter
bool updatePath(StateMachine & SM) {
    EMUDeweighting & sm = static_cast<EMUDeweighting &>(SM); //Cast to specific StateMachine type

    std::vector<double> params;

    //Path update assistance
    if ( sm.UIserver->isCmd("UDPA", params) ) {
        std::shared_ptr<M3PathState> s = sm.state<M3PathState>("PathState");

        if(params.size() == 1) {
            if(params[0]>-30. &&  params[0]<30.) {
                s->setAssistanceLevel(params[0]);
            }
            sm.UIserver->sendCmd(string("OKUP"));
            spdlog::debug("updatePath: {}", params[0]);
        }
        else {
            spdlog::warn("updatePath: Error: number of command parameters.");
            sm.UIserver->sendCmd(string("ERUD"));
        }
    }

    return false;
}

//Fake transition (return false all the time) used to update the mass parameter
bool updateMass(StateMachine & SM) {
    EMUDeweighting & sm = static_cast<EMUDeweighting &>(SM); //Cast to specific StateMachine type

    std::vector<double> params;

    //Path update assistance
    if ( sm.UIserver->isCmd("UDMA", params) ) {
        std::shared_ptr<M3MassCompensation> s = sm.state<M3MassCompensation>("StandbyState");

        if(params.size() == 1) {
            if(params[0]>=0. &&  params[0]<=3.) {
                s->setMass(params[0]);
            }
            sm.UIserver->sendCmd(string("OKUM"));
            spdlog::debug("updateMass: {}", params[0]);
        }
        else {
            spdlog::warn("updateMass: Error: number of command parameters.");
            sm.UIserver->sendCmd(string("ERUD"));
        }
    }

    return false;
}



EMUDeweighting::EMUDeweighting() {
    //Create a Robot and set it to generic state machine
    setRobot(std::make_unique<RobotM3>("EMU_FOURIER", "M3_params.yaml"));
    // setRobot(std::make_unique<RobotM3>("EMU_MELB", "M3_params.yaml"));

    //TODO: include new FLNL state and associated transitions
    //TODO: proper logic and transitions

    //Create state instances and add to the State Machine
    addState("DoNothingState", std::make_shared<M3NothingState>(robot(), this));
    addState("ResetState", std::make_shared<M3NothingState>(robot(), this));
    addState("CalibState", std::make_shared<M3CalibState>(robot(), this));
    addState("StandbyState", std::make_shared<M3MassCompensation>(robot(), this));
    addState("DeweighState", std::make_shared<M3AdvDeweighting>(robot(), this));
    addState("MinJerkState", std::make_shared<M3MinJerkPosition>(robot(), this));
    addState("PathState", std::make_shared<M3PathState>(robot(), this));
    addState("LockState", std::make_shared<M3LockState>(robot(), this));

    //Define transitions between states
    addTransition("DoNothingState", &goToCalib, "CalibState");
    addTransition("CalibState", &endCalib, "StandbyState");

    addTransition("StandbyState", &goToJerk, "MinJerkState");
    addTransition("MinJerkState", &goToJerk, "MinJerkState");
    addTransition("PathState", &goToJerk, "MinJerkState");

    addTransition("StandbyState", &goToPath, "PathState");
    addTransition("PathState", &goToPath, "PathState");
    addTransition("MinJerkState", &goToPath, "PathState");
    addTransition("PathState", &updatePath, "PathState"); //Fake transition never returning true

    addTransition("StandbyState", &goToGravity, "DeweighState");
    addTransition("DeweighState", &goToGravity, "DeweighState");
    addTransition("MinJerkState", &goToGravity, "DeweighState");
    addTransition("PathState", &goToGravity, "DeweighState");

    addTransition("DeweighState", &goToUnlock, "StandbyState");
    addTransition("StandbyState", &updateMass, "StandbyState"); //Fake transition never returning true

    addTransition("PathState", &goToLock, "LockState");
    addTransition("MinJerkState", &goToLock, "LockState");
    addTransition("StandbyState", &goToLock, "LockState");
    addTransition("LockState", &goToUnlock, "StandbyState");

    addTransition("StandbyState", &goToReset, "ResetState");
    addTransition("MinJerkState", &goToReset, "ResetState");
    addTransition("PathState", &goToReset, "ResetState");
    addTransition("LockState", &goToReset, "ResetState");
    addTransition("DeweighState", &goToReset, "ResetState");
    addTransition("ResetState", &goToGravity, "DeweighState");

    addTransitionFromAny(&quit, "StandbyState");
    addTransition("StandbyState", &quit, "StandbyState"); //From any does not apply to self (destination state)
}
EMUDeweighting::~EMUDeweighting() {
}

/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */
void EMUDeweighting::init() {
    spdlog::debug("EMUDeweighting::init()");
    if(robot()->initialise()) {
        logHelper.initLogger("EMUDeweightingLog", "logs/EMUDeweighting.csv", LogFormat::CSV, true);
        logHelper.add(runningTime(), "Time (s)");
        logHelper.add(robot()->getEndEffPosition(), "X");
        logHelper.add(robot()->getEndEffVelocity(), "dX");
        logHelper.add(robot()->getInteractionForce(), "F");
        logHelper.add(robot()->getEndEffAcceleration(), "ddX");
        logHelper.add(robot()->getEndEffVelocityFiltered(), "dXFilt");
        #ifdef NOROBOT
            UIserver = std::make_shared<FLNLHelper>(*robot(), "127.0.0.1");
            // UIserver = std::make_shared<FLNLHelper>(*robot(), "192.168.7.2");
        #else
            UIserver = std::make_shared<FLNLHelper>(*robot(), "127.0.0.1");
            // UIserver = std::make_shared<FLNLHelper>(*robot(), "192.168.7.2");
        #endif // NOROBOT
        UIserver->registerState(Command);
        UIserver->registerState(MvtProgress);
        UIserver->registerState(Contribution);
    }
    else {
        spdlog::critical("Failed robot initialisation. Exiting...");
        std::raise(SIGTERM); //Clean exit
    }
}

void EMUDeweighting::end() {
    if(running())
        UIserver->closeConnection();
    StateMachine::end();
}


/**
 * \brief Statemachine to hardware interface method. Run any hardware update methods
 * that need to run every program loop update cycle.
 *
 */
void EMUDeweighting::hwStateUpdate() {
    StateMachine::hwStateUpdate();
    //Also send robot state over network
    UIserver->sendState();
    //Attempt to reconnect (if not already waiting for connection)
    UIserver->reconnect();
}
