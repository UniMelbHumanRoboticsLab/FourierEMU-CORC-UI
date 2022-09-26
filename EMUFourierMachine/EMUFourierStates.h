/**
 * \file M3DemoState.h
 * \author Vincent Crocher
 * \date 2022-08-17
 *
 * \copyright Copyright (c) 2022
 *
 */

#ifndef M3STATE_H_DEF
#define M3STATE_H_DEF

#include "State.h"
#include "RobotM3.h"

class EMUFourierMachine;


typedef struct M3TrajPt
{
    M3TrajPt (double x, double y, double z, double tf): X(VM3(x, y, z)), T(tf) {};
    VM3 X;
    double T;
} M3TrajPt;


/**
 * \brief Generic state type for used with M3DemoMachine, providing running time and iterations number: been superseeded by default state, not very much useful anymore.
 *
 */
class EMUFourierState : public State {
   protected:
    RobotM3 * robot;                               //!< Pointer to state machines robot object

    EMUFourierState(RobotM3* M3, EMUFourierMachine *sm_, const char *name = NULL): State(name), robot(M3), sm(sm_){spdlog::debug("Created EMUFourierState {}", name);};
   private:
    void entry(void) final {
        /*std::cout
        << "==================================" << std::endl
        << " STARTING  " << getName() << std::endl
        << "----------------------------------" << std::endl
        << std::endl;*/

        //Actual state entry
        entryCode();
    };
    void during(void) final {
        //Actual state during
        duringCode();
    };
    void exit(void) final {
        exitCode();
        /*std::cout
        << "----------------------------------" << std::endl
        << "EXIT "<< getName() << std::endl
        << "==================================" << std::endl
        << std::endl;*/
    };

   public:
    virtual void entryCode(){};
    virtual void duringCode(){};
    virtual void exitCode(){};

   protected:
    EMUFourierMachine *sm;
};


/**
 * \brief Does nothing waiting for a calib command. Set drives in torque control mode.
 *
 */
class M3NothingState : public EMUFourierState {

   public:
    M3NothingState(RobotM3 * M3, EMUFourierMachine *sm, const char *name = "M3 Do Nothing"):EMUFourierState(M3, sm, name){};

    void entryCode(void) { robot->initTorqueControl(); robot->setJointTorque(VM3(0,0,0)); }
    void duringCode(void) { robot->setJointTorque(VM3(0,0,0)); }
    void exitCode(void) { robot->setJointTorque(VM3(0,0,0)); }
};

/**
 * \brief Position calibration of M3. Go to the bottom left stops of robot at constant torque for absolute position calibration. Set drives in torque control mode.
 *
 */
class M3CalibState : public EMUFourierState {

   public:
    M3CalibState(RobotM3 * M3, EMUFourierMachine *sm, const char *name = "M3 Calib State"):EMUFourierState(M3, sm, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    bool isCalibDone() {return calibDone;}

   private:
     VM3 qi;
     VM3 stop_reached_time;
     bool at_stop[3];
     bool calibDone=false;
};


/**
 * \brief Lock in place: position control around current point. Assumes drives in torque control already.
 *
 */
class M3LockState : public EMUFourierState {

   public:
    M3LockState(RobotM3 * M3, EMUFourierMachine *sm, const char *name = "M3 Lock"):EMUFourierState(M3, sm, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

   private:
    VM3 X0;
    double k = 1100.;                //! Impedance proportional gain (spring)
    double d = 3.;                   //! Impedance derivative gain (damper)
};


/**
 * \brief Provide end-effector mass compensation on M3. Mass is controllable through keyboard inputs. Assumes drives in torque control already.
 *
 */
class M3MassCompensation : public EMUFourierState {

   public:
    M3MassCompensation(RobotM3 * M3, EMUFourierMachine *sm, const char *name = "M3 Mass Compensation"):EMUFourierState(M3, sm, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    void setMass(double m) {mass=m; std::cout << "Mass: " << mass << std::endl;}

   private:
     const double mass_limit = 10;  //!< Maximum applicable mass (+ and -)
     double mass = 0;               //!< Desired mass to apply: might differ from applied_mass during transition (i.e. setMass)
     double applied_mass = 0;       //!< Currently appied mass
     double change_mass_rate = 2.;  //!< Rate at which mass will increase/decrease during change mass transition (in kg/s)
};


/**
 * \brief Generic pt to pt state: to be derived.
 *
 */
class M3PtToPt: public EMUFourierState {

   public:
    M3PtToPt(RobotM3 * M3, EMUFourierMachine *sm, const char *name = "M3 Pt to Pt"):EMUFourierState(M3, sm, name) {};

    virtual void entryCode(void) = 0;
    virtual void duringCode(void) = 0;
    virtual void exitCode(void) = 0;

    void clearPts() {
        trajPts.clear();
        stop=true;
    }

    bool addPt(double x, double y, double z, double tf) {
        //Lock robot: don't use pts while fed in
        stop=true;
        if(tf<0) {
            return false;
        }
        trajPts.push_back(M3TrajPt(x,y,z,tf));
        //Unlock
        stop=false;

        return true;
    }

    unsigned int getPtsIdx() { return trajPtIdx; }
    double getStatus() { return status; }

   protected:
    double status=0;            //!< Represents the progress (0-1) along the path/trajectory
    bool stop;                  //!< Flag to stop when feeding pts
    bool loop_through_points;
    unsigned int trajPtIdx=0;
    double startTime;
    std::vector<M3TrajPt> trajPts;
    VM3 Xi, Xf;
    double T;
};


/**
 * \brief Path contraint with viscous assistance. Assumes drives in torque control already.
 *
 */
class M3PathState : public M3PtToPt {

   public:
    M3PathState(RobotM3 * M3, EMUFourierMachine *sm, const char *name = "M3 Path State"):M3PtToPt(M3, sm, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    short int sign(double val) {return (val>0)?1:((val<0)?-1:0); };

    void setAssistanceLevel(double a) { viscous_assistance = fmax(-30., fmin(30., a)); }

   private:
    double k = 600;                 //! Impedance proportional gain (spring)
    double d = 6;                   //! Impedance derivative gain (damper)
    double viscous_assistance=0;    //! Viscous assistance along path
};


/**
 * \brief Point to point position control with min jerk trajectory interpolation. Assumes drives in torque control already.
 *
 */
class M3MinJerkPosition: public M3PtToPt {

   public:
    M3MinJerkPosition(RobotM3 * M3, EMUFourierMachine *sm, const char *name = "M3 Minimum Jerk Position"):M3PtToPt(M3, sm, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

   private:
    //float k_i=1.; //! Integral gain
    double k = 1100.;                //! Impedance proportional gain (spring)
    double d = 3.;                   //! Impedance derivative gain (damper)
};


#endif
