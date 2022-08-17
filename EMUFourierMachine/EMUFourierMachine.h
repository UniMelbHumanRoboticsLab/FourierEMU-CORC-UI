/**
 * \file EMUFourierMachine.h
 * \author Vincent Crocher
 * /brief The EMUFourierMachine class represents an example implementation of an M3 state machine.
 * \date 2022-08-17
 *
 * \copyright Copyright (c) 2020 - 2022
 *
 */
#ifndef M3_SM_H
#define M3_SM_H


#include "StateMachine.h"
#include "RobotM3.h"
#include "FLNLHelper.h"

// State Classes
#include "EMUFourierStates.h"


/**
 * @brief Example implementation of a StateMachine for the M3Robot class. States should implemented M3DemoState
 *
 */
class EMUFourierMachine : public StateMachine {

   public:
    EMUFourierMachine();
    ~EMUFourierMachine();
    void init();
    void end();

    void hwStateUpdate();

    RobotM3 *robot() { return static_cast<RobotM3*>(_robot.get()); } //!< Robot getter with specialised type (lifetime is managed by Base StateMachine)

    std::shared_ptr<FLNLHelper> UIserver = nullptr;     //!< Pointer to communication server

    //TODO: place in struct and pass to states (instead of whole state machine)
    double Command = 0;         //!< Command (state) currently applied
    double MvtProgress = 0;     //!< Progress (status) along mvt
    double Contribution = 0;    //!< User contribution to mvt
};

#endif /*M3_SM_H*/
