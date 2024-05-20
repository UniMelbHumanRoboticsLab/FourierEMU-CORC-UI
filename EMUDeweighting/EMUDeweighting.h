/**
 * \file EMUDeweighting.h
 * \author Vincent Crocher
 * /brief The EMUDeweighting class is a state machine aims to evaluate deweighting algorithms on EMU device
 * \date 2024-03-26
 *
 * \copyright Copyright (c) 2024
 *
 */
#ifndef M3_SM_H
#define M3_SM_H


#include "StateMachine.h"
#include "RobotM3.h"
#include "FLNLHelper.h"

// State Classes
#include "EMUDeweightingStates.h"


/**
 * @brief Example implementation of a StateMachine for the M3Robot class. States should implemented M3DemoState
 *
 */
class EMUDeweighting : public StateMachine {

   public:
    EMUDeweighting();
    ~EMUDeweighting();
    void init();
    void end();

    void hwStateUpdate();

    RobotM3 *robot() { return static_cast<RobotM3*>(_robot.get()); } //!< Robot getter with specialised type (lifetime is managed by Base StateMachine)

    std::shared_ptr<FLNLHelper> UIserver = nullptr;     //!< Pointer to communication server

    //TODO: place in struct and pass to states (instead of whole state machine)
    double Command = 0;         //!< Command (state) currently applied
    double MvtProgress = 0;     //!< Progress (status) along mvt
    double Contribution = 0;    //!< User contribution to mvt
    double MassComp =0;         //!< Mass comp value used for standard operations
    Deweight_s DwData;


    void UpdateEnergy();
    double Energy = 0;
};

#endif /*M3_SM_H*/
