#include "EMUFourierStates.h"
#include "EMUFourierMachine.h"

using namespace std;

double timeval_to_sec(struct timespec *ts) {
    return (double)(ts->tv_sec + ts->tv_nsec / 1000000000.0);
}

VM3 impedance(Eigen::Matrix3d K, Eigen::Matrix3d D, VM3 X0, VM3 X, VM3 dX, VM3 dXd=VM3::Zero()) {
    return K*(X0-X) + D*(dXd-dX);
}

double JerkIt(VM3 X0, VM3 Xf, double T, double t, VM3 &Xd, VM3 &dXd) {
    t = std::max(std::min(t, T), .0001); //Bound time
    double tn=std::max(std::min(t/T, 1.0), .0);//Normalised time bounded 0-1
    double tn3=pow(tn,3.);
    double tn4=tn*tn3;
    double tn5=tn*tn4;
    Xd = X0 + ( (X0-Xf) * (15.*tn4-6.*tn5-10.*tn3) );
    dXd = (X0-Xf) * (4.*15.*tn4-5.*6.*tn5-10.*3*tn3)/t;
    return tn;
}


void M3CalibState::entryCode(void) {
    calibDone=false;
    for(unsigned int i=0; i<3; i++) {
        stop_reached_time[i] = .0;
        at_stop[i] = false;
    }
    robot->decalibrate();
    robot->initTorqueControl();
    robot->printJointStatus();
    sm->Command = 1;
    sm->Contribution = .0;
    sm->MvtProgress = .0;
    std::cout << "Calibrating (keep clear)..." << std::flush;
}
//Move slowly on each joint until max force detected
void M3CalibState::duringCode(void) {
    VM3 tau(0, 0, 0);

    //Apply constant torque (with damping) unless stop has been detected for more than 0.5s
    VM3 vel=robot->getVelocity();
    double b = 7.;
    for(unsigned int i=0; i<3; i++) {
        tau(i) = std::min(std::max(8 - b * vel(i), .0), 8.);
        if(stop_reached_time(i)>0.5) {
            at_stop[i]=true;
        }
        if(vel(i)<0.01) {
            stop_reached_time(i) += dt();
        }
    }

    //Switch to gravity control when done
    if(robot->isCalibrated()) {
        robot->setEndEffForceWithCompensation(VM3(0,0,0));
        calibDone=true; //Trigger event
    }
    else {
        //If all joints are calibrated
        if(at_stop[0] && at_stop[1] && at_stop[2]) {
            robot->applyCalibration();
            std::cout << "OK." << std::endl;
        }
        else {
            robot->setJointTorque(tau);
            if(iterations()%100==1) {
                std::cout << "." << std::flush;
            }
        }
    }
}
void M3CalibState::exitCode(void) {
    robot->setEndEffForceWithCompensation(VM3(0,0,0));
}


void M3LockState::entryCode(void) {
    //robot->initTorqueControl();
    robot->setEndEffForceWithCompensation(VM3::Zero(), false);
    //robot->initVelocityControl();
    //robot->setJointVelocity(VM3::Zero());

    sm->Command = 5;
    sm->Contribution = .0;
    sm->MvtProgress = .0;

    X0 = robot->getEndEffPosition();
}
void M3LockState::duringCode(void) {
    //robot->setEndEffVelocity(VM3::Zero());

    //Impedance on point
    Eigen::Matrix3d K = k*Eigen::Matrix3d::Identity();
    Eigen::Matrix3d D = d*Eigen::Matrix3d::Identity();
    VM3 Fd = impedance(K, D, X0, robot->getEndEffPosition(), robot->getEndEffVelocity());
    robot->setEndEffForceWithCompensation(Fd, false);


    /*
    //K tuning
    if(robot->keyboard->getS()) {
        k -= 5;
        std::cout << "K=" << k << " D=" << d<< std::endl;
    }
    if(robot->keyboard->getW()) {
        k += 5;
        std::cout << "K=" << k << " D=" << d<< std::endl;
    }

    //D tuning
    if(robot->keyboard->getD()) {
        d -= 1;
        std::cout << "K=" << k << " D=" << d<< std::endl;
    }
    if(robot->keyboard->getA()) {
        d += 1;
        std::cout << "K=" << k << " D=" << d<< std::endl;
    }*/
}
void M3LockState::exitCode(void) {
    //robot->setEndEffVelocity(VM3::Zero());
    robot->setEndEffForceWithCompensation(VM3::Zero(), false);
}


void M3MassCompensation::entryCode(void) {
    //robot->initTorqueControl();
    robot->setEndEffForceWithCompensation(VM3::Zero(), false);
    sm->Command = 2;
    sm->Contribution = .0;
    sm->MvtProgress = .0;
    std::cout << "Press S to decrease mass (-100g), W to increase (+100g)." << std::endl;
}
void M3MassCompensation::duringCode(void) {

    //Bound mass to +-5kg
    if(mass>mass_limit) {
        mass = mass_limit;
    }
    if(mass<-mass_limit) {
        mass = -mass_limit;
    }

    //Calculate effective applied mass based on possible transition (change mass
    applied_mass += sign(mass - applied_mass)*change_mass_rate*dt();

    //Apply corresponding force
    robot->setEndEffForceWithCompensation(VM3(0,0,applied_mass*9.8), true);

    //Mass controllable through keyboard inputs
    if(robot->keyboard->getS()) {
        mass -=0.5;robot->printStatus();
        robot->printJointStatus();
        std::cout << "Mass: " << mass << std::endl;
    }
    if(robot->keyboard->getW()) {
        mass +=0.5;robot->printStatus();
        robot->printJointStatus();
        std::cout << "Mass: " << mass << std::endl;
    }
}
void M3MassCompensation::exitCode(void) {
    robot->setEndEffForceWithCompensation(VM3(0,0,0));
}


void M3PathState::entryCode(void) {
    //robot->initTorqueControl();
    robot->setEndEffForceWithCompensation(VM3::Zero(), false);

    sm->Command = 3;
    sm->Contribution = .0;
    sm->MvtProgress = .0;

    trajPtIdx=0;
    Xi=robot->getEndEffPosition();
    if(trajPts.size()>0) {
        Xf=trajPts[trajPtIdx].X;
    }
    else {
        Xf=robot->getEndEffPosition();
    }

    status=.0;
}
void M3PathState::duringCode(void) {
    VM3 X=robot->getEndEffPosition();
    VM3 dX=robot->getEndEffVelocity();
    VM3 Xd=Xi;
    VM3 Fd(0,0,0);

    //Find closest point on path
    VM3 PathUnitV=(Xf-Xi)/(Xf-Xi).norm();
    Xd = Xi + ( (X-Xi).dot(PathUnitV)*PathUnitV);

    //Progress along path
    double status = sign((X-Xi).dot(PathUnitV))*(Xd-Xi).norm()/(Xf-Xi).norm();
    double b=0;
    //Assistance progressive up and down in first and last 10% of movement
    if(status>=0.1 && status<0.9) {
        b = 1.0;
    }
    if(status<0.1) {
        b = status*10.;
    }
    if(status>0.9) {
        b = (1-status)*10.;
    }
    if(status<=0) {
        //"Before" Xi
        Xd = Xi;
        b = 0.;
    }
    if(status>=1) {
        //After "Xf"
        Xd = Xf;
        b = 0.;
    }

    //Velocity vector projection
    VM3 dX_path = dX.dot(PathUnitV)*PathUnitV;
    VM3 dX_ortho = dX - dX_path;

    //Impedance towards path  point with only ortho velocity component
    Eigen::Matrix3d K = k*Eigen::Matrix3d::Identity();
    Eigen::Matrix3d D = d*Eigen::Matrix3d::Identity();
    Fd = impedance(K, D, Xd, X, dX_ortho);

    //Assitive viscosity in path direction
    Fd+=viscous_assistance*b*dX_path;

    //Apply force with gravity compensation but w/o friction compensation
    robot->setEndEffForceWithCompensation(Fd, false);


    //Have we reached a point? And not currently feeding the pts list
    if(status>=1. && !stop) {
        //Go to next point
        trajPtIdx++;
        if(trajPtIdx>=trajPts.size()){
            trajPtIdx=0;
        }
        //From where we are
        Xi=robot->getEndEffPosition();
        //to next pt (if it exists)
        if(trajPts.size()>0) {
            Xf=trajPts[trajPtIdx].X;
        }
        else {
            Xf=robot->getEndEffPosition();
        }
    }
     sm->MvtProgress = trajPtIdx+status;

    //Display progression
    if(spdlog::get_level()<=spdlog::level::debug) {
        if(iterations()%100==1) {
            std::cout << "Path progress: (Point "<< trajPtIdx << ") |";
            for(int i=0; i<round(status*50.); i++)
                std::cout << "=";
            for(int i=0; i<round((1-status)*50.); i++)
                std::cout << "-";

            std::cout << "| (" << status*100 << "%)";
            robot->printStatus();
        }
    }
}
void M3PathState::exitCode(void) {
    robot->setEndEffForceWithCompensation(VM3::Zero());
}


void M3MinJerkPosition::entryCode(void) {
    //Setup velocity control for position over velocity loop
    /*robot->initVelocityControl();
    robot->setJointVelocity(VM3::Zero());
    k_i=1.;*/
    robot->setEndEffForceWithCompensation(VM3::Zero(), false);

    sm->Command = 4;
    sm->Contribution = .0;
    sm->MvtProgress = .0;

    //Initialise to first target point
    trajPtIdx=0;
    startTime=running();
    Xi=robot->getEndEffPosition();
    if(trajPts.size()>0) {
        Xf=trajPts[trajPtIdx].X;
        T=trajPts[trajPtIdx].T;
    }
    else {
        Xf=robot->getEndEffPosition();
        T=.1;
    }

    status=.0;
}
void M3MinJerkPosition::duringCode(void) {

    VM3 Xd, dXd;
    //Compute current desired interpolated point
    status=JerkIt(Xi, Xf, T, running()-startTime, Xd, dXd);

    /*Apply position control
    robot->setEndEffVelocity(dXd+k_i*(Xd-robot->getEndEffPosition()));*/
    //Impedance on current point
    Eigen::Matrix3d K = k*Eigen::Matrix3d::Identity();
    Eigen::Matrix3d D = d*Eigen::Matrix3d::Identity();
    VM3 Fd = impedance(K, D, Xd, robot->getEndEffPosition(), robot->getEndEffVelocity(), dXd);
    robot->setEndEffForceWithCompensation(Fd, false);

    //Have we reached a point? And not currently feeding the pts list
    if(status>=1. && !stop) {
        //Go to next point
        trajPtIdx++;
        if(trajPtIdx>=trajPts.size()){
            trajPtIdx=0;
        }
        //From where we were
        Xi=Xf;
        //to next pt (if it exists)
        if(trajPts.size()>0) {
            Xf=trajPts[trajPtIdx].X;
            T=trajPts[trajPtIdx].T;
        }
        else {
            Xf=robot->getEndEffPosition();
            T=0;
        }
        startTime=running();
    }
    sm->MvtProgress = trajPtIdx+status;

    //Display progression
    if(spdlog::get_level()<=spdlog::level::debug) {
        if(iterations()%100==1) {
            std::cout << "Progress (Point "<< trajPtIdx << ") |";
            for(int i=0; i<round(status*50.); i++)
                std::cout << "=";
            for(int i=0; i<round((1-status)*50.); i++)
                std::cout << "-";

            std::cout << "| (" << status*100 << "%)  ";
            robot->printStatus();
        }
    }
}
void M3MinJerkPosition::exitCode(void) {
    //robot->setJointVelocity(VM3::Zero());
    robot->setEndEffForceWithCompensation(VM3::Zero());
}
