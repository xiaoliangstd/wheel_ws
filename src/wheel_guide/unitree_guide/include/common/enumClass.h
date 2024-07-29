/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef ENUMCLASS_H
#define ENUMCLASS_H

#include <iostream>
#include <sstream>

enum class CtrlPlatform{
    GAZEBO,
    REALROBOT,
};

enum class RobotType{
    A1,
    Go1
};

enum class UserCommand{
    // EXIT,
    NONE,
    START,      // trotting
    L2_A,       // fixedStand
    L2_B,       // passive
    L2_X,       // freeStand
#ifdef COMPILE_WITH_MOVE_BASE
    L2_Y,       // move_base
#endif  // COMPILE_WITH_MOVE_BASE
    L1_X,       // balanceTest
    L1_A,       // swingTest
    L1_Y        // stepTest
};

enum class FrameType{
    BODY,
    HIP,
    GLOBAL
};

enum class WaveStatus{
    STANCE_ALL,
    SWING_ALL,
    WAVE_ALL
};

enum class FSMMode{
    NORMAL,
    CHANGE
};

struct legStruct {

    // real
    double theta1,theta2,theta3;
    double theta1_dot,theta2_dot,theta3_dot;
    double tau1,tau2,vel3;

    // just for test
    double end_x,end_y;
    double end_x_dot,end_y_dot;

    // virtual
    double L0;
    double theta0;
    double L0_dot;
    double theta0_dot;
};

enum class FSMStateName{
    // EXIT,
    INVALID,
    PASSIVE,
    FIXEDSTAND,
    FREESTAND,
    TROTTING,
#ifdef COMPILE_WITH_MOVE_BASE
    MOVE_BASE,       // move_base
#endif  // COMPILE_WITH_MOVE_BASE
    BALANCETEST,
    SWINGTEST,
    STEPTEST,
    WHEELVMCTEST,
    WHEELRLVMC
};

#endif  // ENUMCLASS_H