/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef WHEELVMCTEST_H
#define WHEELVMCTEST_H

#include "FSM/FSMState.h"

class State_WheelVmcTest : public FSMState{
public:
    State_WheelVmcTest(CtrlComponents *ctrlComp);
    ~State_WheelVmcTest(){}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

private:
    void forward_kinematics(legStruct& _legStruct);
    void forward_kinematicsXY(legStruct& _legStruct);
    // float _targetPos[12] = {0.5, 0.35, 0.0, -0.5, -0.35, 0.0, 
    //                         0.0, 0.67, -1.3, 0.0, 0.67, -1.3};

    void VMC(legStruct& _legStruct,float F, float T);
    void VMCXY(legStruct& _legStruct,float x, float y);
    
    float _targetPos[12] = {0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 
                            0.0, 0.67, -1.3, 0.0, 0.67, -1.3};
    
    float _startPos[12];
    float _duration = 1000;   //steps
    float _percent = 0;       //%
    
    float offset = 0.054;
    float l1 = 0.15;
    float l2 = 0.25;

    // float kp_l0 = 100.0;
    // float kd_l0 = 10.0;

    // float kp_theta = 10.0;
    // float kd_theta = 1;

    float kp_l0 = 900.0;
    float kd_l0 = 20.0;

    // float kp_theta = 50.0;
    // float kd_theta = 3;

    float kp_theta = 90.0;
    float kd_theta = 3;


    float kp_x = 900.0;
    float kd_x = 20.0;

    float kp_y = 900.0;
    float kd_y = 20.0;


    // Vec2 theta0; // the angle for virtual leg, and the following leg order is left then right. 
    // Vec2 theta1; 
    // Vec2 theta2;
    // Vec2 l0;

    legStruct left_kin,right_kin;
    legStruct left_kin_virtual,right_kin_virtual;

    float counter;
    float test_phase;
};

#endif  // RLWHEELVMC_H