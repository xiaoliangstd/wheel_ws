/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <iostream>
#include "FSM/State_WheelVmcTest.h"

State_WheelVmcTest::State_WheelVmcTest(CtrlComponents *ctrlComp)
                :FSMState(ctrlComp, FSMStateName::WHEELVMCTEST, "wheel vmc test"){}

void State_WheelVmcTest::enter(){
    for(int i=0; i<4; i++){
        if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
            _lowCmd->setSimStanceGain(i);
        }
        else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
            _lowCmd->setRealStanceGain(i);
        }
        _lowCmd->setZeroDq(i);
        _lowCmd->setZeroTau(i);
    }
    for(int i=0; i<12; i++){
        _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
        _startPos[i] = _lowState->motorState[i].q;
    }
    _ctrlComp->setAllStance();

    // for debug
    counter = 0;
}

void State_WheelVmcTest::run()
{
    _lowCmd->motorCmd[2].Kp = 0; 
    _lowCmd->motorCmd[2].Kd = 2; 
    _lowCmd->motorCmd[2].dq = 2; 
    _lowCmd->motorCmd[5].Kp = 0; 
    _lowCmd->motorCmd[5].Kd = 2; 
    _lowCmd->motorCmd[5].dq = -2; 

    left_kin.theta1 = _lowState->motorState[0].q;
    left_kin.theta1_dot = _lowState->motorState[0].dq;
    left_kin.theta2 = _lowState->motorState[1].q + 1.5707;
    left_kin.theta2_dot = _lowState->motorState[1].dq;

    right_kin.theta1 = -_lowState->motorState[3].q;
    right_kin.theta1_dot = -_lowState->motorState[3].dq;
    right_kin.theta2 = -_lowState->motorState[4].q + 1.5707;
    right_kin.theta2_dot = -_lowState->motorState[4].dq;

    forward_kinematics(left_kin); 
    forward_kinematics(right_kin); 
   
    counter = counter + 0.5*_ctrlComp->dt;
    test_phase = fmod(counter,1);

    float dirsed_l0 = 0.2 + 0.1 *sin(2*3.1415926*test_phase);
    float dirsed_theta0 = 0.6 *sin(2*3.1415926*test_phase);
    
    float left_force_leg = kp_l0 * (dirsed_l0 - left_kin.L0) - kd_l0 * left_kin.L0_dot; 
    float left_torque_leg = kp_theta * (dirsed_theta0 - left_kin.theta0) - kd_theta * left_kin.theta0_dot;
    VMC(left_kin,left_force_leg,left_torque_leg);
    _lowCmd->motorCmd[0].Kp = 0; 
    _lowCmd->motorCmd[0].Kd = 0; 
    _lowCmd->motorCmd[0].tau = left_kin.tau1;
    _lowCmd->motorCmd[1].Kp = 0; 
    _lowCmd->motorCmd[1].Kd = 0; 
    _lowCmd->motorCmd[1].tau = left_kin.tau2;

    float dirsed_theta0_right = 0;
    float dirsed_l0_right = 0.2 + 0.1 *sin(2*3.1415926*test_phase);
    float right_force_leg = kp_l0 * (dirsed_l0_right - right_kin.L0) - kd_l0 * right_kin.L0_dot; 
    float right_torque_leg = kp_theta * (dirsed_theta0_right - right_kin.theta0) - kd_theta * right_kin.theta0_dot;
    VMC(right_kin,right_force_leg,right_torque_leg);
    _lowCmd->motorCmd[3].Kp = 0; 
    _lowCmd->motorCmd[3].Kd = 0; 
    _lowCmd->motorCmd[3].tau = -right_kin.tau1;
    _lowCmd->motorCmd[4].Kp = 0; 
    _lowCmd->motorCmd[4].Kd = 0; 
    _lowCmd->motorCmd[4].tau = -right_kin.tau2;
}

void State_WheelVmcTest::VMC(legStruct& _legStruct,float F, float T)
{
    float theta0 = _legStruct.theta0 + M_PI / 2;

    float phi1 = this->l2 * cos(_legStruct.theta1 + _legStruct.theta2 - theta0);
    float phi2 = this->l2 * sin(_legStruct.theta1 + _legStruct.theta2 - theta0);

    float t11 = this->l1 * sin(theta0 - _legStruct.theta1) - phi2;

    float t12 = this->l1 * cos(theta0 - _legStruct.theta1) - phi1;
    t12 = t12 / _legStruct.L0;

    float t21 = -phi2;

    float t22 = -phi1;
    t22 = t22 / _legStruct.L0;

    _legStruct.tau1 = t11 * F - t12 * T;
    _legStruct.tau2 = t21 * F - t22 * T;
}

void State_WheelVmcTest::forward_kinematics(legStruct& _legStruct)
{
    float theta1 = _legStruct.theta1;
    float theta2 = _legStruct.theta2;

    float end_x = offset + l1 * cos(theta1) + l2 * cos(theta1+theta2);
    float end_y = l1 * sin(theta1) + l2 * sin(theta1+theta2);
  
    float L0 = sqrt(pow(end_x,2)+pow(end_y,2));
    float theta0 = atan2(end_y,end_x) - M_PI / 2;

    _legStruct.L0 = L0;
    _legStruct.theta0 = theta0;

    float dt = _ctrlComp->dt;// run at 500 hz
    float theta1_dt = theta1 + _legStruct.theta1_dot*dt;
    float theta2_dt = theta2 + _legStruct.theta2_dot*dt;

    float end_x_dt = offset + l1 * cos(theta1_dt) + l2 * cos(theta1_dt+theta2_dt);
    float end_y_dt = l1 * sin(theta1_dt) + l2 * sin(theta1_dt+theta2_dt);
    
    float L0_dt = sqrt(pow(end_x_dt,2)+pow(end_y_dt,2));
    float theta0_dt = atan2(end_y_dt,end_x_dt) - M_PI / 2;

    _legStruct.L0_dot = (L0_dt - _legStruct.L0) / dt;
    _legStruct.theta0_dot = (theta0_dt - _legStruct.theta0) / dt;
}

void State_WheelVmcTest::forward_kinematicsXY(legStruct& _legStruct)
{
    float theta1 = _legStruct.theta1;
    float theta2 = _legStruct.theta2;

    _legStruct.end_x = offset + l1 * cos(theta1) + l2 * cos(theta1+theta2);
    _legStruct.end_y = l1 * sin(theta1) + l2 * sin(theta1+theta2);

    float dt = _ctrlComp->dt;// run at 500 hz
    float theta1_dt = theta1 + _legStruct.theta1_dot*dt;
    float theta2_dt = theta2 + _legStruct.theta2_dot*dt;

    float end_x_dt = offset + l1 * cos(theta1_dt) + l2 * cos(theta1_dt+theta2_dt);
    float end_y_dt = l1 * sin(theta1_dt) + l2 * sin(theta1_dt+theta2_dt);

    _legStruct.end_x_dot = (end_x_dt - _legStruct.end_x) / dt;
    _legStruct.end_y_dot = (end_y_dt - _legStruct.end_y) / dt;;
}

void State_WheelVmcTest::VMCXY(legStruct& _legStruct,float force_x, float force_y)
{
    float t11 = -this->l1 * sin(_legStruct.theta1) - this->l2 * sin(_legStruct.theta1+_legStruct.theta2);

    float t12 = this->l1 * cos(_legStruct.theta1) + this->l2 * cos(_legStruct.theta1+_legStruct.theta2);

    float t21 = -this->l2 * sin(_legStruct.theta1+_legStruct.theta2);

    float t22 = this->l2 * cos(_legStruct.theta1+_legStruct.theta2);

    _legStruct.tau1 = t11 * force_x + t12 * force_y;
    _legStruct.tau2 = t21 * force_x + t22 * force_y;
}

void State_WheelVmcTest::exit(){
    _percent = 0;
}

FSMStateName State_WheelVmcTest::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else{
        return FSMStateName::WHEELVMCTEST;
    }
}