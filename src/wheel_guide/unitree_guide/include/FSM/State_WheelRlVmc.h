/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef WHEELRLVMC_H
#define WHEELRLVMC_H

#include <thread>
#include <MNN/Interpreter.hpp>
#include "FSM/FSMState.h"
#include "common/obsHistory.h"
#include "common/getWorkingDir.h"


class State_WheelRlVmc : public FSMState{
public:
    State_WheelRlVmc(CtrlComponents *ctrlComp);
    ~State_WheelRlVmc(){}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

private:
    void VMC(legStruct& _legStruct,float F, float T);
    void forward_kinematics(legStruct& _legStruct);

    float _startPos[12];
    
    float offset = 0.054; // x-axis
    float l1 = 0.15;
    float l2 = 0.25;
    float dirsed_l0 = 0.18; // Virtual leg length, can be used to control the height of the robot
    float kp_l0 = 900.0;
    float kd_l0 = 20.0;
    float kp_theta = 90.0;
    float kd_theta = 3;
    float kd_wheel = 0.5;
    float feedforward_force = 40;

    legStruct leg_left,leg_right;

    float counter;
    float test_phase;

    Eigen::VectorXf dof_vel_limits;
    Eigen::VectorXf torque_limits;
    Eigen::VectorXf obs_scales;
    Eigen::VectorXf obs_scaled;
    Eigen::VectorXf act_mean;
    Eigen::VectorXf act_scales;
    Eigen::VectorXf act_scaled;
    Eigen::VectorXf obs_current;
    Eigen::VectorXf act_prev;

    float dt, kp, kd;
    int obs_dim,act_dim , obs_history_steps;
    float clip_actions, clip_obs;

    obsHistory obs_buf;

    void loadPolicy();
    
    std::array<float, 3> gyro;

    RotMat _B2G_RotMat; // the rotation matrix from the body to the world.
    RotMat _G2B_RotMat; // the rotation matrix from the world to the body.

    // Values used to the inferenceThread 
    std::thread _thread;
    long long _startTime;
    bool threadRunning = true;
    void inferenceThreadFunction();

    // MNN
    std::shared_ptr<MNN::Interpreter> net = nullptr;
    MNN::Session* session = nullptr;
    MNN::Tensor *obs_tensor = nullptr;
    MNN::Tensor *act_tensor = nullptr;

    // Helper function
    double periodic_function(double x) 
    {
        double period = 4 * M_PI;
        double y_periodic = std::fmod(x, period);
        if (y_periodic < -2 * M_PI) 
        {
            y_periodic += period;
        } 
        else if (y_periodic > 2 * M_PI) 
        {
            y_periodic -= period;
        }
        return y_periodic;
    }
};

#endif  // WHEELVMCTEST_H 