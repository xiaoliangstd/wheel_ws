/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <iostream>
#include "FSM/State_WheelRlVmc.h"

State_WheelRlVmc::State_WheelRlVmc(CtrlComponents *ctrlComp)
                :FSMState(ctrlComp, FSMStateName::WHEELRLVMC, "wheel rl vmc")
{

}

void State_WheelRlVmc::enter()
{
    for(int i=0; i<4; i++)
    {
        if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO)
        {
            _lowCmd->setSimStanceGain(i);
        }
        else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT)
        {
            _lowCmd->setRealStanceGain(i);
        }
        _lowCmd->setZeroDq(i);
        _lowCmd->setZeroTau(i);
    }

    loadPolicy();

    for (int num = 0; num < obs_history_steps; ++num)
    {
        obs_current.setZero(obs_dim);
        for (int i = 0; i < 3; ++i)
        {
            obs_current(i) = _lowState->getGyro()[i];
        }
        _B2G_RotMat = _lowState->getRotMat();
        _G2B_RotMat = _B2G_RotMat.transpose();
        Vec3 projected_gravity_body;
        Vec3 projected_gravity_world;
        projected_gravity_world<< 0,0,-1;
        projected_gravity_body = _G2B_RotMat * projected_gravity_world;
        for (int i = 0; i < 3; ++i)
        {
            obs_current(i + 3) = projected_gravity_body(i);
        }

        _userValue = _lowState->userValue;
        obs_current(6) = _userValue.ly;
        obs_current(7) = -_userValue.lx;
        obs_current(8) = dirsed_l0;

        obs_current(9) = 0.0; // theta0
        obs_current(10) = 0.0;

        obs_current(11) = 0.0; // theta0_dot
        obs_current(12) = 0.0;

        obs_current(13) = 0.0; // L0
        obs_current(14) = 0.0;

        obs_current(15) = 0.0; // L0_dot
        obs_current(16) = 0.0;

        obs_current(17) = 0.0; // wheel_pos
        obs_current(18) = 0.0;

        obs_current(19) = 0.0; // wheel_vel
        obs_current(20) = 0.0;

        act_prev.setZero(); 
        obs_current.segment<6>(21) << act_prev;

        obs_scaled = obs_current.cwiseProduct(obs_scales).cwiseMax(-this->clip_obs).cwiseMin(this->clip_obs);
        obs_buf.insert(obs_scaled);
    }
    threadRunning = true;
    _thread = std::thread(&State_WheelRlVmc::inferenceThreadFunction, this);
}

void State_WheelRlVmc::loadPolicy()
{
    std::string policy_file_path;
	policy_file_path = getWorkingDir() + "/src/wheel_guide/unitree_guide/policy/liang_wheel_Apr12_00-12-43_model_97400.mnn";
    net = std::shared_ptr<MNN::Interpreter>(MNN::Interpreter::createFromFile(policy_file_path.c_str()));
    
    MNN::ScheduleConfig config;
    config.numThread = 2;
    session = net->createSession(config);

    obs_tensor = net->getSessionInput(session, "obs");
    act_tensor = net->getSessionOutput(session, "act");

    // RL
    this->dt = 0.01; // run at 100 hz
    this->clip_actions = 100.0; 
    this->clip_obs = 100.0; 
    
    obs_history_steps = 5;
    obs_dim = obs_tensor->shape().back() / obs_history_steps;
    act_dim = act_tensor->shape().back();
    obs_buf = obsHistory(obs_history_steps,obs_dim);
    obs_scales.setZero(obs_dim);
    obs_scaled.setZero(obs_dim);
    obs_current.setZero(obs_dim);
    act_mean.setZero(act_dim);
    act_scales.setZero(act_dim);
    act_prev.setZero(act_dim);
    act_scaled.setZero(act_dim);
    dof_vel_limits.setZero(act_dim);
    torque_limits.setZero(act_dim);

    // obs scale 27 dimension
    obs_scales(0) = 0.25; // angular vel
    obs_scales(1) = 0.25;
    obs_scales(2) = 0.25;

    obs_scales(3) = 1.0; // gravity force
    obs_scales(4) = 1.0;
    obs_scales(5) = 1.0;

    obs_scales(6) = 2.0; // commands_scale
    obs_scales(7) = 0.25;
    obs_scales(8) = 5.0;

    obs_scales(9) = 1.0; // dof_pos
    obs_scales(10) = 1.0;

    obs_scales(11) = 0.05; // dof_vel
    obs_scales(12) = 0.05;

    obs_scales(13) = 5.0; // l0
    obs_scales(14) = 5.0;

    obs_scales(15) = 0.25; // l0_dot
    obs_scales(16) = 0.25;

    obs_scales(17) = 1.0; // dof_pos
    obs_scales(18) = 1.0;

    obs_scales(19) = 0.05; // dof_vel
    obs_scales(20) = 0.05;

    obs_scales(21) = 1.0; // action
    obs_scales(22) = 1.0;
    obs_scales(23) = 1.0; 
    obs_scales(24) = 1.0;
    obs_scales(25) = 1.0; 
    obs_scales(26) = 1.0;
   
    for(int i=0;i<6;i++)
        act_scales(i) = 1.0;
    
    act_scales(0) = 0.5; // left leg action_scale_theta
    act_scales(1) = 0.1; // left leg action_scale_l0
    act_scales(2) = 10.0; // left leg action_scale_vel
    act_scales(3) = 0.5; // right leg action_scale_theta
    act_scales(4) = 0.1; // right leg action_scale_l0
    act_scales(5) = 10.0; // right leg action_scale_vel

    for(int i=0;i<6;i++)
        act_mean(i) = 0.0;
    act_mean(1) = 0.175;
    act_mean(4) = 0.175;
}

void State_WheelRlVmc::inferenceThreadFunction()
{
    while(threadRunning)
    {
        _startTime = getSystemTime();

        obs_current.setZero(obs_dim);
        for (int i = 0; i < 3; ++i)
        {
            obs_current(i) = _lowState->getGyro()[i];
        }

        _B2G_RotMat = _lowState->getRotMat();
        _G2B_RotMat = _B2G_RotMat.transpose();
        Vec3 projected_gravity_body,projected_gravity_world;
        projected_gravity_world<<0,0,-1;
        projected_gravity_body = _G2B_RotMat * projected_gravity_world;
        for (int i = 0; i < 3; ++i)
        {
            obs_current(i + 3) = projected_gravity_body(i);
        }

        _userValue = _lowState->userValue;
        obs_current(6) = _userValue.ly;
        obs_current(7) = -_userValue.lx;
        obs_current(8) = dirsed_l0;

        obs_current(9) = leg_left.theta0; // theta0
        obs_current(10) = leg_right.theta0;

        obs_current(11) = leg_left.theta0_dot; // theta0_dot
        obs_current(12) = leg_right.theta0_dot;

        obs_current(13) = leg_left.L0; // L0
        obs_current(14) = leg_right.L0;

        obs_current(15) = leg_left.L0_dot; // L0_dot
        obs_current(16) = leg_right.L0_dot;

        obs_current(17) = leg_left.theta3; // wheel_pos
        obs_current(18) = leg_right.theta3;

        obs_current(19) = leg_left.theta3_dot; // wheel_vel
        obs_current(20) = leg_right.theta3_dot;

        obs_current.segment<6>(21)<< act_prev;
        
        obs_scaled = obs_current.cwiseProduct(obs_scales).cwiseMax(-this->clip_obs).cwiseMin(this->clip_obs);
        
        obs_buf.insert(obs_scaled);

        // inference
        Eigen::VectorXf obs = obs_buf.get_obs_vec();  
        for (int i = 0; i < obs_dim * obs_history_steps; ++i)
        {
            obs_tensor->host<float>()[i] = obs(i);
        }

        net->runSession(session);

        for (int i = 0; i < 6; ++i)
        {
            act_prev(i) = act_tensor->host<float>()[i];
        }
        // inference
        act_prev = act_prev.cwiseMax(-this->clip_actions).cwiseMin(this->clip_actions);
        act_scaled = act_prev.cwiseProduct(act_scales) + act_mean;

        absoluteWait(_startTime, (long long)(this->dt * 1000000));
    }
    threadRunning = false;
}

void State_WheelRlVmc::run()
{
    leg_left.theta1 = _lowState->motorState[0].q;
    leg_left.theta1_dot = _lowState->motorState[0].dq;
    leg_left.theta2 = _lowState->motorState[1].q + 1.5707;
    leg_left.theta2_dot = _lowState->motorState[1].dq;
    leg_left.theta3 = periodic_function(_lowState->motorState[2].q); // Restrict the joint angle output to the range of -2pi to 2pi.
    leg_left.theta3_dot = _lowState->motorState[2].dq;

    forward_kinematics(leg_left);
    float leg_torque_left = kp_theta * (act_scaled(0) - leg_left.theta0) - kd_theta * leg_left.theta0_dot;
    float leg_force_left = kp_l0 * (act_scaled(1) - leg_left.L0) - kd_l0 * leg_left.L0_dot; 
    float wheel_torque_left = kd_wheel * (act_scaled(2) - leg_left.theta3_dot);
    
    VMC(leg_left,leg_force_left+feedforward_force,leg_torque_left);
    _lowCmd->motorCmd[0].Kp = 0; 
    _lowCmd->motorCmd[0].Kd = 0;
    _lowCmd->motorCmd[0].tau = leg_left.tau1;
    _lowCmd->motorCmd[1].Kp = 0; 
    _lowCmd->motorCmd[1].Kd = 0; 
    _lowCmd->motorCmd[1].tau = leg_left.tau2;
    _lowCmd->motorCmd[2].Kp = 0; 
    _lowCmd->motorCmd[2].Kd = 0.0; 
    _lowCmd->motorCmd[2].tau = wheel_torque_left;

    leg_right.theta1 = -_lowState->motorState[3].q;
    leg_right.theta1_dot = -_lowState->motorState[3].dq;
    leg_right.theta2 = -_lowState->motorState[4].q + 1.5707;
    leg_right.theta2_dot = -_lowState->motorState[4].dq;
    leg_right.theta3 = periodic_function(_lowState->motorState[5].q); // Restrict the joint angle output to the range of -2pi to 2pi.
    leg_right.theta3_dot = _lowState->motorState[5].dq;

    forward_kinematics(leg_right); 
    float leg_torque_right = kp_theta * (act_scaled(3) - leg_right.theta0) - kd_theta * leg_right.theta0_dot;
    float leg_force_right = kp_l0 * (act_scaled(4)  - leg_right.L0) - kd_l0 * leg_right.L0_dot; 
    float wheel_torque_right = kd_wheel * (act_scaled(5) - leg_right.theta3_dot);
    
    VMC(leg_right,leg_force_right+feedforward_force,leg_torque_right);
    _lowCmd->motorCmd[3].Kp = 0; 
    _lowCmd->motorCmd[3].Kd = 0; 
    _lowCmd->motorCmd[3].tau = -leg_right.tau1;
    _lowCmd->motorCmd[4].Kp = 0; 
    _lowCmd->motorCmd[4].Kd = 0; 
    _lowCmd->motorCmd[4].tau = -leg_right.tau2;
    _lowCmd->motorCmd[5].Kp = 0; 
    _lowCmd->motorCmd[5].Kd = 0.0; 
    _lowCmd->motorCmd[5].tau = wheel_torque_right;
}

void State_WheelRlVmc::forward_kinematics(legStruct& _legStruct)
{
    float theta1 = _legStruct.theta1;
    float theta2 = _legStruct.theta2;

    float end_x = offset + l1 * cos(theta1) + l2 * cos(theta1+theta2);
    float end_y = l1 * sin(theta1) + l2 * sin(theta1+theta2);
  
    float L0 = sqrt(pow(end_x,2)+pow(end_y,2));
    float theta0 = atan2(end_y,end_x) - M_PI / 2;

    _legStruct.L0 = L0;
    _legStruct.theta0 = theta0;

    float dt = _ctrlComp->dt; // run at 500 hz
    float theta1_dt = theta1 + _legStruct.theta1_dot*dt;
    float theta2_dt = theta2 + _legStruct.theta2_dot*dt;

    float end_x_dt = offset + l1 * cos(theta1_dt) + l2 * cos(theta1_dt+theta2_dt);
    float end_y_dt = l1 * sin(theta1_dt) + l2 * sin(theta1_dt+theta2_dt);
    
    float L0_dt = sqrt(pow(end_x_dt,2)+pow(end_y_dt,2));
    float theta0_dt = atan2(end_y_dt,end_x_dt) - M_PI / 2;

    _legStruct.L0_dot = (L0_dt - _legStruct.L0) / dt;
    _legStruct.theta0_dot = (theta0_dt - _legStruct.theta0) / dt;
}

void State_WheelRlVmc::VMC(legStruct& _legStruct,float F, float T)
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

void State_WheelRlVmc::exit()
{
    threadRunning = false;
    _thread.join();
}

FSMStateName State_WheelRlVmc::checkChange()
{
    if(_lowState->userCmd == UserCommand::L2_B)
    {
        return FSMStateName::PASSIVE;
    }
    return FSMStateName::WHEELRLVMC;
}