/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <iostream>
#include <unistd.h>
#include "common/obsHistory.h"


int main(int argc, char **argv)
{
    int obs_history_length = 5;
    int num_obs = 27;

    obsHistory historyTest = obsHistory(obs_history_length,num_obs);

    Eigen::VectorXf obs;
    obs.setZero(num_obs);
    obs(0) = 4;
    obs(2) = 99;
    historyTest.reset(obs);
    Eigen::VectorXf newVector;
    newVector.setZero(num_obs);
    newVector << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27;
    historyTest.updateMatrix(newVector);
    newVector << 2, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27;
    historyTest.updateMatrix(newVector);
    newVector << 3, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27;
    historyTest.updateMatrix(newVector);
    newVector << 4, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27;
    historyTest.updateMatrix(newVector);
    newVector << 5, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27;
    historyTest.updateMatrix(newVector);
    newVector << 6, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27;
    historyTest.updateMatrix(newVector);
}
