// panda_controller.h
#ifndef PANDA_CONTROLLER_H
#define PANDA_CONTROLLER_H

#define MAXLINVEL = 0.25;

#include <string>
#include <franka/robot.h>
#include <franka/model.h>
#include <cmath>
#include <iostream>
#include <franka/exception.h>
#include "examples_common.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

class PandaController
{
private:
    std::string ipAddress;
    char mode;  // mode = 1 for controlling robot pose, mode = 0 for retrieving state information, mode = -1 for nothing
    double *posTargetPtr;
    double (*orienTargetPtr)[3];
    double *currState;
    franka::Robot *robot;
    franka::Model *modelPtr;

public:
    PandaController();
    PandaController(std::string ipAddress, char mode, double *posTargetPtr, double orienTargetPtr[][3], double currState[7]);
    void setBehavior();
    void runController(double = 10);
    void getPose();
    void convertPose(const double T[16], double currState[7]);
    static void geomIkin(double*, double*);
    double getRotationVelocity(const franka::RobotState& robotState, const Eigen::Matrix<double,3,3> &finalRotationMat, Eigen::Vector3d &omega);
    void moveToStart(double, double, double);
    void multiplyMat(double arr1[3][3], double arr2[3][3], double arr3[3][3]);
    void setAcceleration(Eigen::Vector3d & currVel, Eigen::Vector3d & desVel, Eigen::Vector3d & outputAccel, double maxVel, double maxAccel);

    void setMode(int mode);
    void velocityMode();

    void pivot();
    void RotateAxis(int axis);
    void RotateJoint(int joint);
};

#endif