/**
 * @file zmp_control.h
 * @author Selvakumar H S (franklinselva10@gmail.com)
 * @brief Implementation of ZMP based dynamic balance control for NAO robot; Reference - Introduction to Humanoid Robotics
 * @version 0.1
 * @date 2021-05-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef ZMPCONTROL_H
#define ZMPCONTROL_H

#include <dynamics_param.h>
#include <balance_control.h>

#include <math.h>

/**
 * @brief Implemented the work on ZMP balance using momentum control based on the work proposed in "Introduction to Humanoid Robotics"
 * 
 * 
 */
class ZMPControl : public dPARAM, robot
{
    /**
     * @brief This class calculates the ZMP and check for its position within the support polygon.
     * 
     */
public:
    ZMPControl();
    ZMPControl(std::string robot_IP, int robot_port, int mode);
    ~ZMPControl();

    Eigen::Vector2d getZMP();
    bool withinSP();
    void checkSupportPhase();
    void check_balance_and_move();
    void balance();
    void unbalanced();
    void DS();

    void begin_imitation(float feetdistance, float distancepiedR, float distancepiedL, float dRotation);

protected:
    //Util functions
    std::vector<Eigen::Vector2d> calculateFootCoordinates(Eigen::Vector3d footCenter, float footAngle);

    bool onSegment(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r);
    bool doIntersect(Eigen::Vector2d p1, Eigen::Vector2d q1, Eigen::Vector2d p2, Eigen::Vector2d q2);
    int orientation(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r);

private:
    dPARAM parameters;
    balanceControl bControl;

    Eigen::Vector2d pZMP; //Position of ZMP in 2D
    Eigen::Vector3d pCoM; //Position of CoM
    Eigen::Vector3d aCoM; //Acceleration of CoM

    Eigen::Vector3d pRFoot; //Position of Right Foot
    Eigen::Vector3d pLFoot; //Position of Left Foot

    const float g = -9.81;      //Gravity
    const float footWidth = 1;  // FootWidth along y-axis
    const float footLength = 1; // FootWidth along x-axis
};

#endif