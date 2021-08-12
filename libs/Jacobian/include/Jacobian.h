//File: Jacobian.h

#ifndef JACOBIAN_H
#define JACOBIAN_H

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
// #include <conio.h>
#include <stdio.h>
#include <sstream>
#include <math.h>
#include <vector>

const float DEG2RAD = M_PI/180;
const float RAD2DEG= 180/M_PI;

class NAO
{
public:
    ///Constructor
    NAO();

    ///Destructor
    ~NAO();

    /// DKM_TORSO
    Eigen::MatrixXd GetJacobianP_LHand(std::vector<float> &q);
    Eigen::MatrixXd GetJacobianP_RHand(std::vector<float> &q);
    Eigen::MatrixXd GetJacobianP_LFoot(std::vector<float> &q);
    Eigen::MatrixXd GetJacobianP_RFoot(std::vector<float> &q);

    Eigen::MatrixXd GetJacobianXY_CoM(std::vector<float> &q);

    /// DGM_TORSO
    Eigen::Vector3d GetP_LHand(std::vector<float> &q);
    Eigen::Vector3d GetP_RHand(std::vector<float> &q);
    Eigen::Vector3d GetP_RFoot(std::vector<float> &q);
    Eigen::Vector3d GetP_LFoot(std::vector<float> &q);

    Eigen::Vector3d GetXYZ_CoM(std::vector<float> &q);

    /// DGM RFoot
    Eigen::Vector3d GetP_LHand_RF(std::vector<float> &q);
    Eigen::Vector3d GetP_RHand_RF(std::vector<float> &q);
    Eigen::Vector3d GetP_LFoot_RF(std::vector<float> &q);
    Eigen::Vector3d GetP_RFoot_RF(std::vector<float> &q);
    Eigen::Vector3d GetP_RFoot_Torso(std::vector<float> &q);

    Eigen::Vector3d GetXYZ_CoM_RF(std::vector<float> &q);

    /// DKM_RFoot

    Eigen::MatrixXd GetJacobianP_LHand_RF(std::vector<float> &q);
    Eigen::MatrixXd GetJacobianP_RHand_RF(std::vector<float> &q);
    Eigen::MatrixXd GetJacobianP_LFoot_RF(std::vector<float> &q);
    Eigen::MatrixXd GetJacobianP_RFoot_RF(std::vector<float> &q);
    Eigen::MatrixXd GetJacobianP_RFoot_Torso(std::vector<float> &q);

    Eigen::MatrixXd GetJacobianXY_CoM_RF(std::vector<float> &q);

    /// RECTIFIED means with a coordinate frame always parallel to the floor. ///
    /// DGM_rectified
    Eigen::VectorXd GetP_LHand_rectified(std::vector<float> &q);
    Eigen::VectorXd GetP_RHand_rectified(std::vector<float> &q);
    Eigen::VectorXd GetP_RFoot_rectified(std::vector<float> &q);
    Eigen::VectorXd GetP_LFoot_rectified(std::vector<float> &q);

    Eigen::Vector3d GetXYZ_CoM_rectified(std::vector<float> &q);
    Eigen::Vector3d GetXYZ_CoM_RF_rectified(std::vector<float> &q);

    /// DKM_rectified
    Eigen::MatrixXd GetJacobianP_LHand_rectified(std::vector<float> &q);
    Eigen::MatrixXd GetJacobianP_RHand_rectified(std::vector<float> &q);
    Eigen::MatrixXd GetJacobianP_LFoot_rectified(std::vector<float> &q);
    Eigen::MatrixXd GetJacobianP_RFoot_rectified(std::vector<float> &q);

    Eigen::MatrixXd GetJacobianP_LHand_RF_rectified(std::vector<float> &q);
    Eigen::MatrixXd GetJacobianP_RHand_RF_rectified(std::vector<float> &q);
    Eigen::MatrixXd GetJacobianP_LFoot_RF_rectified(std::vector<float> &q);
    Eigen::MatrixXd GetJacobianP_RFoot_RF_rectified(std::vector<float> &q);

    Eigen::MatrixXd GetJacobianXY_CoM_rectified(std::vector<float> &q);
    Eigen::MatrixXd GetJacobianXY_CoM_RF_rectified(std::vector<float> &q);

private:
    //AL::ALMemoryProxy *m_memory;
};

#endif
