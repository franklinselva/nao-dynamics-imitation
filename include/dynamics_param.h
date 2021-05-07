#ifndef DYNAMICS_PARAM_H
#define DYNAMICS_PARAM_H

#include <Jacobian.h>
#include <vector>

class dPARAM
{
    /**
     * @brief This file holds the dynamic parameters of NAO robot v5 H25. 
     * The parameters are acquired from http://doc.aldebaran.com/2-1/family/robots/masses_robot.html#robot-masses
     * 
     */
public:
    dPARAM();
    ~dPARAM();

    std::vector<float> getMass() const;
    float getTotalMass() const;
    std::vector<Eigen::Vector3d> getCoM() const;
    std::vector<Eigen::Matrix3f> getInertia() const;

    float getMass(uint segment) const;
    Eigen::Vector3d getCoM(uint segment) const;
    Eigen::Matrix3f getInertia(uint segment) const;

    void defMass();
    void defCoM();
    void defInertiaMatrix();

    typedef enum
    {
        HEAD,
        NECK,
        TORSO, //Head Chain
        RIGHT_SHOULDER,
        RIGHT_BICEPS,
        RIGHT_ELBOW,
        RIGHT_FOREARM,
        RIGHT_HAND, //Right Arm Chain
        LEFT_SHOULDER,
        LEFT_BICEPS,
        LEFT_ELBOW,
        LEFT_FOREARM,
        LEFT_HAND, //Left Arm Chain
        RIGHT_PELVIS,
        RIGHT_HIP,
        RIGHT_THIGH,
        RIGHT_TIBIA,
        RIGHT_ANKLE,
        RIGHT_FOOT, //Right Foot Chain
        LEFT_PELVIS,
        LEFT_HIP,
        LEFT_THIGH,
        LEFT_TIBIA,
        LEFT_ANKLE,
        LEFT_FOOT //Left Foot Chain
    } dSegment;

private:
    std::vector<float> Mass;
    float TotalMass;
    std::vector<Eigen::Vector3d> CentreOfMass;
    std::vector<Eigen::Matrix3f> InertiaMatrix;
};

#endif
