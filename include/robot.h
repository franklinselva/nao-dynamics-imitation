#ifndef ROBOT_H
#define ROBOT_H


/// Jacobian libraries
#include <Jacobian.h>
#include "xsens_tool.h"


/// Include robot softbank
#include <qi/os.hpp>
#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <alcommon/albroker.h>
#include <boost/shared_ptr.hpp>

class robot
{
public:
    /// CONSTRUCTOR
    robot();
    robot(std::string robot_IP, int robot_port, int mode);

    /// DESTRUCTOR
    ~robot();

    ///FUNCTIONS

    void connect_robot(); /// Connect the robot via a Proxy/Broker --> Done in the constructor with dynamic allocations.

    void def_DOF(); /// Define robot's number of degree of freedom

    void def_joint_names(); /// Define robot's joints names

    void wake_up(); /// Turn on the robot
    void go_to_sleep(); /// Turn off the robot
    void stand_zero(); /// Go to StandZero posture.

    void def_interpreted_robot_angle_wakeup(); /// Define and stock the wake_up joint configuration

    /// Define and stock the wake_up joint configuration (INPUT: xsens' packages)
    void def_interpreted_robot_angle_wakeup_from_xsens_joints(std::vector<joint_buffer> xsens_joint_Npose);

    /// Define and stock the interpreted robot angles (INPUT: xsens' packages)
    void def_interpreted_robot_angle_from_xsens_joints( std::vector<joint_buffer> xsens_joint);

    void def_joint_limits(std::vector<float> joint_limits_max, std::vector<float> joint_limits_min); /// Define robot's joint limits

    /// GETTERS

    int getDOF() const; /// Get the robot's number of degree of freedom

    AL::ALValue get_joint_names() const; /// Get the robot's joints names

    std::vector<float> get_interpreted_robot_angle_from_xsens_joints() const; /// Get the interpreted robot angles

    void check_joint_limits(); /// Check the joint limits and set the robot's configuration (set robot_joint_value)

    AL::ALValue get_joint_values() const; /// Get the joint configuration which is sent to the robot

    std::vector<float> get_interpreted_robot_angle_wakeup() const; /// Get the wake up / N-pose vector joint configuration from the robot.

    void getHeight(float RFoot, float LFoot); /// Get the height of the feet of the human.


    ///SETTER

    void set_security_distances(); /// Set a security distance

    void set_speed(float speed); ///Set robot's speed

    void set_balance(); /// Set robot's balance configuration

    void setHeight(float RFoot, float LFoot); /// Set the height of the feet of the human.

    void setEndEffector(Eigen::Vector3d RHand, Eigen::Vector3d LHand, Eigen::Vector3d RFoot, Eigen::Vector3d LFoot);

    void copy_joint_values(robot host); ///Set robot joint value

    void copy_velocity(robot host);

    void update_velocity(float feetdistance, float distancepiedR, float distancepiedL, float angletete); ///Set velocity if feet are far or head is turned

    /// Begin imitation of Body joints
    void begin_imitation(float rotation_tete, float distancepiedR, float distancepiedL, float speed);

    void interprete_velocity(); ///Check velocity and moove if (movex!=0)or(movetheta!=0) go to stand if both are null and m_moving is true.

    /// Return true if the next robot's configuration is balanced
    void def_CoM_limits();
    void def_FeetHeight(float RFoot_z, float LFoot_z);
    void GetCoM();
    void DGM(Eigen::VectorXd &q_current);
    void DKM(Eigen::VectorXd &q_current);
    void scale(Eigen::MatrixXd Body_segments);
    void scale2();


    /// HQP
    Eigen::VectorXd HQP_solver(Eigen::MatrixXd J, Eigen::VectorXd dx, Eigen::VectorXd q_c);
    Eigen::VectorXd HQP_solver2(Eigen::MatrixXd J, Eigen::VectorXd dx, Eigen::VectorXd q_d);

    ///Close a txt file.
    void closeFile();
    float getData();
    void terminate();

protected:

    //MATH
    const float DEG2RAD = M_PI/180;
    const float RAD2DEG= 180/M_PI;

    //Robot
    const float ROBOT_COM_X = -11.5; //At wake up
    const float ROBOT_COM_Y = 77.5; // At wake up

    const float ROBOT_COM_THRESHOLD_X = 50; /// MAYBE plus or minus 50 (mm) // 37.5
    const float ROBOT_COM_THRESHOLD_Y = 75;

    int m_numberDOF;

    ///connect to SOFTBANK ROBOTICS robots.
    AL::ALMotionProxy *m_motion;
    AL::ALTextToSpeechProxy *m_speech;
    AL::ALRobotPostureProxy *m_posture;
    AL::ALMemoryProxy *m_memory;

    std::ofstream m_BalanceFile;

    Eigen::MatrixXd m_JacobianEndEff;
    Eigen::MatrixXd m_JacobianXY_CoM;
    Eigen::MatrixXd m_JacobianXY_CoM_RF;
    Eigen::VectorXd m_X_current;
    Eigen::VectorXd m_X_desired;
    Eigen::Vector2d m_XY_CoM_current;
    Eigen::Vector2d m_XY_CoM_desired;
    Eigen::VectorXd m_X_Npose;
    Eigen::Vector2d m_X_CoM_Npose;

    Eigen::VectorXd m_q_rectified;
    Eigen::VectorXd m_q_balanced;
    Eigen::MatrixXd m_q_results_byStepsFinal;

    /// Keep track of the Center of Mass position. EQUILIBRUM
    bool m_isBalanced;
    bool m_isRectified;
    bool m_isDS;
    bool m_isRS;
    bool m_isLS;

    std::vector<float> m_security_XY_CoM;
    std::vector<float> m_pos_CoM;
    std::vector<float> m_pos_CoM_mean;
    std::vector<float> m_vel_CoM;
    std::vector<float> m_pos_CoM_wakeup;
    std::vector<float> m_vel_CoM_wakeup;
    std::vector<float> m_q_current;
    std::vector<float> m_q_desired;

    float m_motors_speed;

    std::vector<float> m_interpreted_robot_angle;
    std::vector<float> m_joint_limits_max;
    std::vector<float> m_joint_limits_min;
    std::vector<float> m_interpreted_robot_angle_wakeup;
    std::vector<float> m_velocity_x_y_theta;
    bool m_moving;
    Eigen::Vector2d m_FeetHeight;

    int m_mode;

    /// End effectors
    Eigen::Vector3d m_RHand_d;
    Eigen::Vector3d m_LHand_d;
    Eigen::Vector3d m_RFoot_d;
    Eigen::Vector3d m_LFoot_d;

    std::vector<joint_buffer> m_xsens_joint_Npose;

    /// ROBOTS
    NAO *Nao;

    AL::ALValue m_robot_joint_names;
    AL::ALValue m_robot_joint_values;

    /// Is the robot imitating?
    bool m_imitate;

private:

    ///Parameters
    std::string m_name = "NAO";
    std::string m_robot_IP;
    int m_robot_port;

};

#include <balance_control.h>

#endif
