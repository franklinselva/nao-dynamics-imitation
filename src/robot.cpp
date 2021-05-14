#include <robot.h>
#include <vector>

using namespace Eigen;
using namespace std;

Eigen::MatrixXd pinvRight2(Eigen::MatrixXd &J);
Eigen::VectorXd changeConfigurationModel(std::vector<float> &V);
Eigen::MatrixXd RotationEuler(float roll, float pitch, float yaw);

/**
 * @brief Construct a new robot::robot object for the NAO robot
 * 
 */
robot::robot() : m_name(0), m_robot_IP(0), m_mode(1), m_moving(false), m_motors_speed(0), m_isBalanced(false), m_velocity_x_y_theta(3, 0),
                 m_pos_CoM_wakeup(2, 0), m_vel_CoM_wakeup(2, 0), m_pos_CoM(2, 0), m_vel_CoM(2, 0), m_security_XY_CoM(4, 0), m_q_rectified(26), m_q_balanced(26),
                 m_pos_CoM_mean(2, 0), m_q_current(26, 0), m_q_desired(26, 0)
{
    m_motion = new AL::ALMotionProxy();
    m_speech = new AL::ALTextToSpeechProxy();
    m_posture = new AL::ALRobotPostureProxy();
    m_memory = new AL::ALMemoryProxy();

    Nao = new NAO(); //Jacobian Instance Initialization

    m_motion->setMoveArmsEnabled(false, false);
}

robot::~robot()
{

    delete m_motion;
    delete m_speech;
    delete m_posture;
    delete m_memory;

    delete Nao;
}

/**
 * @brief Construct a new robot::robot object for the NAO robot 
 * 
 * @param robot_IP Provide the IP address of the NAO robot
 * @param robot_port Provide the Port for the NAO robot
 * @param mode (1/2) for Immersion or teleoperation mode respectively
 */
robot::robot(std::string robot_IP, int robot_port, int mode) : m_robot_IP(robot_IP), m_robot_port(robot_port), m_mode(mode), m_moving(false), m_motors_speed(0),
                                                               m_isBalanced(false), m_velocity_x_y_theta(3, 0), m_pos_CoM_wakeup(2, 0), m_vel_CoM_wakeup(2, 0), m_pos_CoM(2, 0), m_vel_CoM(2, 0), m_security_XY_CoM(4, 0),
                                                               m_q_rectified(26), m_q_balanced(26), m_pos_CoM_mean(2, 0), m_q_current(26, 0), m_q_desired(26, 0)
{
    m_motion = new AL::ALMotionProxy(m_robot_IP, m_robot_port);
    m_speech = new AL::ALTextToSpeechProxy(m_robot_IP, m_robot_port);
    m_posture = new AL::ALRobotPostureProxy(m_robot_IP, m_robot_port);
    m_memory = new AL::ALMemoryProxy(m_robot_IP, m_robot_port);

    Nao = new NAO();

    m_motion->setMoveArmsEnabled(false, false);

    if (m_name == "PEPPER")
    {
        std::cout << "PEPPER ROBOT is not supported with this application..." << std::endl;
        exit(2);
    }
}

void robot::def_FeetHeight(float RFoot_z, float LFoot_z)
{
    std::cout << "RFoot height: " << std::endl
              << RFoot_z << std::endl;
    std::cout << "LFoot height: " << std::endl
              << LFoot_z << std::endl;
    //m_FeetHeight << X_current(2), X_current(5); /// RFoot and LFoot height.
}

void robot::def_CoM_limits()
{
    float X_CoM_max, X_CoM_min, Y_CoM_max, Y_CoM_min;

    /// Center of Mass in NPose -- Center of the polygon of balance
    Vector2d XY_CoM_wake_up;
    XY_CoM_wake_up << ROBOT_COM_X, ROBOT_COM_Y;

    X_CoM_max = m_X_CoM_Npose[0] + ROBOT_COM_THRESHOLD_X;
    X_CoM_min = m_X_CoM_Npose[0] - ROBOT_COM_THRESHOLD_X;
    Y_CoM_max = m_X_CoM_Npose[1] + ROBOT_COM_THRESHOLD_Y;
    Y_CoM_min = m_X_CoM_Npose[1] - ROBOT_COM_THRESHOLD_Y;

    m_security_XY_CoM[0] = X_CoM_max;
    m_security_XY_CoM[1] = X_CoM_min;
    m_security_XY_CoM[2] = Y_CoM_max;
    m_security_XY_CoM[3] = Y_CoM_min;
}

void robot::setEndEffector(Eigen::Vector3d RHand, Eigen::Vector3d LHand, Eigen::Vector3d RFoot, Eigen::Vector3d LFoot)
{
    m_RHand_d = RHand;
    m_LHand_d = LHand;
    m_RFoot_d = RFoot;
    m_LFoot_d = LFoot;

    Eigen::VectorXd X_desired(12);

    X_desired << m_RHand_d,
        m_LHand_d,
        m_RFoot_d,
        m_LFoot_d;

    m_X_desired = X_desired;
}

void robot::def_DOF()
{
    if (m_name == "NAO")
    {
        m_numberDOF = 26;
    }
}

void robot::def_joint_names()
{
    m_robot_joint_names.arraySetSize(m_numberDOF);

    // "Head" chain
    m_robot_joint_names[0] = "HeadYaw";
    m_robot_joint_names[1] = "HeadPitch";
    // "LArm" chain
    m_robot_joint_names[2] = "LShoulderPitch";
    m_robot_joint_names[3] = "LShoulderRoll";
    m_robot_joint_names[4] = "LElbowYaw";
    m_robot_joint_names[5] = "LElbowRoll";
    m_robot_joint_names[6] = "LWristYaw";
    m_robot_joint_names[7] = "LHand";
    // "LLeg" chain
    m_robot_joint_names[8] = "LHipYawPitch";
    m_robot_joint_names[9] = "LHipRoll";
    m_robot_joint_names[10] = "LHipPitch";
    m_robot_joint_names[11] = "LKneePitch";
    m_robot_joint_names[12] = "LAnklePitch";
    m_robot_joint_names[13] = "LAnkleRoll";
    // "RLeg" chain
    m_robot_joint_names[14] = "RHipYawPitch";
    m_robot_joint_names[15] = "RHipRoll";
    m_robot_joint_names[16] = "RHipPitch";
    m_robot_joint_names[17] = "RKneePitch";
    m_robot_joint_names[18] = "RAnklePitch";
    m_robot_joint_names[19] = "RAnkleRoll";
    // "RArm" chain
    m_robot_joint_names[20] = "RShoulderPitch";
    m_robot_joint_names[21] = "RShoulderRoll";
    m_robot_joint_names[22] = "RElbowYaw";
    m_robot_joint_names[23] = "RElbowRoll";
    m_robot_joint_names[24] = "RWristYaw";
    m_robot_joint_names[25] = "RHand";
}

void robot::go_to_sleep()
{
    m_motion->rest();
}

void robot::wake_up()
{
    if (!m_motion->robotIsWakeUp())
        m_motion->wakeUp();
    else
        m_posture->goToPosture("Stand", 0.2F);
}

void robot::stand_zero()
{
    /// Go to StandZero posture
    m_posture->goToPosture("StandZero", 0.2F);
}

void robot::def_interpreted_robot_angle_wakeup()
{
    std::vector<float> interpreted_robot_angle_wakeup(m_numberDOF, 0);

    interpreted_robot_angle_wakeup = m_motion->getAngles(m_robot_joint_names, false);
    m_q_current = interpreted_robot_angle_wakeup;
    Eigen::VectorXd q(m_numberDOF);
    for (int i = 0; i < m_numberDOF; i++)
        q[i] = m_q_current[i];
    //m_X_Npose << 0,0,0,0,0,0; /// INITIALIZATION.
    DGM(q);
    m_X_Npose = m_X_current;
    m_X_CoM_Npose = m_XY_CoM_current;

    std::cout << "Position of Feet wrt RF (NPose): " << std::endl
              << m_X_Npose << std::endl;
    std::cout << "Position of CoM wrt RF (NPose): " << std::endl
              << m_X_CoM_Npose << std::endl;

    for (int i = 0; i < m_numberDOF; i++)
    {
        interpreted_robot_angle_wakeup[i] = interpreted_robot_angle_wakeup[i] * RAD2DEG;
    }

    m_interpreted_robot_angle_wakeup = interpreted_robot_angle_wakeup;
    m_interpreted_robot_angle = m_interpreted_robot_angle_wakeup;
}

void robot::def_interpreted_robot_angle_wakeup_from_xsens_joints(std::vector<joint_buffer> xsens_joint_Npose)
{
    m_xsens_joint_Npose.clear();
    // Receive new pose
    m_xsens_joint_Npose = xsens_joint_Npose;
}

void robot::def_interpreted_robot_angle_from_xsens_joints(std::vector<joint_buffer> xsens_joint)
{
    std::vector<float> interpreted_robot_angle(m_numberDOF, 0);

    // TEST
    int A = 40; //A and B are the Xsens angle range for the Hands caption so L and RHand
    int B = 30;

    float coef_shoulder = 2;

    if (m_name == "NAO" && m_mode == 2)
    {

        //HEAD
        // HeadYaw
        interpreted_robot_angle[0] = xsens_joint[5].rotation_z - m_xsens_joint_Npose[5].rotation_z + m_interpreted_robot_angle_wakeup[0];

        // HeadPitch
        interpreted_robot_angle[1] = xsens_joint[5].rotation_y - m_xsens_joint_Npose[5].rotation_y + m_interpreted_robot_angle_wakeup[1];

        // LShoulderPitch
        interpreted_robot_angle[2] = (xsens_joint[11].rotation_y - m_xsens_joint_Npose[11].rotation_y) + m_interpreted_robot_angle_wakeup[2];
        if (interpreted_robot_angle[2] < 0)
            interpreted_robot_angle[2] = interpreted_robot_angle[2] * coef_shoulder;

        // LShoulderRoll
        interpreted_robot_angle[3] = xsens_joint[11].rotation_x - m_xsens_joint_Npose[11].rotation_x + m_interpreted_robot_angle_wakeup[3];

        // LElbowYaw
        interpreted_robot_angle[4] = (xsens_joint[11].rotation_z - m_xsens_joint_Npose[11].rotation_z) + m_interpreted_robot_angle_wakeup[4];

        // LElbowRoll
        interpreted_robot_angle[5] = (xsens_joint[12].rotation_x - m_xsens_joint_Npose[12].rotation_x) + m_interpreted_robot_angle_wakeup[5];

        //LWristYaw
        interpreted_robot_angle[6] = (xsens_joint[12].rotation_z - m_xsens_joint_Npose[12].rotation_z) + m_interpreted_robot_angle_wakeup[6];

        //LHand
        interpreted_robot_angle[7] = (((xsens_joint[13].rotation_y - m_xsens_joint_Npose[13].rotation_y) + A) / (A + B)) * RAD2DEG;

        //LHipYawPitch
        interpreted_robot_angle[8] = +m_interpreted_robot_angle_wakeup[8];

        //LHipRoll
        interpreted_robot_angle[9] = (xsens_joint[18].rotation_x - m_xsens_joint_Npose[18].rotation_x) + m_interpreted_robot_angle_wakeup[9];
        //        (xsens_joint[0].rotation_x - m_xsens_joint_Npose[0].rotation_x)
        //                                        + (xsens_joint[1].rotation_x - m_xsens_joint_Npose[1].rotation_x)
        //                                        + (xsens_joint[2].rotation_x - m_xsens_joint_Npose[2].rotation_x)
        //                                        + (xsens_joint[3].rotation_x - m_xsens_joint_Npose[3].rotation_x)+ m_interpreted_robot_angle_wakeup[9];
        //LHipPitch
        interpreted_robot_angle[10] = (xsens_joint[18].rotation_y - m_xsens_joint_Npose[18].rotation_y) + m_interpreted_robot_angle_wakeup[10];
        //+ m_interpreted_robot_angle_wakeup[10];

        //LKneePitch
        interpreted_robot_angle[11] = (xsens_joint[19].rotation_y - m_xsens_joint_Npose[19].rotation_y) + m_interpreted_robot_angle_wakeup[11];

        //LAnklePitch
        interpreted_robot_angle[12] = (xsens_joint[20].rotation_y - m_xsens_joint_Npose[20].rotation_y)
                                      //(xsens_joint[19].rotation_y - m_xsens_joint_Npose[19].rotation_y)
                                      + m_interpreted_robot_angle_wakeup[12];

        //LAnkleRoll
        interpreted_robot_angle[13] = (xsens_joint[20].rotation_x - m_xsens_joint_Npose[20].rotation_x) + m_interpreted_robot_angle_wakeup[13];

        //Right
        //RHipYawPitch
        interpreted_robot_angle[14] = +m_interpreted_robot_angle_wakeup[14];

        //RHipRoll
        interpreted_robot_angle[15] = (xsens_joint[14].rotation_x - m_xsens_joint_Npose[14].rotation_x) + m_interpreted_robot_angle_wakeup[15];
        //        (xsens_joint[0].rotation_x - m_xsens_joint_Npose[0].rotation_x)
        //                                        + (xsens_joint[1].rotation_x - m_xsens_joint_Npose[1].rotation_x)
        //                                        + (xsens_joint[2].rotation_x - m_xsens_joint_Npose[2].rotation_x)
        //                                        + (xsens_joint[3].rotation_x - m_xsens_joint_Npose[3].rotation_x)+ m_interpreted_robot_angle_wakeup[15];

        //RHipPitch
        interpreted_robot_angle[16] = (xsens_joint[14].rotation_y - m_xsens_joint_Npose[14].rotation_y) + m_interpreted_robot_angle_wakeup[16];

        //RKneePitch
        interpreted_robot_angle[17] = (xsens_joint[15].rotation_y - m_xsens_joint_Npose[15].rotation_y) + m_interpreted_robot_angle_wakeup[17];

        //RAnklePitch
        interpreted_robot_angle[18] = (xsens_joint[16].rotation_y - m_xsens_joint_Npose[16].rotation_y)
                                      //(xsens_joint[15].rotation_y - m_xsens_joint_Npose[15].rotation_y)
                                      + m_interpreted_robot_angle_wakeup[18];

        //RAnkleRoll
        interpreted_robot_angle[19] = (xsens_joint[16].rotation_x - m_xsens_joint_Npose[16].rotation_x) + m_interpreted_robot_angle_wakeup[19];

        //RShoulderPitch
        interpreted_robot_angle[20] = (xsens_joint[7].rotation_y - m_xsens_joint_Npose[7].rotation_y) + m_interpreted_robot_angle_wakeup[20];
        if (interpreted_robot_angle[20] < 0)
            interpreted_robot_angle[20] = interpreted_robot_angle[20] * coef_shoulder;

        //RShoulderRoll
        interpreted_robot_angle[21] = (xsens_joint[7].rotation_x - m_xsens_joint_Npose[7].rotation_x) + m_interpreted_robot_angle_wakeup[21];

        //RElbowYaw
        interpreted_robot_angle[22] = (xsens_joint[7].rotation_z - m_xsens_joint_Npose[7].rotation_z) + m_interpreted_robot_angle_wakeup[22];
        //xsens_joint[9].rotation_x - m_xsens_joint_Npose[9].rotation_x

        //RElbowRoll
        interpreted_robot_angle[23] = xsens_joint[8].rotation_x - m_xsens_joint_Npose[8].rotation_x + m_interpreted_robot_angle_wakeup[23];

        //RWristYaw
        interpreted_robot_angle[24] = (xsens_joint[8].rotation_z - m_xsens_joint_Npose[8].rotation_z) + m_interpreted_robot_angle_wakeup[24];

        //RHand
        interpreted_robot_angle[25] = (((xsens_joint[9].rotation_y - m_xsens_joint_Npose[9].rotation_y) + A) / (A + B)) * RAD2DEG;
    }
    else if (m_name == "NAO" && m_mode == 1)
    {

        //HEAD
        // HeadYaw
        interpreted_robot_angle[0] = xsens_joint[5].rotation_z - m_xsens_joint_Npose[5].rotation_z + m_interpreted_robot_angle_wakeup[0];

        // HeadPitch
        interpreted_robot_angle[1] = xsens_joint[5].rotation_y - m_xsens_joint_Npose[5].rotation_y + m_interpreted_robot_angle_wakeup[1];

        // LShoulderPitch
        interpreted_robot_angle[2] = (xsens_joint[11].rotation_y - m_xsens_joint_Npose[11].rotation_y) + m_interpreted_robot_angle_wakeup[2];
        if (interpreted_robot_angle[2] < 0)
            interpreted_robot_angle[2] = interpreted_robot_angle[2] * coef_shoulder;

        // LShoulderRoll
        interpreted_robot_angle[3] = xsens_joint[11].rotation_x - m_xsens_joint_Npose[11].rotation_x + m_interpreted_robot_angle_wakeup[3];

        // LElbowYaw
        interpreted_robot_angle[4] = (xsens_joint[11].rotation_z - m_xsens_joint_Npose[11].rotation_z) + m_interpreted_robot_angle_wakeup[4];

        // LElbowRoll
        interpreted_robot_angle[5] = (xsens_joint[12].rotation_x - m_xsens_joint_Npose[12].rotation_x) + m_interpreted_robot_angle_wakeup[5];

        //LWristYaw
        interpreted_robot_angle[6] = (xsens_joint[12].rotation_z - m_xsens_joint_Npose[12].rotation_z) + m_interpreted_robot_angle_wakeup[6];

        //LHand
        interpreted_robot_angle[7] = (((xsens_joint[13].rotation_y - m_xsens_joint_Npose[13].rotation_y) + A) / (A + B)) * RAD2DEG;

        //LHipYawPitch
        interpreted_robot_angle[8] = +m_interpreted_robot_angle_wakeup[8];

        //LHipRoll
        interpreted_robot_angle[9] = +m_interpreted_robot_angle_wakeup[9];

        //LHipPitch
        interpreted_robot_angle[10] = +m_interpreted_robot_angle_wakeup[10];

        //LKneePitch
        interpreted_robot_angle[11] = +m_interpreted_robot_angle_wakeup[11];

        //LAnklePitch
        interpreted_robot_angle[12] = +m_interpreted_robot_angle_wakeup[12];

        //LAnkleRoll
        interpreted_robot_angle[13] = +m_interpreted_robot_angle_wakeup[13];

        //Right
        //RHipYawPitch
        interpreted_robot_angle[14] = +m_interpreted_robot_angle_wakeup[14];

        //RHipRoll
        interpreted_robot_angle[15] = +m_interpreted_robot_angle_wakeup[15];

        //RHipPitch
        interpreted_robot_angle[16] = +m_interpreted_robot_angle_wakeup[16];

        //RKneePitch
        interpreted_robot_angle[17] = +m_interpreted_robot_angle_wakeup[17];

        //RAnklePitch
        interpreted_robot_angle[18] = +m_interpreted_robot_angle_wakeup[18];

        //RAnkleRoll
        interpreted_robot_angle[19] = +m_interpreted_robot_angle_wakeup[19];

        //RShoulderPitch
        interpreted_robot_angle[20] = (xsens_joint[7].rotation_y - m_xsens_joint_Npose[7].rotation_y) + m_interpreted_robot_angle_wakeup[20];
        if (interpreted_robot_angle[20] < 0)
            interpreted_robot_angle[20] = interpreted_robot_angle[20] * coef_shoulder;

        //RShoulderRoll
        interpreted_robot_angle[21] = (xsens_joint[7].rotation_x - m_xsens_joint_Npose[7].rotation_x) + m_interpreted_robot_angle_wakeup[21];

        //RElbowYaw
        interpreted_robot_angle[22] = (xsens_joint[7].rotation_z - m_xsens_joint_Npose[7].rotation_z) + m_interpreted_robot_angle_wakeup[22];
        //xsens_joint[9].rotation_x - m_xsens_joint_Npose[9].rotation_x

        //RElbowRoll
        interpreted_robot_angle[23] = xsens_joint[8].rotation_x - m_xsens_joint_Npose[8].rotation_x + m_interpreted_robot_angle_wakeup[23];

        //RWristYaw
        interpreted_robot_angle[24] = (xsens_joint[8].rotation_z - m_xsens_joint_Npose[8].rotation_z) + m_interpreted_robot_angle_wakeup[24];

        //RHand
        interpreted_robot_angle[25] = (((xsens_joint[9].rotation_y - m_xsens_joint_Npose[9].rotation_y) + A) / (A + B)) * RAD2DEG;
    }

    m_interpreted_robot_angle = interpreted_robot_angle;
}

void robot::def_joint_limits(std::vector<float> joint_limits_max, std::vector<float> joint_limits_min)
{
    AL::ALMotionProxy motion(m_robot_IP, 9559);
    for (int i = 0; i < m_numberDOF; i++)
    {
        AL::ALValue limits = m_motion->getLimits(m_robot_joint_names[i]);
        joint_limits_min[i] = limits[0][0];
        joint_limits_max[i] = limits[0][1];
    }
    m_joint_limits_min = joint_limits_min;
    m_joint_limits_max = joint_limits_max;
}

void robot::check_joint_limits()
{
    m_robot_joint_values.arraySetSize(m_numberDOF);

    //calibration for the shoulder pitch so that it does not go beyond -100° which is not a natural movement
    m_joint_limits_min[3] = -100 * DEG2RAD;
    m_joint_limits_min[8] = -100 * DEG2RAD;

    for (int i = 0; i < m_numberDOF; i++)
    {
        m_interpreted_robot_angle[i] = round(m_interpreted_robot_angle[i] * 20) / 20 * DEG2RAD; //we give a precision of 0.05 degrees*********************************
        if (m_interpreted_robot_angle[i] > m_joint_limits_max[i])
            m_interpreted_robot_angle[i] = m_joint_limits_max[i];
        if (m_interpreted_robot_angle[i] < m_joint_limits_min[i])
            m_interpreted_robot_angle[i] = m_joint_limits_min[i];
        //put q_human data into an al::value element to be sent to the robot
        m_robot_joint_values[i] = m_interpreted_robot_angle[i];
    }
}

int robot::getDOF() const
{
    return m_numberDOF;
}

AL::ALValue robot::get_joint_names() const
{
    return m_robot_joint_names;
}

std::vector<float> robot::get_interpreted_robot_angle_from_xsens_joints() const
{
    return m_interpreted_robot_angle;
}

AL::ALValue robot::get_joint_values() const
{
    return m_robot_joint_values;
}

std::vector<float> robot::get_interpreted_robot_angle_wakeup() const
{
    return m_interpreted_robot_angle_wakeup;
}

void robot::set_security_distances()
{
    if (m_name == "PEPPER")
    {
        m_motion->setOrthogonalSecurityDistance(0.2);  //Security distance in front of Pepper 0.2 m
        m_motion->setTangentialSecurityDistance(0.05); //Lateral Security distance 5 cm (allow Pepper to pass the door)
    }
}

void robot::set_speed(float speed)
{
    m_motors_speed = speed;
}

void robot::copy_joint_values(robot host)
{
    m_robot_joint_values = host.m_robot_joint_values;
}

void robot::copy_velocity(robot host)
{
    m_velocity_x_y_theta = host.m_velocity_x_y_theta;
}

void robot::update_velocity(float feetdistance, float distancepiedR, float distancepiedL, float rotation_tete)
{
    ///INITIALIZE CONST
    /// Minimum & maximum distances between feet to move along x (mm).
    //const float min_deplacement=0.3;
    //const float max_deplacement=0.6;
    const float min_deplacement = 250;
    const float max_deplacement = 400;

    /// Minimum and maximum rotation of the head to rotate along z.
    //    const float threshold_rotation_deg=30 ;   //Sets the minimum angle of the head to trigger torso rotation
    //    const float max_rotation_deg=45;
    const float threshold_rotation_deg = 20; //Sets the minimum angle of the head to trigger torso rotation
    const float max_rotation_deg = 45;

    /// Minimum & maximum velocities alowed.
    float v_max;
    float v_min;

    /// Velocities of robot. We only allow displacements along x and rotation around himself (angular velocity along z).
    //    float movex=0;
    float movetheta = 0;

    //    float r; /// Ratio

    if (feetdistance > max_deplacement) // TEST
        m_imitate = false;
    std::cout << "\n [WARN] Imitation is disabled due since feet distance is greater than max deplacement" << std::endl;
    robot::stand_zero();

    if (m_name == "NAO" && m_mode == 1)
    {

        v_max = 0.15;
        v_min = 0.1;

        rotation_tete = rotation_tete + m_interpreted_robot_angle_wakeup[0]; /// Calibration of the head z rotation with the wakeup pose.

        /// Moving forwards.
        if (distancepiedR > min_deplacement)
        {
            m_moving = true;
            m_velocity_x_y_theta[0] = v_max;
            m_velocity_x_y_theta[1] = 0;
            m_velocity_x_y_theta[2] = 0;
            m_motion->move(m_velocity_x_y_theta[0], m_velocity_x_y_theta[1], m_velocity_x_y_theta[2]);
            std::cout << "Moving forwards" << std::endl;
            std::cout << "movex: " << m_velocity_x_y_theta[0] << std::endl
                      << "movey: " << m_velocity_x_y_theta[1] << std::endl
                      << "movetheta: " << m_velocity_x_y_theta[2] << std::endl;
        }

        /// Moving backwards.
        else if (distancepiedL > min_deplacement)
        {
            m_moving = true;
            m_velocity_x_y_theta[0] = -v_min;
            m_velocity_x_y_theta[1] = 0;
            m_velocity_x_y_theta[2] = 0;
            m_motion->move(m_velocity_x_y_theta[0], m_velocity_x_y_theta[1], m_velocity_x_y_theta[2]);
            std::cout << "Moving backwards" << std::endl;
            std::cout << "movex: " << m_velocity_x_y_theta[0] << std::endl
                      << "movey: " << m_velocity_x_y_theta[1] << std::endl
                      << "movetheta: " << m_velocity_x_y_theta[2] << std::endl;
        }

        /// Rotating.
        else if (abs(rotation_tete) > threshold_rotation_deg)
        {
            m_moving = true;
            if (rotation_tete > 0)
                movetheta = 0.2 + ((rotation_tete - threshold_rotation_deg) / (max_rotation_deg - threshold_rotation_deg));
            else
                movetheta = -0.2 + ((rotation_tete + threshold_rotation_deg) / (max_rotation_deg - threshold_rotation_deg));
            m_velocity_x_y_theta[0] = 0;
            m_velocity_x_y_theta[1] = 0;
            m_velocity_x_y_theta[2] = movetheta;
            m_motion->move(m_velocity_x_y_theta[0], m_velocity_x_y_theta[1], m_velocity_x_y_theta[2]);
            std::cout << "Rotating" << std::endl;
            std::cout << "movex: " << m_velocity_x_y_theta[0] << std::endl
                      << "movey: " << m_velocity_x_y_theta[1] << std::endl
                      << "movetheta: " << m_velocity_x_y_theta[2] << std::endl;
        }

        /// STOP.
        if (distancepiedL < min_deplacement && distancepiedR < min_deplacement && abs(rotation_tete) < threshold_rotation_deg && m_moving)
        {
            m_velocity_x_y_theta[0] = 0;
            m_velocity_x_y_theta[1] = 0;
            m_velocity_x_y_theta[2] = 0;
            m_motion->move(m_velocity_x_y_theta[0], m_velocity_x_y_theta[1], m_velocity_x_y_theta[2]);
            wake_up();
            std::cout << "STOP!!" << std::endl;
            m_moving = false;
        }
    }
}

void robot::interprete_velocity()
{
    if ((m_velocity_x_y_theta[0] != 0) | (m_velocity_x_y_theta[2] != 0))
    {
        m_motion->move(m_velocity_x_y_theta[0], m_velocity_x_y_theta[1], m_velocity_x_y_theta[2]);
        m_moving = true;
        std::cout << "WE ARE MOVING !"
                  << " is moving? " << m_moving << " movex= " << m_velocity_x_y_theta[0] << "movetheta= " << m_velocity_x_y_theta[2] << std::endl;
        std::cout << "m_velocity_x_y_theta: " << std::endl
                  << m_velocity_x_y_theta << std::endl;
        std::cout << m_motion->moveIsActive() << std::endl;
    }
    else if ((m_velocity_x_y_theta[0] == 0) && (m_velocity_x_y_theta[2] == 0) && m_moving)
    //else if (m_moving)
    //else
    {
        //m_motion->move(m_velocity_x_y_theta[0],m_velocity_x_y_theta[1],m_velocity_x_y_theta[2]);
        m_motion->move(0, 0, 0);
        std::cout << "STOP!!!!" << std::endl;
        std::cout << m_motion->moveIsActive() << std::endl;

        if (m_name == "NAO")
        {
            while (!m_posture->goToPosture("Stand", 0.2))
            {
                //wake_up();
            }
            std::cout << "WOKE UP" << std::endl;
        }

        m_moving = false;
    }
    else
        std::cout << m_motion->moveIsActive() << std::endl;
}

void robot::closeFile()
{
    m_BalanceFile.close();
}

// float robot::getData()
// {
//     float _data;
//     _data = m_memory->getData("Device/SubDeviceList/Platform/Front/Sonar/Sensor/Value");
//     return _data;
// }

void robot::scale2()
{
    std::vector<float> q = m_robot_joint_values;

    Eigen::Vector3d RHand;
    Eigen::Vector3d LHand;
    Eigen::Vector3d RFoot;
    Eigen::Vector3d LFoot;

    RHand = Nao->GetP_RHand(q);
    LHand = Nao->GetP_LHand(q);
    RFoot = Nao->GetP_RFoot(q);
    LFoot = Nao->GetP_LFoot(q);

    setEndEffector(RHand, LHand, RFoot, LFoot);
}

void robot::DGM(Eigen::VectorXd &q_current)
{
    if (m_name == "NAO")
    {
        std::vector<float> q(m_numberDOF, 0);

        for (int i = 0; i < m_numberDOF; i++)
        {
            q[i] = q_current[i];
        }

        Eigen::Vector3d X_current_RHand, X_current_LHand, X_current_RFoot, X_current_LFoot, X_current_RFoot_Torso;
        Eigen::Vector3d XYZ_CoM, XYZ_CoM_RF;
        Eigen::VectorXd X_current(6);

        X_current_RHand = Nao->GetP_RHand_RF(q);
        X_current_LHand = Nao->GetP_LHand_RF(q);
        X_current_RFoot = Nao->GetP_RFoot_RF(q);
        X_current_LFoot = Nao->GetP_LFoot_RF(q);
        X_current_RFoot_Torso = Nao->GetP_RFoot_Torso(q);

        XYZ_CoM_RF = Nao->GetXYZ_CoM_RF(q); /// RFoot FRAME

        /// To RFOOT frame
        m_XY_CoM_current[0] = XYZ_CoM_RF[0];
        m_XY_CoM_current[1] = XYZ_CoM_RF[1];

        X_current << X_current_RFoot, X_current_LFoot;

        m_X_current = X_current;
    }
}

void robot::DKM(Eigen::VectorXd &q_current)
{
    std::vector<float> q(m_numberDOF, 0);

    for (int i = 0; i < m_numberDOF; i++)
    {
        q[i] = q_current[i];
    }

    Eigen::MatrixXd Jacobian_RHand(3, m_numberDOF), Jacobian_LHand(3, m_numberDOF), Jacobian_RFoot(3, m_numberDOF), Jacobian_LFoot(3, m_numberDOF), Jacobian_RFoot_Torso(3, m_numberDOF);
    Eigen::MatrixXd JacobianEndEff(6, m_numberDOF), JacobianXY_CoM(2, m_numberDOF), JacobianXY_CoM_RF(2, m_numberDOF);

    Jacobian_RHand = Nao->GetJacobianP_RHand_RF(q);
    Jacobian_LHand = Nao->GetJacobianP_LHand_RF(q);
    Jacobian_RFoot = Nao->GetJacobianP_RFoot_RF(q);
    Jacobian_LFoot = Nao->GetJacobianP_LFoot_RF(q);
    Jacobian_RFoot_Torso = Nao->GetJacobianP_RFoot_Torso(q);

    JacobianXY_CoM_RF = Nao->GetJacobianXY_CoM_RF(q);
    m_JacobianXY_CoM_RF = JacobianXY_CoM_RF;

    JacobianEndEff << Jacobian_RFoot,
        Jacobian_LFoot;

    m_JacobianEndEff = JacobianEndEff;
}

Eigen::MatrixXd RotationEuler(float roll, float pitch, float yaw)
{
    Eigen::MatrixXd Rx(3, 3), Ry(3, 3), Rz(3, 3), R(3, 3);

    Rx << 1, 0, 0,
        0, cos(roll), -sin(roll),
        0, sin(roll), cos(roll);

    Ry << cos(pitch), 0, sin(pitch),
        0, 1, 0,
        -sin(pitch), 0, cos(pitch);

    Rz << cos(yaw), -sin(yaw), 0,
        sin(yaw), cos(yaw), 0,
        0, 0, 1;

    R = Rz * Ry;
    R = R * Rx;

    return R;
}

Eigen::VectorXd robot::HQP_solver(Eigen::MatrixXd J, Eigen::VectorXd dx, Eigen::VectorXd q_c)
{
    /// Solving with Quadratic Programming : * ///
    /// ************ min ||Jdq-dx||2 ********* ///
    /// ********* st dq- <= dq <= dq+ ******** ///
    /// and with prioritization of taks. ***** ///

    USING_NAMESPACE_QPOASES

    Eigen::VectorXd dq(m_numberDOF);

    int_t n_V = m_numberDOF; /// number of variables = num_DoF
    int_t n_C = 1;           /// number of constraint.
    int_t H_rows = n_V;
    int_t H_cols = n_V;
    int_t H_size = H_rows * H_cols;

    Eigen::MatrixXd P(m_numberDOF, m_numberDOF);
    P = 2 * J.transpose() * J;

    Eigen::Map<Eigen::VectorXd> P_vectorized(P.data(), P.size());

    real_t H[n_V * n_V];
    for (int i = 0; i < n_V * n_V; i++)
    {
        H[i] = P_vectorized[i];
    }

    Eigen::VectorXd r(m_numberDOF);
    r = -2 * (J.transpose() * dx);
    int_t g_size = n_V;
    real_t g[n_V];
    for (int i = 0; i < n_V; i++)
    {
        g[i] = r[i];
    }

    real_t A[n_C * n_V];
    for (int i = 0; i < n_C * n_V; i++)
        A[i] = 0;
    A[8] = -1.;
    A[14] = 1.0;

    // {
    //      0,0,0,0,0,0,0,0,-1.0,0,0,0,0,0,1.0,0,0,0,0,0,0,0,0,0,0,0
    // };

    Eigen::VectorXd dq_min(m_numberDOF), dq_max(m_numberDOF);
    real_t lb[n_V], ub[n_V];

    for (int i = 0; i < m_numberDOF; i++)
    {
        dq_min[i] = 0.5 * (m_joint_limits_min[i] - q_c(i));
        dq_max[i] = 0.5 * (m_joint_limits_max[i] - q_c(i));
        lb[i] = dq_min[i];
        ub[i] = dq_max[i];
    }

    real_t lbA[n_C];
    real_t ubA[n_C];

    for (int i = 0; i < n_C; i++)
    {
        lbA[i] = 0;
        ubA[i] = 0;
    }

    HessianType hessianType = HST_INDEF;
    QProblem example(n_V, n_C, hessianType);

    Options options;
    example.setOptions(options);

    /* Solve first QP. */
    int_t nWSR = 10;
    //example.init( H,g,A,lb,ub,lbA,ubA, nWSR );
    example.init(H, g, A, lb, ub, lbA, ubA, nWSR, nullptr, 0, 0);

    /* Get and print solution of first QP. */
    real_t xOpt[n_V];
    real_t yOpt[n_V + n_C];
    example.getPrimalSolution(xOpt);
    example.getDualSolution(yOpt);
    for (int i = 0; i < n_V; i++)
    {
        printf("\nxOpt[%d] = %e ;  yOpt[%d] = %e;  objVal = %e\n\n",
               i, xOpt[i], i, yOpt[i], example.getObjVal());

        dq(i) = xOpt[i];
    }

    return dq;
}

Eigen::VectorXd robot::HQP_solver2(Eigen::MatrixXd J, Eigen::VectorXd dx, Eigen::VectorXd q_d)
{
    /// Solving with Quadratic Programming : * ///
    /// ************ min ||dq-dq*||2 ********* ///
    /// ********* st dq- <= dq <= dq+ ******** ///
    /// ******** st J_CoM*dq - dX_CoM <= 0 *** ///
    /// ****** st J_RF*dq = dX_RF -> RF fixed  ///
    /// ****** st J_LF*dq = dX_LF -> LF fixed  ///
    /// and with prioritization of taks. ***** ///

    USING_NAMESPACE_QPOASES

    /// REMEMBER: q_d = robot's configuration that's not balanced ///

    std::vector<float> q(m_numberDOF, 0);

    Eigen::VectorXd dq_des(m_numberDOF), dq(m_numberDOF), q_current(m_numberDOF);

    Eigen::MatrixXd J_RF(3, m_numberDOF), J_LF(3, m_numberDOF);
    Eigen::VectorXd X_RF_c(m_numberDOF), X_LF_c(m_numberDOF), X_LF_d(m_numberDOF);
    Eigen::Vector2d dX_CoM_min, dX_CoM_max;

    for (int i = 0; i < m_numberDOF; i++)
    {
        q[i] = q_d[i];
        q_current[i] = m_q_current[i];
        dq_des[i] = m_q_desired[i] - m_q_current[i];
    }

    J_RF = Nao->GetJacobianP_RFoot_RF(q);
    J_LF = Nao->GetJacobianP_LFoot_RF(q);

    X_RF_c = Nao->GetP_RFoot(m_q_current) - Nao->GetP_RFoot(m_q_current); /// Feet position wrt to RFoot ///
    X_LF_c = Nao->GetP_LFoot(m_q_current) - Nao->GetP_RFoot(m_q_current);
    X_LF_d = Nao->GetP_LFoot(q) - Nao->GetP_RFoot(q);

    dX_CoM_max << m_security_XY_CoM[0] - m_XY_CoM_current(0), /// SUPPORT POLYGON ///
        m_security_XY_CoM[2] - m_XY_CoM_current(1);
    dX_CoM_min << m_security_XY_CoM[1] - m_XY_CoM_current(0),
        m_security_XY_CoM[3] - m_XY_CoM_current(1);

    int_t n_V = m_numberDOF; /// number of variables = num_DoF
    int_t n_C = 8;           /// number of constraint. 6 for both feet + 2 for CoM.
    int_t H_rows = n_V;
    int_t H_cols = n_V;
    int_t H_size = H_rows * H_cols;

    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(m_numberDOF, m_numberDOF);
    P = 2 * P;
    Eigen::Map<Eigen::VectorXd> P_vectorized(P.data(), P.size());

    real_t H[n_V * n_V];
    for (int i = 0; i < n_V * n_V; i++)
    {
        H[i] = P_vectorized[i];
    }

    Eigen::VectorXd r(m_numberDOF);
    r = -(dq_des);
    int_t g_size = n_V;

    real_t g[n_V];
    for (int i = 0; i < m_numberDOF; i++)
    {
        g[i] = r[i];
    }

    Eigen::Map<Eigen::VectorXd> J_vectorized(J.data(), J.size());
    Eigen::Map<Eigen::VectorXd> J_LF_vectorized(J_LF.data(), J_LF.size());
    Eigen::Map<Eigen::VectorXd> J_RF_vectorized(J_RF.data(), J_RF.size());

    real_t A[n_C * n_V];
    for (int i = 0; i < n_C * n_V; i++)
    {
        if (i < J_vectorized.size())
        {
            A[i] = J_vectorized[i];
        }
        else if ((i >= J_vectorized.size()) && (i < J_RF_vectorized.size()))
        {
            A[i] = J_RF_vectorized[i];
        }
        else if ((i >= J_vectorized.size() + J_RF_vectorized.size() - 1) && i < J_LF_vectorized.size())
        {
            A[i] = J_LF_vectorized[i];
        }
    }

    Eigen::VectorXd dq_min(m_numberDOF), dq_max(m_numberDOF);

    float K_lim = 0.5;

    real_t lb[n_V];
    real_t ub[n_V];

    for (int i = 0; i < m_numberDOF; i++)
    {
        dq_min[i] = K_lim * (m_joint_limits_min[i] - q[i]);
        dq_max[i] = K_lim * (m_joint_limits_max[i] - q[i]);
        lb[i] = dq_min[i];
        ub[i] = dq_max[i];
    }

    real_t lbA[n_C];
    lbA[0] = dX_CoM_min(0);
    lbA[1] = dX_CoM_min(1);

    lbA[2] = -X_RF_c(0);
    lbA[3] = -X_RF_c(1);
    lbA[4] = -X_RF_c(2);

    lbA[5] = X_LF_d(0) - X_LF_c(0);
    lbA[6] = X_LF_d(1) - X_LF_c(1);
    lbA[7] = -X_LF_c(2);

    real_t ubA[n_C];
    ubA[0] = dX_CoM_max(0);
    ubA[1] = dX_CoM_max(1);

    ubA[2] = -X_RF_c(0);
    ubA[3] = -X_RF_c(1);
    ubA[4] = -X_RF_c(2);

    ubA[5] = X_LF_d(0) - X_LF_c(0);
    ubA[6] = X_LF_d(1) - X_LF_c(1);
    ubA[7] = -X_LF_c(2);

    HessianType hessianType = HST_IDENTITY;
    //QProblem example(n_V,n_C, hessianType);
    QProblem example(n_V, n_C);

    //	Options options;
    //	example.setOptions( options );

    /* Solve first QP. */
    int_t nWSR = 10;

    example.init(H, g, A, lb, ub, lbA, ubA, nWSR, nullptr, 0, 0);

    /* Get and print solution of first QP. */
    real_t xOpt[n_V];
    real_t yOpt[n_V + n_C];
    example.getPrimalSolution(xOpt);
    example.getDualSolution(yOpt);
    for (int i = 0; i < n_V; i++)
    {
        printf("\nxOpt[%d] = %e ;  yOpt[%d] = %e;  objVal = %e\n\n",
               i, xOpt[i], i, yOpt[i], example.getObjVal());

        dq(i) = xOpt[i];
    }

    std::cout << "dq: " << std::endl
              << dq << std::endl;
    std::cout << "Nouvelle config: " << std::endl
              << dq + q_current << std::endl;

    return dq + q_current;
}
