/**
 * @file zmp_control.cpp
 * @author Selvakumar H S (franklinselva10@gmail.com)
 * @brief Implementation of ZMP Control based on the references from "D. Gucci et al. 2018; 
 * Robust Real-time Whole-Body Motion Retargeting from Human to Humanoid"
 * @version 0.1
 * @date 2021-05-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <zmp_control.h>

#define INF 10000
using namespace std;
using namespace Eigen;

/**
 * @brief get ZMP position based on the current CoM;
 * ! Note that the ZMP is assumed to be only 2D here.
 * The model is assumed to be 3D-LIPM as referenced in Kajita. et. al. 2003 
 * Balance Control Analysis of Humanoid Robot based on ZMP Feedback Control
 * 
 * @return Eigen::Vector2d the position of ZMP in 2D
 */

ZMPControl::ZMPControl() : robot()
{
    std::cout << "[INFO] Initialized robot object" << std::endl;
}

ZMPControl::ZMPControl(std::string robot_IP, int robot_port, int mode) : robot(robot_IP, robot_port, mode)
{
    std::cout << "[INFO] Initialized robot object" << std::endl;
}

Eigen::Vector2d ZMPControl::getZMP()
{
    //Get current CoM from the robot state
    ZMPControl::pCoM = Nao->GetXYZ_CoM(robot::m_q_current);

    ZMPControl::aCoM[0] = g * (pCoM[0] - pZMP[0]) / pCoM[2];
    ZMPControl::aCoM[1] = g * (pCoM[1] - pZMP[1]) / pCoM[2];

    ZMPControl::pZMP[0] = pCoM[0] - pCoM[2] * aCoM[0] / ZMPControl::g;
    ZMPControl::pZMP[1] = pCoM[1] - pCoM[2] * aCoM[1] / ZMPControl::g;
    // ZMPControl::pZMP[2] = 0;

    return ZMPControl::pZMP;
}

/**
 * @brief Checks for the ZMP within the support polygon. 
 * ! Note that this function is approximately the same as the check_balance_and_save() function in balanceControl class. 
 * Reference taken for point within polygon from https://www.eecs.umich.edu/courses/eecs380/HANDOUTS/PROJ2/InsidePoly.html
 * 
 * @return true if the ZMP is within the support polygon
 * @return false if the ZMP goes outside the support polygon
 */
bool ZMPControl::withinSP()
{
    robot::m_withinSP = false;
    std::vector<float> q = robot::m_robot_joint_values;
    std::vector<Eigen::Vector2d> footVertex, lFootVertex, rFootVertex; //Vector of foot vertices
    const uint nVertices = 8;

    pLFoot = Nao->GetP_LFoot(q);
    pLFoot = Nao->GetP_LFoot(q);

    lFootVertex = ZMPControl::calculateFootCoordinates(pLFoot, 0);
    rFootVertex = ZMPControl::calculateFootCoordinates(pRFoot, 0);
    Eigen::Vector2d pExtreme{INF, pZMP[1]}; // The point acts as an ray with point to ZMP

    int count = 0, i = 0;

    do
    {
        int next = (i + 1) & nVertices;

        if (doIntersect(footVertex[i], footVertex[next], pZMP, pExtreme))
        {
            if (orientation(footVertex[i], pZMP, footVertex[next]) == 0)
            {
                robot::m_withinSP = onSegment(footVertex[i], pZMP, footVertex[next]);
                return robot::m_withinSP;
            }
            count++;
        }
        i = next;
    } while (i != 0);

    robot::m_withinSP = count & 1; //Odd number bool return
    return robot::m_withinSP;
}

/**
 * @brief Checks for left, right or double support and stores the value in: m_isDS, m_isLS, m_isRS
 * 
 * 
 */
void ZMPControl::checkSupportPhase()
{
    ///NAO only imitate angle if he is not moving
    if (m_mode == 2)
    {
        /// Check if the CoM is in the balance polygon and imitate.
        robot::m_isDS = false;
        robot::m_isRS = false;
        robot::m_isLS = false;

        if (m_FeetHeight(0) < 10 && m_FeetHeight(1) < 10)
        {
            robot::m_isDS = true;
        }
        else if (m_FeetHeight(0) > 10)
        {
            robot::m_isLS = true;
        }
        else if (m_FeetHeight(1) > 10)
        {
            robot::m_isRS = true;
        }
    }
}

/**
 * @brief This function checks the balance of the robot and pushes the joint velocity variation
 * ! Note that this function is approximately the same as the balance() function in balanceControl class. 
 * Returns none
 */
void ZMPControl::balance()
{
    std::vector<float> interpreted_robot_angle_wakeup(m_numberDOF, 0);
    std::vector<float> robot_joint_values(m_numberDOF, 0);

    Eigen::MatrixXd JacobianXY_CoM_current(2, 26);
    Eigen::Vector3d XYZ_CoM_desired, XYZ_CoM_current;
    float max_error_CoM = 1;

    m_q_desired = m_robot_joint_values;

    Eigen::Vector3d XYZ_CoM_T_desired, XYZ_CoM_RF_desired;
    Eigen::Vector3d XYZ_CoM_T_current, XYZ_CoM_RF_current;

    XYZ_CoM_T_desired = Nao->GetXYZ_CoM(m_q_desired);
    XYZ_CoM_RF_desired = Nao->GetXYZ_CoM_RF(m_q_desired);

    XYZ_CoM_T_current = Nao->GetXYZ_CoM(m_q_current);
    XYZ_CoM_RF_current = Nao->GetXYZ_CoM_RF(m_q_current);

    /// m_pos_CoM is based on a RFOOT frame.
    m_pos_CoM[0] = XYZ_CoM_RF_desired(0); // - m_pos_CoM_wakeup[0];
    m_pos_CoM[1] = XYZ_CoM_RF_desired(1); // - m_pos_CoM_wakeup[1];

    def_CoM_limits();

    std::vector<float> q_CoM = m_motion->getCOM("Body", 2, true);
    std::vector<float> q_RF = m_motion->getPosition("RLeg", 2, true);
    std::vector<float> q_CoM2 = m_motion->getCOM("Body", 0, true);
    std::vector<float> q_RF2 = m_motion->getPosition("RLeg", 0, true);

    for (int i = 0; i < q_CoM.size(); i++)
        q_CoM[i] = q_CoM[i] * 1000;

    for (int i = 0; i < q_CoM2.size(); i++)
        q_CoM2[i] = q_CoM2[i] * 1000;

    std::cout << "Current(AL) CoM position -- Center Feet Frame: " << q_CoM << std::endl;
    std::cout << "Current(AL) CoM position -- Torso Frame: " << q_CoM2 << std::endl;

    for (int i = 0; i < q_CoM.size(); i++)
        q_CoM[i] = q_CoM[i] - q_RF[i] * 1000;
    for (int i = 0; i < q_CoM2.size(); i++)
        q_CoM2[i] = q_CoM2[i] - q_RF2[i] * 1000;

    std::cout << "Current(AL) CoM position -- RFoot Frame: " << q_CoM << std::endl;
    std::cout << "Current(AL) CoM position 2 -- RFoot Frame: " << q_CoM2 << std::endl;

    if (m_pos_CoM[0] > (m_security_XY_CoM[0] + max_error_CoM) |
        m_pos_CoM[0]<(m_security_XY_CoM[1] - max_error_CoM) |
                     m_pos_CoM[1]>(m_security_XY_CoM[2] + max_error_CoM) |
        m_pos_CoM[1] < (m_security_XY_CoM[3] - max_error_CoM))
    {
        m_isBalanced = false;

        unbalanced();

        std::cout << "\033[1;32m Not balanced accomplished \033[0m" << std::endl;

        if (m_isBalanced)
        {

            for (int i = 0; i < m_numberDOF; i++)
                m_robot_joint_values[i] = m_q_balanced[i];

            m_robot_joint_values[8] = m_robot_joint_values[14]; /// LHipYawPitch = RHipYawPitch

            std::vector<float> q_desired(m_numberDOF, 0);
            /// m_robot_joint_values in joint limits?? Another verification?? Just for make sure.
            for (int i = 0; i < m_numberDOF; i++)
            {
                q_desired[i] = m_robot_joint_values[i];
                if (q_desired[i] < m_joint_limits_max[i] && q_desired[i] > m_joint_limits_min[i])
                {
                    //std::cout << "Balance compensed" << std::endl;
                }
                else if (q_desired[i] > m_joint_limits_max[i])
                {
                    m_robot_joint_values[i] = m_joint_limits_max[i];
                    m_isBalanced = false;
                }
                else if (q_desired[i] < m_joint_limits_min[i])
                {
                    m_robot_joint_values[i] = m_joint_limits_min[i];
                    m_isBalanced = false;
                }
            }
            if (m_isBalanced)
            {
                m_motion->angleInterpolationWithSpeed(m_robot_joint_names, m_robot_joint_values, m_motors_speed); //Angle Interpolation with speed
                //m_motion->setAngles(m_robot_joint_names, m_robot_joint_values, m_motors_speed);
                m_q_current = m_robot_joint_values;
            }
            m_isBalanced = false;
        }
    }

    else if (m_isDS)
    {
        m_isRectified = false;

        DS();

        if (m_isRectified)
        {
            for (int i = 0; i < m_numberDOF; i++)
                m_robot_joint_values[i] = m_q_rectified[i];

            m_robot_joint_values[8] = m_robot_joint_values[14]; /// LHipYawPitch = RHipYawPitch

            std::vector<float> q_desired(m_numberDOF, 0);

            //? m_robot_joint_values in joint limits?? Another verification?? Just for make sure.
            for (int i = 0; i < m_numberDOF; i++)
            {
                q_desired[i] = m_robot_joint_values[i];
                if (q_desired[i] < m_joint_limits_max[i] && q_desired[i] > m_joint_limits_min[i])
                {
                    //std::cout << "Balance compensed" << std::endl;
                }
                else if (q_desired[i] > m_joint_limits_max[i])
                {
                    std::cout << "Joint " << i << " is over its limits." << std::endl;
                    m_robot_joint_values[i] = m_joint_limits_max[i];
                    m_isRectified = false;
                }
                else if (q_desired[i] < m_joint_limits_min[i])
                {
                    std::cout << "Joint " << i << " is under its limits." << std::endl;
                    m_robot_joint_values[i] = m_joint_limits_min[i];
                    m_isRectified = false;
                }
            }
            if (m_isRectified)
            {
                m_motion->angleInterpolationWithSpeed(m_robot_joint_names, m_robot_joint_values, m_motors_speed); //Angle Interpolation with speed
                //m_motion->setAngles(m_robot_joint_names, m_robot_joint_values, m_motors_speed);
                m_q_current = m_robot_joint_values;
            }
            m_isRectified = false;
        }
    }
}

/**
 * @brief This functions is called and is rectified when the robot reaches a unbalanced state.
 * ! Note that this function is approximately the same as the non_balanced() function in balanceControl class. 
 * 
 */
void ZMPControl::unbalanced()
{
    std::cout << "\033[1;31m NOT BALANCED \033[0m" << std::endl;
    std::cout << std::endl;

    //int modifiedConfiguration[] = {16,17,11,12,19,14,6,7,4,3,8,2};
    int modifiedConfiguration[] = {2, 3, 5, 8, 9, 10, 11, 12, 15, 16, 17, 18, 20, 21, 23};

    ///DGM
    Eigen::Vector2d CoM_current, CoM_desired, CoM_future;
    Eigen::VectorXd X_future(6), X_desired(6);

    std::vector<float> dq_minimization(m_numberDOF, 0);
    Eigen::VectorXd ZAngleDiff(m_numberDOF), ZLimits(m_numberDOF);

    Eigen::Vector2d dCoM;
    Eigen::VectorXd dX(6);

    Eigen::VectorXd q_limits_mean(m_numberDOF), q_limits_delta(m_numberDOF);

    for (int i = 0; i < m_numberDOF; i++)
    {
        q_limits_mean[i] = (m_joint_limits_max[i] + m_joint_limits_min[i]) / 2.0;
        q_limits_delta[i] = m_joint_limits_max[i] - m_joint_limits_min[i];
    }

    Eigen::VectorXd dq(m_numberDOF), dq1(m_numberDOF), dq2(m_numberDOF), dq3(m_numberDOF);

    float weightAngleDiff = -0.1; /// Weight of a minimization task (-)
    float weightLimit = -0.1;     /// Weight of a minimization task (-)

    int iIteration = 0, maxIterations = 10;
    float max_error_CoM = 1; // 0.5;
    float max_error_X = 1;
    int Sp_CoM = 10; // Threshold (milimeters) of how big the step of the IK can be in task space (dXp_CoM)

    bool ClampingOn = true, SolutionFound = false, ReducedX = true;

    Eigen::VectorXd q_results(m_numberDOF);

    Eigen::MatrixXd q_results_bySteps;

    CoM_desired(0) = m_pos_CoM[0];
    CoM_desired(1) = m_pos_CoM[1];

    if (m_pos_CoM[0] > m_security_XY_CoM[0]) ///m_pos_CoM_x > CoM_max_x
    {
        CoM_desired[0] = m_security_XY_CoM[0];
    }
    if (m_pos_CoM[0] < m_security_XY_CoM[1]) ///m_pos_CoM_x < CoM_min_x
    {
        CoM_desired[0] = m_security_XY_CoM[1];
    }
    if (m_pos_CoM[1] > m_security_XY_CoM[2]) ///m_pos_CoM_y > CoM_max_y
    {
        CoM_desired[1] = m_security_XY_CoM[2];
    }
    if (m_pos_CoM[1] < m_security_XY_CoM[3]) ///m_pos_CoM_y < CoM_min_y
    {
        CoM_desired[1] = m_security_XY_CoM[3];
    }

    Eigen::VectorXd q_current(m_numberDOF);
    Eigen::VectorXd q_future(m_numberDOF);
    Eigen::VectorXd q_desired(m_numberDOF);
    for (int i = 0; i < m_numberDOF; i++)
    {
        q_current[i] = m_q_current[i];
        q_desired[i] = m_q_desired[i];
        q_future[i] = m_q_desired[i];
    }

    while (1)
    {
        std::cout << std::endl
                  << "Beginning the iteration loop..." << std::endl
                  << std::endl;
        if (iIteration != 0)
            q_future = q_current;

        /// Start the Direct Geometrical Model
        DGM(q_desired);
        X_desired = m_X_current;

        DGM(q_future);
        CoM_future = m_XY_CoM_current;

        dCoM(1) = (CoM_desired[1] - CoM_future[1]);
        dCoM(0) = (CoM_desired[0] - CoM_future[0]);

        std::cout << "Desired CoM pos : " << std::endl
                  << CoM_desired.transpose() << std::endl;
        std::cout << "Next Not Balanced CoM pos: " << std::endl
                  << CoM_future.transpose() << std::endl;
        std::cout << "dCoM: " << std::endl
                  << dCoM.transpose() << std::endl;

        X_future = m_X_current;

        /// WE FIXE THE RFOOT AND WE KEEP THE LFOOT IN A PLANE/
        X_desired << m_X_Npose(0), m_X_Npose(1), m_X_Npose(2), /// RFoot_XYZ wrt RFoot
            X_desired(3), X_desired(4), m_X_Npose(5);          /// LFoot_XYZ wrt RFoot

        dX = X_desired - X_future;

        std::cout << "Desired Eff pos : " << std::endl
                  << X_desired.transpose() << std::endl;
        std::cout << "Current Eff pos: " << std::endl
                  << X_future.transpose() << std::endl;
        std::cout << "dX: " << std::endl
                  << dX.transpose() << std::endl;

        /// Start the Direct Kinematic Model
        DKM(q_future);

        std::cout << "Current joint configuration : " << std::endl
                  << q_current.transpose() << std::endl;
        std::cout << "Future joint configuration (NOT BALANCED): " << std::endl
                  << q_future.transpose() << std::endl;

        // CHECK IF SOLUTION WAS FOUND
        // Limits
        bool LimitsOK = true;
        for (int i = 0; i < m_numberDOF; i++)
            if (q_current[i] < m_joint_limits_min[i] || q_current[i] > m_joint_limits_max[i])
            {
                LimitsOK = false;
                std::cout << " i: " << i;
            }
        if (!LimitsOK)
            std::cout << std::endl;

        // Balance
        std::cout << "|CoM_desired_x - CoM_current_x| = " << std::endl
                  << fabs(dCoM(0)) << std::endl;
        std::cout << "|CoM_desired_y - CoM_current_y| = " << std::endl
                  << fabs(dCoM(1)) << std::endl;
        bool BalanceOK = true;
        if ((fabs(dCoM(0)) > max_error_CoM) || (fabs(dCoM(1)) > max_error_CoM))
        {
            BalanceOK = false;
        }

        std::cout << "|X_RF_desired_x - X_RF_current_x| = " << std::endl
                  << fabs(dX(0)) << std::endl;
        std::cout << "|X_RF_desired_y - X_RF_current_y| = " << std::endl
                  << fabs(dX(1)) << std::endl;
        std::cout << "|X_RF_desired_z - X_RF_current_z| = " << std::endl
                  << fabs(dX(2)) << std::endl;
        std::cout << "|X_LF_desired_x - X_LF_current_x| = " << std::endl
                  << fabs(dX(3)) << std::endl;
        std::cout << "|X_LF_desired_y - X_LF_current_y| = " << std::endl
                  << fabs(dX(4)) << std::endl;
        std::cout << "|X_LF_desired_z - X_LF_current_z| = " << std::endl
                  << fabs(dX(5)) << std::endl;

        // End effectors
        bool EndEffectorsOK = true;
        VectorXd dXAbs = dX.cwiseAbs();
        int numberRows = dXAbs.rows();
        if (dXAbs.maxCoeff() > max_error_X)
            EndEffectorsOK = false;

        // Complete check
        if (LimitsOK && BalanceOK && EndEffectorsOK)
        {
            SolutionFound = true;

            m_q_balanced = q_future;
            m_isBalanced = true;
            std::cout << "new q_current" << std::endl
                      << m_q_balanced.transpose() << std::endl;

            std::cout << "\033[1;32mConversion!!! In " << iIteration << " iterations. \033[0m" << std::endl; // in dark green
            break;
        }

        // The limit of iterations has been reached
        if (iIteration > maxIterations)
        {
            // The results stay the same as the previous configuration.
            std::cout << "\033[1;31mNo conversion after " << iIteration << " iterations.\033[0m";
            SolutionFound = false;

            if (!LimitsOK)
            {
                std::cout << "\033[1;31m LIMITS FAIL \033[0m";
            }
            else
            {
                std::cout << "\033[1;32m LIMITS OK \033[0m";
            }
            if (!BalanceOK)
            {
                std::cout << "\033[1;31m BALANCE FAIL \033[0m";
            }
            else
            {
                std::cout << "\033[1;32m BALANCE OK \033[0m";
            }
            if (!EndEffectorsOK)
            {
                std::cout << "\033[1;31m END EFF. FAIL \033[0m" << std::endl;
            }
            else
            {
                std::cout << "\033[1;32m END EFF. OK \033[0m" << std::endl;
            }

            break;
        }

        /// Minimize joint differences
        /// Difference between desired joint angles and current ones
        for (int i = 0; i < m_numberDOF; i++)
        {
            if (std::count(modifiedConfiguration, modifiedConfiguration + 15, i) == 1)
            {
                dq_minimization[i] = q_current[i] - q_desired[i];
            }
            else
                dq_minimization[i] = 0;
        }

        /// Low priority: Optimization tasks -- AngleDifferenciation & JointLimits
        for (int i = 0; i < m_numberDOF; i++)
        {
            ZAngleDiff(i) = 2.0 * weightAngleDiff * dq_minimization[i];
            ZLimits(i) = (2.0 * weightLimit * (q_current[i] - q_limits_mean(i))) / (pow(q_limits_delta(i), 2));
        }

        /// Choose priority -- tracking / balance
        /// To choose, exchange J1 and J2, X1 and X2, dimension 1 and dimension2.
        /// It's also possible to experiment only with a specific end-effector, by using their Jacobians defined above and adjusting dX and dimension accordingly
        int dimension1 = m_JacobianXY_CoM_RF.rows();
        int dimension2 = m_JacobianEndEff.rows();

        Eigen::VectorXd q_current_temp(m_numberDOF), q_current_temp2(m_numberDOF);

        /// Priority 1 -> BALANCE
        Eigen::MatrixXd J1 = m_JacobianXY_CoM_RF;

        J1(0, 14) = J1(0, 8);
        J1(1, 14) = J1(1, 8);

        Eigen::Vector2d dX1 = dCoM;

        /// Priority 2 -> Track EndEffectors
        Eigen::MatrixXd J2 = m_JacobianEndEff;
        J2(0, 14) = J2(0, 8);
        J2(1, 14) = J2(1, 8);
        J2(2, 14) = J2(2, 8);
        J2(3, 14) = J2(3, 8);
        J2(4, 14) = J2(4, 8);
        J2(5, 14) = J2(5, 8);
        Eigen::VectorXd dX2 = dX;

        /// Clamping loop according to Baerlocher
        /// Clamps joints that are outside limits to their limits and solves IK with the rest of joints
        bool Clamping = true;
        //q_current = q_future;
        while (Clamping)
        {
            Clamping = false;

            // Get augmented Jacobians
            // J2a
            Eigen::MatrixXd J2a(dimension1 + dimension2, m_numberDOF);
            for (int i = 0; i < m_numberDOF; i++)
            {
                // First task
                for (int j = 0; j < dimension1; j++)
                    J2a(j, i) = J1(j, i);
                // Second task
                for (int j = 0; j < dimension2; j++)
                    J2a(dimension1 + j, i) = J2(j, i);
            }

            std::cout << std::endl
                      << "Begin the clamping loop..." << std::endl
                      << std::endl;

            std::cout << "X_desired - X_future = " << std::endl
                      << dX2.transpose() << std::endl;
            std::cout << "CoM_desired - CoM_future = " << std::endl
                      << dX1.transpose() << std::endl;
            std::cout << "" << std::endl
                      << J1(0, 17) << std::endl;
            std::cout << "" << std::endl
                      << J2(0, 17) << std::endl;

            /// Get pseudo inverses
            /// These matrices are sparse and wide. The right pinv seems to be working, but I don't know how robust it is.
            //Eigen::MatrixXd J1_i = pinvRight2(J1);

            Eigen::MatrixXd J1_ = J1;
            CompleteOrthogonalDecomposition<Ref<Eigen::MatrixXd>> COD_1(J1_);
            Eigen::MatrixXd J2a_ = J2a;
            CompleteOrthogonalDecomposition<Ref<Eigen::MatrixXd>> COD_2(J2a_);

            ///Projection matrix.
            Eigen::MatrixXd P1 = Eigen::MatrixXd::Identity(m_numberDOF, m_numberDOF) - COD_1.solve(J1);
            Eigen::MatrixXd P2a = Eigen::MatrixXd::Identity(m_numberDOF, m_numberDOF) - COD_2.solve(J2a);

            MatrixXd J2P1a = J2 * P1;
            Eigen::MatrixXd J2P1a_ = J2P1a;
            CompleteOrthogonalDecomposition<Ref<Eigen::MatrixXd>> COD_3(J2P1a_);

            dq1 = COD_1.solve(dX1);
            dq = dq1 + COD_3.solve(dX2 - J2 * dq1);

            std::cout << "dq: " << std::endl
                      << dq << std::endl;
            std::cout << "q_future: " << std::endl
                      << q_future << std::endl
                      << std::endl;

            q_current_temp = q_future;
            q_current = q_future + dq;
            q_current_temp2 = q_current;
            std::cout << "q_current + dq: " << std::endl
                      << q_current << std::endl
                      << std::endl;

            //Eigen::VectorXd dQ(m_numberDOF);
            //dq = HQP_solver2(J1,dX1,q_future);

            //std::cout << "Difference de méthode: " << std::endl << q_current_temp2 - dq << std::endl << std::endl;

            // Baerlocher
            // Check if result is within joint limits
            if (ClampingOn)
            {

                double margin = 0.001;
                for (int i = 0; i < m_numberDOF; i++)
                {
                    if (q_current_temp2[i] < m_joint_limits_min[i] || q_current_temp2(i) > m_joint_limits_max[i])
                    {
                        std::cout << "Joint over its limits" << std::endl;
                        Clamping = true;
                        q_future = q_current_temp;
                        // Fix the trouble angle to its limit
                        if (q_current_temp2(i) < m_joint_limits_min[i])
                        {
                            q_future[i] = m_joint_limits_min[i] + margin;
                            std::cout << "Joint " << i + 1 << " (" << i << ") clamped to its min limit." << std::endl;
                        }
                        if (q_current_temp2(i) > m_joint_limits_max[i])
                        {
                            q_future[i] = m_joint_limits_max[i] - margin;
                            std::cout << "Joint " << i + 1 << " (" << i << ") clamped to its max limit." << std::endl;
                        }
                        // Zero column i of the Jacobian
                        for (int l = 0; l < dimension1; l++)
                        {
                            J1(l, i) = 0;
                            std::cout << i << std::endl
                                      << J1(l, i) << std::endl;
                        }
                        for (int l = 0; l < dimension2; l++)
                        {
                            J2(l, i) = 0;
                            //std::cout << dimension2 << std::endl;
                            std::cout << i << std::endl
                                      << J2(l, i) << std::endl;
                        }
                    }
                    else
                        std::cout << "Joints in their limits!!!" << std::endl;
                }
            }

            //q_current_mod = q_current_temp2;
        } /// end clamping loop

        std::cout << std::endl
                  << "Clamping loop ended" << std::endl
                  << std::endl;

        if (iIteration == 0)
        {
            q_results_bySteps = dq.transpose();
        }
        else
        {
            // Append this dq
            int numberRows = q_results_bySteps.rows();
            q_results_bySteps.conservativeResize(numberRows + 1, Eigen::NoChange);
            q_results_bySteps.row(numberRows) = dq.transpose();
        }

        // This makes sure no crazy results come out, because sometimes nans and infs come and destroy everything
        for (int i = 0; i < m_numberDOF; i++)
        {
            if (q_current[i] != q_current[i]) // check if its NaN
                q_current[i] = q_current_temp(i);
            if (q_current[i] > 1.0e2) // thats a very large upper limit, no?
            {
                std::cout << "inf " << i << std::endl;
                q_current[i] = q_current_temp(i);
            }
        }

        // Making sure the results are in the desired quadrant -- it seems to always be --  get rid of temp2?
        // Limit to 2*pi
        for (int i = 0; i < m_numberDOF; i++)
            q_current[i] = fmod((double)q_current[i], 2 * M_PI);
        // Limit to between -pi and pi
        for (int i = 0; i < m_numberDOF; i++)
        {
            if ((q_current[i] >= M_PI) && (q_current[i] <= 2 * M_PI))
                q_current[i] = q_current[i] - 2 * M_PI;
            else if ((q_current[i] < -M_PI) && (q_current[i] > -2 * M_PI))
                q_current[i] = q_current[i] + 2 * M_PI;
        }

        std::vector<float> q2(m_numberDOF, 0);
        for (int i = 0; i < m_numberDOF; i++)
            q2[i] = q_current[i];
        //        std::cout<<std::endl<<"New CoM: "<<std::endl<< Nao->GetXYZ_CoM_RF_rectified(q2)<<std::endl;
        std::cout << std::endl
                  << "New CoM: " << std::endl
                  << Nao->GetXYZ_CoM_RF(q2) << std::endl;

        std::cout << "Iteration n° " << iIteration << std::endl;
        std::cout << std::endl
                  << std::endl;
        iIteration = iIteration + 1;
        std::cout << "new q_current: " << std::endl
                  << q_current << std::endl
                  << std::endl;

    } /// end of while loop
    if (SolutionFound)
    {
        std::cout << "\033[1;32m SOLUTION FOUND \033[0m" << std::endl
                  << std::endl;
        m_q_results_byStepsFinal = q_results_bySteps;
        q_results_bySteps.conservativeResize(1, m_numberDOF);
    }

    std::cout << std::endl
              << std::endl;
}

/**
 * @brief Move the robot to be in balance position in double support phase
 * 
 */
void ZMPControl::DS()
{

    std::cout << "\033[1;31m BALANCED IN DOUBLE SUPPORT \033[0m" << std::endl;
    std::cout << std::endl;

    //int modifiedConfiguration[] = {16,17,11,12,19,14,6,7,4,3,8,2};
    int modifiedConfiguration[] = {2, 3, 5, 8, 9, 10, 11, 12, 15, 16, 17, 18, 20, 21, 23};

    ///DGM
    Eigen::Vector2d CoM_current, CoM_desired, CoM_future;
    //Eigen::VectorXd X_future(12), X_desired(12);
    Eigen::VectorXd X_future(6), X_desired(6);

    std::vector<float> dq_minimization(m_numberDOF, 0);
    Eigen::VectorXd ZAngleDiff(m_numberDOF), ZLimits(m_numberDOF);

    Eigen::Vector2d dCoM;
    //Eigen::VectorXd dX(12);
    Eigen::VectorXd dX(6);

    Eigen::VectorXd q_limits_mean(m_numberDOF), q_limits_delta(m_numberDOF);

    for (int i = 0; i < m_numberDOF; i++)
    {
        q_limits_mean[i] = (m_joint_limits_max[i] + m_joint_limits_min[i]) / 2.0;
        q_limits_delta[i] = m_joint_limits_max[i] - m_joint_limits_min[i];
    }

    Eigen::VectorXd dq(m_numberDOF), dq1(m_numberDOF), dq2(m_numberDOF), dq3(m_numberDOF);

    float weightAngleDiff = -0.1; /// Weight of a minimization task (-)
    float weightLimit = -0.1;     /// Weight of a minimization task (-)

    int iIteration = 0, maxIterations = 10;
    float max_error_X = 1;

    bool ClampingOn = true, SolutionFound = false, ReducedX = true;

    Eigen::VectorXd q_results(m_numberDOF);

    Eigen::MatrixXd q_results_bySteps;

    Eigen::VectorXd q_current(m_numberDOF);
    Eigen::VectorXd q_future(m_numberDOF);
    Eigen::VectorXd q_desired(m_numberDOF);
    for (int i = 0; i < m_numberDOF; i++)
    {
        q_current[i] = m_q_current[i];
        q_desired[i] = m_q_desired[i];
        q_future[i] = m_q_desired[i];
    }

    while (1)
    {
        std::cout << std::endl
                  << "Beginning the iteration loop..." << std::endl
                  << std::endl;
        if (iIteration != 0)
            q_future = q_current;
        /// Start the Direct Geometrical Model

        DGM(q_future);
        X_future = m_X_current;

        /// WE FIXE THE RFOOT AND WE KEEP THE LFOOT IN THE XY PLANE ///
        X_desired << m_X_Npose(0), m_X_Npose(1), m_X_Npose(2), /// RFoot_6D wrt RFoot
            X_future(3), X_future(4), m_X_Npose(5);            /// LFoot_6D wrt RFoot

        dX = X_desired - X_future;

        std::cout << "Desired Eff pos : " << std::endl
                  << X_desired.transpose() << std::endl;
        std::cout << "Current Eff pos: " << std::endl
                  << X_future.transpose() << std::endl;
        std::cout << "dX: " << std::endl
                  << dX.transpose() << std::endl;

        /// Start the Direct Kinematic Model
        DKM(q_future);

        std::cout << "Current joint configuration : " << std::endl
                  << q_current.transpose() << std::endl;
        std::cout << "Future joint configuration (NOT BALANCED): " << std::endl
                  << q_future.transpose() << std::endl;

        // CHECK IF SOLUTION WAS FOUND
        // Limits
        bool LimitsOK = true;
        for (int i = 0; i < m_numberDOF; i++)
            if (q_current[i] < m_joint_limits_min[i] || q_current[i] > m_joint_limits_max[i])
            {
                LimitsOK = false;
                std::cout << " i: " << i;
            }
        if (!LimitsOK)
            std::cout << std::endl;

        std::cout << "|X_desired_x - X_current_x| = " << std::endl
                  << fabs(dX(0)) << std::endl;
        std::cout << "|X_desired_y - X_current_y| = " << std::endl
                  << fabs(dX(1)) << std::endl;
        // End effectors
        bool EndEffectorsOK = true;
        VectorXd dXAbs = dX.cwiseAbs();
        int numberRows = dXAbs.rows();
        if (dXAbs.maxCoeff() > max_error_X)
            EndEffectorsOK = false;

        // Complete check
        if (LimitsOK && EndEffectorsOK)
        {
            SolutionFound = true;

            m_q_rectified = q_future;
            m_isRectified = true;
            std::cout << "new q_current" << std::endl
                      << m_q_rectified.transpose() << std::endl;

            std::cout << "\033[1;32mConversion!!! In " << iIteration << " iterations. \033[0m" << std::endl; // in dark green
            break;
        }

        // The limit of iterations has been reached
        if (iIteration > maxIterations)
        {
            // The results stay the same as the previous configuration.
            std::cout << "\033[1;31mNo conversion after " << iIteration << " iterations.\033[0m";
            SolutionFound = false;

            if (!LimitsOK)
            {
                std::cout << "\033[1;31m LIMITS FAIL \033[0m";
            }
            else
            {
                std::cout << "\033[1;32m LIMITS OK \033[0m";
            }
            if (!EndEffectorsOK)
            {
                std::cout << "\033[1;31m END EFF. FAIL \033[0m" << std::endl;
            }
            else
            {
                std::cout << "\033[1;32m END EFF. OK \033[0m" << std::endl;
            }

            break;
        }

        /// Minimize joint differences
        /// Difference between desired joint angles and current ones
        for (int i = 0; i < m_numberDOF; i++)
        {
            if (std::count(modifiedConfiguration, modifiedConfiguration + 15, i) == 1)
            {
                dq_minimization[i] = q_current[i] - q_desired[i];
            }
            else
                dq_minimization[i] = 0;
        }

        /// Low priority: Optimization tasks -- AngleDifferenciation & JointLimits
        for (int i = 0; i < m_numberDOF; i++)
        {
            ZAngleDiff(i) = 2.0 * weightAngleDiff * dq_minimization[i];
            ZLimits(i) = (2.0 * weightLimit * (q_current[i] - q_limits_mean(i))) / (pow(q_limits_delta(i), 2));
        }

        /// SINGLE TASK REQUIERED:
        /// RFoot fixed ans LFoot constrained in the XY plane (LFoot height = 0);
        int dimension1 = m_JacobianEndEff.rows();

        Eigen::VectorXd q_current_temp(m_numberDOF), q_current_temp2(m_numberDOF);

        /// Track EndEffectors
        Eigen::MatrixXd J1 = m_JacobianEndEff;
        Eigen::VectorXd dX1 = dX;

        /// ESSAI --> Converge vers 0???
        int lambda = 1;
        //dX_CoM = -lambda*dX_CoM;

        /// Clamping loop according to Baerlocher
        /// Clamps joints that are outside limits to their limits and solves IK with the rest of joints
        bool Clamping = true;
        q_current = q_future;
        while (Clamping)
        {
            Clamping = false;

            std::cout << std::endl
                      << "Begin the clamping loop..." << std::endl
                      << std::endl;

            std::cout << "X_desired - X_future = " << std::endl
                      << dX1.transpose() << std::endl;

            /// Get pseudo inverses
            /// These matrices are sparse and wide. The right pinv seems to be working, but I don't know how robust it is.
            //Eigen::MatrixXd J1_i = pinvRight2(J1);

            Eigen::MatrixXd J1_ = J1;
            CompleteOrthogonalDecomposition<Ref<Eigen::MatrixXd>> COD_1(J1_);

            ///Projection matrix.
            Eigen::MatrixXd P1 = Eigen::MatrixXd::Identity(m_numberDOF, m_numberDOF) - COD_1.solve(J1);

            dq2 = COD_1.solve(dX1);
            dq = dq2 + P1 * ZAngleDiff;
            //dq = dq1 + P1*ZLimits;

            std::cout << "dq: " << std::endl
                      << dq << std::endl;

            std::cout << "q_future: " << std::endl
                      << q_current << std::endl
                      << std::endl;
            q_current_temp = q_current;
            q_current = q_current + dq;
            q_current_temp2 = q_current;
            std::cout << "q_current + dq: " << std::endl
                      << q_current << std::endl
                      << std::endl;

            // Baerlocher
            // Check if result is within joint limits
            if (ClampingOn)
            {

                double margin = 0.01;
                for (int i = 0; i < m_numberDOF; i++)
                {
                    if (q_current_temp2[i] < m_joint_limits_min[i] || q_current_temp2(i) > m_joint_limits_max[i])
                    {
                        std::cout << "Joint over its limits" << std::endl;
                        Clamping = true;
                        // Back to previous solution
                        q_current = q_current_temp;
                        //q_current = q_current_temp;
                        // Fix the trouble angle to its limit
                        if (q_current_temp2(i) < m_joint_limits_min[i])
                        {
                            q_current[i] = m_joint_limits_min[i] + margin;
                            std::cout << "Joint " << i + 1 << " (" << i << ") clamped to its min limit." << std::endl;
                        }
                        if (q_current_temp2(i) > m_joint_limits_max[i])
                        {
                            q_current[i] = m_joint_limits_max[i] - margin;
                            std::cout << "Joint " << i + 1 << " (" << i << ") clamped to its max limit." << std::endl;
                        }
                        // Zero column i of the Jacobian
                        for (int j = 0; j < dimension1; j++)
                            J1(j, i) = 0;
                    }
                    else
                        std::cout << "Joints in their limits!!!" << std::endl;
                }
            }

            //q_current_mod = q_current_temp2;
        } /// end clamping loop

        std::cout << std::endl
                  << "Clamping loop ended" << std::endl
                  << std::endl;

        if (iIteration == 0)
        {
            q_results_bySteps = dq.transpose();
        }
        else
        {
            // Append this dq
            int numberRows = q_results_bySteps.rows();
            q_results_bySteps.conservativeResize(numberRows + 1, Eigen::NoChange);
            q_results_bySteps.row(numberRows) = dq.transpose();
        }

        // This makes sure no crazy results come out, because sometimes nans and infs come and destroy everything
        for (int i = 0; i < m_numberDOF; i++)
        {
            if (q_current[i] != q_current[i]) // check if its NaN
                q_current[i] = q_current_temp(i);
            if (q_current[i] > 1.0e2) // thats a very large upper limit, no?
            {
                std::cout << "inf " << i << std::endl;
                q_current[i] = q_current_temp(i);
            }
        }

        // Making sure the results are in the desired quadrant -- it seems to always be --  get rid of temp2?
        // Limit to 2*pi
        for (int i = 0; i < m_numberDOF; i++)
            q_current[i] = fmod((double)q_current[i], 2 * M_PI);
        // Limit to between -pi and pi
        for (int i = 0; i < m_numberDOF; i++)
        {
            if ((q_current[i] >= M_PI) && (q_current[i] <= 2 * M_PI))
                q_current[i] = q_current[i] - 2 * M_PI;
            else if ((q_current[i] < -M_PI) && (q_current[i] > -2 * M_PI))
                q_current[i] = q_current[i] + 2 * M_PI;
        }

        std::cout << "Iteration n° " << iIteration << std::endl;
        std::cout << std::endl
                  << std::endl;
        iIteration = iIteration + 1;
        std::cout << "new q_current: " << std::endl
                  << q_current << std::endl
                  << std::endl;

    } /// end of while loop
    if (SolutionFound)
    {
        std::cout << "\033[1;32m SOLUTION FOUND \033[0m" << std::endl
                  << std::endl;
        m_q_results_byStepsFinal = q_results_bySteps;
        q_results_bySteps.conservativeResize(1, m_numberDOF);
    }

    std::cout << std::endl
              << std::endl;
}

/**
 * @brief Check for dynamic robot balance based on mode(1/2);
 * Focuses on single and double support for mode 2;
 * Starts the imitation if mode 1 is selected
 * 
 */
void ZMPControl::check_balance_and_move()
{
    ZMPControl::checkSupportPhase();
    ZMPControl::withinSP();

    if (robot::m_mode == 2 && (robot::m_isDS || robot::m_isLS || robot::m_isRS))
    {
        if (robot::m_withinSP)
            ZMPControl::balance();
        else
            ZMPControl::unbalanced();
    }
    else if (robot::m_mode == 1)
    {
        // Start Body Imitation
        m_motion->setAngles(m_robot_joint_names, m_robot_joint_values, m_motors_speed);
        m_q_current = m_robot_joint_values;
    }
}

/**
 * @brief Starts the imitation of the human actor based on the mode selected (1/2); 
 * ! Note that dynamic balance control is implemented with this function.
 * 
 * @param feetdistance The euclidean distance betweem the two feet
 * @param distancepiedR The euclidean distance from right foot to torso projection
 * @param distancepiedL The euclidean distance from left foot to torso projection
 * @param dRotation Rotation difference along z axis from Xsens Analyze
 */
void ZMPControl::begin_imitation(float feetdistance, float distancepiedR, float distancepiedL, float dRotation)
{
    ///update m_velocity if feet are far or head is turned
    robot::update_velocity(feetdistance, distancepiedR, distancepiedL, dRotation);
    robot::m_imitate = true;

    ///Check if the robot is balanced then proceed to imitate without falling
    ZMPControl::check_balance_and_move();
}

/**
 * @brief Calculates the foot vertex coordinates in 2D projection
 * 
 * @param footCenter Eigen::Vector3d - Center of the position of left foot or right foot 
 * @param footAngle float(rad) - FootAngle of the left or right foot
 * @return std::vector<Eigen::Vector2d> - Returns the vertex coordinates in order "Top Right, Top Left, Bottom Right, Bottom Left"
 */
std::vector<Eigen::Vector2d> ZMPControl::calculateFootCoordinates(Eigen::Vector3d footCenter, float footAngle)
{
    Eigen::Vector2d TR_vertex, TL_vertex, BL_vertex, BR_vertex;
    Eigen::Vector2d footCenter2D{footCenter[0], footCenter[1]};

    TR_vertex[0] = footCenter2D[0] + ((footWidth / 2) * cos(footAngle) - (footLength / 2) * sin(footAngle));
    TR_vertex[1] = footCenter2D[1] + ((footWidth / 2) * sin(footAngle) + (footLength / 2) * cos(footAngle));

    TL_vertex[0] = footCenter2D[0] - ((footWidth / 2) * cos(footAngle) - (footLength / 2) * sin(footAngle));
    TL_vertex[1] = footCenter2D[1] - ((footWidth / 2) * sin(footAngle) + (footLength / 2) * cos(footAngle));

    BL_vertex[0] = footCenter2D[0] - ((footWidth / 2) * cos(footAngle) + (footLength / 2) * sin(footAngle));
    BL_vertex[1] = footCenter2D[1] - ((footWidth / 2) * sin(footAngle) - (footLength / 2) * cos(footAngle));

    BR_vertex[0] = footCenter2D[0] + ((footWidth / 2) * cos(footAngle) + (footLength / 2) * sin(footAngle));
    BR_vertex[1] = footCenter2D[1] + ((footWidth / 2) * sin(footAngle) - (footLength / 2) * cos(footAngle));

    return std::vector<Eigen::Vector2d>{TR_vertex, TL_vertex, BR_vertex, BL_vertex};
}

/**
 * @brief Given the collinear points, the function checks for q in line pr
 * 
 * @param p Eigen::Vector2d - Coordinates of the line 
 * @param q Eigen::Vector2d - Point to be tested
 * @param r Eigen::Vector2d - Coordinates of the line 
 * @return true The point q is on the line pr
 * @return false The point q is not on the line pr
 */
bool ZMPControl::onSegment(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r)
{
    if (q[0] <= std::max(p[0], r[0]) && q[0] >= std::min(p[0], r[0]) &&
        q[1] <= std::max(p[1], r[1]) && q[1] >= std::min(p[1], r[1]))
        return true;
    return false;
}

/**
 * @brief To find orientation of ordered triplet (p, q, r).
 * 
 * @param p Eigen::Vector2d
 * @param q Eigen::Vector2d
 * @param r Eigen::Vector2d
 * @return int - 0 if points are collinear; 1 if the points are in clockwise order; 2 if the points are in anticlockwise order
 */
int ZMPControl::orientation(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r)
{
    int val = (q[1] - p[1]) * (r[0] - q[0]) -
              (q[0] - p[0]) * (r[1] - q[1]);

    if (val == 0)
        return 0;             // colinear
    return (val > 0) ? 1 : 2; // clock or counterclock wise
}

/**
 * @brief This function checks for intersection of two lines given its 2D coordinates
 * 
 * @param p1 Eigen::Vector2d - Coordinates of line 1
 * @param q1 Eigen::Vector2d - Coordinates of line 1
 * @param p2 Eigen::Vector2d - Coordinates of line 2
 * @param q2 Eigen::Vector2d - Coordinates of line 2
 * @return true The two given lines do intersect
 * @return false The two lines doesn't intersect
 */
bool ZMPControl::doIntersect(Eigen::Vector2d p1, Eigen::Vector2d q1, Eigen::Vector2d p2, Eigen::Vector2d q2)
{
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1))
        return true;

    // p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1))
        return true;

    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2))
        return true;

    // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2))
        return true;

    return false; // Doesn't fall in any of the above cases
}

/**
 * @brief This function returns the expected CoM projection of the robot on the 2D horizontal plane
 * The approach is baed on the paper "Gucci. et al. 2018, Robust Real-time Whole-Body Motion 
 * Retargeting from Human to Humanoid"
 * 
 * @param lFootHuman Eigen::Vector3f - The position of the left foot of the human actor from Xsens
 * @param rFootHuman Eigen::Vector3f - The position of the right foot of the human actor from Xsens
 * @return Eigen::Vector3f The expected CoM projection of the robot on the 2D horizontal plane
 */
Eigen::Vector3f ZMPControl::CoMRetargetting(Eigen::Vector3f lFootHuman, Eigen::Vector3f rFootHuman, Eigen::Vector3f CoMHuman)
{
    float o = (CoMHuman - lFootHuman).dot(rFootHuman - lFootHuman) / (rFootHuman - lFootHuman).norm();
}