#include <robot.h>
#include <vector>

using namespace Eigen;
using namespace std;

Eigen::MatrixXd pinvRight2(Eigen::MatrixXd& J);
Eigen::VectorXd changeConfigurationModel(std::vector<float> & V);
Eigen::MatrixXd RotationEuler(float roll, float pitch, float yaw);

robot::robot() : m_name(0), m_robot_IP(0), m_mode(1), m_moving(false), m_motors_speed(0), m_isBalanced(false), m_velocity_x_y_theta(3,0),
                m_pos_CoM_wakeup(2,0), m_vel_CoM_wakeup(2,0), m_pos_CoM(2,0), m_vel_CoM(2,0), m_security_XY_CoM(4,0), m_q_rectified(26),m_q_balanced(26),
                m_pos_CoM_mean(2,0), m_q_current(26,0), m_q_desired(26,0)
{
    m_motion = new AL::ALMotionProxy();
    m_speech = new AL::ALTextToSpeechProxy();
    m_posture = new AL::ALRobotPostureProxy();
    m_memory = new AL::ALMemoryProxy();

    Nao = new NAO(); //Jacobian Instance Initialization

    m_motion->setMoveArmsEnabled(false,false);

}

robot::~robot()
{

    delete m_motion;
    delete m_speech;
    delete m_posture;
    delete m_memory;

    delete Nao;

}

robot::robot(std::string robot_IP, int robot_port, int mode) : m_robot_IP(robot_IP), m_robot_port(robot_port), m_mode(mode), m_moving(false), m_motors_speed(0),
            m_isBalanced(false), m_velocity_x_y_theta(3,0), m_pos_CoM_wakeup(2,0), m_vel_CoM_wakeup(2,0), m_pos_CoM(2,0), m_vel_CoM(2,0), m_security_XY_CoM(4,0),
            m_q_rectified(26), m_q_balanced(26), m_pos_CoM_mean(2,0), m_q_current(26,0), m_q_desired(26,0)
{
    m_motion = new AL::ALMotionProxy(m_robot_IP,m_robot_port);
    m_speech = new AL::ALTextToSpeechProxy(m_robot_IP,m_robot_port);
    m_posture = new AL::ALRobotPostureProxy(m_robot_IP, m_robot_port);
    m_memory = new AL::ALMemoryProxy(m_robot_IP, m_robot_port);

    Nao = new NAO();

    m_motion->setMoveArmsEnabled(false,false);

    if(m_name == "PEPPER")
    {
        std::cout<<"PEPPER ROBOT is not supported with this application..."<<std::endl;
    }

}

void robot::balance()
{
    std::cout << "\033[1;32m CHECKING BALANCE... \033[0m" << std::endl;
    std::cout << std::endl;

    std::vector<float> interpreted_robot_angle_wakeup(m_numberDOF,0);
    std::vector<float> robot_joint_values(m_numberDOF,0);

    Eigen::MatrixXd JacobianXY_CoM_current(2,26);
    Eigen::Vector3d XYZ_CoM_desired, XYZ_CoM_current;
    float max_error_CoM = 1;

    m_q_desired = m_robot_joint_values;

    /// Get the Jacobian for ΔX_CoM = JacobianXY_CoM_RFoot*Δq
    //JacobianXY_CoM_current = Nao->GetJacobianXY_CoM_RF_rectified(m_q_current);

    std::cout << "Current joint configuration: " << std::endl << m_q_current << std::endl;
    std::cout << "Desired joint configuration: " << std::endl << m_q_desired << std::endl;

    Eigen::Vector3d XYZ_CoM_T_desired, XYZ_CoM_RF_desired;
    Eigen::Vector3d XYZ_CoM_T_current, XYZ_CoM_RF_current;
    //    Eigen::Vector3d XYZ_CoM_T_desired_c, XYZ_CoM_RF_desired_c;
    //    Eigen::Vector3d XYZ_CoM_T_current_c, XYZ_CoM_RF_current_c;

    XYZ_CoM_T_desired = Nao->GetXYZ_CoM(m_q_desired);
    XYZ_CoM_RF_desired = Nao->GetXYZ_CoM_RF(m_q_desired);
    //    XYZ_CoM_T_desired_c = Nao->GetXYZ_CoM_rectified(m_q_desired);
    //    XYZ_CoM_RF_desired_c = Nao->GetXYZ_CoM_RF_rectified(m_q_desired);

    XYZ_CoM_T_current = Nao->GetXYZ_CoM(m_q_current);
    XYZ_CoM_RF_current = Nao->GetXYZ_CoM_RF(m_q_current);
    //    XYZ_CoM_T_current_c = Nao->GetXYZ_CoM_rectified(m_q_current);
    //    XYZ_CoM_RF_current_c = Nao->GetXYZ_CoM_RF_rectified(m_q_current);

    std::cout << "CoMd wrt Torso (Parallel to floor frame): " << std::endl << XYZ_CoM_T_desired.transpose() << std::endl;
    std::cout << "CoMc wrt Torso (Parallel to floor frame): " << std::endl << XYZ_CoM_T_current.transpose() << std::endl;

    std::cout << "CoMd wrt RFoot (Parallel to floor frame): " << std::endl << XYZ_CoM_RF_desired.transpose() << std::endl;
    std::cout << "CoMc wrt RFoot (Parallel to floor frame): " << std::endl << XYZ_CoM_RF_current.transpose() << std::endl;
    //std::cout << "CoM wrt RFoot2: " << std::endl << Nao->GetXYZ_CoM_RF(m_q_desired) << std::endl;

    //    XYZ_CoM_desired = Nao->GetXYZ_CoM(m_q_desired);
    //    XYZ_CoM_current = Nao->GetXYZ_CoM(m_q_current);

        //m_JacobianXY_CoM = JacobianXY_CoM_current;

    //    /// m_pos_CoM is based on a TORSO frame.
    //    m_pos_CoM[0] = XYZ_CoM_desired(0);// - m_pos_CoM_wakeup[0];
    //    m_pos_CoM[1] = XYZ_CoM_desired(1);// - m_pos_CoM_wakeup[1];

    /// m_pos_CoM is based on a RFOOT frame.
    m_pos_CoM[0] = XYZ_CoM_RF_desired(0);// - m_pos_CoM_wakeup[0];
    m_pos_CoM[1] = XYZ_CoM_RF_desired(1);// - m_pos_CoM_wakeup[1];

    def_CoM_limits();

    //std::cout << "Desired CoM position: " << m_pos_CoM_mean << std::endl;
    std::cout << "Desired CoM position wrt RFoot: " << XYZ_CoM_RF_desired.transpose() << std::endl;
    std::cout << "Current CoM position wrt RFoot: " << XYZ_CoM_RF_current.transpose() << std::endl;
    std::vector<float> q_CoM = m_motion->getCOM("Body",2,true);
    std::vector<float> q_RF = m_motion->getPosition("RLeg",2,true);
    std::vector<float> q_CoM2 = m_motion->getCOM("Body",0,true);
    std::vector<float> q_RF2 = m_motion->getPosition("RLeg",0,true);
    for(int i=0;i<q_CoM.size();i++) q_CoM[i] = q_CoM[i]*1000;
    for(int i=0;i<q_CoM2.size();i++) q_CoM2[i] = q_CoM2[i]*1000;
    std::cout << "Current(AL) CoM position -- Center Feet Frame: " << q_CoM  << std::endl;
    std::cout << "Current(AL) CoM position -- Torso Frame: " << q_CoM2  << std::endl;
    for(int i=0;i<q_CoM.size();i++) q_CoM[i] = q_CoM[i] - q_RF[i]*1000;
    for(int i=0;i<q_CoM2.size();i++) q_CoM2[i] = q_CoM2[i] - q_RF2[i]*1000;
    std::cout << "Current(AL) CoM position -- RFoot Frame: " << q_CoM  << std::endl;
    std::cout << "Current(AL) CoM position 2 -- RFoot Frame: " << q_CoM2  << std::endl;

    if( m_pos_CoM[0] > (m_security_XY_CoM[0] + max_error_CoM) |
       m_pos_CoM[0] < (m_security_XY_CoM[1] - max_error_CoM) |
       m_pos_CoM[1] > (m_security_XY_CoM[2] + max_error_CoM) |
       m_pos_CoM[1] < (m_security_XY_CoM[3] - max_error_CoM))
    //if(true)
       {
            m_isBalanced = false;

            not_balanced();

            std::cout << "\033[1;32m Not balanced accomplished \033[0m" << std::endl;

            //m_isBalanced = true;

            if(m_isBalanced)
            {


                    std::cout << "m_q_balanced: " << std::endl << m_q_balanced.transpose() << std::endl;
                    for(int i=0; i<m_numberDOF; i++)
                            m_robot_joint_values[i] = m_q_balanced[i];
                    std::cout << std::endl << "Configuration send to the robot: " << std::endl << m_robot_joint_values << std::endl;


                    m_robot_joint_values[8] = m_robot_joint_values[14]; /// LHipYawPitch = RHipYawPitch


                    std::vector<float> q_desired(m_numberDOF,0);
                    /// m_robot_joint_values in joint limits?? Another verification?? Just for make sure.
                    for (int i = 0; i<m_numberDOF; i++)
                    {
                        q_desired[i] = m_robot_joint_values[i];
                        if (q_desired[i] < m_joint_limits_max[i] && q_desired[i] > m_joint_limits_min[i])
                        {
                            //std::cout << "Balance compensed" << std::endl;
                        }
                        else if(q_desired[i] > m_joint_limits_max[i])
                        {
                            std::cout << "Joint " << i << " is over its limits." << std::endl;
                            m_robot_joint_values[i] = m_joint_limits_max[i];
                            m_isBalanced = false;
                        }
                        else if(q_desired[i] < m_joint_limits_min[i])
                        {
                            std::cout << "Joint " << i << " is under its limits." << std::endl;
                            m_robot_joint_values[i] = m_joint_limits_min[i];
                            m_isBalanced = false;
                        }

                    }
                    if(m_isBalanced)
                    {
                        std::cout << "angleInterpolation" << std::endl;
                        m_motion->angleInterpolationWithSpeed(m_robot_joint_names, m_robot_joint_values, m_motors_speed);
                        //m_motion->setAngles(m_robot_joint_names, m_robot_joint_values, m_motors_speed);
                        m_q_current = m_robot_joint_values;
                    }
                    std::cout << std::endl << "Robot's rectified configuration: " << std::endl << m_robot_joint_values << std::endl;


                m_isBalanced = false;

            }
    }

    else if (m_isDS)
    {
        m_isRectified = false;

        DS();

        std::cout << "\033[1;32m Sending data to robot \033[0m" << std::endl;

        //m_isBalanced = true;

        if(m_isRectified)
        {

            std::cout << "m_q_rectified: " << std::endl << m_q_rectified.transpose() << std::endl;
            for(int i=0; i<m_numberDOF; i++)
                    m_robot_joint_values[i] = m_q_rectified[i];
            std::cout << std::endl << "Configuration send to the robot: " << std::endl << m_robot_joint_values << std::endl;


            m_robot_joint_values[8] = m_robot_joint_values[14]; /// LHipYawPitch = RHipYawPitch


            std::vector<float> q_desired(m_numberDOF,0);
            /// m_robot_joint_values in joint limits?? Another verification?? Just for make sure.
            for (int i = 0; i<m_numberDOF; i++)
            {
                q_desired[i] = m_robot_joint_values[i];
                if (q_desired[i] < m_joint_limits_max[i] && q_desired[i] > m_joint_limits_min[i])
                {
                    //std::cout << "Balance compensed" << std::endl;
                }
                else if(q_desired[i] > m_joint_limits_max[i])
                {
                    std::cout << "Joint " << i << " is over its limits." << std::endl;
                    m_robot_joint_values[i] = m_joint_limits_max[i];
                    m_isRectified = false;
                }
                else if(q_desired[i] < m_joint_limits_min[i])
                {
                    std::cout << "Joint " << i << " is under its limits." << std::endl;
                    m_robot_joint_values[i] = m_joint_limits_min[i];
                    m_isRectified = false;
                }

            }
            if(m_isRectified)
            {
                std::cout << "angleInterpolation" << std::endl;
                m_motion->angleInterpolationWithSpeed(m_robot_joint_names, m_robot_joint_values, m_motors_speed);
                //m_motion->setAngles(m_robot_joint_names, m_robot_joint_values, m_motors_speed);
                m_q_current = m_robot_joint_values;
            }
            std::cout << std::endl << "Robot's rectified configuration: " << std::endl << m_robot_joint_values << std::endl;


            m_isRectified = false;
        }
    }

}

void robot::def_FeetHeight(float RFoot_z, float LFoot_z)
{
    std::cout << "RFoot height: " << std::endl << RFoot_z << std::endl;
    std::cout << "LFoot height: " << std::endl << LFoot_z << std::endl;
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

void robot::not_balanced()
{
    /// *********************************************** ///
    /// Move the joints to achieve a balanced position. ///
    /// *********************************************** ///

    std::cout << "\033[1;31m NOT BALANCED \033[0m" << std::endl;
    std::cout <<std::endl;

    //int modifiedConfiguration[] = {16,17,11,12,19,14,6,7,4,3,8,2};
    int modifiedConfiguration[] = {2,3,5,8,9,10,11,12,15,16,17,18,20,21,23};

    ///DGM
    Eigen::Vector2d CoM_current, CoM_desired, CoM_future;
    Eigen::VectorXd X_future(6), X_desired(6);

    std::vector<float> dq_minimization(m_numberDOF,0);
    Eigen::VectorXd ZAngleDiff(m_numberDOF), ZLimits(m_numberDOF);

    Eigen::Vector2d dCoM;
    //Eigen::VectorXd dX(12);
    Eigen::VectorXd dX(6);

    Eigen::VectorXd q_limits_mean(m_numberDOF), q_limits_delta(m_numberDOF);

    for(int i =0;i<m_numberDOF;i++)
    {
        q_limits_mean[i] = (m_joint_limits_max[i]+m_joint_limits_min[i])/2.0;
        q_limits_delta[i] = m_joint_limits_max[i]-m_joint_limits_min[i];
    }

    Eigen::VectorXd dq(m_numberDOF),dq1(m_numberDOF),dq2(m_numberDOF),dq3(m_numberDOF);

    float weightAngleDiff = -0.1; /// Weight of a minimization task (-)
    float weightLimit = -0.1; /// Weight of a minimization task (-)

    int iIteration = 0, maxIterations = 10;
    float max_error_CoM =1;// 0.5;
    float max_error_X = 1;
    int Sp_CoM = 10; // Threshold (milimeters) of how big the step of the IK can be in task space (dXp_CoM)

    bool ClampingOn = true, SolutionFound = false, ReducedX = true;

    Eigen::VectorXd q_results(m_numberDOF);

    Eigen::MatrixXd q_results_bySteps;

    CoM_desired(0) = m_pos_CoM[0];
    CoM_desired(1) = m_pos_CoM[1];

    if( m_pos_CoM[0] > m_security_XY_CoM[0]) ///m_pos_CoM_x > CoM_max_x
    {
        CoM_desired[0] = m_security_XY_CoM[0];
    }
    if(m_pos_CoM[0] < m_security_XY_CoM[1]) ///m_pos_CoM_x < CoM_min_x
    {
        CoM_desired[0] = m_security_XY_CoM[1];
    }
    if(m_pos_CoM[1] > m_security_XY_CoM[2]) ///m_pos_CoM_y > CoM_max_y
    {
        CoM_desired[1] = m_security_XY_CoM[2];
    }
    if(m_pos_CoM[1] < m_security_XY_CoM[3]) ///m_pos_CoM_y < CoM_min_y
    {
        CoM_desired[1] = m_security_XY_CoM[3];
    }

    Eigen::VectorXd q_current(m_numberDOF);
    Eigen::VectorXd q_future(m_numberDOF);
    Eigen::VectorXd q_desired(m_numberDOF);
    for(int i=0;i<m_numberDOF;i++)
    {
        q_current[i] = m_q_current[i];
        q_desired[i] = m_q_desired[i];
        q_future[i] = m_q_desired[i];
    }


    while(1)
    {
        std::cout << std::endl << "Beginning the iteration loop..." << std::endl << std::endl;
        if(iIteration!=0)
        q_future = q_current;

        /// Start the Direct Geometrical Model
        DGM(q_desired);
        X_desired = m_X_current;

        DGM(q_future);
        CoM_future = m_XY_CoM_current;

        dCoM(1) = (CoM_desired[1] - CoM_future[1]);
        dCoM(0) = (CoM_desired[0] - CoM_future[0]);

        std::cout << "Desired CoM pos : " << std::endl << CoM_desired.transpose() << std::endl;
        std::cout << "Next Not Balanced CoM pos: " << std::endl << CoM_future.transpose() << std::endl;
        std::cout << "dCoM: " << std::endl << dCoM.transpose() << std::endl;

        X_future = m_X_current;

        /// WE FIXE THE RFOOT AND WE KEEP THE LFOOT IN A PLANE/
        X_desired << m_X_Npose(0), m_X_Npose(1), m_X_Npose(2),          /// RFoot_XYZ wrt RFoot
                    X_desired(3), X_desired(4), m_X_Npose(5);             /// LFoot_XYZ wrt RFoot


        dX = X_desired - X_future;

        std::cout << "Desired Eff pos : " << std::endl << X_desired.transpose() << std::endl;
        std::cout << "Current Eff pos: " << std::endl << X_future.transpose() << std::endl;
        std::cout << "dX: " << std::endl << dX.transpose() << std::endl;

        /// Start the Direct Kinematic Model
        DKM(q_future);

        std::cout << "Current joint configuration : " << std::endl << q_current.transpose() << std::endl;
        std::cout << "Future joint configuration (NOT BALANCED): " << std::endl << q_future.transpose() << std::endl;

    //        /// Reduce XY_CoM_desired to the allowed step -- only once per solve
    //        if(ReducedX){
    //            ReducedX = false;
    //
    //            float dCoM_X	  =  CoM_desired[0]-CoM_current(0);
    //            float dCoM_Y	  =  CoM_desired[1]-CoM_current(1);
    //
    //            // This will only work for mm. Radians will be lower than Sp
    //            if(abs(dCoM_X)>Sp_CoM) {
    //                dCoM_X = dCoM_X/abs(dCoM_X)*Sp_CoM;
    //            }
    //            if(abs(dCoM_Y)>Sp_CoM) {
    //                dCoM_Y = dCoM_Y/abs(dCoM_Y)*Sp_CoM;
    //            }
    //            CoM_desired[0] = dCoM_X+CoM_current(0);
    //            CoM_desired[1] = dCoM_Y+CoM_current(1);
    //
    //            //std::cout << "Desired XY CoM: " << std::endl << CoM_desired << std::endl;
    //            //std::cout << "Current XY CoM: " << std::endl << CoM_current << std::endl;
    //            //dCoM(0) = (CoM_desired[0] - CoM_current[0]);
    //            //dCoM(1) = (CoM_desired[1] - CoM_current[1]);
    //        }

        // CHECK IF SOLUTION WAS FOUND
        // Limits
        bool LimitsOK = true;
        for(int i=0; i<m_numberDOF; i++)
            if (q_current[i] < m_joint_limits_min[i] || q_current[i] > m_joint_limits_max[i]){
                LimitsOK = false;
                std::cout << " i: " << i;
            }
        if (!LimitsOK) std::cout << std::endl;

        // Balance
        std::cout << "|CoM_desired_x - CoM_current_x| = " << std::endl << fabs(dCoM(0)) << std::endl;
        std::cout << "|CoM_desired_y - CoM_current_y| = " << std::endl << fabs(dCoM(1)) << std::endl;
        bool BalanceOK = true;
        if ((fabs(dCoM(0))>max_error_CoM)||(fabs(dCoM(1))>max_error_CoM))
        {
            BalanceOK = false;
        }

        std::cout << "|X_RF_desired_x - X_RF_current_x| = " << std::endl << fabs(dX(0)) << std::endl;
        std::cout << "|X_RF_desired_y - X_RF_current_y| = " << std::endl << fabs(dX(1)) << std::endl;
        std::cout << "|X_RF_desired_z - X_RF_current_z| = " << std::endl << fabs(dX(2)) << std::endl;
        std::cout << "|X_LF_desired_x - X_LF_current_x| = " << std::endl << fabs(dX(3)) << std::endl;
        std::cout << "|X_LF_desired_y - X_LF_current_y| = " << std::endl << fabs(dX(4)) << std::endl;
        std::cout << "|X_LF_desired_z - X_LF_current_z| = " << std::endl << fabs(dX(5)) << std::endl;
        // End effectors
        bool EndEffectorsOK = true;
        VectorXd dXAbs = dX.cwiseAbs();
        int numberRows = dXAbs.rows();
        if( dXAbs.maxCoeff() > max_error_X )
            EndEffectorsOK = false;

        // Complete check
        if(LimitsOK && BalanceOK && EndEffectorsOK) {
            SolutionFound = true;

            m_q_balanced=q_future;
            m_isBalanced = true;
            std::cout << "new q_current" << std::endl << m_q_balanced.transpose() << std::endl;

            std::cout << "\033[1;32mConversion!!! In " << iIteration << " iterations. \033[0m" << std::endl; // in dark green
            break;
        }

        // The limit of iterations has been reached
        if(iIteration>maxIterations) {
            // The results stay the same as the previous configuration.
            std::cout << "\033[1;31mNo conversion after " << iIteration << " iterations.\033[0m";
            SolutionFound = false;

            if(!LimitsOK) {
                std::cout << "\033[1;31m LIMITS FAIL \033[0m";
            } else {
                std::cout << "\033[1;32m LIMITS OK \033[0m";
            }
            if(!BalanceOK) {
                std::cout << "\033[1;31m BALANCE FAIL \033[0m";
            } else {
                std::cout << "\033[1;32m BALANCE OK \033[0m";
            }
            if(!EndEffectorsOK) {
                std::cout << "\033[1;31m END EFF. FAIL \033[0m"<< std::endl;
            } else {
                std::cout << "\033[1;32m END EFF. OK \033[0m"<< std::endl;
            }

            break;
        }

        /// Minimize joint differences
        /// Difference between desired joint angles and current ones
        for (int i = 0; i<m_numberDOF;i++) {
            if (std::count(modifiedConfiguration,modifiedConfiguration+15,i)==1)
            {
                dq_minimization[i] = q_current[i]  - q_desired[i];
            }
            else
            dq_minimization[i] = 0;
        }

        /// Low priority: Optimization tasks -- AngleDifferenciation & JointLimits
        for (int i=0; i<m_numberDOF; i++)
        {
            ZAngleDiff(i) = 2.0*weightAngleDiff*dq_minimization[i];
            ZLimits(i) = (2.0*weightLimit*(q_current[i]-q_limits_mean(i)))/(pow(q_limits_delta(i),2));
        }

        /// Choose priority -- tracking / balance
        /// To choose, exchange J1 and J2, X1 and X2, dimension 1 and dimension2.
        /// It's also possible to experiment only with a specific end-effector, by using their Jacobians defined above and adjusting dX and dimension accordingly
        int dimension1 = m_JacobianXY_CoM_RF.rows();
        int dimension2 = m_JacobianEndEff.rows();

        Eigen::VectorXd q_current_temp(m_numberDOF),q_current_temp2(m_numberDOF);

        /// Priority 1 -> BALANCE
        Eigen::MatrixXd J1  = m_JacobianXY_CoM_RF;

    //        J1(0,8)=0; /// LHipYawPitch doesn't take part in the calculation of new_XY_CoM
    //        J1(1,8)=0;
    //        J1(0,14)=0; /// RHipYawPitch doesn't take part in the calculation of new_XY_CoM
    //        J1(1,14)=0;
        J1(0,14)=J1(0,8);
        J1(1,14)=J1(1,8);

        Eigen::Vector2d dX1 = dCoM;

        /// Priority 2 -> Track EndEffectors
        Eigen::MatrixXd J2  = m_JacobianEndEff;
        J2(0,14)=J2(0,8);
        J2(1,14)=J2(1,8);
        J2(2,14)=J2(2,8);
        J2(3,14)=J2(3,8);
        J2(4,14)=J2(4,8);
        J2(5,14)=J2(5,8);
        Eigen::VectorXd dX2 = dX;

        /// Clamping loop according to Baerlocher
        /// Clamps joints that are outside limits to their limits and solves IK with the rest of joints
        bool Clamping = true;
        //q_current = q_future;
        while (Clamping){
            Clamping = false;

            // Get augmented Jacobians
            // J2a
            Eigen::MatrixXd J2a(dimension1+dimension2,m_numberDOF);
            for(int i=0; i<m_numberDOF; i++) {
                // First task
                for (int j=0; j<dimension1; j++)
                    J2a(j,i) = J1(j,i);
                // Second task
                for (int j=0; j<dimension2; j++)
                    J2a(dimension1+j,i) = J2(j,i);
            }

            std::cout << std::endl << "Begin the clamping loop..." << std::endl << std::endl;

            std::cout << "X_desired - X_future = " << std::endl << dX2.transpose() << std::endl;
            std::cout << "CoM_desired - CoM_future = " << std::endl << dX1.transpose() << std::endl;
            std::cout << "" << std::endl << J1(0,17) << std::endl;
            std::cout << "" << std::endl << J2(0,17) << std::endl;

            /// Get pseudo inverses
            /// These matrices are sparse and wide. The right pinv seems to be working, but I don't know how robust it is.
            //Eigen::MatrixXd J1_i = pinvRight2(J1);

            Eigen::MatrixXd J1_=J1;
            CompleteOrthogonalDecomposition<Ref<MatrixXd> > COD_1(J1_);
            Eigen::MatrixXd J2a_=J2a;
            CompleteOrthogonalDecomposition<Ref<MatrixXd> > COD_2(J2a_);

            ///Projection matrix.
            Eigen::MatrixXd P1 = Eigen::MatrixXd::Identity(m_numberDOF,m_numberDOF) - COD_1.solve(J1);
            Eigen::MatrixXd P2a = Eigen::MatrixXd::Identity(m_numberDOF,m_numberDOF) - COD_2.solve(J2a);

            MatrixXd J2P1a = J2*P1;
            Eigen::MatrixXd J2P1a_=J2P1a;
            CompleteOrthogonalDecomposition<Ref<MatrixXd> > COD_3(J2P1a_);

            dq1 = COD_1.solve(dX1);
            dq = dq1 + COD_3.solve(dX2-J2*dq1);
            //dq2 = dq1 + COD_3.solve(dX2-J2*dq1);
    //            dq3 = dq2 + P2a*ZLimits;
                //dq = dq2 + P2a*ZAngleDiff;


            std::cout<<"dq: "<<std::endl<< dq <<std::endl;

            /// ON CHOISIT LA PREMIERE METHODE. POURQUOI????
    //            std::cout << "q_current: " << std::endl << q_current << std::endl << std::endl;
    //            q_current_temp = q_current;
    //            q_current = q_current + dq;
    //            q_current_temp2 = q_current;
    //            std::cout << "q_current + dq: " << std::endl << q_current << std::endl << std::endl;

            //dq = HQP_solver2(J1,dX1,q_future);
    //            std::cout << "q_future: " << std::endl << q_current << std::endl << std::endl;
            std::cout << "q_future: " << std::endl << q_future << std::endl << std::endl;
    //            q_current_temp = q_current;
    //            q_current = q_current + dq;
            q_current_temp = q_future;
            q_current = q_future + dq;
            q_current_temp2 = q_current;
            std::cout << "q_current + dq: " << std::endl << q_current << std::endl << std::endl;

            //Eigen::VectorXd dQ(m_numberDOF);
            //dq = HQP_solver2(J1,dX1,q_future);

            //std::cout << "Difference de méthode: " << std::endl << q_current_temp2 - dq << std::endl << std::endl;

            // Baerlocher
            // Check if result is within joint limits
            if(ClampingOn) {

                double margin = 0.001;
                for(int i=0; i<m_numberDOF; i++) {
                    if (q_current_temp2[i] < m_joint_limits_min[i] || q_current_temp2(i) > m_joint_limits_max[i]){
                            std::cout << "Joint over its limits" << std::endl;
                            Clamping = true;
                            q_future = q_current_temp;
                            // Fix the trouble angle to its limit
                            if (q_current_temp2(i) < m_joint_limits_min[i] ) {
                                q_future[i] = m_joint_limits_min[i]+margin;
                                std::cout << "Joint " << i+1 << " (" << i << ") clamped to its min limit." << std::endl;
                            }
                            if (q_current_temp2(i) > m_joint_limits_max[i] ) {
                                q_future[i] = m_joint_limits_max[i]-margin;
                                std::cout << "Joint " << i+1 << " (" << i << ") clamped to its max limit." << std::endl;
                            }
                            // Zero column i of the Jacobian
                            for (int l=0; l<dimension1; l++){
                                    J1(l,i) = 0;
                                    std::cout << i << std::endl << J1(l,i) << std::endl;
                            }
                            for (int l=0; l<dimension2; l++){
                                    J2(l,i) = 0;
                                    //std::cout << dimension2 << std::endl;
                                    std::cout << i << std::endl << J2(l,i) << std::endl;
                            }
                    }
                    else
                        std::cout << "Joints in their limits!!!" << std::endl;
                }
            }

        //q_current_mod = q_current_temp2;
        } /// end clamping loop

        std::cout << std::endl << "Clamping loop ended" << std::endl << std::endl;

        if (iIteration == 0) {
            q_results_bySteps = dq.transpose();
        }
        else {
            // Append this dq
            int numberRows = q_results_bySteps.rows();
            q_results_bySteps.conservativeResize(numberRows+1,Eigen::NoChange);
            q_results_bySteps.row(numberRows) = dq.transpose();
        }

        // This makes sure no crazy results come out, because sometimes nans and infs come and destroy everything
        for(int i=0; i<m_numberDOF; i++) {
            if( q_current[i] != q_current[i] )	// check if its NaN
                q_current[i] = q_current_temp(i);
            if( q_current[i] > 1.0e2)// thats a very large upper limit, no?
            {
                std::cout << "inf " << i << std::endl;
                q_current[i] = q_current_temp(i);
            }
        }

        // Making sure the results are in the desired quadrant -- it seems to always be --  get rid of temp2?
        // Limit to 2*pi
        for(int i=0; i<m_numberDOF; i++)
            q_current[i] = fmod((double) q_current[i], 2*M_PI);
        // Limit to between -pi and pi
        for(int i=0; i<m_numberDOF; i++) {
            if(( q_current[i] >= M_PI ) && ( q_current[i] <= 2*M_PI))
                    q_current[i] = q_current[i] - 2*M_PI;
            else if(( q_current[i] <- M_PI ) && ( q_current[i] >- 2*M_PI ))
                q_current[i] = q_current[i] + 2*M_PI;
        }

        std::vector<float> q2(m_numberDOF,0);
        for(int i =0; i<m_numberDOF;i++) q2[i] = q_current[i];
    //        std::cout<<std::endl<<"New CoM: "<<std::endl<< Nao->GetXYZ_CoM_RF_rectified(q2)<<std::endl;
        std::cout<<std::endl<<"New CoM: "<<std::endl<< Nao->GetXYZ_CoM_RF(q2)<<std::endl;

        std::cout << "Iteration n° " << iIteration << std::endl;
        std::cout << std::endl << std::endl;
        iIteration=iIteration+1;
        std::cout << "new q_current: " << std::endl << q_current << std::endl << std::endl;

    } /// end of while loop
    if(SolutionFound)
    {
        std::cout << "\033[1;32m SOLUTION FOUND \033[0m" << std::endl << std::endl;
        m_q_results_byStepsFinal = q_results_bySteps;
        q_results_bySteps.conservativeResize(1,m_numberDOF);
    }

    std::cout << std::endl << std::endl;

}

void robot::DS()
{
    /// *********************************************** ///
    /// Move the joints to achieve a balanced position. ///
    /// *********************************************** ///

    std::cout << "\033[1;31m BALANCED IN DOUBLE SUPPORT \033[0m" << std::endl;
    std::cout <<std::endl;

    //int modifiedConfiguration[] = {16,17,11,12,19,14,6,7,4,3,8,2};
    int modifiedConfiguration[] = {2,3,5,8,9,10,11,12,15,16,17,18,20,21,23};

    ///DGM
    Eigen::Vector2d CoM_current, CoM_desired, CoM_future;
    //Eigen::VectorXd X_future(12), X_desired(12);
    Eigen::VectorXd X_future(6), X_desired(6);

    std::vector<float> dq_minimization(m_numberDOF,0);
    Eigen::VectorXd ZAngleDiff(m_numberDOF), ZLimits(m_numberDOF);

    Eigen::Vector2d dCoM;
    //Eigen::VectorXd dX(12);
    Eigen::VectorXd dX(6);

    Eigen::VectorXd q_limits_mean(m_numberDOF), q_limits_delta(m_numberDOF);

    for(int i =0;i<m_numberDOF;i++)
    {
        q_limits_mean[i] = (m_joint_limits_max[i]+m_joint_limits_min[i])/2.0;
        q_limits_delta[i] = m_joint_limits_max[i]-m_joint_limits_min[i];
    }

    Eigen::VectorXd dq(m_numberDOF),dq1(m_numberDOF),dq2(m_numberDOF),dq3(m_numberDOF);

    float weightAngleDiff = -0.1; /// Weight of a minimization task (-)
    float weightLimit = -0.1; /// Weight of a minimization task (-)

    int iIteration = 0, maxIterations = 10;
    float max_error_X = 1;

    bool ClampingOn = true, SolutionFound = false, ReducedX = true;

    Eigen::VectorXd q_results(m_numberDOF);

    Eigen::MatrixXd q_results_bySteps;

    Eigen::VectorXd q_current(m_numberDOF);
    Eigen::VectorXd q_future(m_numberDOF);
    Eigen::VectorXd q_desired(m_numberDOF);
    for(int i=0;i<m_numberDOF;i++)
    {
        q_current[i] = m_q_current[i];
        q_desired[i] = m_q_desired[i];
        q_future[i] = m_q_desired[i];
    }


    while(1)
    {
        std::cout << std::endl << "Beginning the iteration loop..." << std::endl << std::endl;
        if(iIteration!=0)
        q_future = q_current;
        /// Start the Direct Geometrical Model

        DGM(q_future);
        X_future = m_X_current;

        /// WE FIXE THE RFOOT AND WE KEEP THE LFOOT IN THE XY PLANE ///
        //        X_desired << m_X_Npose(0), m_X_Npose(1), m_X_Npose(2), m_X_Npose(3), m_X_Npose(4), m_X_Npose(5),         /// RFoot_6D wrt RFoot
        //                    X_future(6), X_future(7), m_X_Npose(8), m_X_Npose(9), m_X_Npose(10), m_X_Npose(11);             /// LFoot_6D wrt RFoot

        X_desired << m_X_Npose(0), m_X_Npose(1), m_X_Npose(2),         /// RFoot_6D wrt RFoot
                    X_future(3), X_future(4), m_X_Npose(5);             /// LFoot_6D wrt RFoot

        dX = X_desired - X_future;

        std::cout << "Desired Eff pos : " << std::endl << X_desired.transpose() << std::endl;
        std::cout << "Current Eff pos: " << std::endl << X_future.transpose() << std::endl;
        std::cout << "dX: " << std::endl << dX.transpose() << std::endl;

        /// Start the Direct Kinematic Model
        DKM(q_future);

        std::cout << "Current joint configuration : " << std::endl << q_current.transpose() << std::endl;
        std::cout << "Future joint configuration (NOT BALANCED): " << std::endl << q_future.transpose() << std::endl;


        // CHECK IF SOLUTION WAS FOUND
        // Limits
        bool LimitsOK = true;
        for(int i=0; i<m_numberDOF; i++)
            if (q_current[i] < m_joint_limits_min[i] || q_current[i] > m_joint_limits_max[i]){
                LimitsOK = false;
                std::cout << " i: " << i;
            }
        if (!LimitsOK) std::cout << std::endl;


        std::cout << "|X_desired_x - X_current_x| = " << std::endl << fabs(dX(0)) << std::endl;
        std::cout << "|X_desired_y - X_current_y| = " << std::endl << fabs(dX(1)) << std::endl;
        // End effectors
        bool EndEffectorsOK = true;
        VectorXd dXAbs = dX.cwiseAbs();
        int numberRows = dXAbs.rows();
        if( dXAbs.maxCoeff() > max_error_X )
            EndEffectorsOK = false;

        // Complete check
        if(LimitsOK && EndEffectorsOK) {
            SolutionFound = true;

            m_q_rectified=q_future;
            m_isRectified = true;
            std::cout << "new q_current" << std::endl << m_q_rectified.transpose() << std::endl;

            std::cout << "\033[1;32mConversion!!! In " << iIteration << " iterations. \033[0m" << std::endl; // in dark green
            break;
        }

        // The limit of iterations has been reached
        if(iIteration>maxIterations) {
            // The results stay the same as the previous configuration.
            std::cout << "\033[1;31mNo conversion after " << iIteration << " iterations.\033[0m";
            SolutionFound = false;

            if(!LimitsOK) {
                std::cout << "\033[1;31m LIMITS FAIL \033[0m";
            } else {
                std::cout << "\033[1;32m LIMITS OK \033[0m";
            }
            if(!EndEffectorsOK) {
                std::cout << "\033[1;31m END EFF. FAIL \033[0m"<< std::endl;
            } else {
                std::cout << "\033[1;32m END EFF. OK \033[0m"<< std::endl;
            }

            break;
        }

        /// Minimize joint differences
        /// Difference between desired joint angles and current ones
        for (int i = 0; i<m_numberDOF;i++) {
            if (std::count(modifiedConfiguration,modifiedConfiguration+15,i)==1)
            {
                dq_minimization[i] = q_current[i]  - q_desired[i];
            }
            else
            dq_minimization[i] = 0;
        }

        /// Low priority: Optimization tasks -- AngleDifferenciation & JointLimits
        for (int i=0; i<m_numberDOF; i++)
        {
            ZAngleDiff(i) = 2.0*weightAngleDiff*dq_minimization[i];
            ZLimits(i) = (2.0*weightLimit*(q_current[i]-q_limits_mean(i)))/(pow(q_limits_delta(i),2));
        }

        /// SINGLE TASK REQUIERED:
        /// RFoot fixed ans LFoot constrained in the XY plane (LFoot height = 0);
        int dimension1 = m_JacobianEndEff.rows();

        Eigen::VectorXd q_current_temp(m_numberDOF),q_current_temp2(m_numberDOF);

        /// Track EndEffectors
        Eigen::MatrixXd J1  = m_JacobianEndEff;
        Eigen::VectorXd dX1 = dX;

        /// ESSAI --> Converge vers 0???
        int lambda = 1;
        //dX_CoM = -lambda*dX_CoM;

        /// Clamping loop according to Baerlocher
        /// Clamps joints that are outside limits to their limits and solves IK with the rest of joints
        bool Clamping = true;
        q_current = q_future;
        while (Clamping){
            Clamping = false;

            std::cout << std::endl << "Begin the clamping loop..." << std::endl << std::endl;

            std::cout << "X_desired - X_future = " << std::endl << dX1.transpose() << std::endl;


            /// Get pseudo inverses
            /// These matrices are sparse and wide. The right pinv seems to be working, but I don't know how robust it is.
            //Eigen::MatrixXd J1_i = pinvRight2(J1);

            Eigen::MatrixXd J1_=J1;
            CompleteOrthogonalDecomposition<Ref<MatrixXd> > COD_1(J1_);

            ///Projection matrix.
            Eigen::MatrixXd P1 = Eigen::MatrixXd::Identity(m_numberDOF,m_numberDOF) - COD_1.solve(J1);


            dq2 = COD_1.solve(dX1);
            dq = dq2 + P1*ZAngleDiff;
            //dq = dq1 + P1*ZLimits;

            std::cout<<"dq: "<<std::endl<< dq <<std::endl;

            std::cout << "q_future: " << std::endl << q_current << std::endl << std::endl;
            q_current_temp = q_current;
            q_current = q_current + dq;
            q_current_temp2 = q_current;
            std::cout << "q_current + dq: " << std::endl << q_current << std::endl << std::endl;

            // Baerlocher
            // Check if result is within joint limits
            if(ClampingOn) {

                double margin = 0.01;
                for(int i=0; i<m_numberDOF; i++) {
                    if (q_current_temp2[i] < m_joint_limits_min[i] || q_current_temp2(i) > m_joint_limits_max[i]){
                            std::cout << "Joint over its limits" << std::endl;
                            Clamping = true;
                            // Back to previous solution
                            q_current = q_current_temp;
                            //q_current = q_current_temp;
                            // Fix the trouble angle to its limit
                            if (q_current_temp2(i) < m_joint_limits_min[i] ) {
                                q_current[i] = m_joint_limits_min[i]+margin;
                                std::cout << "Joint " << i+1 << " (" << i << ") clamped to its min limit." << std::endl;
                            }
                            if (q_current_temp2(i) > m_joint_limits_max[i] ) {
                                q_current[i] = m_joint_limits_max[i]-margin;
                                std::cout << "Joint " << i+1 << " (" << i << ") clamped to its max limit." << std::endl;
                            }
                            // Zero column i of the Jacobian
//                            for (int j=0; j<dimension1; j++) J1(j,i) = 0;
                            for (int j=0; j<dimension1; j++) J1(j,i) = 0;
                    }
                    else
                        std::cout << "Joints in their limits!!!" << std::endl;
                }
            }

        //q_current_mod = q_current_temp2;
        } /// end clamping loop

        std::cout << std::endl << "Clamping loop ended" << std::endl << std::endl;

        if (iIteration == 0) {
            q_results_bySteps = dq.transpose();
        }
        else {
            // Append this dq
            int numberRows = q_results_bySteps.rows();
            q_results_bySteps.conservativeResize(numberRows+1,Eigen::NoChange);
            q_results_bySteps.row(numberRows) = dq.transpose();
        }

        // This makes sure no crazy results come out, because sometimes nans and infs come and destroy everything
        for(int i=0; i<m_numberDOF; i++) {
            if( q_current[i] != q_current[i] )	// check if its NaN
                q_current[i] = q_current_temp(i);
            if( q_current[i] > 1.0e2)// thats a very large upper limit, no?
            {
                std::cout << "inf " << i << std::endl;
                q_current[i] = q_current_temp(i);
            }
        }

        // Making sure the results are in the desired quadrant -- it seems to always be --  get rid of temp2?
        // Limit to 2*pi
        for(int i=0; i<m_numberDOF; i++)
            q_current[i] = fmod((double) q_current[i], 2*M_PI);
        // Limit to between -pi and pi
        for(int i=0; i<m_numberDOF; i++) {
            if(( q_current[i] >= M_PI ) && ( q_current[i] <= 2*M_PI))
                    q_current[i] = q_current[i] - 2*M_PI;
            else if(( q_current[i] <- M_PI ) && ( q_current[i] >- 2*M_PI ))
                q_current[i] = q_current[i] + 2*M_PI;
        }

        std::cout << "Iteration n° " << iIteration << std::endl;
        std::cout << std::endl << std::endl;
        iIteration=iIteration+1;
        std::cout << "new q_current: " << std::endl << q_current << std::endl << std::endl;

    } /// end of while loop
    if(SolutionFound)
    {
        std::cout << "\033[1;32m SOLUTION FOUND \033[0m" << std::endl << std::endl;
        m_q_results_byStepsFinal = q_results_bySteps;
        q_results_bySteps.conservativeResize(1,m_numberDOF);
    }

    std::cout << std::endl << std::endl;

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
    if(m_name == "NAO")
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
        m_posture->goToPosture("Stand",0.2F);

}

void robot::stand_zero()
{
    /// Go to StandZero posture
    m_posture->goToPosture("StandZero",0.2F);
}

void robot::def_interpreted_robot_angle_wakeup()
{
    std::vector<float> interpreted_robot_angle_wakeup(m_numberDOF,0);

    interpreted_robot_angle_wakeup = m_motion->getAngles(m_robot_joint_names, false);
    m_q_current = interpreted_robot_angle_wakeup;
    Eigen::VectorXd q(m_numberDOF);
    for(int i=0;i<m_numberDOF;i++)q[i]=m_q_current[i];
    //m_X_Npose << 0,0,0,0,0,0; /// INITIALIZATION.
    DGM(q);
    m_X_Npose = m_X_current;
    m_X_CoM_Npose = m_XY_CoM_current;

    std::cout << "Position of Feet wrt RF (NPose): " << std::endl << m_X_Npose << std::endl;
    std::cout << "Position of CoM wrt RF (NPose): " << std::endl << m_X_CoM_Npose << std::endl;

    for(int i=0; i<m_numberDOF; i++)
    {
        interpreted_robot_angle_wakeup[i] = interpreted_robot_angle_wakeup[i]*RAD2DEG;
    }

    m_interpreted_robot_angle_wakeup= interpreted_robot_angle_wakeup;
    m_interpreted_robot_angle = m_interpreted_robot_angle_wakeup;
}

void robot::def_interpreted_robot_angle_wakeup_from_xsens_joints(std::vector<joint_buffer> xsens_joint_Npose)
{
    m_xsens_joint_Npose.clear();
    // Receive new pose
    m_xsens_joint_Npose = xsens_joint_Npose;
}

void robot::def_interpreted_robot_angle_from_xsens_joints( std::vector<joint_buffer> xsens_joint)
{
    std::vector<float> interpreted_robot_angle(m_numberDOF,0);

    // TEST
    int A=40;  //A and B are the Xsens angle range for the Hands caption so L and RHand
    int B=30;

    float coef_shoulder=2;

    if(m_name == "NAO" && m_mode == 2)
    {

        //HEAD
        // HeadYaw
        interpreted_robot_angle[0]     = xsens_joint[5].rotation_z - m_xsens_joint_Npose[5].rotation_z + m_interpreted_robot_angle_wakeup[0];

        // HeadPitch
        interpreted_robot_angle[1]     = xsens_joint[5].rotation_y - m_xsens_joint_Npose[5].rotation_y + m_interpreted_robot_angle_wakeup[1];

        // LShoulderPitch
        interpreted_robot_angle[2]		= (xsens_joint[11].rotation_y - m_xsens_joint_Npose[11].rotation_y) + m_interpreted_robot_angle_wakeup[2];
        if (interpreted_robot_angle[2]<0)
            interpreted_robot_angle[2]=interpreted_robot_angle[2]*coef_shoulder;

        // LShoulderRoll
        interpreted_robot_angle[3]		= xsens_joint[11].rotation_x - m_xsens_joint_Npose[11].rotation_x + m_interpreted_robot_angle_wakeup[3];

        // LElbowYaw
        interpreted_robot_angle[4]     = (xsens_joint[11].rotation_z - m_xsens_joint_Npose[11].rotation_z) + m_interpreted_robot_angle_wakeup[4];

        // LElbowRoll
        interpreted_robot_angle[5]     = (xsens_joint[12].rotation_x - m_xsens_joint_Npose[12].rotation_x) + m_interpreted_robot_angle_wakeup[5];

        //LWristYaw
        interpreted_robot_angle[6]     = (xsens_joint[12].rotation_z - m_xsens_joint_Npose[12].rotation_z) + m_interpreted_robot_angle_wakeup[6];

        //LHand
        interpreted_robot_angle[7]     = (((xsens_joint[13].rotation_y-m_xsens_joint_Npose[13].rotation_y)+A)/(A+B))*RAD2DEG;

        //LHipYawPitch
        interpreted_robot_angle[8]	= + m_interpreted_robot_angle_wakeup[8];

        //LHipRoll
        interpreted_robot_angle[9]	=  (xsens_joint[18].rotation_x - m_xsens_joint_Npose[18].rotation_x) + m_interpreted_robot_angle_wakeup[9];
        //        (xsens_joint[0].rotation_x - m_xsens_joint_Npose[0].rotation_x)
        //                                        + (xsens_joint[1].rotation_x - m_xsens_joint_Npose[1].rotation_x)
        //                                        + (xsens_joint[2].rotation_x - m_xsens_joint_Npose[2].rotation_x)
        //                                        + (xsens_joint[3].rotation_x - m_xsens_joint_Npose[3].rotation_x)+ m_interpreted_robot_angle_wakeup[9];
        //LHipPitch
        interpreted_robot_angle[10]	= (xsens_joint[18].rotation_y - m_xsens_joint_Npose[18].rotation_y) + m_interpreted_robot_angle_wakeup[10];
        //+ m_interpreted_robot_angle_wakeup[10];

        //LKneePitch
        interpreted_robot_angle[11]	= (xsens_joint[19].rotation_y - m_xsens_joint_Npose[19].rotation_y) + m_interpreted_robot_angle_wakeup[11];

        //LAnklePitch
        interpreted_robot_angle[12]	= (xsens_joint[20].rotation_y - m_xsens_joint_Npose[20].rotation_y)
        //(xsens_joint[19].rotation_y - m_xsens_joint_Npose[19].rotation_y)
        + m_interpreted_robot_angle_wakeup[12];

        //LAnkleRoll
        interpreted_robot_angle[13]	= (xsens_joint[20].rotation_x - m_xsens_joint_Npose[20].rotation_x)
        + m_interpreted_robot_angle_wakeup[13];

        //Right
        //RHipYawPitch
        interpreted_robot_angle[14]	= + m_interpreted_robot_angle_wakeup[14];

        //RHipRoll
        interpreted_robot_angle[15]	=  (xsens_joint[14].rotation_x - m_xsens_joint_Npose[14].rotation_x) + m_interpreted_robot_angle_wakeup[15];
        //        (xsens_joint[0].rotation_x - m_xsens_joint_Npose[0].rotation_x)
        //                                        + (xsens_joint[1].rotation_x - m_xsens_joint_Npose[1].rotation_x)
        //                                        + (xsens_joint[2].rotation_x - m_xsens_joint_Npose[2].rotation_x)
        //                                        + (xsens_joint[3].rotation_x - m_xsens_joint_Npose[3].rotation_x)+ m_interpreted_robot_angle_wakeup[15];

        //RHipPitch
        interpreted_robot_angle[16]	= (xsens_joint[14].rotation_y - m_xsens_joint_Npose[14].rotation_y) + m_interpreted_robot_angle_wakeup[16];

        //RKneePitch
        interpreted_robot_angle[17]	= (xsens_joint[15].rotation_y - m_xsens_joint_Npose[15].rotation_y) + m_interpreted_robot_angle_wakeup[17];

        //RAnklePitch
        interpreted_robot_angle[18]	= (xsens_joint[16].rotation_y - m_xsens_joint_Npose[16].rotation_y)
        //(xsens_joint[15].rotation_y - m_xsens_joint_Npose[15].rotation_y)
        + m_interpreted_robot_angle_wakeup[18];

        //RAnkleRoll
        interpreted_robot_angle[19]	= (xsens_joint[16].rotation_x - m_xsens_joint_Npose[16].rotation_x)
        + m_interpreted_robot_angle_wakeup[19];

        //RShoulderPitch
        interpreted_robot_angle[20]	= (xsens_joint[7].rotation_y - m_xsens_joint_Npose[7].rotation_y) + m_interpreted_robot_angle_wakeup[20];
        if (interpreted_robot_angle[20]<0)
            interpreted_robot_angle[20]=interpreted_robot_angle[20]*coef_shoulder;

        //RShoulderRoll
        interpreted_robot_angle[21]	= (xsens_joint[7].rotation_x - m_xsens_joint_Npose[7].rotation_x) + m_interpreted_robot_angle_wakeup[21];

        //RElbowYaw
        interpreted_robot_angle[22]    = (xsens_joint[7].rotation_z - m_xsens_joint_Npose[7].rotation_z) + m_interpreted_robot_angle_wakeup[22];
        //xsens_joint[9].rotation_x - m_xsens_joint_Npose[9].rotation_x

        //RElbowRoll
        interpreted_robot_angle[23]    = xsens_joint[8].rotation_x - m_xsens_joint_Npose[8].rotation_x + m_interpreted_robot_angle_wakeup[23];

        //RWristYaw
        interpreted_robot_angle[24]    = (xsens_joint[8].rotation_z - m_xsens_joint_Npose[8].rotation_z) + m_interpreted_robot_angle_wakeup[24];

        //RHand
        interpreted_robot_angle[25]    = (((xsens_joint[9].rotation_y-m_xsens_joint_Npose[9].rotation_y)+A)/(A+B))*RAD2DEG;
    }
    else if(m_name == "NAO" && m_mode == 1)
    {

        //HEAD
        // HeadYaw
        interpreted_robot_angle[0]     = xsens_joint[5].rotation_z - m_xsens_joint_Npose[5].rotation_z + m_interpreted_robot_angle_wakeup[0];

        // HeadPitch
        interpreted_robot_angle[1]     = xsens_joint[5].rotation_y - m_xsens_joint_Npose[5].rotation_y + m_interpreted_robot_angle_wakeup[1];

        // LShoulderPitch
        interpreted_robot_angle[2]		= (xsens_joint[11].rotation_y - m_xsens_joint_Npose[11].rotation_y) + m_interpreted_robot_angle_wakeup[2];
        if (interpreted_robot_angle[2]<0)
            interpreted_robot_angle[2]=interpreted_robot_angle[2]*coef_shoulder;

        // LShoulderRoll
        interpreted_robot_angle[3]		= xsens_joint[11].rotation_x - m_xsens_joint_Npose[11].rotation_x + m_interpreted_robot_angle_wakeup[3];

        // LElbowYaw
        interpreted_robot_angle[4]     = (xsens_joint[11].rotation_z - m_xsens_joint_Npose[11].rotation_z) + m_interpreted_robot_angle_wakeup[4];

        // LElbowRoll
        interpreted_robot_angle[5]     = (xsens_joint[12].rotation_x - m_xsens_joint_Npose[12].rotation_x) + m_interpreted_robot_angle_wakeup[5];

        //LWristYaw
        interpreted_robot_angle[6]     = (xsens_joint[12].rotation_z - m_xsens_joint_Npose[12].rotation_z) + m_interpreted_robot_angle_wakeup[6];

        //LHand
        interpreted_robot_angle[7]     = (((xsens_joint[13].rotation_y-m_xsens_joint_Npose[13].rotation_y)+A)/(A+B))*RAD2DEG;

        //LHipYawPitch
        interpreted_robot_angle[8]	= + m_interpreted_robot_angle_wakeup[8];

        //LHipRoll
        interpreted_robot_angle[9]	=  + m_interpreted_robot_angle_wakeup[9];

        //LHipPitch
        interpreted_robot_angle[10]	= + m_interpreted_robot_angle_wakeup[10];

        //LKneePitch
        interpreted_robot_angle[11]	= + m_interpreted_robot_angle_wakeup[11];

        //LAnklePitch
        interpreted_robot_angle[12]	= + m_interpreted_robot_angle_wakeup[12];

        //LAnkleRoll
        interpreted_robot_angle[13]	= + m_interpreted_robot_angle_wakeup[13];

        //Right
        //RHipYawPitch
        interpreted_robot_angle[14]	= + m_interpreted_robot_angle_wakeup[14];

        //RHipRoll
        interpreted_robot_angle[15]	= + m_interpreted_robot_angle_wakeup[15];

        //RHipPitch
        interpreted_robot_angle[16]	= + m_interpreted_robot_angle_wakeup[16];

        //RKneePitch
        interpreted_robot_angle[17]	= + m_interpreted_robot_angle_wakeup[17];

        //RAnklePitch
        interpreted_robot_angle[18]	= + m_interpreted_robot_angle_wakeup[18];

        //RAnkleRoll
        interpreted_robot_angle[19]	= + m_interpreted_robot_angle_wakeup[19];

        //RShoulderPitch
        interpreted_robot_angle[20]	= (xsens_joint[7].rotation_y - m_xsens_joint_Npose[7].rotation_y) + m_interpreted_robot_angle_wakeup[20];
        if (interpreted_robot_angle[20]<0)
            interpreted_robot_angle[20]=interpreted_robot_angle[20]*coef_shoulder;

        //RShoulderRoll
        interpreted_robot_angle[21]	= (xsens_joint[7].rotation_x - m_xsens_joint_Npose[7].rotation_x) + m_interpreted_robot_angle_wakeup[21];

        //RElbowYaw
        interpreted_robot_angle[22]    = (xsens_joint[7].rotation_z - m_xsens_joint_Npose[7].rotation_z) + m_interpreted_robot_angle_wakeup[22];
        //xsens_joint[9].rotation_x - m_xsens_joint_Npose[9].rotation_x

        //RElbowRoll
        interpreted_robot_angle[23]    = xsens_joint[8].rotation_x - m_xsens_joint_Npose[8].rotation_x + m_interpreted_robot_angle_wakeup[23];

        //RWristYaw
        interpreted_robot_angle[24]    = (xsens_joint[8].rotation_z - m_xsens_joint_Npose[8].rotation_z) + m_interpreted_robot_angle_wakeup[24];

        //RHand
        interpreted_robot_angle[25]    = (((xsens_joint[9].rotation_y-m_xsens_joint_Npose[9].rotation_y)+A)/(A+B))*RAD2DEG;
    }

    m_interpreted_robot_angle = interpreted_robot_angle;
}

void robot::def_joint_limits(std::vector<float> joint_limits_max, std::vector<float> joint_limits_min)
{
    AL::ALMotionProxy motion(m_robot_IP, 9559);
    for(int i=0; i<m_numberDOF; i++)
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
    m_joint_limits_min[3]=-100*DEG2RAD;
    m_joint_limits_min[8]=-100*DEG2RAD;

    for (int i = 0; i<m_numberDOF; i++)
    {
        m_interpreted_robot_angle[i]=round(m_interpreted_robot_angle[i]*20)/20*DEG2RAD; //we give a precision of 0.05 degrees*********************************
        if (m_interpreted_robot_angle[i] > m_joint_limits_max[i])
            m_interpreted_robot_angle[i] = m_joint_limits_max[i];
        if (m_interpreted_robot_angle[i] < m_joint_limits_min[i])
            m_interpreted_robot_angle[i] = m_joint_limits_min[i];
        //put q_human data into an al::value element to be sent to the robot
        m_robot_joint_values[i]=m_interpreted_robot_angle[i];

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
    if (m_name=="PEPPER")
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
    const float min_deplacement=250;
    const float max_deplacement=400;

    /// Minimum and maximum rotation of the head to rotate along z.
    //    const float threshold_rotation_deg=30 ;   //Sets the minimum angle of the head to trigger torso rotation
    //    const float max_rotation_deg=45;
    const float threshold_rotation_deg=20 ;   //Sets the minimum angle of the head to trigger torso rotation
    const float max_rotation_deg=45;

    /// Minimum & maximum velocities alowed.
    float v_max;
    float v_min;

    /// Velocities of robot. We only allow displacements along x and rotation around himself (angular velocity along z).
//    float movex=0;
    float movetheta=0;

//    float r; /// Ratio

    if(feetdistance > max_deplacement) // TEST
        m_imitate=false;
        std::cout<<"\n Imitation is disabled due since feet distance is greater than max deplacement"<<std::endl;
        robot::stand_zero();

    if(m_name == "NAO" && m_mode == 1)
    {

        v_max = 0.15;
        v_min = 0.1;

        rotation_tete = rotation_tete + m_interpreted_robot_angle_wakeup[0]; /// Calibration of the head z rotation with the wakeup pose.

        /// Moving forwards.
        if (distancepiedR>min_deplacement)
        {
            m_moving = true;
            m_velocity_x_y_theta[0] = v_max;
            m_velocity_x_y_theta[1] = 0;
            m_velocity_x_y_theta[2] = 0;
            m_motion->move(m_velocity_x_y_theta[0],m_velocity_x_y_theta[1],m_velocity_x_y_theta[2]);
            std::cout << "Moving forwards" << std::endl;
            std::cout << "movex: " << m_velocity_x_y_theta[0] << std::endl<< "movey: " << m_velocity_x_y_theta[1] << std::endl<< "movetheta: " << m_velocity_x_y_theta[2] << std::endl;
        }

        /// Moving backwards.
        else if (distancepiedL>min_deplacement)
        {
            m_moving = true;
            m_velocity_x_y_theta[0] = -v_min;
            m_velocity_x_y_theta[1] = 0;
            m_velocity_x_y_theta[2] = 0;
            m_motion->move(m_velocity_x_y_theta[0],m_velocity_x_y_theta[1],m_velocity_x_y_theta[2]);
            std::cout << "Moving backwards" << std::endl;
            std::cout << "movex: " << m_velocity_x_y_theta[0] << std::endl<< "movey: " << m_velocity_x_y_theta[1] << std::endl<< "movetheta: " << m_velocity_x_y_theta[2] << std::endl;
        }

        /// Rotating.
        else if (abs(rotation_tete)>threshold_rotation_deg)
        {
            m_moving = true;
            if (rotation_tete>0)
                movetheta=0.2+((rotation_tete - threshold_rotation_deg)/(max_rotation_deg-threshold_rotation_deg));
            else
                movetheta=-0.2+((rotation_tete + threshold_rotation_deg)/(max_rotation_deg-threshold_rotation_deg));
            m_velocity_x_y_theta[0] = 0;
            m_velocity_x_y_theta[1] = 0;
            m_velocity_x_y_theta[2] = movetheta;
            m_motion->move(m_velocity_x_y_theta[0],m_velocity_x_y_theta[1],m_velocity_x_y_theta[2]);
            std::cout << "Rotating" << std::endl;
            std::cout << "movex: " << m_velocity_x_y_theta[0] << std::endl<< "movey: " << m_velocity_x_y_theta[1] << std::endl<< "movetheta: " << m_velocity_x_y_theta[2] << std::endl;
        }

        /// STOP.
        if(distancepiedL<min_deplacement && distancepiedR<min_deplacement && abs(rotation_tete)<threshold_rotation_deg && m_moving)
        {
            m_velocity_x_y_theta[0] = 0;
            m_velocity_x_y_theta[1] = 0;
            m_velocity_x_y_theta[2] = 0;
            m_motion->move(m_velocity_x_y_theta[0],m_velocity_x_y_theta[1],m_velocity_x_y_theta[2]);
            wake_up();
            std::cout << "STOP!!" << std::endl;
            m_moving = false;
        }
    }
}

void robot::interprete_velocity()
{
    if ((m_velocity_x_y_theta[0]!=0) | (m_velocity_x_y_theta[2]!=0))
    {
        m_motion->move(m_velocity_x_y_theta[0],m_velocity_x_y_theta[1],m_velocity_x_y_theta[2]);
        m_moving=true;
        std::cout<<"WE ARE MOVING !"<<" is moving? "<<m_moving<<" movex= "<<m_velocity_x_y_theta[0]<< "movetheta= " << m_velocity_x_y_theta[2]<<std::endl;
        std::cout << "m_velocity_x_y_theta: " << std::endl << m_velocity_x_y_theta << std::endl;
        std::cout << m_motion->moveIsActive() << std::endl;
    }
    else if ((m_velocity_x_y_theta[0]==0) && (m_velocity_x_y_theta[2]==0) && m_moving)
    //else if (m_moving)
    //else
    {
        //m_motion->move(m_velocity_x_y_theta[0],m_velocity_x_y_theta[1],m_velocity_x_y_theta[2]);
        m_motion->move(0,0,0);
        std::cout<<"STOP!!!!"<<std::endl;
        std::cout << m_motion->moveIsActive() << std::endl;

        if(m_name == "NAO")
        {
            while(!m_posture->goToPosture("Stand",0.2))
            {
                //wake_up();
            }
        std::cout<<"WOKE UP"<<std::endl;
        }

        m_moving=false;
    }
    else
        std::cout << m_motion->moveIsActive() << std::endl;
}

void robot::check_balance_and_move()
{
    if(m_name == "NAO")
    {
        ///NAO only imitate angle if he is not moving
        if (m_mode==2)
        {
            /// Check if the CoM is in the balance polygon and imitate.
            m_isDS=false;
            m_isRS=false;
            m_isLS=false;

            if(m_FeetHeight(0)<10 && m_FeetHeight(1)<10)
            {
                m_isDS = true;
                std::cout << std::endl << "DOUBLE SUPPORT" << std::endl << std::endl;
            }
            else if(m_FeetHeight(0)>10)
            {
                m_isLS=true;
                std::cout << std::endl << "LEFT SUPPORT" << std::endl << std::endl;
            }
            else if(m_FeetHeight(1)>10)
            {
                m_isRS=true;
                std::cout << std::endl << "RIGHT SUPPORT" << std::endl << std::endl;
            }

            balance();
        }
        else if(m_mode==1)
        {
            /// Body imitation
            /// Send robot_joint_value to the robot
            std::cout<<"Motor speed : " << m_motors_speed <<std::endl;
            m_motion->setAngles(m_robot_joint_names, m_robot_joint_values, m_motors_speed);

            m_q_current = m_robot_joint_values;
        }
    }

}

void robot::imitation_bis(float feetdistance, float distancepiedR, float distancepiedL, float rotation_tete)
{
    ///update m_velocity if feet are far or head is turned
    update_velocity(feetdistance, distancepiedR, distancepiedL, rotation_tete);
    m_imitate = true;

    ///Check if the robot is balanced then proceed to imitate without falling
    check_balance_and_move();
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

    setEndEffector(RHand,LHand,RFoot,LFoot);
}

void robot::DGM(Eigen::VectorXd &q_current)
{
    if(m_name == "NAO")
    {
        std::vector<float> q(m_numberDOF,0);

        for(int i=0;i<m_numberDOF;i++)
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
    std::vector<float> q(m_numberDOF,0);

    for(int i=0;i<m_numberDOF;i++)
    {
        q[i] = q_current[i];
    }

    Eigen::MatrixXd Jacobian_RHand(3,m_numberDOF), Jacobian_LHand(3,m_numberDOF), Jacobian_RFoot(3,m_numberDOF), Jacobian_LFoot(3,m_numberDOF), Jacobian_RFoot_Torso(3,m_numberDOF);
    Eigen::MatrixXd JacobianEndEff(6,m_numberDOF), JacobianXY_CoM(2,m_numberDOF), JacobianXY_CoM_RF(2,m_numberDOF);

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
    Eigen::MatrixXd Rx(3,3), Ry(3,3), Rz(3,3), R(3,3);

    Rx << 1, 0, 0,
      0, cos(roll), -sin(roll),
      0, sin(roll), cos(roll);

    Ry << cos(pitch), 0, sin(pitch),
      0, 1, 0,
      -sin(pitch), 0, cos(pitch);

    Rz << cos(yaw), -sin(yaw), 0,
      sin(yaw), cos(yaw), 0,
      0, 0, 1;

    R = Rz*Ry;
    R = R*Rx;

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
    int_t n_C = 1; /// number of constraint.
    int_t H_rows = n_V;
	int_t H_cols = n_V;
    int_t H_size = H_rows*H_cols;

	Eigen::MatrixXd P(m_numberDOF,m_numberDOF);
	P = 2*J.transpose()*J;

    Eigen::Map<Eigen::VectorXd> P_vectorized(P.data(), P.size());

	real_t H[n_V*n_V];
    for (int i = 0; i < n_V*n_V; i++)
    {
        H[i] = P_vectorized[i];
    }

	// real_t H[n_V*n_V] =
	// {
	//      P(0,0), P(0,1), P(0,2), P(0,3), P(0,4), P(0,5), P(0,6), P(0,7), P(0,8), P(0,9), P(0,10), P(0,11), P(0,12), P(0,13), P(0,14),
	//      P(0,15), P(0,16), P(0,17), P(0,18), P(0,19), P(0,20), P(0,21), P(0,22), P(0,23), P(0,24), P(0,25),
	//      P(1,0), P(1,1), P(1,2), P(1,3), P(1,4), P(1,5), P(1,6), P(1,7), P(1,8), P(1,9), P(1,10), P(1,11), P(1,12), P(1,13), P(1,14),
	//      P(1,15), P(1,16), P(1,17), P(1,18), P(1,19), P(1,20), P(1,21), P(1,22), P(1,23), P(1,24), P(1,25),
	//      P(2,0), P(2,1), P(2,2), P(2,3), P(2,4), P(2,5), P(2,6), P(2,7), P(2,8), P(2,9), P(2,10), P(2,11), P(2,12), P(2,13), P(2,14),
	//      P(2,15), P(2,16), P(2,17), P(2,18), P(2,19), P(2,20), P(2,21), P(2,22), P(2,23), P(2,24), P(2,25),
	//      P(3,0), P(3,1), P(3,2), P(3,3), P(3,4), P(3,5), P(3,6), P(3,7), P(3,8), P(3,9), P(3,10), P(3,11), P(3,12), P(3,13), P(3,14),
	//      P(3,15), P(3,16), P(3,17), P(3,18), P(3,19), P(3,20), P(3,21), P(3,22), P(3,23), P(3,24), P(3,25),
	//      P(4,0), P(4,1), P(4,2), P(4,3), P(4,4), P(4,5), P(4,6), P(4,7), P(4,8), P(4,9), P(4,10), P(4,11), P(4,12), P(4,13), P(4,14),
	//      P(4,15), P(4,16), P(4,17), P(4,18), P(4,19), P(4,20), P(4,21), P(4,22), P(4,23), P(4,24), P(4,25),
	//      P(5,0), P(5,1), P(5,2), P(5,3), P(5,4), P(5,5), P(5,6), P(5,7), P(5,8), P(5,9), P(5,10), P(5,11), P(5,12), P(5,13), P(5,14),
	//      P(5,15), P(5,16), P(5,17), P(5,18), P(5,19), P(5,20), P(5,21), P(5,22), P(5,23), P(5,24), P(5,25),
	//      P(6,0), P(6,1), P(6,2), P(6,3), P(6,4), P(6,5), P(6,6), P(6,7), P(6,8), P(6,9), P(6,10), P(6,11), P(6,12), P(6,13), P(6,14),
	//      P(6,15), P(6,16), P(6,17), P(6,18), P(6,19), P(6,20), P(6,21), P(6,22), P(6,23), P(6,24), P(6,25),
	//      P(7,0), P(7,1), P(7,2), P(7,3), P(7,4), P(7,5), P(7,6), P(7,7), P(7,8), P(7,9), P(7,10), P(7,11), P(7,12), P(7,13), P(7,14),
	//      P(7,15), P(7,16), P(7,17), P(7,18), P(7,19), P(7,20), P(7,21), P(7,22), P(7,23), P(7,24), P(7,25),
	//      P(8,0), P(8,1), P(8,2), P(8,3), P(8,4), P(8,5), P(8,6), P(8,7), P(8,8), P(8,9), P(8,10), P(8,11), P(8,12), P(8,13), P(8,14),
	//      P(8,15), P(8,16), P(8,17), P(8,18), P(8,19), P(8,20), P(8,21), P(8,22), P(8,23), P(8,24), P(8,25),
	//      P(9,0), P(9,1), P(9,2), P(9,3), P(9,4), P(9,5), P(9,6), P(9,7), P(9,8), P(9,9), P(9,10), P(9,11), P(9,12), P(9,13), P(9,14),
	//      P(9,15), P(9,16), P(9,17), P(9,18), P(9,19), P(9,20), P(9,21), P(9,22), P(9,23), P(9,24), P(9,25),
	//      P(10,0), P(10,1), P(10,2), P(10,3), P(10,4), P(10,5), P(10,6), P(10,7), P(10,8), P(10,9), P(10,10), P(10,11), P(10,12), P(10,13), P(10,14),
	//      P(10,15), P(10,16), P(10,17), P(10,18), P(10,19), P(10,20), P(10,21), P(10,22), P(10,23), P(10,24), P(10,25),
	//      P(11,0), P(11,1), P(11,2), P(11,3), P(11,4), P(11,5), P(11,6), P(11,7), P(11,8), P(11,9), P(11,10), P(11,11), P(11,12), P(11,13), P(11,14),
	//      P(11,15), P(11,16), P(11,17), P(11,18), P(11,19), P(11,20), P(11,21), P(11,22), P(11,23), P(11,24), P(11,25),
	//      P(12,0), P(12,1), P(12,2), P(12,3), P(12,4), P(12,5), P(12,6), P(12,7), P(12,8), P(12,9), P(12,10), P(12,11), P(12,12), P(12,13), P(12,14),
	//      P(12,15), P(12,16), P(12,17), P(12,18), P(12,19), P(12,20), P(12,21), P(12,22), P(12,23), P(12,24), P(12,25),
	//      P(13,0), P(13,1), P(13,2), P(13,3), P(13,4), P(13,5), P(13,6), P(13,7), P(13,8), P(13,9), P(13,10), P(13,11), P(13,12), P(13,13), P(13,14),
	//      P(13,15), P(13,16), P(13,17), P(13,18), P(13,19), P(13,20), P(13,21), P(13,22), P(13,23), P(13,24), P(13,25),
	//      P(14,0), P(14,1), P(14,2), P(14,3), P(14,4), P(14,5), P(14,6), P(14,7), P(14,8), P(14,9), P(14,10), P(14,11), P(14,12), P(14,13), P(14,14),
	//      P(14,15), P(14,16), P(14,17), P(14,18), P(14,19), P(14,20), P(14,21), P(14,22), P(14,23), P(14,24), P(14,25),
	//      P(15,0), P(15,1), P(15,2), P(15,3), P(15,4), P(15,5), P(15,6), P(15,7), P(15,8), P(15,9), P(15,10), P(15,11), P(15,12), P(15,13), P(15,14),
	//      P(15,15), P(15,16), P(15,17), P(15,18), P(15,19), P(15,20), P(15,21), P(15,22), P(15,23), P(15,24), P(15,25),
	//      P(16,0), P(16,1), P(16,2), P(16,3), P(16,4), P(16,5), P(16,6), P(16,7), P(16,8), P(16,9), P(16,10), P(16,11), P(16,12), P(16,13), P(16,14),
	//      P(16,15), P(16,16), P(16,17), P(16,18), P(16,19), P(16,20), P(16,21), P(16,22), P(16,23), P(16,24), P(16,25),
	//      P(17,0), P(17,1), P(17,2), P(17,3), P(17,4), P(17,5), P(17,6), P(17,7), P(17,8), P(17,9), P(17,10), P(17,11), P(17,12), P(17,13), P(17,14),
	//      P(17,15), P(17,16), P(17,17), P(17,18), P(17,19), P(17,20), P(17,21), P(17,22), P(17,23), P(17,24), P(17,25),
	//      P(18,0), P(18,1), P(18,2), P(18,3), P(18,4), P(18,5), P(18,6), P(18,7), P(18,8), P(18,9), P(18,10), P(18,11), P(18,12), P(18,13), P(18,14),
	//      P(18,15), P(18,16), P(18,17), P(18,18), P(18,19), P(18,20), P(18,21), P(18,22), P(18,23), P(18,24), P(18,25),
	//      P(19,0), P(19,1), P(19,2), P(19,3), P(19,4), P(19,5), P(19,6), P(19,7), P(19,8), P(19,9), P(19,10), P(19,11), P(19,12), P(19,13), P(19,14),
	//      P(19,15), P(19,16), P(19,17), P(19,18), P(19,19), P(19,20), P(19,21), P(19,22), P(19,23), P(19,24), P(19,25),
	//      P(20,0), P(20,1), P(20,2), P(20,3), P(20,4), P(20,5), P(20,6), P(20,7), P(20,8), P(20,9), P(20,10), P(20,11), P(20,12), P(20,13), P(20,14),
	//      P(20,15), P(20,16), P(20,17), P(20,18), P(20,19), P(20,20), P(20,21), P(20,22), P(20,23), P(20,24), P(20,25),
	//      P(21,0), P(21,1), P(21,2), P(21,3), P(21,4), P(21,5), P(21,6), P(21,7), P(21,8), P(21,9), P(21,10), P(21,11), P(21,12), P(21,13), P(21,14),
	//      P(21,15), P(21,16), P(21,17), P(21,18), P(21,19), P(21,20), P(21,21), P(21,22), P(21,23), P(21,24), P(21,25),
	//      P(22,0), P(22,1), P(22,2), P(22,3), P(22,4), P(22,5), P(22,6), P(22,7), P(22,8), P(22,9), P(22,10), P(22,11), P(22,12), P(22,13), P(22,14),
	//      P(22,15), P(22,16), P(22,17), P(22,18), P(22,19), P(22,20), P(22,21), P(22,22), P(22,23), P(22,24), P(22,25),
	//      P(23,0), P(23,1), P(23,2), P(23,3), P(23,4), P(23,5), P(23,6), P(23,7), P(23,8), P(23,9), P(23,10), P(23,11), P(23,12), P(23,13), P(23,14),
	//      P(23,15), P(23,16), P(23,17), P(23,18), P(23,19), P(23,20), P(23,21), P(23,22), P(23,23), P(23,24), P(23,25),
	//      P(24,0), P(24,1), P(24,2), P(24,3), P(24,4), P(24,5), P(24,6), P(24,7), P(24,8), P(24,9), P(24,10), P(24,11), P(24,12), P(24,13), P(24,14),
	//      P(24,15), P(24,16), P(24,17), P(24,18), P(24,19), P(24,20), P(24,21), P(24,22), P(24,23), P(24,24), P(24,25),
	//      P(25,0), P(25,1), P(25,2), P(25,3), P(25,4), P(25,5), P(25,6), P(25,7), P(25,8), P(25,9), P(25,10), P(25,11), P(25,12), P(25,13), P(25,14),
	//      P(25,15), P(25,16), P(25,17), P(25,18), P(25,19), P(25,20), P(25,21), P(25,22), P(25,23), P(25,24), P(25,25)

    // };

    Eigen::VectorXd r(m_numberDOF);
    r = -2*(J.transpose()*dx);
    int_t g_size = n_V;
    real_t g[n_V];
    for (int i = 0; i < n_V; i++)
    {
        g[i] = r[i];
    }

    real_t A[n_C*n_V];
    for (int i = 0; i < n_C * n_V; i++) A[i] = 0;
    A[8] = -1.;
    A[14] = 1.0;

    // {
    //      0,0,0,0,0,0,0,0,-1.0,0,0,0,0,0,1.0,0,0,0,0,0,0,0,0,0,0,0
    // };

    Eigen::VectorXd dq_min(m_numberDOF), dq_max(m_numberDOF);
    real_t lb[n_V], ub[n_V];

    for(int i=0;i<m_numberDOF;i++)
    {
        dq_min[i] = 0.5*(m_joint_limits_min[i] - q_c(i));
        dq_max[i] = 0.5*(m_joint_limits_max[i] - q_c(i));
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

	HessianType hessianType=HST_INDEF;
	QProblem example(n_V,n_C, hessianType);

	Options options;
	example.setOptions( options );

	/* Solve first QP. */
	int_t nWSR = 10;
	//example.init( H,g,A,lb,ub,lbA,ubA, nWSR );
	example.init( H,g,A,lb,ub,lbA,ubA, nWSR,nullptr,0,0 );

	/* Get and print solution of first QP. */
	real_t xOpt[n_V];
	real_t yOpt[n_V+n_C];
	example.getPrimalSolution( xOpt );
	example.getDualSolution( yOpt );
	for(int i =0;i<n_V;i++)
    {
        printf( "\nxOpt[%d] = %e ;  yOpt[%d] = %e;  objVal = %e\n\n",
			i, xOpt[i],i,yOpt[i],example.getObjVal() );

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

    std::vector<float> q(m_numberDOF,0);

    Eigen::VectorXd dq_des(m_numberDOF), dq(m_numberDOF), q_current(m_numberDOF);

    Eigen::MatrixXd J_RF(3,m_numberDOF), J_LF(3,m_numberDOF);
    Eigen::VectorXd X_RF_c(m_numberDOF), X_LF_c(m_numberDOF), X_LF_d(m_numberDOF);
    Eigen::Vector2d dX_CoM_min, dX_CoM_max;

    for(int i=0;i<m_numberDOF;i++)
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

    dX_CoM_max << m_security_XY_CoM[0] - m_XY_CoM_current(0),   /// SUPPORT POLYGON ///
                m_security_XY_CoM[2] - m_XY_CoM_current(1);
    dX_CoM_min << m_security_XY_CoM[1] - m_XY_CoM_current(0),
                m_security_XY_CoM[3] - m_XY_CoM_current(1);

    int_t n_V = m_numberDOF; /// number of variables = num_DoF
    int_t n_C = 8; /// number of constraint. 6 for both feet + 2 for CoM.
    int_t H_rows = n_V;
	int_t H_cols = n_V;
    int_t H_size = H_rows*H_cols;

	Eigen::MatrixXd P = Eigen::MatrixXd::Identity(m_numberDOF,m_numberDOF);
	P = 2*P;
    Eigen::Map<Eigen::VectorXd> P_vectorized(P.data(), P.size());

	real_t H[n_V*n_V];
    for (int i = 0; i < n_V*n_V; i++)
    {
        H[i] = P_vectorized[i];
    }

    //  =
	// {
	//      P(0,0), P(0,1), P(0,2), P(0,3), P(0,4), P(0,5), P(0,6), P(0,7), P(0,8), P(0,9), P(0,10), P(0,11), P(0,12), P(0,13), P(0,14),
	//      P(0,15), P(0,16), P(0,17), P(0,18), P(0,19), P(0,20), P(0,21), P(0,22), P(0,23), P(0,24), P(0,25),
	//      P(1,0), P(1,1), P(1,2), P(1,3), P(1,4), P(1,5), P(1,6), P(1,7), P(1,8), P(1,9), P(1,10), P(1,11), P(1,12), P(1,13), P(1,14),
	//      P(1,15), P(1,16), P(1,17), P(1,18), P(1,19), P(1,20), P(1,21), P(1,22), P(1,23), P(1,24), P(1,25),
	//      P(2,0), P(2,1), P(2,2), P(2,3), P(2,4), P(2,5), P(2,6), P(2,7), P(2,8), P(2,9), P(2,10), P(2,11), P(2,12), P(2,13), P(2,14),
	//      P(2,15), P(2,16), P(2,17), P(2,18), P(2,19), P(2,20), P(2,21), P(2,22), P(2,23), P(2,24), P(2,25),
	//      P(3,0), P(3,1), P(3,2), P(3,3), P(3,4), P(3,5), P(3,6), P(3,7), P(3,8), P(3,9), P(3,10), P(3,11), P(3,12), P(3,13), P(3,14),
	//      P(3,15), P(3,16), P(3,17), P(3,18), P(3,19), P(3,20), P(3,21), P(3,22), P(3,23), P(3,24), P(3,25),
	//      P(4,0), P(4,1), P(4,2), P(4,3), P(4,4), P(4,5), P(4,6), P(4,7), P(4,8), P(4,9), P(4,10), P(4,11), P(4,12), P(4,13), P(4,14),
	//      P(4,15), P(4,16), P(4,17), P(4,18), P(4,19), P(4,20), P(4,21), P(4,22), P(4,23), P(4,24), P(4,25),
	//      P(5,0), P(5,1), P(5,2), P(5,3), P(5,4), P(5,5), P(5,6), P(5,7), P(5,8), P(5,9), P(5,10), P(5,11), P(5,12), P(5,13), P(5,14),
	//      P(5,15), P(5,16), P(5,17), P(5,18), P(5,19), P(5,20), P(5,21), P(5,22), P(5,23), P(5,24), P(5,25),
	//      P(6,0), P(6,1), P(6,2), P(6,3), P(6,4), P(6,5), P(6,6), P(6,7), P(6,8), P(6,9), P(6,10), P(6,11), P(6,12), P(6,13), P(6,14),
	//      P(6,15), P(6,16), P(6,17), P(6,18), P(6,19), P(6,20), P(6,21), P(6,22), P(6,23), P(6,24), P(6,25),
	//      P(7,0), P(7,1), P(7,2), P(7,3), P(7,4), P(7,5), P(7,6), P(7,7), P(7,8), P(7,9), P(7,10), P(7,11), P(7,12), P(7,13), P(7,14),
	//      P(7,15), P(7,16), P(7,17), P(7,18), P(7,19), P(7,20), P(7,21), P(7,22), P(7,23), P(7,24), P(7,25),
	//      P(8,0), P(8,1), P(8,2), P(8,3), P(8,4), P(8,5), P(8,6), P(8,7), P(8,8), P(8,9), P(8,10), P(8,11), P(8,12), P(8,13), P(8,14),
	//      P(8,15), P(8,16), P(8,17), P(8,18), P(8,19), P(8,20), P(8,21), P(8,22), P(8,23), P(8,24), P(8,25),
	//      P(9,0), P(9,1), P(9,2), P(9,3), P(9,4), P(9,5), P(9,6), P(9,7), P(9,8), P(9,9), P(9,10), P(9,11), P(9,12), P(9,13), P(9,14),
	//      P(9,15), P(9,16), P(9,17), P(9,18), P(9,19), P(9,20), P(9,21), P(9,22), P(9,23), P(9,24), P(9,25),
	//      P(10,0), P(10,1), P(10,2), P(10,3), P(10,4), P(10,5), P(10,6), P(10,7), P(10,8), P(10,9), P(10,10), P(10,11), P(10,12), P(10,13), P(10,14),
	//      P(10,15), P(10,16), P(10,17), P(10,18), P(10,19), P(10,20), P(10,21), P(10,22), P(10,23), P(10,24), P(10,25),
	//      P(11,0), P(11,1), P(11,2), P(11,3), P(11,4), P(11,5), P(11,6), P(11,7), P(11,8), P(11,9), P(11,10), P(11,11), P(11,12), P(11,13), P(11,14),
	//      P(11,15), P(11,16), P(11,17), P(11,18), P(11,19), P(11,20), P(11,21), P(11,22), P(11,23), P(11,24), P(11,25),
	//      P(12,0), P(12,1), P(12,2), P(12,3), P(12,4), P(12,5), P(12,6), P(12,7), P(12,8), P(12,9), P(12,10), P(12,11), P(12,12), P(12,13), P(12,14),
	//      P(12,15), P(12,16), P(12,17), P(12,18), P(12,19), P(12,20), P(12,21), P(12,22), P(12,23), P(12,24), P(12,25),
	//      P(13,0), P(13,1), P(13,2), P(13,3), P(13,4), P(13,5), P(13,6), P(13,7), P(13,8), P(13,9), P(13,10), P(13,11), P(13,12), P(13,13), P(13,14),
	//      P(13,15), P(13,16), P(13,17), P(13,18), P(13,19), P(13,20), P(13,21), P(13,22), P(13,23), P(13,24), P(13,25),
	//      P(14,0), P(14,1), P(14,2), P(14,3), P(14,4), P(14,5), P(14,6), P(14,7), P(14,8), P(14,9), P(14,10), P(14,11), P(14,12), P(14,13), P(14,14),
	//      P(14,15), P(14,16), P(14,17), P(14,18), P(14,19), P(14,20), P(14,21), P(14,22), P(14,23), P(14,24), P(14,25),
	//      P(15,0), P(15,1), P(15,2), P(15,3), P(15,4), P(15,5), P(15,6), P(15,7), P(15,8), P(15,9), P(15,10), P(15,11), P(15,12), P(15,13), P(15,14),
	//      P(15,15), P(15,16), P(15,17), P(15,18), P(15,19), P(15,20), P(15,21), P(15,22), P(15,23), P(15,24), P(15,25),
	//      P(16,0), P(16,1), P(16,2), P(16,3), P(16,4), P(16,5), P(16,6), P(16,7), P(16,8), P(16,9), P(16,10), P(16,11), P(16,12), P(16,13), P(16,14),
	//      P(16,15), P(16,16), P(16,17), P(16,18), P(16,19), P(16,20), P(16,21), P(16,22), P(16,23), P(16,24), P(16,25),
	//      P(17,0), P(17,1), P(17,2), P(17,3), P(17,4), P(17,5), P(17,6), P(17,7), P(17,8), P(17,9), P(17,10), P(17,11), P(17,12), P(17,13), P(17,14),
	//      P(17,15), P(17,16), P(17,17), P(17,18), P(17,19), P(17,20), P(17,21), P(17,22), P(17,23), P(17,24), P(17,25),
	//      P(18,0), P(18,1), P(18,2), P(18,3), P(18,4), P(18,5), P(18,6), P(18,7), P(18,8), P(18,9), P(18,10), P(18,11), P(18,12), P(18,13), P(18,14),
	//      P(18,15), P(18,16), P(18,17), P(18,18), P(18,19), P(18,20), P(18,21), P(18,22), P(18,23), P(18,24), P(18,25),
	//      P(19,0), P(19,1), P(19,2), P(19,3), P(19,4), P(19,5), P(19,6), P(19,7), P(19,8), P(19,9), P(19,10), P(19,11), P(19,12), P(19,13), P(19,14),
	//      P(19,15), P(19,16), P(19,17), P(19,18), P(19,19), P(19,20), P(19,21), P(19,22), P(19,23), P(19,24), P(19,25),
	//      P(20,0), P(20,1), P(20,2), P(20,3), P(20,4), P(20,5), P(20,6), P(20,7), P(20,8), P(20,9), P(20,10), P(20,11), P(20,12), P(20,13), P(20,14),
	//      P(20,15), P(20,16), P(20,17), P(20,18), P(20,19), P(20,20), P(20,21), P(20,22), P(20,23), P(20,24), P(20,25),
	//      P(21,0), P(21,1), P(21,2), P(21,3), P(21,4), P(21,5), P(21,6), P(21,7), P(21,8), P(21,9), P(21,10), P(21,11), P(21,12), P(21,13), P(21,14),
	//      P(21,15), P(21,16), P(21,17), P(21,18), P(21,19), P(21,20), P(21,21), P(21,22), P(21,23), P(21,24), P(21,25),
	//      P(22,0), P(22,1), P(22,2), P(22,3), P(22,4), P(22,5), P(22,6), P(22,7), P(22,8), P(22,9), P(22,10), P(22,11), P(22,12), P(22,13), P(22,14),
	//      P(22,15), P(22,16), P(22,17), P(22,18), P(22,19), P(22,20), P(22,21), P(22,22), P(22,23), P(22,24), P(22,25),
	//      P(23,0), P(23,1), P(23,2), P(23,3), P(23,4), P(23,5), P(23,6), P(23,7), P(23,8), P(23,9), P(23,10), P(23,11), P(23,12), P(23,13), P(23,14),
	//      P(23,15), P(23,16), P(23,17), P(23,18), P(23,19), P(23,20), P(23,21), P(23,22), P(23,23), P(23,24), P(23,25),
	//      P(24,0), P(24,1), P(24,2), P(24,3), P(24,4), P(24,5), P(24,6), P(24,7), P(24,8), P(24,9), P(24,10), P(24,11), P(24,12), P(24,13), P(24,14),
	//      P(24,15), P(24,16), P(24,17), P(24,18), P(24,19), P(24,20), P(24,21), P(24,22), P(24,23), P(24,24), P(24,25),
	//      P(25,0), P(25,1), P(25,2), P(25,3), P(25,4), P(25,5), P(25,6), P(25,7), P(25,8), P(25,9), P(25,10), P(25,11), P(25,12), P(25,13), P(25,14),
	//      P(25,15), P(25,16), P(25,17), P(25,18), P(25,19), P(25,20), P(25,21), P(25,22), P(25,23), P(25,24), P(25,25)

    // };

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

    real_t A[n_C*n_V];
    for (int i = 0; i < n_C*n_V; i++)
    {
        if (i < J_vectorized.size())
        {
            A[i] = J_vectorized[i];
        }
        else if ((i >= J_vectorized.size()) && (i < J_RF_vectorized.size()))
        {
            A[i] = J_RF_vectorized[i];
        }
        else if ( (i >= J_vectorized.size() + J_RF_vectorized.size() - 1) && i < J_LF_vectorized.size())
        {
            A[i] = J_LF_vectorized[i];
        }
    }
    
    //  =
    // {
    //      J(0,0),J(0,1),J(0,2),J(0,3),J(0,4),J(0,5),J(0,6),J(0,7),J(0,8),J(0,9),J(0,10),J(0,11),J(0,12),J(0,13),J(0,14),J(0,15),J(0,16),J(0,17),J(0,18),J(0,19),J(0,20),J(0,21),J(0,22),J(0,23),J(0,24),J(0,25),
    //      J(1,0),J(1,1),J(1,2),J(1,3),J(1,4),J(1,5),J(1,6),J(1,7),J(1,8),J(1,9),J(1,10),J(1,11),J(1,12),J(1,13),J(1,14),J(1,15),J(1,16),J(1,17),J(1,18),J(1,19),J(1,20),J(1,21),J(1,22),J(1,23),J(1,24),J(1,25),

    //      J_RF(0,0),J_RF(0,1),J_RF(0,2),J_RF(0,3),J_RF(0,4),J_RF(0,5),J_RF(0,6),J_RF(0,7),J_RF(0,8),J_RF(0,9),J_RF(0,10),J_RF(0,11),J_RF(0,12),J_RF(0,13),J_RF(0,14),J_RF(0,15),J_RF(0,16),J_RF(0,17),J_RF(0,18),J_RF(0,19),J_RF(0,20),J_RF(0,21),J_RF(0,22),J_RF(0,23),J_RF(0,24),J_RF(0,25),
    //      J_RF(1,0),J_RF(1,1),J_RF(1,2),J_RF(1,3),J_RF(1,4),J_RF(1,5),J_RF(1,6),J_RF(1,7),J_RF(1,8),J_RF(1,9),J_RF(1,10),J_RF(1,11),J_RF(1,12),J_RF(1,13),J_RF(1,14),J_RF(1,15),J_RF(1,16),J_RF(1,17),J_RF(1,18),J_RF(1,19),J_RF(1,20),J_RF(1,21),J_RF(1,22),J_RF(1,23),J_RF(1,24),J_RF(1,25),
    //      J_RF(2,0),J_RF(2,1),J_RF(2,2),J_RF(2,3),J_RF(2,4),J_RF(2,5),J_RF(2,6),J_RF(2,7),J_RF(2,8),J_RF(2,9),J_RF(2,10),J_RF(2,11),J_RF(2,12),J_RF(2,13),J_RF(2,14),J_RF(2,15),J_RF(2,16),J_RF(2,17),J_RF(2,18),J_RF(2,19),J_RF(2,20),J_RF(2,21),J_RF(2,22),J_RF(2,23),J_RF(2,24),J_RF(2,25),
    //      //J_RF(0,0),J_RF(0,1),J_RF(0,2),J_RF(0,3),J_RF(0,4),J_RF(0,5),J_RF(0,6),J_RF(0,7),J_RF(0,8),J_RF(0,9),J_RF(0,10),J_RF(0,11),J_RF(0,12),J_RF(0,13),J_RF(0,14),J_RF(0,15),J_RF(0,16),J_RF(0,17),J_RF(0,18),J_RF(0,19),J_RF(0,20),J_RF(0,21),J_RF(0,22),J_RF(0,23),J_RF(0,24),J_RF(0,25),
    //      //J_RF(1,0),J_RF(1,1),J_RF(1,2),J_RF(1,3),J_RF(1,4),J_RF(1,5),J_RF(1,6),J_RF(1,7),J_RF(1,8),J_RF(1,9),J_RF(1,10),J_RF(1,11),J_RF(1,12),J_RF(1,13),J_RF(1,14),J_RF(1,15),J_RF(1,16),J_RF(1,17),J_RF(1,18),J_RF(1,19),J_RF(1,20),J_RF(1,21),J_RF(1,22),J_RF(1,23),J_RF(1,24),J_RF(1,25),
    //      //J_RF(2,0),J_RF(2,1),J_RF(2,2),J_RF(2,3),J_RF(2,4),J_RF(2,5),J_RF(2,6),J_RF(2,7),J_RF(2,8),J_RF(2,9),J_RF(2,10),J_RF(2,11),J_RF(2,12),J_RF(2,13),J_RF(2,14),J_RF(2,15),J_RF(2,16),J_RF(2,17),J_RF(2,18),J_RF(2,19),J_RF(2,20),J_RF(2,21),J_RF(2,22),J_RF(2,23),J_RF(2,24),J_RF(2,25),

    //      J_LF(0,0),J_LF(0,1),J_LF(0,2),J_LF(0,3),J_LF(0,4),J_LF(0,5),J_LF(0,6),J_LF(0,7),J_LF(0,8),J_LF(0,9),J_LF(0,10),J_LF(0,11),J_LF(0,12),J_LF(0,13),J_LF(0,14),J_LF(0,15),J_LF(0,16),J_LF(0,17),J_LF(0,18),J_LF(0,19),J_LF(0,20),J_LF(0,21),J_LF(0,22),J_LF(0,23),J_LF(0,24),J_LF(0,25),
    //      J_LF(1,0),J_LF(1,1),J_LF(1,2),J_LF(1,3),J_LF(1,4),J_LF(1,5),J_LF(1,6),J_LF(1,7),J_LF(1,8),J_LF(1,9),J_LF(1,10),J_LF(1,11),J_LF(1,12),J_LF(1,13),J_LF(1,14),J_LF(1,15),J_LF(1,16),J_LF(1,17),J_LF(1,18),J_LF(1,19),J_LF(1,20),J_LF(1,21),J_LF(1,22),J_LF(1,23),J_LF(1,24),J_LF(1,25),
    //      J_LF(2,0),J_LF(2,1),J_LF(2,2),J_LF(2,3),J_LF(2,4),J_LF(2,5),J_LF(2,6),J_LF(2,7),J_LF(2,8),J_LF(2,9),J_LF(2,10),J_LF(2,11),J_LF(2,12),J_LF(2,13),J_LF(2,14),J_LF(2,15),J_LF(2,16),J_LF(2,17),J_LF(2,18),J_LF(2,19),J_LF(2,20),J_LF(2,21),J_LF(2,22),J_LF(2,23),J_LF(2,24),J_LF(2,25),
    //      //J_LF(0,0),J_LF(0,1),J_LF(0,2),J_LF(0,3),J_LF(0,4),J_LF(0,5),J_LF(0,6),J_LF(0,7),J_LF(0,8),J_LF(0,9),J_LF(0,10),J_LF(0,11),J_LF(0,12),J_LF(0,13),J_LF(0,14),J_LF(0,15),J_LF(0,16),J_LF(0,17),J_LF(0,18),J_LF(0,19),J_LF(0,20),J_LF(0,21),J_LF(0,22),J_LF(0,23),J_LF(0,24),J_LF(0,25),
    //      //J_LF(1,0),J_LF(1,1),J_LF(1,2),J_LF(1,3),J_LF(1,4),J_LF(1,5),J_LF(1,6),J_LF(1,7),J_LF(1,8),J_LF(1,9),J_LF(1,10),J_LF(1,11),J_LF(1,12),J_LF(1,13),J_LF(1,14),J_LF(1,15),J_LF(1,16),J_LF(1,17),J_LF(1,18),J_LF(1,19),J_LF(1,20),J_LF(1,21),J_LF(1,22),J_LF(1,23),J_LF(1,24),J_LF(1,25),
    //      //J_LF(2,0),J_LF(2,1),J_LF(2,2),J_LF(2,3),J_LF(2,4),J_LF(2,5),J_LF(2,6),J_LF(2,7),J_LF(2,8),J_LF(2,9),J_LF(2,10),J_LF(2,11),J_LF(2,12),J_LF(2,13),J_LF(2,14),J_LF(2,15),J_LF(2,16),J_LF(2,17),J_LF(2,18),J_LF(2,19),J_LF(2,20),J_LF(2,21),J_LF(2,22),J_LF(2,23),J_LF(2,24),J_LF(2,25)

    // };

    Eigen::VectorXd dq_min(m_numberDOF), dq_max(m_numberDOF);

    float K_lim = 0.5;

	real_t lb[n_V];
    real_t ub[n_V];

    for(int i=0;i<m_numberDOF;i++)
    {
        dq_min[i] = K_lim*(m_joint_limits_min[i] - q[i]);
        dq_max[i] = K_lim*(m_joint_limits_max[i] - q[i]);
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
    lbA[7] =  -X_LF_c(2);

	real_t ubA[n_C];
    ubA[0] = dX_CoM_max(0);
    ubA[1] = dX_CoM_max(1);

    ubA[2] = -X_RF_c(0);
    ubA[3] = -X_RF_c(1);
    ubA[4] = -X_RF_c(2);

    ubA[5] = X_LF_d(0) - X_LF_c(0);
    ubA[6] = X_LF_d(1) - X_LF_c(1);
    ubA[7] =  -X_LF_c(2);

	HessianType hessianType=HST_IDENTITY;
	//QProblem example(n_V,n_C, hessianType);
	QProblem example(n_V,n_C);

//	Options options;
//	example.setOptions( options );

	/* Solve first QP. */
	int_t nWSR = 10;

	example.init( H,g,A,lb,ub,lbA,ubA, nWSR,nullptr,0,0 );

	/* Get and print solution of first QP. */
	real_t xOpt[n_V];
	real_t yOpt[n_V+n_C];
	example.getPrimalSolution( xOpt );
	example.getDualSolution( yOpt );
	for(int i =0;i<n_V;i++)
    {
        printf( "\nxOpt[%d] = %e ;  yOpt[%d] = %e;  objVal = %e\n\n",
			i, xOpt[i],i,yOpt[i],example.getObjVal() );

        dq(i) = xOpt[i];

    }

    std::cout << "dq: " << std::endl << dq << std::endl;
    std::cout << "Nouvelle config: " << std::endl << dq + q_current << std::endl;

    return dq + q_current;
}


