/**
 * @file xsens_imitation.cpp    
 * @author Selvakumar H S(franklinselva10@gmail.com)
 * @brief The modified version of the exisiting xsens_imitation package with static balance and zmp control
 * @version 0.1
 * @date 2021-05-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <robot.h>
#include <stdio.h>
#include <balance_control.h>

#define PORT_XSENS 9764;
#define XSENS_segments = 23;

using namespace std;
using namespace Eigen;

int main(int argc, char *argv[])
{
    if (argc < 7)
    {
        std::cout << "[Error] You must specify: \n"
                  << "Xsens IP, \n"
                  << "Xsens port, \n"
                  << "Robot's IP, \n"
                  << "Robot's Port, \n"
                  << "Robot's speed, \n"
                  << "Teleoperation mode (1 or 2 for NAO, 0 otherwise)." << endl;
        exit(2);
    }

    ///*********************INITIALISATION ROBOT**************************///

    std::cout << "Prepare yourself at NPose" << std::endl;
    std::cout << "...5..." << std::endl;
    sleep(1);
    std::cout << "...4..." << std::endl;
    sleep(1);
    std::cout << "...3..." << std::endl;
    sleep(1);
    std::cout << "...2..." << std::endl;
    sleep(1);
    std::cout << "...1..." << std::endl;
    sleep(1); /// Let some time for operator preparation.

    /// Read the arguments
    std::string robot_IP, xsens_IP;
    int xsens_port, robot_port, mode;
    float speed; // Fraction of maximum speed (normalized value (0-1))

    xsens_IP = argv[1];
    xsens_port = atof(argv[2]);
    robot_IP = argv[3];
    robot_port = atof(argv[4]);
    speed = atof(argv[5]);
    mode = atof(argv[6]);

    if (mode != 1 && mode != 2)
    {
        cerr << "You must select mode 1 or mode 2 if you want to teleoperate NAO!" << endl;
        exit(2);
    }

    /// New object of the robot class.
    robot r(robot_IP, robot_port, mode); /// Connect to "name" robot.
    balanceControl bControl;

    int numberDOF;
    r.def_DOF();
    numberDOF = r.getDOF();

    AL::ALValue robot_joint_names;
    robot_joint_names.arraySetSize(numberDOF);
    r.def_joint_names(); /// Robot's joints names
    robot_joint_names = r.get_joint_names();

    ///*********************INITILISATION SOCKET**************************///

    //connection to the xsens as a server
    SOCKET sock = init_connection_server(xsens_IP, xsens_port);
    SOCKADDR_IN from = {0};
    unsigned int fromsize = sizeof from;

    ///*********************INITILISATION VARIABLES**************************///

    char init_buffer[24];
    string init_ID;
    int buffer_count;
    vector<joint_buffer> xsens_joint;
    vector<euler_buffer> xsens_euler;
    vector<joint_buffer> xsens_joint_Npose;
    joint_buffer temp_joint;
    euler_buffer temp_euler;
    bool first = true;
    bool first_time = true;
    char buffer[10000]; //buffer to stock the packages of XSENS
    int n = 0;

    float FeetDistance;
    float distanceRFoot_torso;
    float distanceLFoot_torso;

    float rotation_tete;

    Eigen::MatrixXd Re_Body_segments(4, 23);
    Eigen::MatrixXd Abs_Body_segments(4, 23);

    Eigen::Vector3d LFoot;
    Eigen::Vector3d RFoot;

    vector<float> joint_limits_max(numberDOF, 0), joint_limits_min(numberDOF, 0);

    /// Definition of chrono
    auto start_chrono_terminal = std::chrono::steady_clock::now();
    auto elapsed_chrono_terminal = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_chrono_terminal);
    auto start_chrono_sonar = std::chrono::steady_clock::now();
    auto elapsed_chrono_sonar = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_chrono_terminal);

    /// Set robot's motor speed
    r.set_speed(speed);

    /// Wake up (if not)
    std::cout << " [INFO] Starting from NPose..." << std::endl;
    r.wake_up();

    /// Definition of the Npose of robot (or WAKE UP pose)
    r.def_interpreted_robot_angle_wakeup();
    std::vector<float> interpreted_robot_angle_wakeup(numberDOF, 0);
    interpreted_robot_angle_wakeup = r.get_interpreted_robot_angle_wakeup();

    /// Get joint limits
    r.def_joint_limits(joint_limits_max, joint_limits_min);

    ///****************************************** MAIN LOOP *******************************************

    int wait = 0; //parameters that allow an alternate command of move and setAngles

    std::cout << "PRESS ENTER TO EXIT" << std::endl;
    while (!kbhit())
    //while(true)
    {
        auto start = std::chrono::steady_clock::now();

        /// RECEIVING THE BUFFERS FROM XSENS PORT.
        if ((n = recvfrom(sock, buffer, sizeof buffer - 1, 0, (SOCKADDR *)&from, &fromsize)) < 0)
        {
            perror("recvfrom()");
            exit(errno);
        }
        buffer[n] = '\0';

        /// 24 bits package --> INITIALIZATION
        for (int i = 0; i < 6; i++)
        {
            init_buffer[i] = buffer[i];
        }
        init_ID = init_buffer;
        buffer_count = 24; //current byte

        /// ************MXTP20 is a message with: *****************///
        /// *******************************************************///
        /// Information about each joint is sent as follows. ***** ///
        /// 4 bytes point ID of parent segment connection. ********///
        /// 4 bytes point ID of child segment connection. *********///
        /// 4 bytes floating point rotation around segment x�axis. ///
        /// 4 bytes floating point rotation around segment y�axis. ///
        /// 4 bytes floating point rotation around segment z�axis. ///
        /// *******************************************************///

        if (init_ID == "MXTP20")
        {
            std::cout << "MXTP20" << std::endl;

            /// 20 bits package --> obtain xsens_vector filled with all joint rotations
            xsens_joint.clear();

            /// The xsens_joint is filled in the order of joints in the xsens software
            while (buffer_count < n)
            {
                temp_joint.define(buffer, buffer_count);
                xsens_joint.push_back(temp_joint);
                buffer_count += 20;
            }

            if (first_time)
            {
                xsens_joint_Npose.clear();
                xsens_joint_Npose = xsens_joint;

                /// Define the wake up pose (XSENS joints) for the calibration.
                r.def_interpreted_robot_angle_wakeup_from_xsens_joints(xsens_joint_Npose);

                first_time = false;
            }

            /// Transform the XSENS joint vector into a interpreted robot set of angles.
            r.def_interpreted_robot_angle_from_xsens_joints(xsens_joint);

            /// CHECK ANGLES IN LIMITS RANGE AND GET THE NEW ROBOT JOINT CONFIGURATION
            r.check_joint_limits();

            //r.scale2();

            rotation_tete = xsens_joint[5].rotation_z - xsens_joint_Npose[5].rotation_z;
        }

        /// ******************MXTP02 is a message with: ************************///
        /// ********************************************************************///
        /// Information about each segment is sent as follows. *****************///
        /// 4 bytes segment ID See 2.5.9 ***************************************///
        /// 4 bytes x�coordinate of segment position ***************************///
        /// 4 bytes y�coordinate of segment position ***************************///
        /// 4 bytes z�coordinate of segment position ***************************///
        /// 4 bytes q1 rotation � segment rotation quaternion component 1 (re). ///
        /// 4 bytes q2 rotation � segment rotation quaternion component 1 (i). *///
        /// 4 bytes q3 rotation � segment rotation quaternion component 1 (j). *///
        /// 4 bytes q4 rotation � segment rotation quaternion component 1 (k). *///
        /// ********************************************************************///

        if (init_ID == "MXTP02" && !first)
        {
            std::cout << "MXTP02" << std::endl;

            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);

            /// 32 bits package --> obtain the absolute XYZ position coordinate.
            int torso_RHand_LHand_Rfoot_Lfoot[] = {4, 10, 14, 17, 21}; /// In the XSENS's model

            xsens_euler.clear();

            while (buffer_count < n)
            {
                temp_euler.define(buffer, buffer_count);
                xsens_euler.push_back(temp_euler);
                buffer_count += 32;
            }

            if (first_time)
            {
                Eigen::MatrixXd Ab_Body_segments = getAbsolutePosition(xsens_euler);
            }

            buffer_count = 24 + torso_RHand_LHand_Rfoot_Lfoot[0] * 32;
            temp_euler.define(buffer, buffer_count);
            float torsox, torsoy, torsoz, r_torsox, r_torsoy, r_torsoz;
            torsox = temp_euler.position_x;
            torsoy = temp_euler.position_y;
            torsoz = temp_euler.position_z;

            buffer_count = 24 + torso_RHand_LHand_Rfoot_Lfoot[3] * 32;
            temp_euler.define(buffer, buffer_count);
            float RFoot_x, RFoot_y, RFoot_z;
            RFoot_x = temp_euler.position_x;
            RFoot_y = temp_euler.position_y;
            RFoot_z = temp_euler.position_z;

            buffer_count = 24 + torso_RHand_LHand_Rfoot_Lfoot[4] * 32;
            temp_euler.define(buffer, buffer_count);
            float LFoot_x, LFoot_y, LFoot_z;
            LFoot_x = temp_euler.position_x;
            LFoot_y = temp_euler.position_y;
            LFoot_z = temp_euler.position_z;

            //            distanceLFoot_torso = sqrt(pow(LFoot_x - torsox, 2) + pow(LFoot_y - torsoy, 2));
            //            distanceRFoot_torso = sqrt(pow(torsox - RFoot_x, 2) + pow(torsoy - RFoot_y, 2));

            //Re_Body_segments = getRelativePosition(xsens_euler);
            Abs_Body_segments = getAbsolutePosition(xsens_euler);

            RFoot = Abs_Body_segments.block<3, 1>(0, 17);
            LFoot = Abs_Body_segments.block<3, 1>(0, 21);

            r.def_FeetHeight(RFoot_z, LFoot_z);

            distanceLFoot_torso = sqrt(pow(LFoot[0], 2) + pow(LFoot[1], 2));
            distanceRFoot_torso = sqrt(pow(RFoot[0], 2) + pow(RFoot[1], 2));
            FeetDistance = sqrt(pow(RFoot[0] - LFoot[0], 2) + pow(LFoot[1] - RFoot[1], 2));

            std::cout << "FeetDistance: " << FeetDistance << std::endl;
            std::cout << "rotation_tete: " << rotation_tete << std::endl;
        }

        ///*************************************************///
        ///******************MOVEMENT***********************///
        ///*************************************************///
        if (!first)
        {
            /// Begin imitation of Body joints
            cout << "\033[1;34mImitating \033[0m" << endl;
            bControl.begin_imitation(FeetDistance, distanceRFoot_torso, distanceLFoot_torso, rotation_tete);
        }

        first = false;

        ///REFRESH TERMINAL DATA
        elapsed_chrono_terminal = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_chrono_terminal);
        float data;
        if (elapsed_chrono_terminal.count() > 2000)
        {
            //cout<<"distance pied: "<<distancepied<<" | moveIsActive: "<<motion.moveIsActive() <<" | movex= "<<movex<<" | movetheta = "<<movetheta<<" | data: "<<data<<endl;
            start_chrono_terminal = std::chrono::steady_clock::now();
        }

    } /// end of while() loop.

    std::cout << " " << std::endl;
    std::cout << "------------------------------------" << std::endl;
    std::cout << "Stoping imitation..." << std::endl;
    std::cout << "Reaching N-pose..." << std::endl;
    r.wake_up();
    std::cout << "N-pose." << endl;
    std::cout << "Can I go to sleep? (y/n) ";
    char answer;
    std::cin >> answer;
    if (answer == 'y')
    {
        std::cout << std::endl
                  << "Going to sleep..." << std::endl;
        r.go_to_sleep();
    }
    std::cerr << "Motion imitation terminated." << std::endl;
    r.closeFile();
    closesocket(sock);
    return 0;
}
