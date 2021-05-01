
#include <iostream>

#include "Socket.h"
#include "robot_v3.h"
#include <conio.h>

int XSENS_segments = 23;
int robot_port = 9559;

/* ERROR LIST :
	-1 = wrong number of arguments.
	-2 = Socket initialisation error.
	-3 = Error receive socket.
*/


int main(int argc, char* argv[])
{

	/*
	---------------------
	INSTANCIATE VARIABLES
	---------------------
	*/


	// INPUT
	if (argc < 4) {
		std::cerr << "You must specify: Robot's name (in capital letters), Robot's IP, Robot's speed and its teleoperation mode (only for NAO, otherwise, put 0).\n For example : NAO 192.168.1.117 0.2 1" << std::endl;
		return -1;
	}
	std::string name = argv[1];
	std::string robot_IP = argv[2];
	float speed = float(atof(argv[3]));
	int mode;
	if(argc == 3) mode=0;
	else mode = atof(argv[4]);
	bool eye_on = true;
	bool alone_time = false;
	int XSENS_port = 9763;
	if (argc == 8) {
		std::cout << "Bool detected " << std::endl;
		eye_on = bool(atoi(argv[5])>0);
		alone_time = bool(atoi(argv[6])>0);
		XSENS_port = atoi(argv[7]);
	}

	if (alone_time)
		Sleep(3000);

	// SOCKET
	if (!Socket::Start()) {
		std::cout << "Erreur initialisation : " << Socket::GetError() << std::endl;
		return -2;
	}
	TCPSocket mainSocket;
	mainSocket.connect_receive(XSENS_port);

	// ROBOT
	robot r(name, robot_IP, mode);
	r.set_speed(speed);
	const int numberDOF = r.getDOF();
	AL::ALValue robot_joint_names;
	robot_joint_names.arraySetSize(numberDOF);
	robot_joint_names = r.get_joint_names();
	std::vector<float> interpreted_robot_angle_wakeup(numberDOF, 0);

	// PRINT QUICK CHECK
	std::cout << "Robot name : " << name
		<< "\nRobot IP : " << robot_IP << ":" << robot_port
		<< "\nRobot speed : " << speed
		<< "\nDOF : " << numberDOF
		<< std::endl;


	// BUFFERS
	char buffer[10000];
	//std::string init_ID;
	int buffer_count;
	std::vector<joint_buffer> xsens_joint, xsens_joint_Npose;
	std::vector<euler_buffer> xsens_euler;
	joint_buffer temp_joint;
	euler_buffer temp_euler;
	SOCKADDR_IN from = { 0 };
	unsigned int fromsize = sizeof from;

	//  GEOMETRIC DATA
	float distanceLFoot_torso, distanceRFoot_torso, rotation_tete;
	float FeetDistance ;
	Eigen::MatrixXd Re_Body_segments(4, 23), Abs_Body_segments(4, 23);
	Eigen::Vector3d LFoot, RFoot;

	// ITERATION INFO
	bool first = true, first_time = true;
	int n = 0, wait = 0;

	/*
	INITIALISATION OF VARIABLE DONE. WE CAN NOW WAKE THE ROBOT.
	WAKE THE ROBOT UP IF NOT
	GET HIS ANGLE VALUE
	SAVE THEM TO COMPARE THEM TO XSENS LATER
	*/
	r.wake_up();
	r.def_interpreted_robot_angle_wakeup();
	interpreted_robot_angle_wakeup = r.get_interpreted_robot_angle_wakeup();

	/*
	BEGIN MAIN COPY LOOP. ONLY STOP IF INPUT COMING OR ERROR.
	*/

	std::cout << "Main loop begin." << std::endl;
	while (!_kbhit()) {

		//if (eye_on)
		//	r.random_leds(0.5);

		//std::cout << "New iteration. PRESS ANY KEY TO STOP !" << std::endl;
		n = mainSocket.ReceiveFrom(buffer, sizeof buffer -1,from,fromsize);
		if (n < 0) {
			std::cout << "Error socket receive ! n is : " << n << std::endl;
			return -3;
		}
		buffer[n] = '\0';
		buffer_count = 24;

		char init_buffer[7] = { buffer[0], buffer[1],buffer[2],buffer[3],buffer[4],buffer[5] };
		//std::cout << "init_buffer is : " << init_buffer << std::endl;

		//THE MESSAGE IS THE ANGLE
		if (strcmp(init_buffer, "MXTP20") == 0) {
			//std::cout << "Receiving MXTP20 buffer" << std::endl;
			xsens_joint.clear();
			/*
			FOR EACH JOINT RECEIVED :
			TURN THE BUFFER INTO ACTUAL DATA
			STORE THE DATA IN XSENS_JOINT
			GO TO THE NEXT JOINT
			*/
			while (buffer_count < n) {
				temp_joint.define(buffer, buffer_count);
				xsens_joint.push_back(temp_joint);
				buffer_count += 20; // angles data contain 20 bits.
			}
			// IF IT HAS NOT BEEN DONE YET, SAVE THE WAKE UP JOINT ANGLE TO COMPARE LATER.
			if (first_time) {
				xsens_joint_Npose.clear();
				xsens_joint_Npose = xsens_joint;
				r.def_interpreted_robot_angle_wakeup_from_xsens_joints(xsens_joint_Npose);
				first_time = false;
			}

			r.def_interpreted_robot_angle_from_xsens_joints(xsens_joint);
			r.check_joint_limits();
			//r.scale2();
			rotation_tete = xsens_joint[5].rotation_z - xsens_joint_Npose[5].rotation_z;
		} //End if MXTP20

		//THE MESSAGE IS THE POSITION
		if ((strcmp(init_buffer, "MXTP02") == 0) && !first) {
			//std::cout << "Receiving MXTP02 buffer" << std::endl;
			xsens_euler.clear();
			/// 32 bits package --> obtain the absolute XYZ position coordinate.
            int torso_RHand_LHand_Rfoot_Lfoot[] = {4,10,14,17,21}; /// In the XSENS's model
			/*
			FOR EACH JOINT RECEIVED :
			TURN THE BUFFER INTO ACTUAL DATA
			STORE THE DATA IN XSENS_JOINT
			GO TO THE NEXT JOINT
			*/
			while (buffer_count < n) {
				temp_euler.define(buffer, buffer_count);
				xsens_euler.push_back(temp_euler);
				buffer_count += 32; // position data contain 32 bits.
			}

			buffer_count=24+torso_RHand_LHand_Rfoot_Lfoot[0]*32;
            temp_euler.define(buffer,buffer_count);
            float torsox, torsoy, torsoz, r_torsox, r_torsoy, r_torsoz;
            torsox=temp_euler.position_x;
            torsoy=temp_euler.position_y;
            torsoz=temp_euler.position_z;

            buffer_count=24+torso_RHand_LHand_Rfoot_Lfoot[3]*32;
            temp_euler.define(buffer,buffer_count);
            float RFoot_x, RFoot_y, RFoot_z;
            RFoot_x=temp_euler.position_x;
            RFoot_y=temp_euler.position_y;
            RFoot_z=temp_euler.position_z;

            buffer_count=24+torso_RHand_LHand_Rfoot_Lfoot[4]*32;
            temp_euler.define(buffer,buffer_count);
            float LFoot_x, LFoot_y, LFoot_z;
            LFoot_x=temp_euler.position_x;
            LFoot_y=temp_euler.position_y;
            LFoot_z=temp_euler.position_z;

			Abs_Body_segments = getAbsolutePosition(xsens_euler);

			RFoot = Abs_Body_segments.block<3, 1>(0, 17);
			LFoot = Abs_Body_segments.block<3, 1>(0, 21);

			distanceLFoot_torso = float(sqrt(pow(LFoot[0], 2) + pow(LFoot[1], 2))); //LFoot distance wrt Torso
			distanceRFoot_torso = float(sqrt(pow(RFoot[0], 2) + pow(RFoot[1], 2))); //RFoot distance wrt Torso

			r.def_FeetHeight(RFoot_z,LFoot_z);

            FeetDistance = sqrt(pow(RFoot[0]-LFoot[0], 2) + pow(LFoot[1]-RFoot[1], 2));

            std::cout << "FeetDistance: " << FeetDistance << std::endl;
            std::cout << "rotation_tete: " << rotation_tete << std::endl;

		} //End if MXTP02

		if(!first) //Some distance aren't defined the first time so we avoid mooving.
			r.imitation_bis(FeetDistance, distanceRFoot_torso, distanceLFoot_torso,rotation_tete);
		first = false;
	} //End main loop

	std::cerr << "Motion imitation terminated." << std::endl;
	Socket::Release();
	r.wake_up();
	std::cout << "Program ending." << std::endl;

	return 0;
}
