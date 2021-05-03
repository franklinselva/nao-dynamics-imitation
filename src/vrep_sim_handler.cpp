#include <vrep_sim_handler.h>

VREP_HANDLER::VREP_HANDLER(std::string VREP_IP_, int VREP_PORT_, std::string ROBOT_IP_, int ROBOT_PORT_): VREP_IP(VREP_IP_), VREP_PORT(VREP_PORT_), 
            ROBOT_IP(ROBOT_IP_), ROBOT_PORT(ROBOT_PORT_)
{
    std::cout<<"[INFO] Initializing Copellia External API handler" <<std::endl;
    try 
    {
		broker = AL::ALBroker::createBroker(
		    "SimBroker",
		    "0.0.0.0",
		    54000,
		    ROBOT_IP,
		    ROBOT_PORT,
		    0);
    	std::cout<<"[INFO] Connected to Robot Broker Service" <<std::endl;
	}
	catch(const AL::ALError& /* e */) 
    {
		std::cerr << "Faild to connect broker to: "
		          << ROBOT_IP<< ":"<< ROBOT_PORT<< std::endl;
		AL::ALBrokerManager::getInstance()->killAllBroker();
		AL::ALBrokerManager::kill();
		all_ok = false;
	}

	cout << "[INFO] Connecting to CopelliaSim on " << VREP_IP << ":" << VREP_PORT << endl;
	int clientID = simxStart((simxChar*)VREP_IP.c_str(), VREP_PORT, 1, 1, 2000, 5);

	cout << "[INFO] clientID = " << clientID << endl;
	if(clientID == -1) {
		std::cerr << "Can't connect to CopelliaSim on " << VREP_IP << ":" << VREP_PORT << endl;
		all_ok = false;
	}

	cout << "[INFO] Successfuly connected to CopelliaSim" << endl;

	cout << "[INFO] Getting Nao joint handler" << endl;
	if(not this->check_robot_handler(clientID)) {
		std::cerr << "Couldn't retrieve Robot Joint Handler" << endl;
		all_ok = false;
	}

	cout << "[INFO] Creating Motion Proxy services" << endl;
    m_motion = new AL::ALMotionProxy();

    cout << "[INFO] Updating Naoqi state" << endl;
	this->update_robot_state(clientID, m_motion);

    cout << "[INFO] Starting simulation" << endl;	
	simxStartSimulation(clientID, simx_opmode_oneshot_wait);
}

VREP_HANDLER::~VREP_HANDLER()
{
    delete m_motion;
}

void VREP_HANDLER::sigint_handler(int s) {
	stop_execution = true;
}

bool VREP_HANDLER::check_robot_handler(int clientID) {
	int i=0;

	//Head
	all_ok &= (simxGetObjectHandle(clientID, "HeadYaw#", 			&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "HeadPitch#", 			&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	//Left Arm
	all_ok &= (simxGetObjectHandle(clientID, "LShoulderPitch3#",	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "LShoulderRoll3#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "LElbowYaw3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "LElbowRoll3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "LWristYaw3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	//Left Fingers
	all_ok &= (simxGetObjectHandle(clientID, "NAO_LThumbBase#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "Revolute_joint8#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "NAO_LLFingerBase#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "Revolute_joint12#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "Revolute_joint14#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "NAO_RLFingerBase#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "Revolute_joint11#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "Revolute_joint13#",	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	//Left Leg
	all_ok &= (simxGetObjectHandle(clientID, "LHipYawPitch3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "LHipRoll3#", 			&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "LHipPitch3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "LKneePitch3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "LAnklePitch3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "LAnkleRoll3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	//Right Leg
	all_ok &= (simxGetObjectHandle(clientID, "RHipYawPitch3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "RHipRoll3#", 			&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "RHipPitch3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "RKneePitch3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "RAnklePitch3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "RAnkleRoll3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	//Right Arm
	all_ok &= (simxGetObjectHandle(clientID, "RShoulderPitch3#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "RShoulderRoll3#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "RElbowYaw3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "RElbowRoll3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "RWristYaw3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	//Right Fingers
	all_ok &= (simxGetObjectHandle(clientID, "NAO_RThumbBase#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "Revolute_joint0#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "NAO_RLFingerBase#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "Revolute_joint5#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "Revolute_joint6#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "NAO_RRFingerBase#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "Revolute_joint2#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "Revolute_joint3#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);

	if(all_ok) {
		std::cout<<"[INFO] Checked Robot Handler - Success"<<std::endl;
	}
	
	return all_ok;
}

void VREP_HANDLER::update_robot_state(int clientID, AL::ALMotionProxy *motionProxy) {
	std::vector<float> joint_positions, times;
	joint_positions.resize(26);
	times.resize(26, 1.0);

	for (int joint = 0; joint < 26; ++joint)
	{
		if(joint > 7)
			simxGetJointPosition(clientID, _nao_joint_handles[joint+7], &joint_positions[joint], simx_opmode_oneshot_wait);
		else
			simxGetJointPosition(clientID, _nao_joint_handles[joint], &joint_positions[joint], simx_opmode_oneshot_wait);	

		if((joint == 7) or (joint == 25))
			joint_positions[joint] = 1.0 - joint_positions[joint];
	}

	motionProxy -> angleInterpolation("Body", joint_positions, times, true);
}

void VREP_HANDLER::set_joint_commands(int clientID, AL::ALMotionProxy *motionProxy) 
{
	std::vector<float> commandAngles = motionProxy -> getAngles("Body", false);

	for (int joint = 0, i=0; joint < 25; ++joint)
	{
		if((joint == 7) or (joint == 25))
			for (int j = 0; j < 8; ++j)
				simxSetJointTargetPosition(clientID, _nao_joint_handles[i++], 1.0 - commandAngles[joint], simx_opmode_streaming);
		else
			simxSetJointTargetPosition(clientID, _nao_joint_handles[i++], commandAngles[joint], simx_opmode_streaming);			
	}
}