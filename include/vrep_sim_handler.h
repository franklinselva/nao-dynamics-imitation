#ifndef VREP_SIM_HANDLER_H
#define VREP_SIM_HANDLER_H

#include <iostream>
#include <thread>
#include <chrono>

#include <v_repConst.h>
#include <vrep_driver.h>

#include <alcommon/albroker.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alerror/alerror.h>
#include <qi/os.hpp>

#include <boost/shared_ptr.hpp>
#include <alerror/alerror.h>
#include <alcommon/albrokermanager.h>

#include <signal.h>
#include <stdlib.h>

using namespace std;

class VREP_HANDLER
{
    public:
        VREP_HANDLER(std::string VREP_IP_, int VREP_PORT_, std::string ROBOT_IP_, int ROBOT_PORT_);
        ~VREP_HANDLER();

        void set_joint_commands(int clientID, AL::ALMotionProxy *motionProxy);
        void update_robot_state(int clientID, AL::ALMotionProxy *motionProxy);
        bool check_robot_handler(int clientID);

        void sigint_handler(int s);

        //Variables
        bool all_ok = true;

    private:
        int _nao_joint_handles[40];
        bool stop_execution = false;
        AL::ALMotionProxy *m_motion;

        boost::shared_ptr<AL::ALBroker> broker;

        std::string VREP_IP, ROBOT_IP;
        int VREP_PORT, ROBOT_PORT;
};

#endif
