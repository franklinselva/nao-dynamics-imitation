/*
    CheckPEPPERmodel

    Move the robot along 3 trajectories, waist, right arm and left arm
    Then save the position of end effectors from aldebaran frames
    and from our DKM
 */

#include "xsens_tool.h"
#include <chrono>


const int numberDOF = 17;   //DOF means Degrees Of Freedom
static const float DEG2RAD = M_PI/180;
static const float RAD2DEG= 180/M_PI;

float threshold_rotation_deg=15 ;   //Sets the minimum angle of the head to trigger Pepper's torso rotation
float max_rotation_deg=40;
float speed; // Fraction of maximum speed
float xsensport;

#define BUF_SIZE 1024
#define PORT_SONAR 1234
#define PORT_XSENS 9763



using namespace std;

///MAIN///

int main(int argc, char *argv[])
{

    if(argc<4)
    {
        cerr << "You must specify: Pepper IP, Robot's speed and Phone_IP, optionnaly you can add udp_off if you don't want to send udp messages else it will be on" << endl;
        exit(2);
    }


    ///************************************************************************************************
    ///------------------------------------------------------------------------------------------------
    ///******************************************INITIALISATION****************************************
    ///------------------------------------------------------------------------------------------------
    ///************************************************************************************************


    ///************INITIALISATION PEPPER***************

    // Set names for Body
    AL::ALValue names;
    names.arraySetSize(17);

    // "RLeg" chain
    names[0] = "KneePitch";
    names[1] = "HipPitch";
    names[2] = "HipRoll";
    // "RArm" chain
    names[3] = "RShoulderPitch";
    names[4] = "RShoulderRoll";
    names[5] = "RElbowYaw";
    names[6] = "RElbowRoll";
    names[7] = "RWristYaw";
    // "LArm" chain
    names[8] = "LShoulderPitch";
    names[9] = "LShoulderRoll";
    names[10] = "LElbowYaw";
    names[11] = "LElbowRoll";
    names[12] = "LWristYaw";
    //"head" chain
    names[13] = "HeadYaw";
    names[14] = "HeadPitch";
    //Hands
    names[15] = "RHand";
    names[16] = "LHand";

    string host ;
    host= argv[1];
    speed = atof(argv[2]); //fraction of speed

    //connection pepper
    /// DEPRECATED  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< A MODIFIER
    cout<<"connexion pepper"<<endl;
    AL::ALRobotPostureProxy posture(host, 9559);
    AL::ALMemoryProxy memory(host, 9559);
    AL::ALMotionProxy motion(host, 9559);
    AL::ALTextToSpeechProxy SpeechProxy(host, 9559);

    //set security distances
    motion.setOrthogonalSecurityDistance(0.2);  //Security distance in front of Pepper 0.2 m
    motion.setTangentialSecurityDistance(0.05); //Lateral Security distance 5 cm (allow Pepper to pass the door)
    cout<<"orthogonal security distance: "<<motion.getOrthogonalSecurityDistance()<<endl;
    cout<<"tangent security distance: "<<motion.getTangentialSecurityDistance()<<endl;

    //enable moving arms when rolling
    motion.setMoveArmsEnabled(false, false);

    //Wake up if robot isn't
    if (!motion.robotIsWakeUp())
        motion.wakeUp();
    else
        posture.goToPosture("Stand",0.2);

    // Joint values which will be sent to the robot -- again the "Body" names inconvenience
    vector<float> angles_human(numberDOF,0);
    AL::ALValue values;
    values.arraySetSize(numberDOF);

    // Get joint limits
    vector<float> q_limits_max(numberDOF,0), q_limits_min(numberDOF,0); // in my model's order
    for(int i=0; i<numberDOF; i++)
    {
        AL::ALValue limits = motion.getLimits(names[i]);
        q_limits_min[i] = limits[0][0];
        q_limits_max[i] = limits[0][1];
    }



    ///**************INITILISATION SOCKET*****************

    //connection to the phone as a client
    const char* ip_phone=argv[3];
    cout<<ip_phone<<endl;
    SOCKADDR_IN sin = { 0 };
    SOCKET sock_sonar=init_connection_client(ip_phone, &sin, PORT_SONAR);


    //connection to the xsens as a server
    SOCKET sock = init_connection_server(PORT_XSENS);
    SOCKADDR_IN from = { 0 };
    unsigned int fromsize = sizeof from;


    //Initialisation of variables
    char init_buffer[24];
    int number_of_data, buffer_count;
    vector<joint_buffer> vector_joint;
    vector<euler_buffer> vector_euler;
    joint_buffer temp_joint;
    euler_buffer temp_euler;
    bool first=true;
    float current_orientation, distancepied;
    float qw, qx, qy, qz;
    float movex,movetheta,movey,previous_theta,previous_x(0);
    bool moving=false;
    char buffer[10000]; //buffer for the
    int n=0;


    //def of chrono
    auto start_chrono_terminal = std::chrono::steady_clock::now( );
    auto elapsed_chrono_terminal = std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::steady_clock::now( ) - start_chrono_terminal );
    auto start_chrono_sonar = std::chrono::steady_clock::now( );
    auto elapsed_chrono_sonar = std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::steady_clock::now( ) - start_chrono_terminal );



    ///************************************************************************************************
    ///------------------------------------------------------------------------------------------------
    ///****************************************** MAIN LOOP *******************************************
    ///------------------------------------------------------------------------------------------------
    ///************************************************************************************************


    int wait=0; //parameters that allow an alternate command of move and setAngles

    while(!kbhit())
    {
        auto start = std::chrono::steady_clock::now( );

        if((n = recvfrom(sock, buffer, sizeof buffer - 1, 0, (SOCKADDR *)&from, &fromsize)) < 0)
        {
            perror("recvfrom()");
            exit(errno);
        }
        std::cout<<"msg received"<<std::endl;
        buffer[n] = '\0';

        /**24 bits d'initialisation**/
        for (int i=0; i<24; i++)
            init_buffer[i]=buffer[i];
        string init_ID;
        init_ID = init_buffer;
        buffer_count=24;//current byte

        if (init_ID=="MXTP20")
        {
            /**20 bits par packet de joint_buffer**/
            number_of_data=(n-24)/20; //Pas utilisé
            vector_joint.clear();

            //the vector_joint is filled in the order of joints in the xsens software
            while(buffer_count<n)
            {
                temp_joint.define(buffer,buffer_count);
                vector_joint.push_back(temp_joint);
                buffer_count+=20;
            }



            //Calibration for each joint:
            float calibration_ShoulderPitch=15; //arm in front of us xsens=calibration... and pepper=0°
            float calibration_WristYaw=90; //when in position N xsens=80 but pepper must be at 0°
            float coef_shoulder=2;


            /// get humanAngles from vector_joint in degree


            ///RIGHT
            angles_human[0] 	= 0;//vector_joint[15]+vector_joint[19];	// LKneePitch
            //On somme les mouvements de chaque vertebre de xsens
            ///!!!!!!!!!!!!!!!!!!!!!!!!!!
            angles_human[1]		= -vector_joint[0].rotation_z-vector_joint[1].rotation_z-vector_joint[2].rotation_z-vector_joint[3].rotation_z-vector_joint[14].rotation_z/2-vector_joint[18].rotation_z/2;	// LHipPitch
            angles_human[2]		= -vector_joint[0].rotation_x-vector_joint[1].rotation_x-vector_joint[2].rotation_x-vector_joint[3].rotation_x;//+(vector_joint[14].rotation_x-vector_joint[18].rotation_x)/2	// LHipRoll

            angles_human[3]		= 90-vector_joint[7].rotation_z-vector_joint[6].rotation_x-calibration_ShoulderPitch;	// RShoulderPitch
            if (angles_human[3]<0)
                angles_human[3]=angles_human[3]*coef_shoulder;

            angles_human[4]		= -1*vector_joint[7].rotation_x+5;	// RShoulderRoll *********** before, it was just -1*

            angles_human[5]      = vector_joint[8].rotation_x+60;	// RElbowYaw *********** before, it was +60
            angles_human[6]      = vector_joint[8].rotation_z;	// RElbowRoll
            angles_human[7]      = calibration_WristYaw-vector_joint[8].rotation_y;	// RWristYaw *********** before, it was 120- (make hand touch torso)


            ///LEFT
            angles_human[8]		= 90-vector_joint[11].rotation_z-vector_joint[10].rotation_x-calibration_ShoulderPitch;	// LShoulderPitch
            if (angles_human[8]<0)
                angles_human[8]=angles_human[8]*coef_shoulder;

            angles_human[9]		= vector_joint[11].rotation_x-5;	// LShoulderRoll

            angles_human[10]     = -vector_joint[12].rotation_x-60;	// LElbowYaw
            angles_human[11]     = -vector_joint[12].rotation_z;	// LElbowRoll
            angles_human[12]     = vector_joint[12].rotation_y-calibration_WristYaw;	// LWristYaw *********** before,
            angles_human[13]     = vector_joint[5].rotation_y;	// HeadYaw
            angles_human[14]     = vector_joint[5].rotation_z;	// HeadPitch


            //next 3 lines added by Nassim
            /*
                      float pr=5.; // Only for the hands command, we want to set a specific accuracy pr in ° degrees
                      angles_human[15] = -limit_interior+pr*int((angles_human[15]+limit_interior)/pr); //belong to [a;b]=[-30;40]
                      angles_human[16] = -limit_interior+pr*int((angles_human[16]+limit_interior)/pr);
              */

            ///HANDS
            int limit_exterior=40;  //limit_exterior and limit_interior are the Xsens angle range for the Hands caption so L and RHand
            int limit_interior=30;
            angles_human[15]    = 1-(vector_joint[9].rotation_z+limit_exterior)/(limit_exterior+limit_interior);   //RHand --> angles roughly between -40 and 100 and we want a ratio so we divide by 140
            angles_human[16]    = 1-(vector_joint[13].rotation_z+limit_exterior)/(limit_exterior+limit_interior);   //LHand



            //*******************************************************************************************************************************
            //*******************************************************************************************************************************
            //**********************************************       TEST        **************************************************************
            //*******************************************************************************************************************************
            //*******************************************************************************************************************************

            //calibration for the shoulder pitch so that it does not go beyond -100° which is not a natural movement
            q_limits_min[3]=-100*DEG2RAD;
            q_limits_min[8]=-100*DEG2RAD;

            for (int i = 0; i<15; i++)
            {
                angles_human[i]=round(angles_human[i]*5)/5*DEG2RAD; //we give a precision of 0.5 degrees*********************************
                if (angles_human[i]>q_limits_max[i])
                    angles_human[i] = q_limits_max[i];
                if (angles_human[i]<q_limits_min[i])
                    angles_human[i] = q_limits_min[i];
                //put q_human data into an al::value element to be sent to the robot
                values[i]=angles_human[i];
            }



            for (int i = 15; i<numberDOF; i++)  //we must not multiply the hands by DEG2RAD as it is already a ratio
            {
                angles_human[i]=round(angles_human[i]*20)/20; //we give a precision of 0.05*********************************************
                if (angles_human[i]>q_limits_max[i])
                    angles_human[i] = q_limits_max[i];
                if (angles_human[i]<q_limits_min[i])
                    angles_human[i] = q_limits_min[i];
                    //put q_human data into an al::value element to be sent to the robot
                values[i]=angles_human[i];
            }
        }

        if (init_ID=="MXTP02")
        {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::steady_clock::now( ) - start );
            //cout<<"temps ecoule1: "<<elapsed.count()<<endl;

            /**32 bits par packet de joint_buffer**/
            number_of_data=(n-24)/32; //Pas utilisé
            int torso_Rfoot_Lfoot[] = {4,18,22};

            ///go forward pepper
            buffer_count=24+torso_Rfoot_Lfoot[1]*32;
            temp_euler.define(buffer,buffer_count);
            float piedRx, piedRy;
            piedRx=temp_euler.position_x;
            piedRy=temp_euler.position_y;

            buffer_count=24+torso_Rfoot_Lfoot[2]*32;
            temp_euler.define(buffer,buffer_count);
            float piedLx, piedLy;
            piedLx=temp_euler.position_x;
            piedLy=temp_euler.position_y;

            distancepied=sqrt(pow(piedRx-piedLx,2)+pow(piedRy-piedLy,2));
            //cout<<"DISTANCE PIED: "<<distancepied<<endl;

        }


        ///*************************************************
        ///*************MOVEMENT****************************
        ///*************************************************
        if (!first)
        {
            ///On fait avancer le robot
            movetheta=0;
            movex=0;

            float min_deplacement=0.45;
            float max_deplacement=0.7;
            //V2 on va vers où on regarde quand on avance! on met un ratio jusqu'à 0.5

            if (distancepied>min_deplacement)
            {
                movex=0.1+round(((distancepied-min_deplacement)/max_deplacement)*min_deplacement*100)/100; //max velocity =0.55 m/s
                movetheta=round(angles_human[13]*100/(50*DEG2RAD))/100; //precision ratio 0.01 radian ,we devide by 50 deg because its roughly the furthest angle we can achieve with our neck
                motion.move(movex,0,movetheta);
                cout<<"ON AVANCE!!!!!!"<<" is moving? "<<motion.moveIsActive()<<" movex= "<<movex<<" movetheta= "<<movetheta<<endl;
                moving=true;
            }

            else if (abs(vector_joint[5].rotation_y)>threshold_rotation_deg)
            {
                if (vector_joint[5].rotation_y>0)
                    movetheta=0.2+((vector_joint[5].rotation_y-threshold_rotation_deg)/(max_rotation_deg-threshold_rotation_deg))*1;
                //motion.move(0,0,0.5);
                else
                    movetheta=-0.2+((vector_joint[5].rotation_y+threshold_rotation_deg)/(max_rotation_deg-threshold_rotation_deg))*1;
                cout<<"movetheta "<< movetheta<<endl;
                motion.move(0,0,movetheta);
                cout<<"ON TOURNE!!!!!!"<<" is moving? "<<motion.moveIsActive()<<" movetheta = "<<movetheta<<endl;
                moving=true;
            }

            if(abs(vector_joint[5].rotation_y)<threshold_rotation_deg && distancepied<0.45 && moving==true)
            {
                movetheta=0;
                movex=0;
                motion.move(movex,0,movetheta);
                cout<<"ON ARRETE TOUT!!!!"<<endl;
                moving=false;
            }

            ///On positione les membres
            // Send results to the robot
            // angleInterpolationWithSpeed is a blocking call. setAngles Is not
            motion.setAngles(names, values, speed);




        }

        first=false;


        ///REFRESH TERMINAL DATA
        elapsed_chrono_terminal = std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::steady_clock::now( ) - start_chrono_terminal );
        float data;
        if (elapsed_chrono_terminal.count()>2000)
        {
            //cout<<"distance pied: "<<distancepied<<" | moveIsActive: "<<motion.moveIsActive() <<" | movex= "<<movex<<" | movetheta = "<<movetheta<<" | data: "<<data<<endl;
            start_chrono_terminal=std::chrono::steady_clock::now( );
        }

        if (argc==4)
        {
            ///SEND SONAR DISTANCE TO PHONE
            elapsed_chrono_sonar = std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::steady_clock::now( ) - start_chrono_sonar );
            if (elapsed_chrono_sonar.count()>20)
            {
                data= memory.getData("Device/SubDeviceList/Platform/Front/Sonar/Sensor/Value");
                char data_cstr[16];
                sprintf(data_cstr,"%.2f",data);
                //We send the sonar data to the phone!
                write_server(sock_sonar, &sin, data_cstr);
                start_chrono_sonar=std::chrono::steady_clock::now( );
            }
        }
        else
        {
            if(strcmp(argv[4],"udp_off")!=0)
            {

                ///SEND SONAR DISTANCE TO PHONE
                elapsed_chrono_sonar = std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::steady_clock::now( ) - start_chrono_sonar );
                if (elapsed_chrono_sonar.count()>20)
                {
                    data= memory.getData("Device/SubDeviceList/Platform/Front/Sonar/Sensor/Value");
                    char data_cstr[16];
                    sprintf(data_cstr,"%.2f",data);
                    //We send the sonar data to the phone!
                    write_server(sock_sonar, &sin, data_cstr);
                    start_chrono_sonar=std::chrono::steady_clock::now( );
                }

            }
        }
    }

    motion.stopMove();
    SpeechProxy.say("J'ai fini!");
    posture.goToPosture("Stand",0.2);

    std::cerr << "Motion imitation terminated." << std::endl;
    closesocket(sock);
    return 0;


}

