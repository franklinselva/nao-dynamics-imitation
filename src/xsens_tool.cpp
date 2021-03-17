//// Keyboard hit -- sees if a key is pressed
#include <xsens_tool.h>

using namespace std;

int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if(ch != EOF)
    {
        //ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

void split1(const string &chaine, char delimiteur, vector<string> &elements)
{
    stringstream ss(chaine);
    string sousChaine;
    while (getline(ss, sousChaine, delimiteur))
    {
        elements.push_back(sousChaine);
    }
}

std::vector <std::string> split(const std::string &chaine, char delimiteur)
{
    vector<string> elements;
    split1(chaine, delimiteur, elements);
    return elements;
}

int bufferToInt(char buffer0, char buffer1, char buffer2, char buffer3)
{
    int a = int((unsigned char)(buffer0) << 24 |
                (unsigned char)(buffer1) << 16 |
                (unsigned char)(buffer2) << 8 |
                (unsigned char)(buffer3));
    return a;
}

float bufferToFloat(char buffer0, char buffer1, char buffer2, char buffer3)
{
    int myFourBytes = bufferToInt(buffer0, buffer1, buffer2, buffer3);
    float*  myFloat = (float*) &myFourBytes;
    return *myFloat;
}

void toEulerAngle(float qw, float qx, float qy, float qz, euler_buffer& euler)
{
    // roll (x-axis rotation)
    double sinr = +2.0 * (qw * qx + qy * qz);
    double cosr = +1.0 - 2.0 * (qx * qx + qy * qy);
    euler.rotation_x = atan2(sinr, cosr);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (qw * qy - qz * qx);
    if (fabs(sinp) >= 1)
        euler.rotation_y = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        euler.rotation_y = asin(sinp);

    // yaw (z-axis rotation)
    double siny = +2.0 * (qw * qz + qx * qy);
    double cosy = +1.0 - 2.0 * (qy * qy + qz * qz);
    euler.rotation_z = atan2(siny, cosy);
}

//simplification of program for pepper:
void joint_buffer::define(char buffer[],int buffer_count)
{
    float xsens_rotation_x, xsens_rotation_y, xsens_rotation_z;

    id_parent = bufferToInt(buffer[buffer_count],buffer[buffer_count+1],buffer[buffer_count+2],buffer[buffer_count+3]);
    id_child = bufferToInt(buffer[buffer_count+4],buffer[buffer_count+5],buffer[buffer_count+6],buffer[buffer_count+7]);
    xsens_rotation_x = bufferToFloat(buffer[buffer_count+8],buffer[buffer_count+9],buffer[buffer_count+10],buffer[buffer_count+11]);
    xsens_rotation_y = bufferToFloat(buffer[buffer_count+12],buffer[buffer_count+13],buffer[buffer_count+14],buffer[buffer_count+15]);
    xsens_rotation_z = bufferToFloat(buffer[buffer_count+16],buffer[buffer_count+17],buffer[buffer_count+18],buffer[buffer_count+19]);

    rotation_x = xsens_rotation_x;  ///*****************************************************************************///
    rotation_y = xsens_rotation_z;  ///************     XSENS JOINT FRAME WORK -- Y-UP right handed     ************///
    rotation_z = xsens_rotation_y;  ///*****************************************************************************///

    if(buffer_count == 164 ) //Right UpperArm
    {
        rotation_x = - xsens_rotation_x;
        rotation_y = - xsens_rotation_z;
        rotation_z = - xsens_rotation_y;

    }
    else if(buffer_count == 184) //Right Elbow
    {
        rotation_x = xsens_rotation_z;
        rotation_y = xsens_rotation_x;
        rotation_z = - xsens_rotation_y;

    }
    else if(buffer_count == 204) //Right Wrist
    {
        rotation_x = - xsens_rotation_x;
        rotation_y = - xsens_rotation_z;
        rotation_z = xsens_rotation_y;

    }

    else if(buffer_count == 244) //Left UpperArm
    {
        rotation_x = xsens_rotation_x;
        rotation_y = - xsens_rotation_z;
        rotation_z = xsens_rotation_y;

    }
    else if(buffer_count == 264) //Left Elbow
    {
        rotation_x = - xsens_rotation_z;
        rotation_y = xsens_rotation_x;
        rotation_z = + xsens_rotation_y;
    }
    else if( buffer_count == 284) //Left Wrist
    {
        rotation_x = xsens_rotation_x;
        rotation_y = - xsens_rotation_z;
        rotation_z = - xsens_rotation_y;

    }
    else if(buffer_count == 344) //Right Ankle
    {
        rotation_x = - xsens_rotation_y;
        rotation_y = - xsens_rotation_z;
        rotation_z = xsens_rotation_x;

    }
    else if(buffer_count == 424) // Left Ankle
    {
        rotation_x = + xsens_rotation_y;
        rotation_y = - xsens_rotation_z;
        rotation_z = xsens_rotation_x;

    }
    else if(buffer_count == 304) // Right Hip
    {
        rotation_x = + xsens_rotation_x;
        rotation_y = - xsens_rotation_z;
        rotation_z = xsens_rotation_y;

    }
    else if(buffer_count == 384) // Left Hip
    {
        rotation_x = - xsens_rotation_x;
        rotation_y = - xsens_rotation_z;
        rotation_z = xsens_rotation_y;

    }


    rotation = {rotation_x,
        rotation_y,
        rotation_z};

}

void euler_buffer::define(char buffer[],int buffer_count)
{
    id_segment= bufferToInt(buffer[buffer_count],buffer[buffer_count+1],buffer[buffer_count+2],buffer[buffer_count+3]);
    position_x= bufferToFloat(buffer[buffer_count+4],buffer[buffer_count+5],buffer[buffer_count+6],buffer[buffer_count+7]);
    position_y= bufferToFloat(buffer[buffer_count+8],buffer[buffer_count+9],buffer[buffer_count+10],buffer[buffer_count+11]);
    position_z= bufferToFloat(buffer[buffer_count+12],buffer[buffer_count+13],buffer[buffer_count+14],buffer[buffer_count+15]);
    float qw= bufferToFloat(buffer[buffer_count+16],buffer[buffer_count+17],buffer[buffer_count+18],buffer[buffer_count+19]);
    float qx= bufferToFloat(buffer[buffer_count+20],buffer[buffer_count+21],buffer[buffer_count+22],buffer[buffer_count+23]);
    float qy= bufferToFloat(buffer[buffer_count+24],buffer[buffer_count+25],buffer[buffer_count+26],buffer[buffer_count+27]);
    float qz= bufferToFloat(buffer[buffer_count+28],buffer[buffer_count+29],buffer[buffer_count+30],buffer[buffer_count+31]);
    toEulerAngle(qw, qx, qy,qz,*this);
}

void euler_pose_buffer::define(char buffer[],int buffer_count)
{
    id_segment = bufferToInt(buffer[buffer_count],buffer[buffer_count+1],buffer[buffer_count+2],buffer[buffer_count+3]);
    position_x = bufferToFloat(buffer[buffer_count+4],buffer[buffer_count+5],buffer[buffer_count+6],buffer[buffer_count+7]);
    position_y = bufferToFloat(buffer[buffer_count+8],buffer[buffer_count+9],buffer[buffer_count+10],buffer[buffer_count+11]);
    position_z = bufferToFloat(buffer[buffer_count+12],buffer[buffer_count+13],buffer[buffer_count+14],buffer[buffer_count+15]);
    rotation_x = bufferToFloat(buffer[buffer_count+16],buffer[buffer_count+17],buffer[buffer_count+18],buffer[buffer_count+19]);
    rotation_y = bufferToFloat(buffer[buffer_count+20],buffer[buffer_count+21],buffer[buffer_count+22],buffer[buffer_count+23]);
    rotation_z = bufferToFloat(buffer[buffer_count+24],buffer[buffer_count+25],buffer[buffer_count+26],buffer[buffer_count+27]);
    position = {position_x,
                position_y,
                position_z};
    rotation = {rotation_x,
                rotation_y,
                rotation_z};

//    int rotation_along_x[3][3]; /// Rotation matrix from XSENS (Y-up right handed) pose to Robot's pose (Z-up right handed)
//            rotation[0][0]=1; rotation[0][1]=0; rotation[0][2]=0;
//            rotation[1][0]=0; rotation[1][1]=0; rotation[1][2]=1;
//            rotation[2][0]=0; rotation[2][1]=-1; rotation[2][2]=0;
//
//    int rotation_along_nx[3][3]; /// Rotation matrix from XSENS (Y-up right handed) pose to Robot's pose (Z-up right handed)
//            rotation[0][0]=1; rotation[0][1]=0; rotation[0][2]=0;
//            rotation[1][0]=0; rotation[1][1]=0; rotation[1][2]=-1;
//            rotation[2][0]=0; rotation[2][1]=1; rotation[2][2]=0;

    std::vector<int> LArm_chain = {13,14,15};
    std::vector<int> RArm_chain = {9,10,11};

    if(id_segment == LArm_chain[1] | id_segment == LArm_chain[2] | id_segment == LArm_chain[3])
    {
        position = {position_x,
                -position_z,
                position_y};
        rotation = {rotation_x,
                -position_z,
                position_y};
    }

    if(id_segment == LArm_chain[1] | id_segment == LArm_chain[2] | id_segment == LArm_chain[3])
    {
        position = {position_x,
                position_z,
                -position_y};
        rotation = {rotation_x,
                position_z,
                -position_y};
    }
}



///TOUT POUR LES PACKETS UDP

//client (send message to phone)
int init_connection_client(const char *address, SOCKADDR_IN *sin, int port)
{
    /* UDP so SOCK_DGRAM */
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct hostent *hostinfo;

    if(sock == INVALID_SOCKET)
    {
        perror("socket()");
        exit(errno);
    }

    hostinfo = gethostbyname(address);
    if (hostinfo == NULL)
    {
        fprintf (stderr, "Unknown host %s.\n", address);
        exit(EXIT_FAILURE);
    }

    sin->sin_addr = *(IN_ADDR *) hostinfo->h_addr;
    sin->sin_port = htons(port);
    sin->sin_family = AF_INET;
    cout<<"bind phone"<<endl;
    return sock;
}

//client (receive message from xsens)
int init_connection_server(int port)
{
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, 0);
    SOCKADDR_IN sin = { 0 };

    if(sock == INVALID_SOCKET)
    {
        perror("socket()");
        exit(errno);
    }

    sin.sin_addr.s_addr = htonl(INADDR_ANY);
    sin.sin_port = htons(port);
    sin.sin_family = AF_INET;

    if(bind(sock,(SOCKADDR *) &sin, sizeof sin) == SOCKET_ERROR)
    {
        perror("bind()");
        exit(errno);
    }
    cout<<"bind"<<endl;
    return sock;
}

void write_server(SOCKET sock, SOCKADDR_IN *sin, const char *buffer)
{
    cout<<"sending sonar buffer: "<<buffer<<endl;
    if(sendto(sock, buffer, strlen(buffer), 0, (SOCKADDR *) sin, sizeof *sin) < 0)
    {
        perror("sendto()");
        exit(errno);
    }
}

void end_connection(int sock)
{
    closesocket(sock);
}

Eigen::MatrixXd getAbsolutePosition(std::vector<euler_buffer> xsens_euler)
{

    ///-------------------------------------------------------------------///
    ///------------------- XSENS SEGMENT NAMES ----------------------///
    ///-------------------------------------------------------------------///
    ///********* AL::ALValue segment_xsens_names; **************************///
    ///********* segment_xsens_names.arraySetSize(XSENS_segments); *********///
    ///-------------------------------------------------------------------///
    ///------------------- "UpperBody" chain -----------------------------///
    ///-------------------------------------------------------------------///
    ///********* segment_xsens_names[0] = "Pelvis"; **************************///
    ///********* segment_xsens_names[1] = "L5"; **************************///
    ///********* segment_xsens_names[2] = "L3"; *************************///
    ///********* segment_xsens_names[3] = "T12"; ***************///
    ///********* segment_xsens_names[4] = "T8"; --> Sternum ******************///
    ///********* segment_xsens_names[5] = "Neck"; ************************///
    ///********* segment_xsens_names[6] = "Head"; ***************///
    ///-------------------------------------------------------------------///
    ///------------------- "RArm" chain ----------------------------------///
    ///-------------------------------------------------------------------///
    ///********* segment_xsens_names[7] = "RShoulder"; ***************///
    ///********* segment_xsens_names[8] = "RUpperArm"; *****************///
    ///********* segment_xsens_names[9] = "RForeArm"; ********************///
    ///********* segment_xsens_names[10] = "RHand"; ********************///
    ///-------------------------------------------------------------------///
    ///------------------- "LArm" chain ----------------------------------///
    ///-------------------------------------------------------------------///
    ///********* segment_xsens_names[11] = "LShoulder"; ***************///
    ///********* segment_xsens_names[12] = "LUpperArm"; *****************///
    ///********* segment_xsens_names[13] = "LForeArm"; ********************///
    ///********* segment_xsens_names[14] = "LHand"; ********************///
    ///-------------------------------------------------------------------///
    ///------------------- "RLeg" chain ----------------------------------///
    ///-------------------------------------------------------------------///
    ///********* segment_xsens_names[15] = "RUpperLeg"; *********************///
    ///********* segment_xsens_names[16] = "RLowerLeg"; ********************///
    ///********* segment_xsens_names[17] = "RFoot"; *******************///
    ///********* segment_xsens_names[18] = "RToe"; *********************///
    ///-------------------------------------------------------------------///
    ///------------------- "LLeg" chain ----------------------------------///
    ///-------------------------------------------------------------------///
    ///********* segment_xsens_names[19] = "LUpperLeg"; **********************///
    ///********* segment_xsens_names[20] = "LLowerLeg"; *********************///
    ///********* segment_xsens_names[21] = "LFoot"; ********************///
    ///********* segment_xsens_names[22] = "LToe"; **********************///
    ///-------------------------------------------------------------------///

    float torsox, torsoy, torsoz, r_torsox, r_torsoy, r_torsoz;
    Eigen::VectorXd Torso(4);
    torsox = xsens_euler[4].position_x;
    torsoy = xsens_euler[4].position_y;
    torsoz = xsens_euler[4].position_z;
    r_torsox = -xsens_euler[4].rotation_x;
    r_torsoy = -xsens_euler[4].rotation_y;
    r_torsoz = -xsens_euler[4].rotation_z;

    Torso <<(torsox)*1000,
            (torsoy)*1000,
            (torsoz)*1000,
            1;

    float Pelvis_x, Pelvis_y, Pelvis_z;
    Eigen::VectorXd Pelvis(4);
    Pelvis_x = xsens_euler[0].position_x;
    Pelvis_y = xsens_euler[0].position_y;
    Pelvis_z = xsens_euler[0].position_z;

    Pelvis <<   (Pelvis_x)*1000,
                (Pelvis_y)*1000,
                (Pelvis_z)*1000,
                1;

    float L5_x, L5_y, L5_z;
    Eigen::VectorXd L5(4);
    L5_x = xsens_euler[1].position_x;
    L5_y = xsens_euler[1].position_y;
    L5_z = xsens_euler[1].position_z;

    L5 <<   (L5_x)*1000,
                (L5_y)*1000,
                (L5_z)*1000,
                1;

    float L3_x, L3_y, L3_z;
    Eigen::VectorXd L3(4);
    L3_x = xsens_euler[2].position_x;
    L3_y = xsens_euler[2].position_y;
    L3_z = xsens_euler[2].position_z;

    L3 <<   (L3_x)*1000,
            (L3_y)*1000,
            (L3_z)*1000,
            1;

    float T12_x, T12_y, T12_z;
    Eigen::VectorXd T12(4);
    T12_x = xsens_euler[3].position_x;
    T12_y = xsens_euler[3].position_y;
    T12_z = xsens_euler[3].position_z;

    T12 <<   (T12_x)*1000,
                (T12_y)*1000,
                (T12_z)*1000,
                1;

    float Neck_x, Neck_y, Neck_z;
    Eigen::VectorXd Neck(4);
    Neck_x = xsens_euler[5].position_x;
    Neck_y = xsens_euler[5].position_y;
    Neck_z = xsens_euler[5].position_z;

    Neck <<   (Neck_x)*1000,
                (Neck_y)*1000,
                (Neck_z)*1000,
                1;

    float Head_x, Head_y, Head_z;
    Eigen::VectorXd Head(4);
    Head_x = xsens_euler[6].position_x;
    Head_y = xsens_euler[6].position_y;
    Head_z = xsens_euler[6].position_z;

    Head <<   (Head_x)*1000,
                (Head_y)*1000,
                (Head_z)*1000,
                1;

    float RShoulder_x, RShoulder_y, RShoulder_z;
    Eigen::VectorXd RShoulder(4);
    RShoulder_x = xsens_euler[7].position_x;
    RShoulder_y = xsens_euler[7].position_y;
    RShoulder_z = xsens_euler[7].position_z;

    RShoulder << (RShoulder_x)*1000,
                (RShoulder_y)*1000,
                (RShoulder_z)*1000,
                1;

    float RUpperArm_x, RUpperArm_y, RUpperArm_z;
    Eigen::VectorXd RUpperArm(4);
    RUpperArm_x = xsens_euler[8].position_x;
    RUpperArm_y = xsens_euler[8].position_y;
    RUpperArm_z = xsens_euler[8].position_z;

    RUpperArm << (RUpperArm_x)*1000,
                (RUpperArm_y)*1000,
                (RUpperArm_z)*1000,
                1;

    float RForeArm_x, RForeArm_y, RForeArm_z;
    Eigen::VectorXd RForeArm(4);
    RForeArm_x = xsens_euler[9].position_x;
    RForeArm_y = xsens_euler[9].position_y;
    RForeArm_z = xsens_euler[9].position_z;

    RForeArm << (RForeArm_x)*1000,
                (RForeArm_y)*1000,
                (RForeArm_z)*1000,
                1;

    float RHand_x, RHand_y, RHand_z;
    Eigen::VectorXd RHand(4);
    RHand_x = xsens_euler[10].position_x;
    RHand_y = xsens_euler[10].position_y;
    RHand_z = xsens_euler[10].position_z;

    RHand <<   (RHand_x)*1000,
                (RHand_y)*1000,
                (RHand_z)*1000,
                1;

    float LShoulder_x, LShoulder_y, LShoulder_z;
    Eigen::VectorXd LShoulder(4);
    LShoulder_x = xsens_euler[11].position_x;
    LShoulder_y = xsens_euler[11].position_y;
    LShoulder_z = xsens_euler[11].position_z;

    LShoulder << (LShoulder_x)*1000,
                (LShoulder_y)*1000,
                (LShoulder_z)*1000,
                1;

    float LUpperArm_x, LUpperArm_y, LUpperArm_z;
    Eigen::VectorXd LUpperArm(4);
    LUpperArm_x = xsens_euler[12].position_x;
    LUpperArm_y = xsens_euler[12].position_y;
    LUpperArm_z = xsens_euler[12].position_z;

    LUpperArm << (LUpperArm_x)*1000,
                (LUpperArm_y)*1000,
                (LUpperArm_z)*1000,
                1;

    float LForeArm_x, LForeArm_y, LForeArm_z;
    Eigen::VectorXd LForeArm(4);
    LForeArm_x = xsens_euler[13].position_x;
    LForeArm_y = xsens_euler[13].position_y;
    LForeArm_z = xsens_euler[13].position_z;

    LForeArm << (LForeArm_x)*1000,
                (LForeArm_y)*1000,
                (LForeArm_z)*1000,
                1;

    float LHand_x, LHand_y, LHand_z;
    Eigen::VectorXd LHand(4);
    LHand_x = xsens_euler[14].position_x;
    LHand_y = xsens_euler[14].position_y;
    LHand_z = xsens_euler[14].position_z;

    LHand <<   (LHand_x)*1000,
                (LHand_y)*1000,
                (LHand_z)*1000,
                1;

    float RUpperLeg_x, RUpperLeg_y, RUpperLeg_z;
    Eigen::VectorXd RUpperLeg(4);
    RUpperLeg_x = xsens_euler[15].position_x;
    RUpperLeg_y = xsens_euler[15].position_y;
    RUpperLeg_z = xsens_euler[15].position_z;

    RUpperLeg << (RUpperLeg_x)*1000,
                (RUpperLeg_y)*1000,
                (RUpperLeg_z)*1000,
                1;

    float RLowerLeg_x, RLowerLeg_y, RLowerLeg_z;
    Eigen::VectorXd RLowerLeg(4);
    RLowerLeg_x = xsens_euler[16].position_x;
    RLowerLeg_y = xsens_euler[16].position_y;
    RLowerLeg_z = xsens_euler[16].position_z;

    RLowerLeg << (RLowerLeg_x)*1000,
                (RLowerLeg_y)*1000,
                (RLowerLeg_z)*1000,
                1;

    float RFoot_x, RFoot_y, RFoot_z;
    Eigen::VectorXd RFoot(4);
    RFoot_x = xsens_euler[17].position_x;
    RFoot_y = xsens_euler[17].position_y;
    RFoot_z = xsens_euler[17].position_z;

    RFoot <<   (RFoot_x)*1000,
                (RFoot_y)*1000,
                (RFoot_z)*1000,
                1;

    float RToe_x, RToe_y, RToe_z;
    Eigen::VectorXd RToe(4);
    RToe_x = xsens_euler[18].position_x;
    RToe_y = xsens_euler[18].position_y;
    RToe_z = xsens_euler[18].position_z;

    RToe <<   (RToe_x)*1000,
                (RToe_y)*1000,
                (RToe_z)*1000,
                1;

    float LUpperLeg_x, LUpperLeg_y, LUpperLeg_z;
    Eigen::VectorXd LUpperLeg(4);
    LUpperLeg_x = xsens_euler[19].position_x;
    LUpperLeg_y = xsens_euler[19].position_y;
    LUpperLeg_z = xsens_euler[19].position_z;

    LUpperLeg << (LUpperLeg_x)*1000,
                (LUpperLeg_y)*1000,
                (LUpperLeg_z)*1000,
                1;

    float LLowerLeg_x, LLowerLeg_y, LLowerLeg_z;
    Eigen::VectorXd LLowerLeg(4);
    LLowerLeg_x = xsens_euler[20].position_x;
    LLowerLeg_y = xsens_euler[20].position_y;
    LLowerLeg_z = xsens_euler[20].position_z;

    LLowerLeg << (LLowerLeg_x)*1000,
                (LLowerLeg_y)*1000,
                (LLowerLeg_z)*1000,
                1;

    float LFoot_x, LFoot_y, LFoot_z;
    Eigen::VectorXd LFoot(4);
    LFoot_x = xsens_euler[21].position_x;
    LFoot_y = xsens_euler[21].position_y;
    LFoot_z = xsens_euler[21].position_z;

    LFoot <<   (LFoot_x)*1000,
                (LFoot_y)*1000,
                (LFoot_z)*1000,
                1;

    float LToe_x, LToe_y, LToe_z;
    Eigen::VectorXd LToe(4);
    LToe_x = xsens_euler[22].position_x;
    LToe_y = xsens_euler[22].position_y;
    LToe_z = xsens_euler[22].position_z;

    LToe <<   (LToe_x)*1000,
                (LToe_y)*1000,
                (LToe_z)*1000,
                1;

    Eigen::MatrixXd RotationTotal(4,4), RotationX(4,4), RotationY(4,4), RotationZ(4,4); /// Remettre dans l'origine.
    Eigen::MatrixXd TranslationTotal(4,4); /// Remettre dans l'origine.

    RotationX << 1, 0, 0,0,
        0, cos(r_torsox), -sin(r_torsox),0,
        0, sin(r_torsox), cos(r_torsox),0,
        0,0,0,1;

    RotationY << cos(r_torsoy), 0, sin(r_torsoy),0,
        0, 1, 0,0,
        -sin(r_torsoy), 0, cos(r_torsoy),0,
        0,0,0,1;

    RotationZ << cos(r_torsoz), -sin(r_torsoz), 0,0,
        sin(r_torsoz), cos(r_torsoz), 0,0,
        0, 0, 1,0,
        0,0,0,1;

    RotationTotal = RotationZ*RotationY;
    RotationTotal = RotationTotal*RotationX;
    RotationTotal = RotationZ;

    TranslationTotal << 1,0,0,(-torsox)*1000,
                        0,1,0,(-torsoy)*1000,
                        0,0,1,(-torsoz)*1000,
                        0,0,0,1;

    Pelvis = TranslationTotal*Pelvis;
    L5 = TranslationTotal*L5;
    L3 = TranslationTotal*L3;
    T12 = TranslationTotal*T12;
    Torso = TranslationTotal*Torso;
    Neck = TranslationTotal*Neck;
    Head = TranslationTotal*Head;
    RShoulder = TranslationTotal*RShoulder;
    RUpperArm = TranslationTotal*RUpperArm;
    RForeArm = TranslationTotal*RForeArm;
    RHand = TranslationTotal*RHand;
    LShoulder = TranslationTotal*LShoulder;
    LUpperArm = TranslationTotal*LUpperArm;
    LForeArm = TranslationTotal*LForeArm;
    LHand = TranslationTotal*LHand;
    RUpperLeg = TranslationTotal*RUpperLeg;
    RLowerLeg = TranslationTotal*RLowerLeg;
    RFoot = TranslationTotal*RFoot;
    RToe = TranslationTotal*RToe;
    LUpperLeg = TranslationTotal*LUpperLeg;
    LLowerLeg = TranslationTotal*LLowerLeg;
    LFoot = TranslationTotal*LFoot;
    LToe = TranslationTotal*LToe;

    Pelvis = RotationTotal*Pelvis;
    L5 = RotationTotal*L5;
    L3 = RotationTotal*L3;
    T12 = RotationTotal*T12;
    Torso = RotationTotal*Torso;
    Neck = RotationTotal*Neck;
    Head = RotationTotal*Head;
    RShoulder = RotationTotal*RShoulder;
    RUpperArm = RotationTotal*RUpperArm;
    RForeArm = RotationTotal*RForeArm;
    RHand = RotationTotal*RHand;
    LShoulder = RotationTotal*LShoulder;
    LUpperArm = RotationTotal*LUpperArm;
    LForeArm = RotationTotal*LForeArm;
    LHand = RotationTotal*LHand;
    RUpperLeg = RotationTotal*RUpperLeg;
    RLowerLeg = RotationTotal*RLowerLeg;
    RFoot = RotationTotal*RFoot;
    RToe = RotationTotal*RToe;
    LUpperLeg = RotationTotal*LUpperLeg;
    LLowerLeg = RotationTotal*LLowerLeg;
    LFoot = RotationTotal*LFoot;
    LToe = RotationTotal*LToe;


    Eigen::MatrixXd Body_segments(4,23);

    Body_segments << Pelvis, L5, L3, T12, Torso,
                    Neck, Head,
                    RShoulder, RUpperArm, RForeArm, RHand,
                    LShoulder, LUpperArm, LForeArm, LHand,
                    RUpperLeg, RLowerLeg, RFoot, RToe,
                    LUpperLeg, LLowerLeg, LFoot, LToe;


//    std::cout << "RFoot: " << std::endl << RFoot << std::endl;
//    std::cout << "LFoot: " << std::endl << LFoot << std::endl;

    return Body_segments;


}

Eigen::MatrixXd getRelativePosition(std::vector<euler_buffer> xsens_euler)
{

    float torsox, torsoy, torsoz, r_torsox, r_torsoy, r_torsoz;
    Eigen::VectorXd Torso(4);
    torsox = xsens_euler[4].position_x;
    torsoy = xsens_euler[4].position_y;
    torsoz = xsens_euler[4].position_z;
    r_torsox = -xsens_euler[4].rotation_x;
    r_torsoy = -xsens_euler[4].rotation_y;
    r_torsoz = -xsens_euler[4].rotation_z;

    Torso <<(torsox)*1000,
            (torsoy)*1000,
            (torsoz)*1000,
            1;

    float Pelvis_x, Pelvis_y, Pelvis_z;
    Eigen::VectorXd Pelvis(4);
    Pelvis_x = xsens_euler[0].position_x;
    Pelvis_y = xsens_euler[0].position_y;
    Pelvis_z = xsens_euler[0].position_z;

    Pelvis <<   (Pelvis_x)*1000,
                (Pelvis_y)*1000,
                (Pelvis_z)*1000,
                1;

    float L5_x, L5_y, L5_z;
    Eigen::VectorXd L5(4);
    L5_x = xsens_euler[1].position_x;
    L5_y = xsens_euler[1].position_y;
    L5_z = xsens_euler[1].position_z;

    L5 <<   (L5_x)*1000,
                (L5_y)*1000,
                (L5_z)*1000,
                1;

    float L3_x, L3_y, L3_z;
    Eigen::VectorXd L3(4);
    L3_x = xsens_euler[2].position_x;
    L3_y = xsens_euler[2].position_y;
    L3_z = xsens_euler[2].position_z;

    L3 <<   (L3_x)*1000,
            (L3_y)*1000,
            (L3_z)*1000,
            1;

    float T12_x, T12_y, T12_z;
    Eigen::VectorXd T12(4);
    T12_x = xsens_euler[3].position_x;
    T12_y = xsens_euler[3].position_y;
    T12_z = xsens_euler[3].position_z;

    T12 <<   (T12_x)*1000,
                (T12_y)*1000,
                (T12_z)*1000,
                1;

    float Neck_x, Neck_y, Neck_z;
    Eigen::VectorXd Neck(4);
    Neck_x = xsens_euler[5].position_x;
    Neck_y = xsens_euler[5].position_y;
    Neck_z = xsens_euler[5].position_z;

    Neck <<   (Neck_x)*1000,
                (Neck_y)*1000,
                (Neck_z)*1000,
                1;

    float Head_x, Head_y, Head_z;
    Eigen::VectorXd Head(4);
    Head_x = xsens_euler[6].position_x;
    Head_y = xsens_euler[6].position_y;
    Head_z = xsens_euler[6].position_z;

    Head <<   (Head_x)*1000,
                (Head_y)*1000,
                (Head_z)*1000,
                1;

    float RShoulder_x, RShoulder_y, RShoulder_z;
    Eigen::VectorXd RShoulder(4);
    RShoulder_x = xsens_euler[7].position_x;
    RShoulder_y = xsens_euler[7].position_y;
    RShoulder_z = xsens_euler[7].position_z;

    RShoulder << (RShoulder_x)*1000,
                (RShoulder_y)*1000,
                (RShoulder_z)*1000,
                1;

    float RUpperArm_x, RUpperArm_y, RUpperArm_z;
    Eigen::VectorXd RUpperArm(4);
    RUpperArm_x = xsens_euler[8].position_x;
    RUpperArm_y = xsens_euler[8].position_y;
    RUpperArm_z = xsens_euler[8].position_z;

    RUpperArm << (RUpperArm_x)*1000,
                (RUpperArm_y)*1000,
                (RUpperArm_z)*1000,
                1;

    float RForeArm_x, RForeArm_y, RForeArm_z;
    Eigen::VectorXd RForeArm(4);
    RForeArm_x = xsens_euler[9].position_x;
    RForeArm_y = xsens_euler[9].position_y;
    RForeArm_z = xsens_euler[9].position_z;

    RForeArm << (RForeArm_x)*1000,
                (RForeArm_y)*1000,
                (RForeArm_z)*1000,
                1;

    float RHand_x, RHand_y, RHand_z;
    Eigen::VectorXd RHand(4);
    RHand_x = xsens_euler[10].position_x;
    RHand_y = xsens_euler[10].position_y;
    RHand_z = xsens_euler[10].position_z;

    RHand <<   (RHand_x)*1000,
                (RHand_y)*1000,
                (RHand_z)*1000,
                1;

    float LShoulder_x, LShoulder_y, LShoulder_z;
    Eigen::VectorXd LShoulder(4);
    LShoulder_x = xsens_euler[11].position_x;
    LShoulder_y = xsens_euler[11].position_y;
    LShoulder_z = xsens_euler[11].position_z;

    LShoulder << (LShoulder_x)*1000,
                (LShoulder_y)*1000,
                (LShoulder_z)*1000,
                1;

    float LUpperArm_x, LUpperArm_y, LUpperArm_z;
    Eigen::VectorXd LUpperArm(4);
    LUpperArm_x = xsens_euler[12].position_x;
    LUpperArm_y = xsens_euler[12].position_y;
    LUpperArm_z = xsens_euler[12].position_z;

    LUpperArm << (LUpperArm_x)*1000,
                (LUpperArm_y)*1000,
                (LUpperArm_z)*1000,
                1;

    float LForeArm_x, LForeArm_y, LForeArm_z;
    Eigen::VectorXd LForeArm(4);
    LForeArm_x = xsens_euler[13].position_x;
    LForeArm_y = xsens_euler[13].position_y;
    LForeArm_z = xsens_euler[13].position_z;

    LForeArm << (LForeArm_x)*1000,
                (LForeArm_y)*1000,
                (LForeArm_z)*1000,
                1;

    float LHand_x, LHand_y, LHand_z;
    Eigen::VectorXd LHand(4);
    LHand_x = xsens_euler[14].position_x;
    LHand_y = xsens_euler[14].position_y;
    LHand_z = xsens_euler[14].position_z;

    LHand <<   (LHand_x)*1000,
                (LHand_y)*1000,
                (LHand_z)*1000,
                1;

    float RUpperLeg_x, RUpperLeg_y, RUpperLeg_z;
    Eigen::VectorXd RUpperLeg(4);
    RUpperLeg_x = xsens_euler[15].position_x;
    RUpperLeg_y = xsens_euler[15].position_y;
    RUpperLeg_z = xsens_euler[15].position_z;

    RUpperLeg << (RUpperLeg_x)*1000,
                (RUpperLeg_y)*1000,
                (RUpperLeg_z)*1000,
                1;

    float RLowerLeg_x, RLowerLeg_y, RLowerLeg_z;
    Eigen::VectorXd RLowerLeg(4);
    RLowerLeg_x = xsens_euler[16].position_x;
    RLowerLeg_y = xsens_euler[16].position_y;
    RLowerLeg_z = xsens_euler[16].position_z;

    RLowerLeg << (RLowerLeg_x)*1000,
                (RLowerLeg_y)*1000,
                (RLowerLeg_z)*1000,
                1;

    float RFoot_x, RFoot_y, RFoot_z;
    Eigen::VectorXd RFoot(4);
    RFoot_x = xsens_euler[17].position_x;
    RFoot_y = xsens_euler[17].position_y;
    RFoot_z = xsens_euler[17].position_z;

    RFoot <<   (RFoot_x)*1000,
                (RFoot_y)*1000,
                (RFoot_z)*1000,
                1;

    float RToe_x, RToe_y, RToe_z;
    Eigen::VectorXd RToe(4);
    RToe_x = xsens_euler[18].position_x;
    RToe_y = xsens_euler[18].position_y;
    RToe_z = xsens_euler[18].position_z;

    RToe <<   (RToe_x)*1000,
                (RToe_y)*1000,
                (RToe_z)*1000,
                1;

    float LUpperLeg_x, LUpperLeg_y, LUpperLeg_z;
    Eigen::VectorXd LUpperLeg(4);
    LUpperLeg_x = xsens_euler[19].position_x;
    LUpperLeg_y = xsens_euler[19].position_y;
    LUpperLeg_z = xsens_euler[19].position_z;

    LUpperLeg << (LUpperLeg_x)*1000,
                (LUpperLeg_y)*1000,
                (LUpperLeg_z)*1000,
                1;

    float LLowerLeg_x, LLowerLeg_y, LLowerLeg_z;
    Eigen::VectorXd LLowerLeg(4);
    LLowerLeg_x = xsens_euler[20].position_x;
    LLowerLeg_y = xsens_euler[20].position_y;
    LLowerLeg_z = xsens_euler[20].position_z;

    LLowerLeg << (LLowerLeg_x)*1000,
                (LLowerLeg_y)*1000,
                (LLowerLeg_z)*1000,
                1;

    float LFoot_x, LFoot_y, LFoot_z;
    Eigen::VectorXd LFoot(4);
    LFoot_x = xsens_euler[21].position_x;
    LFoot_y = xsens_euler[21].position_y;
    LFoot_z = xsens_euler[21].position_z;

    LFoot <<   (LFoot_x)*1000,
                (LFoot_y)*1000,
                (LFoot_z)*1000,
                1;

    float LToe_x, LToe_y, LToe_z;
    Eigen::VectorXd LToe(4);
    LToe_x = xsens_euler[22].position_x;
    LToe_y = xsens_euler[22].position_y;
    LToe_z = xsens_euler[22].position_z;

    LToe <<   (LToe_x)*1000,
                (LToe_y)*1000,
                (LToe_z)*1000,
                1;

    Eigen::MatrixXd RotationTotal(4,4), RotationX(4,4), RotationY(4,4), RotationZ(4,4); /// Remettre dans l'origine.
    Eigen::MatrixXd TranslationTotal(4,4); /// Remettre dans l'origine.

    RotationX << 1, 0, 0,0,
        0, cos(r_torsox), -sin(r_torsox),0,
        0, sin(r_torsox), cos(r_torsox),0,
        0,0,0,1;

    RotationY << cos(r_torsoy), 0, sin(r_torsoy),0,
        0, 1, 0,0,
        -sin(r_torsoy), 0, cos(r_torsoy),0,
        0,0,0,1;

    RotationZ << cos(r_torsoz), -sin(r_torsoz), 0,0,
        sin(r_torsoz), cos(r_torsoz), 0,0,
        0, 0, 1,0,
        0,0,0,1;

    RotationTotal = RotationZ*RotationY;
    RotationTotal = RotationTotal*RotationX;
    RotationTotal = RotationZ;

    TranslationTotal << 1,0,0,(-torsox)*1000,
                        0,1,0,(-torsoy)*1000,
                        0,0,1,(-torsoz)*1000,
                        0,0,0,1;

    Pelvis = TranslationTotal*Pelvis;
    L5 = TranslationTotal*L5;
    L3 = TranslationTotal*L3;
    T12 = TranslationTotal*T12;
    Torso = TranslationTotal*Torso;
    Neck = TranslationTotal*Neck;
    Head = TranslationTotal*Head;
    RShoulder = TranslationTotal*RShoulder;
    RUpperArm = TranslationTotal*RUpperArm;
    RForeArm = TranslationTotal*RForeArm;
    RHand = TranslationTotal*RHand;
    LShoulder = TranslationTotal*LShoulder;
    LUpperArm = TranslationTotal*LUpperArm;
    LForeArm = TranslationTotal*LForeArm;
    LHand = TranslationTotal*LHand;
    RUpperLeg = TranslationTotal*RUpperLeg;
    RLowerLeg = TranslationTotal*RLowerLeg;
    RFoot = TranslationTotal*RFoot;
    RToe = TranslationTotal*RToe;
    LUpperLeg = TranslationTotal*LUpperLeg;
    LLowerLeg = TranslationTotal*LLowerLeg;
    LFoot = TranslationTotal*LFoot;
    LToe = TranslationTotal*LToe;

    Pelvis = RotationTotal*Pelvis;
    L5 = RotationTotal*L5;
    L3 = RotationTotal*L3;
    T12 = RotationTotal*T12;
    Torso = RotationTotal*Torso;
    Neck = RotationTotal*Neck;
    Head = RotationTotal*Head;
    RShoulder = RotationTotal*RShoulder;
    RUpperArm = RotationTotal*RUpperArm;
    RForeArm = RotationTotal*RForeArm;
    RHand = RotationTotal*RHand;
    LShoulder = RotationTotal*LShoulder;
    LUpperArm = RotationTotal*LUpperArm;
    LForeArm = RotationTotal*LForeArm;
    LHand = RotationTotal*LHand;
    RUpperLeg = RotationTotal*RUpperLeg;
    RLowerLeg = RotationTotal*RLowerLeg;
    RFoot = RotationTotal*RFoot;
    RToe = RotationTotal*RToe;
    LUpperLeg = RotationTotal*LUpperLeg;
    LLowerLeg = RotationTotal*LLowerLeg;
    LFoot = RotationTotal*LFoot;
    LToe = RotationTotal*LToe;

    Eigen::VectorXd relative_Head(4);
    Eigen::VectorXd relative_RForeArm(4), relative_RHand(4);
    Eigen::VectorXd relative_LForeArm(4), relative_LHand(4);
    Eigen::VectorXd relative_RLowerLeg(4), relative_RFoot(4), relative_RToe(4);
    Eigen::VectorXd relative_LLowerLeg(4), relative_LFoot(4), relative_LToe(4);

    relative_Head << Head.block<3,1>(0,0) - Neck.block<3,1>(0,0),
                    1;

    relative_RForeArm << RForeArm.block<3,1>(0,0) - RUpperArm.block<3,1>(0,0),
                        1;
    relative_RHand << RHand.block<3,1>(0,0) - RForeArm.block<3,1>(0,0),
                        1;

    relative_LForeArm << LForeArm.block<3,1>(0,0) - LUpperArm.block<3,1>(0,0),
                        1;
    relative_LHand << LHand.block<3,1>(0,0) - LForeArm.block<3,1>(0,0),
                        1;

    relative_RLowerLeg << RLowerLeg.block<3,1>(0,0) - RUpperLeg.block<3,1>(0,0),
                            1;
    relative_RFoot << RFoot.block<3,1>(0,0) - RLowerLeg.block<3,1>(0,0),
                            1;
    relative_RToe << RToe.block<3,1>(0,0) - RFoot.block<3,1>(0,0),
                            1;

    relative_LLowerLeg << LLowerLeg.block<3,1>(0,0) - LUpperLeg.block<3,1>(0,0),
                            1;
    relative_LFoot << LFoot.block<3,1>(0,0) - LLowerLeg.block<3,1>(0,0),
                            1;
    relative_LToe << LToe.block<3,1>(0,0) - LFoot.block<3,1>(0,0),
                            1;

    Eigen::MatrixXd Body_segments(4,23);

    Body_segments << Pelvis, L5, L3, T12, Torso,
                    Neck, relative_Head,
                    RShoulder, RUpperArm, relative_RForeArm, relative_RHand,
                    LShoulder, LUpperArm, relative_LForeArm, relative_LHand,
                    RUpperLeg, relative_RLowerLeg, relative_RFoot, relative_RToe,
                    LUpperLeg, relative_LLowerLeg, relative_LFoot, relative_LToe;



    return Body_segments;


}



