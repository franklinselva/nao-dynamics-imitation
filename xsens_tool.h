/// Standard libraries
#include <termios.h>
#include <fcntl.h>
#include <string>
#include <vector>
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <memory.h>
#include <stdio.h>
#include <math.h>
#include <chrono>

/// qpOASES library
#include <qpOASES/qpOASES.hpp>

/// Jacobian libraries
#include "Jacobian/Jacobian.h>

/// Include socket
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>

/// Include robot softbank
#include <qi/os.hpp>
#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <alcommon/albroker.h>
#include <boost/shared_ptr.hpp>

#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define closesocket(s) close(s)
typedef int SOCKET;
typedef struct sockaddr_in SOCKADDR_IN;
typedef struct sockaddr SOCKADDR;
typedef struct in_addr IN_ADDR;

class joint_buffer
{
public:
    int id_parent;
    int id_child;
    float rotation_x;
    float rotation_y;
    float rotation_z;
    std::vector<float> rotation;
    void define(char buffer[],int buffer_count);
};

class euler_buffer
{
public:
    int id_segment;
    float position_x;
    float position_y;
    float position_z;
    float rotation_x;
    float rotation_y;
    float rotation_z;
    void define(char buffer[],int buffer_count);

};

class euler_pose_buffer
{
public:
    int id_segment;
    float position_x;
    float position_y;
    float position_z;
    float rotation_x;
    float rotation_y;
    float rotation_z;
    std::vector<float> position;
    std::vector<float> rotation;
    void define(char buffer[],int buffer_count);

};

std::vector <std::string> split(const std::string &chaine, char delimiteur);
int kbhit(void);
int bufferToInt(char buffer0, char buffer1, char buffer2, char buffer3);
float bufferToFloat(char buffer3, char buffer2, char buffer1, char buffer0);
void toEulerAngle(float qw, float qx, float qy, float qz, euler_buffer& euler);

/// Get all the XSENS segment positions w.r.t. the Torso.
Eigen::MatrixXd getAbsolutePosition(std::vector<euler_buffer> xsens_euler);

/// Get all the XSENS segment positions w.r.t. the precedent segment (to scale).
Eigen::MatrixXd getRelativePosition(std::vector<euler_buffer> xsens_euler);


///TOUT POUR LES PACKETS UDP

int init_connection_client(const char *address, SOCKADDR_IN *sin, int port);
int init_connection_server(int port);
void write_server(SOCKET sock, SOCKADDR_IN *sin, const char *buffer);
void end_connection(int sock);


