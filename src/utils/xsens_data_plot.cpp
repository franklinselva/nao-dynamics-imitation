#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include <sys/time.h>
#include <netdb.h>
#include <arpa/inet.h>

#include <thread>
#include <chrono>
#include <stdexcept>
#include <memory>
#include <signal.h>

#define SERVER_PORT 9763
#define BUFF_LEN 1024

#define VERBOSE true

#define PI 3.141592

bool first = true;
int server_fd, ret;
struct sockaddr_in ser_addr;

typedef struct header
{
    char ID_String[6];
    int sample_counter;
    char datagram_counter;
    char number_of_items;
    float time_code;
    char character_ID;
    char reserved_for_future_use[7];
} Header;

struct JointState
{
    Header header;
    std::string frame_id;

    std::vector<std::string> name{"x", "y", "z"};
    std::vector<float> position[3];
    std::vector<float> velocity[4];
    std::vector<float> effort[3];
};

JointState pelvis, l5, l3, t12, t8, neck, head, right_shoulder, right_upper_arm, right_forearm, right_hand,
    left_shoulder, left_upper_arm, left_forearm, left_hand,
    right_upper_leg, right_lower_leg, right_foot, right_toe,
    left_upper_leg, left_lower_leg, left_foot, left_toe;

void parse_header(Header *header, char *buf, int *time_stamp_sec, int *time_stamp_nsec)
{
    int hour = 0;
    int min = 0;
    int sec = 0;
    int nanosec = 0;
    float a = 0.0;
    int to_stamp_sec = 0;
    float nano_1000;
    float temp;
    float float_time_code;
    double b = 0.0;

    memcpy(header->ID_String, buf, 6);
    memcpy(&(header->sample_counter), buf + 6, 4);
    header->sample_counter = ntohl(header->sample_counter);
    header->datagram_counter = buf[11];
    header->number_of_items = buf[12];
    memcpy(&(header->time_code), buf + 12, 4);
    header->time_code = ntohl(header->time_code);
    float_time_code = (float)header->time_code;
    //decoding time_code;
    if (header->time_code <= 99999)
    { //if time_code < 99.999;
        float_time_code /= 1000;
        //(debug) printf("float_time_code = header->time_code/1000 = %f\n", float_time_code);
        sec = (int)float_time_code;
        //(debug) printf("sec = (int)float_time_code = %d\n", sec);
        nanosec = (int)((float_time_code - sec) * 1000);
        //(debug)printf("nanosec = %d\n", nanosec);

        //**convert to time stamp**
        *time_stamp_sec = sec;
        *time_stamp_nsec = nanosec;
    }
    else if (header->time_code > 99999 && header->time_code <= 9999999)
    { //if 99.999 < time_code <= 99:99.999;
        float_time_code = float_time_code / 100000;
        min = (int)float_time_code;
        a = (int)((float_time_code - min) * 100000);
        float_time_code = (float)a;
        float_time_code /= 1000;
        sec = (int)float_time_code;
        nanosec = (int)((float_time_code - sec) * 1000);
        to_stamp_sec = sec + min * 60;

        //**convert to time stamp**
        *time_stamp_sec = sec;
        *time_stamp_nsec = nanosec;
    }
    else if (header->time_code > 9999999 && header->time_code <= 999999999)
    { //if 99:99.999 < time_code < 99:99:99.999;
        float_time_code /= 10000000;
        hour = (int)float_time_code;
        a = (int)((float_time_code - hour) * 10000000);
        float_time_code = (float)a;
        float_time_code = float_time_code / 100000;
        min = (int)float_time_code;
        a = (int)((float_time_code - min) * 100000);
        float_time_code = (float)a;
        float_time_code /= 1000;
        sec = (int)float_time_code;
        nanosec = (int)((float_time_code - sec) * 1000);
        to_stamp_sec = sec + min * 60 + hour * 60 * 60;

        //convert to time stamp
        *time_stamp_sec = sec;
        *time_stamp_nsec = nanosec;
    }
    header->character_ID = buf[17];
    memcpy(header->reserved_for_future_use, buf + 18, 7);
    if (VERBOSE || first)
    {
        printf("ID_String = %s\n", header->ID_String);
        printf("sample_counter = %d\n", header->sample_counter); //endian conversion
        printf("datagram_counter = %d\n", header->datagram_counter);
        printf("number_of_items = %d\n", header->number_of_items);
        printf("time_code = %d\n", (int)header->time_code);
        printf("time = %d:%d:%d.%d\n", hour, min, sec, nanosec);
        printf("character_ID = %d\n", header->character_ID);
        first = false;
    }
}

float parse_coordinates(float coordinate, int count, char *buf)
{
    //test endian

    unsigned char byte_array[4];
    memcpy(&byte_array, buf + count, sizeof(coordinate));
    *((unsigned char *)(&coordinate) + 3) = byte_array[0];
    *((unsigned char *)(&coordinate) + 2) = byte_array[1];
    *((unsigned char *)(&coordinate) + 1) = byte_array[2];
    *((unsigned char *)(&coordinate) + 0) = byte_array[3];
    //coordinate /= 100; (Only need this in Type 01)
    return coordinate;
}

void convertFromYupToZup(float *x, float *y, float *z)
{
    float x1 = *x;
    float y1 = *y;
    float z1 = *z;

    *x = z1;
    *y = x1;
    *z = y1;
}

/**
 * @brief Convert the provided float from radians to degree
 * 
 * @param rad float - Value in radians
 * @return float - Value in degree
 */
float convertFromRadToDeg(float rad)
{
    float deg = rad * 180 / PI;
    return deg;
}

void parse_body(char *buf, int *segment_id, float *x, float *y, float *z)
{
    int seg_id = 0;
    float x_p, y_p, z_p = 0.0;
    float q1_r, q2_r, q3_r, q4_r = 0.0;
    memcpy(&seg_id, buf, 4);
    seg_id = ntohl(seg_id);
    *segment_id = seg_id;
    x_p = parse_coordinates(x_p, 4, buf);
    *x = x_p;

    y_p = parse_coordinates(y_p, 8, buf);
    *y = y_p;

    z_p = parse_coordinates(z_p, 12, buf);
    *z = z_p;

    q1_r = parse_coordinates(q1_r, 16, buf);
    q1_r = convertFromRadToDeg(q1_r);
    // *re = q1_r;

    q2_r = parse_coordinates(q2_r, 20, buf);
    q2_r = convertFromRadToDeg(q2_r);
    // *i = q2_r;

    q3_r = parse_coordinates(q3_r, 24, buf);
    q3_r = convertFromRadToDeg(q3_r);
    //*j = q3_r;

    q4_r = parse_coordinates(q4_r, 28, buf);
    q4_r = convertFromRadToDeg(q4_r);
    //*k = q4_r;
}

/**
 * @brief This function allows you to visualize the xsens data in terms of graph
 * 
 * @param fd  int - Socket file descriptor
 * @param argc 
 * @param argv 
 */
void visualize_xsens_data(int fd, int argc, char *argv[])
{
    char *buf = (char *)malloc(1024);
    int read_bytes = 0;
    int cur_index = 0;
    int count;
    int time_stamp_sec;
    int time_stamp_nsec;
    int segment_id;
    float x, y, z;
    struct sockaddr_in client_addr;
    socklen_t len = sizeof(client_addr);
    Header header;
    int round = 0;

    std::cout << "[INFO] Starting to receive data from Xsens!" << std::endl;

    while (1)
    {
        memset(buf, 0, BUFF_LEN);
        count = recvfrom(fd, buf + read_bytes, BUFF_LEN, 0, (struct sockaddr *)&client_addr, &len);
        if (count == -1)
        {
            printf("[ERROR] Receieve data fail!\n");
            return;
        }
        read_bytes += count;
        if (read_bytes >= 24)
        {
            parse_header(&header, buf + cur_index, &time_stamp_sec, &time_stamp_nsec);
            cur_index += 24;
            if (strncmp(header.ID_String, "MXTP02", 6) == 0)
            {
                //Message type 02 - Pose data (Quaternion)
                if (read_bytes >= 24 + 32 * header.datagram_counter)
                {
                    int i = 0;
                    //printf("Message type 02: Pose data (Quaternion)\n");
                    for (i = 0; i < header.datagram_counter; i++)
                    {
                        parse_body(buf + cur_index + i * 32, &segment_id, &x, &y, &z);
                        switch (segment_id)
                        {
                        case 1:
                            pelvis.frame_id = "pelvis";
                            pelvis.position->push_back(x);
                            pelvis.position->push_back(y);
                            pelvis.position->push_back(z);
                            //pelvis.velocity = {i, j, k, re};
                            break;
                        case 2:
                            l5.frame_id = "l5";
                            l5.position->push_back(x);
                            l5.position->push_back(y);
                            l5.position->push_back(z);
                            //l5.velocity = {i, j, k, re};
                            break;
                        case 3:
                            l3.frame_id = "l3";
                            l3.position->push_back(x);
                            l3.position->push_back(y);
                            l3.position->push_back(z);
                            //l3.velocity = {i, j, k, re};
                            break;
                        case 4:
                            t12.frame_id = "t12";
                            t12.position->push_back(x);
                            t12.position->push_back(y);
                            t12.position->push_back(z);
                            //t12.velocity = {i, j, k, re};
                            break;
                        case 5:
                            t8.frame_id = "t8";
                            t8.position->push_back(x);
                            t8.position->push_back(y);
                            t8.position->push_back(z);
                            //t8.velocity = {i, j, k, re};
                            break;
                        case 6:
                            neck.frame_id = "neck";
                            neck.position->push_back(x);
                            neck.position->push_back(y);
                            neck.position->push_back(z);
                            //neck.velocity = {i, j, k, re};
                            break;
                        case 7:
                            head.frame_id = "head";
                            head.position->push_back(x);
                            head.position->push_back(y);
                            head.position->push_back(z);
                            //head.velocity = {i, j, k, re};
                            break;
                        case 8:
                            right_shoulder.frame_id = "right_shoulder";
                            right_shoulder.position->push_back(x);
                            right_shoulder.position->push_back(y);
                            right_shoulder.position->push_back(z);
                            //right_shoulder.velocity = {i, j, k, re};
                            break;
                        case 9:
                            right_upper_arm.frame_id = "right_upper_arm";
                            right_upper_arm.position->push_back(x);
                            right_upper_arm.position->push_back(y);
                            right_upper_arm.position->push_back(z);
                            //right_upper_arm.velocity = {i, j, k, re};
                            break;
                        case 10:
                            right_forearm.frame_id = "right_forearm";
                            right_forearm.position->push_back(x);
                            right_forearm.position->push_back(y);
                            right_forearm.position->push_back(z);
                            //right_forearm.velocity = {i, j, k, re};
                            break;
                        case 11:
                            right_hand.frame_id = "right_hand";
                            right_hand.position->push_back(x);
                            right_hand.position->push_back(y);
                            right_hand.position->push_back(z);
                            //right_hand.velocity = {i, j, k, re};
                            break;
                        case 12:
                            left_shoulder.frame_id = "left_shoulder";
                            left_shoulder.position->push_back(x);
                            left_shoulder.position->push_back(y);
                            left_shoulder.position->push_back(z);
                            //left_shoulder.velocity = {i, j, k, re};
                            break;
                        case 13:
                            left_upper_arm.frame_id = "left_upper_arm";
                            left_upper_arm.position->push_back(x);
                            left_upper_arm.position->push_back(y);
                            left_upper_arm.position->push_back(z);
                            //left_upper_arm.velocity = {i, j, k, re};
                            break;
                        case 14:
                            left_forearm.frame_id = "left_forearm";
                            left_forearm.position->push_back(x);
                            left_forearm.position->push_back(y);
                            left_forearm.position->push_back(z);
                            //left_forearm.velocity = {i, j, k, re};
                            break;
                        case 15:
                            left_hand.frame_id = "left_hand";
                            left_hand.position->push_back(x);
                            left_hand.position->push_back(y);
                            left_hand.position->push_back(z);
                            //left_hand.velocity = {i, j, k, re};
                            break;
                        case 16:
                            right_upper_leg.frame_id = "right_upper_leg";
                            right_upper_leg.position->push_back(x);
                            right_upper_leg.position->push_back(y);
                            right_upper_leg.position->push_back(z);
                            //right_upper_leg.velocity = {i, j, k, re};
                            break;
                        case 17:
                            right_lower_leg.frame_id = "right_lower_leg";
                            right_lower_leg.position->push_back(x);
                            right_lower_leg.position->push_back(y);
                            right_lower_leg.position->push_back(z);
                            //right_lower_leg.velocity = {i, j, k, re};
                            break;
                        case 18:
                            right_foot.frame_id = "right_foot";
                            right_foot.position->push_back(x);
                            right_foot.position->push_back(y);
                            right_foot.position->push_back(z);
                            //right_foot.velocity = {i, j, k, re};
                            break;
                        case 19:
                            right_toe.frame_id = "right_toe";
                            right_toe.position->push_back(x);
                            right_toe.position->push_back(y);
                            right_toe.position->push_back(z);
                            //right_toe.velocity = {i, j, k, re};
                            break;
                        case 20:
                            left_upper_leg.frame_id = "left_upper_leg";
                            left_upper_leg.position->push_back(x);
                            left_upper_leg.position->push_back(y);
                            left_upper_leg.position->push_back(z);
                            //left_upper_leg.velocity = {i, j, k, re};
                            break;
                        case 21:
                            left_lower_leg.frame_id = "left_lower_leg";
                            left_lower_leg.position->push_back(x);
                            left_lower_leg.position->push_back(y);
                            left_lower_leg.position->push_back(z);
                            //left_lower_leg.velocity = {i, j, k, re};
                            break;
                        case 22:
                            left_foot.frame_id = "left_foot";
                            left_foot.position->push_back(x);
                            left_foot.position->push_back(y);
                            left_foot.position->push_back(z);
                            //left_lower_leg.velocity = {i, j, k, re};
                            break;
                        case 23:
                            left_toe.frame_id = "left_toe";
                            left_toe.position->push_back(x);
                            left_toe.position->push_back(y);
                            left_toe.position->push_back(z);
                            //left_lower_leg.velocity = {i, j, k, re};
                            break;
                        }
                    }
                    cur_index += 32 * header.datagram_counter;
                    int left = read_bytes - cur_index;
                    //printf("bytes left to parse = %d\n", left);
                    if (left > 0)
                        memcpy(buf, buf + cur_index, left);
                    read_bytes = left;
                    cur_index = 0;
                }
                else
                {
                    cur_index -= 24;
                }
            }
            else
            {
                //TODO
                printf("other type\n");
                std::cout << "The received message is: " << header.ID_String << std::endl;
            }
            //break;
        }
        round++;
        printf("round = %d\n", round);
    }
    free(buf);
}

/*
  server:
  socket-->bind-->recvfrom-->sendto-->close
*/
void signal_callback_handler(int signum)
{
    std::cout << "[INFO] Closing Connection!" << std::endl;
    close(server_fd);
    // Terminate program
    exit(signum);
}

int main(int argc, char *argv[])
{
    const std::string SERVER_IP = argv[1];

    server_fd = socket(AF_INET, SOCK_DGRAM, 0); //AF_INET: IPV4; SOCK_DGRAM: UDP
    if (server_fd < 0)
    {
        printf("[ERROR] Create socket fail!\n");
        return -1;
    }

    //Catch Keyboard Interrupt
    signal(SIGINT, signal_callback_handler);

    memset(&ser_addr, 0, sizeof(ser_addr));
    ser_addr.sin_family = AF_INET;
    ser_addr.sin_addr.s_addr = inet_addr(SERVER_IP.c_str());
    ser_addr.sin_port = htons(SERVER_PORT);

    ret = bind(server_fd, (struct sockaddr *)&ser_addr, sizeof(ser_addr));
    if (ret < 0)
    {
        perror("[Error] Binding port to Socket Initialization failed!");
        exit(EXIT_FAILURE);
    }

    visualize_xsens_data(server_fd, argc, argv);

    close(server_fd);
}
