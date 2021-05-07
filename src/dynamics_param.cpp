/**
 * @file dynamics_param.cpp
 * @author Selvakumar H S (franklinselva10@gmail.com)
 * @brief This file holds the dynamic parameters of NAO robot v5 H25. 
 * The parameters are acquired from http://doc.aldebaran.com/2-1/family/robots/masses_robot.html#robot-masses
 * @version 0.1
 * @date 2021-05-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <dynamics_param.h>

dPARAM::dPARAM()
{
    std::cout << "[INFO] Importing Mass, Inertial and CoM parameters of the robot" << std::endl;
    this->defMass();
    this->defCoM();
    this->defInertiaMatrix();
    std::cout << "[INFO] Imported Dynamic Parameters of the robot" << std::endl;
}

dPARAM::~dPARAM()
{
}

float dPARAM::getTotalMass() const
{
    return TotalMass;
}

std::vector<float> dPARAM::getMass() const
{
    return Mass;
}

std::vector<Eigen::Vector3d> dPARAM::getCoM() const
{
    return CentreOfMass;
}

std::vector<Eigen::Matrix3f> dPARAM::getInertia() const
{
    return InertiaMatrix;
}

float dPARAM::getMass(uint segment) const
{
    return Mass[segment];
}

Eigen::Vector3d dPARAM::getCoM(uint segment) const
{
    return CentreOfMass[segment];
}

Eigen::Matrix3f dPARAM::getInertia(uint segment) const
{
    return InertiaMatrix[segment];
}

void dPARAM::defMass()
{
    TotalMass = 5.305350006;

    //Appending individual Mass
    Mass.reserve(25);        //For a total of 25 segments
    Mass.push_back(0.60533); //Head
    Mass.push_back(0.07842); //Neck
    Mass.push_back(1.0496);  //Torso

    //Right arm chain
    Mass.push_back(0.09304); //Right Shoulder
    Mass.push_back(0.15777); //Right Biceps
    Mass.push_back(0.06483); //Right Elbow
    Mass.push_back(0.07761); //RIght ForeArm
    Mass.push_back(0.18533); //Right Hand

    //Left Arm chain
    Mass.push_back(0.09304); //Left Shoulder
    Mass.push_back(0.15777); //Left Biceps
    Mass.push_back(0.06483); //Left Elbow
    Mass.push_back(0.07761); //Left ForeArm
    Mass.push_back(0.18533); //Left Hand

    //Right Leg chain
    Mass.push_back(0.06981); //Right Pelvis
    Mass.push_back(0.14053); //RightHip
    Mass.push_back(0.38968); //Right Thigh
    Mass.push_back(0.30142); //Right Tibia
    Mass.push_back(0.13416); //Right Ankle
    Mass.push_back(0.17184); //Right Foot

    //Left Leg chain
    Mass.push_back(0.06981); //Left Pelvis
    Mass.push_back(0.14053); //LeftHip
    Mass.push_back(0.38968); //Left Thigh
    Mass.push_back(0.30142); //Left Tibia
    Mass.push_back(0.13416); //Left Ankle
    Mass.push_back(0.17184); //Left Foot
}

void dPARAM::defCoM()
{
    CentreOfMass.reserve(25);

    CentreOfMass.push_back(Eigen::Vector3d(-0.00112, 0, 0.05258)); //Head
    CentreOfMass.push_back(Eigen::Vector3d(-1e-05, 0, -0.02742));  //Neck
    CentreOfMass.push_back(Eigen::Vector3d(-0.00413, 0, 0.04342)); //Torso

    //Right Arm chain
    CentreOfMass.push_back(Eigen::Vector3d(-0.00165, 0.02663, 0.00014)); // Right Shoulder
    CentreOfMass.push_back(Eigen::Vector3d(0.02455, -0.00563, 0.0033));  //Right Biceps
    CentreOfMass.push_back(Eigen::Vector3d(-0.02744, 0, -0.00014));      //Right Elbow
    CentreOfMass.push_back(Eigen::Vector3d(0.02556, -0.00281, 0.00076)); //Right ForeArm
    CentreOfMass.push_back(Eigen::Vector3d(0.03434, 0.00088, 0.00308));  //Right Hand

    //Left Arm chain
    CentreOfMass.push_back(Eigen::Vector3d(-0.00165, -0.02663, 0.00014)); //Left Shoulder
    CentreOfMass.push_back(Eigen::Vector3d(0.02455, 0.00563, 0.0033));    //Left Biceps
    CentreOfMass.push_back(Eigen::Vector3d(-0.02744, 0, -0.00014));       //Left Elbow
    CentreOfMass.push_back(Eigen::Vector3d(0.02556, 0.00281, 0.00076));   //Left ForeArm
    CentreOfMass.push_back(Eigen::Vector3d(0.03434, -0.00088, 0.00308));  //Left Hand

    //Right Leg chain
    CentreOfMass.push_back(Eigen::Vector3d(-0.00781, 0.01114, 0.02661));    //Right Pelvis
    CentreOfMass.push_back(Eigen::Vector3d(-0.015149, -0.00029, -0.00515)); //Right Hip
    CentreOfMass.push_back(Eigen::Vector3d(0.00138, -0.00221, -0.05373));   //Right Thigh
    CentreOfMass.push_back(Eigen::Vector3d(0.00453, -0.00225, -0.04936));   //Right Tibia
    CentreOfMass.push_back(Eigen::Vector3d(0.00045, -0.00029, 0.00685));    //Right Ankle
    CentreOfMass.push_back(Eigen::Vector3d(0.02542, -0.0033, -0.03239));    //Right Foot

    //Left Leg chain
    CentreOfMass.push_back(Eigen::Vector3d(-0.00781, -0.01114, 0.02661)); //Left Pelvis
    CentreOfMass.push_back(Eigen::Vector3d(-0.01549, 0.00029, -0.00515)); //Left Hip
    CentreOfMass.push_back(Eigen::Vector3d(0.00138, 0.00221, -0.05373));  //Left Thigh
    CentreOfMass.push_back(Eigen::Vector3d(0.00453, 0.00225, -0.04936));  //Left Tibia
    CentreOfMass.push_back(Eigen::Vector3d(0.00045, 0.00029, 0.00685));   //Left Ankle
    CentreOfMass.push_back(Eigen::Vector3d(0.02542, 0.0033, -0.03239));   //Left Foot
}

void dPARAM::defInertiaMatrix()
{
    InertiaMatrix.reserve(25);
    Eigen::Matrix3f temp;

    //Head chain
    temp << 0.0026312952396, 8.788139894e-06, 4.0984661609e-05,
        8.788139894e-06, 0.0024911249056, -2.995792056e-05,
        4.0984661609e-05, -2.995792056e-05, 0.00098573567811; //Head
    InertiaMatrix.push_back(temp);
    temp.setZero();

    temp << 7.4992953159e-05, 1.570000089e-09, -1.8339999741e-08,
        1.5700000189e-09, 7.5999952969e-05, -5.294999994e-08,
        -1.8339999741e-08, -5.294999994e-08, 5.5337300182e-06; //Neck
    InertiaMatrix.push_back(temp);
    temp.setZero();

    temp << 0.0050623407587, 1.431158344e-05, 0.00015519082081,
        1.4311580344e-05, 0.0048801358789, -2.7079340725e-05,
        0.00015519082081, -2.7079340725e-05, 0.00161300038; //Torso
    InertiaMatrix.push_back(temp);
    temp.setZero();

    //Right Arm chain
    temp << 8.4284300101e-05, 2.0280199351e-06, 2.3380000158e-08,
        2.0280199351e-06, 1.4155610188e-05, 1.9719999855e-08,
        2.3380000158e-08, 1.9719999855e-08, 8.6419488071e-05; //Right Shoulder
    InertiaMatrix.push_back(temp);
    temp.setZero();

    temp << 0.00011012030882, 7.6691307186e-05, -2.6046069252e-05,
        7.6691307186e-05, 0.00036757651833, 1.2098280422e-05,
        -2.6994710424e-05, -2.4597700303e-06, 0.00034190082806; //Right Bicep
    InertiaMatrix.push_back(temp);
    temp.setZero();

    temp << 5.5971499933e-06, 4.209999042e-09, 4.3189999133e-08,
        4.209999042e-09, 7.5433119491e-05, -1.8400000412e-09,
        4.3189999133e-08, -1.8400000412e-09, 7.6443393482e-05; //Right Elbow
    InertiaMatrix.push_back(temp);
    temp.setZero();

    temp << 2.5390700102e-05, 2.3324300855e-06, -6.0116997247e-07,
        2.3324300855e-06, 8.9220360678e-05, 2.6940000453e-08,
        -6.0116997247e-07, 2.6940000453e-08, 8.7248430646e-05; //Right Forearm
    InertiaMatrix.push_back(temp);
    temp.setZero();

    temp << 7.0549329394e-05, 5.715990028e-06, -2.247437078e-05,
        5.715990028e-06, 0.0003560623154, 3.1777099139e-06,
        -2.247437078e-05, 3.1777099139e-06, 0.00035191932693; //Right Hand
    InertiaMatrix.push_back(temp);
    temp.setZero();

    //Left Arm chain
    temp << 8.4284300101e-05, -2.0280199351e-06, 2.3380000158e-08,
        -2.0280199351e-06, 1.4155610188e-05, -1.9719999855e-08,
        2.3380000158e-08, -1.9719999855e-08, 8.6419488071e-05; //Left Shoulder
    InertiaMatrix.push_back(temp);
    temp.setZero();

    temp << 9.3899929198e-05, -4.7144520067e-05, -2.6994710424e-05,
        -4.71445200067e-05, 0.00037151877768, -2.4597700303e-06,
        -2.6994710424e-05, -2.4597700303e-06, 0.00034190082806; //Left Bicep
    InertiaMatrix.push_back(temp);
    temp.setZero();

    temp << 5.5971499933e-06, 4.209999042e-09, 4.3189999133e-08,
        4.209999042e-09, 7.5433119491e-05, -1.8400000412e-09,
        4.3189999133e-08, -1.8400000412e-09, 7.6443393482e-05; //Left Elbow
    InertiaMatrix.push_back(temp);
    temp.setZero();

    temp << 2.5332199584e-05, -2.3427101041e-06, 7.4589998178e-08,
        -2.3427101041e-06, 8.91321979e-05, -2.6549999532e-08,
        7.4589998178e-08, -2.6549999532e-08, 8.7287262431e-05; //Left Forearm
    InertiaMatrix.push_back(temp);
    temp.setZero();

    temp << 7.0549329394e-05, 5.715990028e-06, -2.247437078e-05,
        5.715990028e-06, 0.0003560623154, 3.1777099139e-06,
        -2.247437078e-05, 3.1777099139e-06, 0.00035191932693; //Left Hand
    InertiaMatrix.push_back(temp);
    temp.setZero();

    //Right Leg chain
    temp << 8.9971952548e-05, 5.0021899369e-06, 1.2735249584e-05,
        5.0021899369e-06, 0.00010552610911, -2.7700800274e-05,
        1.2735249584e-04, -2.7700800274e-05, 6.6887238062e-05; //Right Pelvis
    InertiaMatrix.push_back(temp);
    temp.setZero();

    temp << 2.7586540455e-05, -1.91900007e-08, -4.108219855e-06,
        -1.91900007e-08, 9.8269956652e-05, 2.5099999856e-09,
        -4.108219855e-06, 2.5099999856e-09, 8.8103319285e-05; //Right Hip
    InertiaMatrix.push_back(temp);
    temp.setZero();

    temp << 0.0016374820843, -8.3954000729e-07, 8.5883009888e-05,
        -8.3954000729e-07, 0.0015922139864, -3.9176258724e-05,
        8.5883009888e-05, -3.9176258724e-05, 0.00030397824594; //Right Thigh
    InertiaMatrix.push_back(temp);
    temp.setZero();

    temp << 0.0011828296119, -8.96500012e-07, 2.7996900826e-05,
        -8.96500012e-07, 0.0011282785563, -3.8476038753e-05,
        2.7996900826e-05, -3.8476038753e-05, 0.00019145276747; //Right Tibia
    InertiaMatrix.push_back(temp);
    temp.setZero();

    temp << 3.8508129364e-05, 6.4339999994e-08, 3.8746597966e-06,
        6.4339999994e-08, 7.4310817581e-05, -4.5799999349e-09,
        3.8746597966e-06, -4.5799999349e-09, 5.491311822e-05; //Right Ankle
    InertiaMatrix.push_back(temp);
    temp.setZero();

    temp << 0.00026930202148, 5.8750501921e-06, 0.00013913327712,
        5.8750501921e-06, 0.00064347387524, -1.8849170374e-05,
        0.00013913327712, -1.8849170374e-05, 0.00052503478946; //Right Foot
    InertiaMatrix.push_back(temp);
    temp.setZero();

    //Left Leg chain
    temp << 8.1502330431e-05, -4.9944901548e-06, 1.2748169866e-05,
        -4.9944901548e-06, 0.00010132555326, 2.3454740585e-05,
        1.2748169866e-05, 2.3454740585e-05, 6.2623628764e-04; // Left Pelvis
    InertiaMatrix.push_back(temp);
    temp.setZero();

    temp << 2.7583539122e-05, -2.2329999183e-08, -4.0816398723e-06,
        -2.2329999183e-08, 9.8270553281e-05, -4.1899999026e-09,
        -4.0816398723e-06, -4.1899999026e-09, 8.8099732238e-05; //Left Hip
    InertiaMatrix.push_back(temp);
    temp.setZero();

    temp << 0.001636719564, 9.2451000455e-07, 8.5306681285e-05,
        9.2451000455e-07, 0.001591072767, 3.8361598854e-05,
        8.5306681285e-05, 3.8361598854e-05, 0.00030374340713; //Left Thigh
    InertiaMatrix.push_back(temp);
    temp.setZero();

    temp << 0.0011820796644, 6.3362000446e-07, 3.6496971006e-05,
        6.3362000446e-07, 0.0011286522495, 3.949522943e-05,
        3.6496971006e-05, 3.949522943e-05, 0.00019322744629; //Left Tibia
    InertiaMatrix.push_back(temp);
    temp.setZero();

    temp << 3.8509781007e-05, -2.6340000403e-08, 3.8619400584e-06,
        -2.6340000403e-08, 7.4265262811e-05, 1.8339999741e-08,
        3.8619400584e-06, 1.8339999741e-08, 5.4865398852e-05; //Left Ankle
    InertiaMatrix.push_back(temp);
    temp.setZero();

    temp << 0.00026944180718, -5.6957201195e-06, 0.00013937948097,
        -5.6957201195e-06, 0.00064434250817, 1.8740920495e-05,
        0.00013937948097, 1.8740920495e-05, 0.00052575673908; //Left Foot
    InertiaMatrix.push_back(temp);
    temp.setZero();
}
