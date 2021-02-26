#include <qi/applicationsession.hpp>
#include <qi/anyobject.hpp>
#include <iostream>
#include <string>

qi::AnyValue fromStringVectorToAnyValue(const std::vector<std::string> &vector)
{
  qi::AnyValue res;
  try
  {
    std::vector<qi::AnyValue> vector_qi;
    vector_qi.reserve(vector.size());
    vector_qi.resize(vector.size());

    std::vector<std::string>::const_iterator it = vector.begin();
    std::vector<qi::AnyValue>::iterator it_qi = vector_qi.begin();
    for(; it != vector.end(); ++it, ++it_qi)
    {
      *it_qi = qi::AnyValue(qi::AnyReference::from(*it), false, false);
    }
    res = qi::AnyValue(qi::AnyReference::from(vector_qi), false, false);
  }
  catch(const std::exception& e)
  {
    std::cout << "Could not convert to qi::AnyValue \n\tTrace: " << e.what() << std::endl;
  }
  return res;
}

qi::AnyValue fromDoubleVectorToAnyValue(const std::vector<double> &vector)
{
  qi::AnyValue res;
  try
  {
    std::vector<qi::AnyValue> vector_qi;
    vector_qi.reserve(vector.size());
    vector_qi.resize(vector.size());

    std::vector<double>::const_iterator it = vector.begin();
    std::vector<qi::AnyValue>::iterator it_qi = vector_qi.begin();
    for(; it != vector.end(); ++it, ++it_qi)
    {
      *it_qi = qi::AnyValue(qi::AnyReference::from(static_cast<float>(*it)), false, false);
    }
    res = qi::AnyValue(qi::AnyReference::from(vector_qi), false, false);
  }
  catch(const std::exception& e)
  {
    std::cout << "Could not convert to qi::AnyValue \n\tTrace: " << e.what() << std::endl;
  }
  return res;
}
int main(int argc, char** argv)
{
    if(argc < 3)
    {
        std::cerr << "You must specify the robot's IP" << std::endl;
        exit(2);
    }


    std::string IP_robot;
    std::string IP_robot_2;
    std::vector<double> angles;
    std::vector<std::string> body_names;
    //const std::string body = "Body";

    int host;
    host = 9559;
    IP_robot = argv[1];
    qi::Url url(IP_robot,"tcp",host);
    qi::SessionPtr session = qi::makeSession();




    IP_robot_2 = argv[2];
    qi::Url url2(IP_robot_2,"tcp",host);
    qi::SessionPtr session2 = qi::makeSession();
//    session2->connect(url2);
//

    session->connect(url);
    session2->connect(url2);


    qi::AnyObject motion = session->service("ALMotion");
//    motion.call<void>("wakeUp");

    std::cout << "Hola" << std::endl;



    qi::AnyObject motion2 = session2->service("ALMotion");

    body_names = motion2.call<std::vector<std::string>>("getBodyNames", "Body");
    //prepare the list of joints names
    qi::AnyValue names_qi = fromStringVectorToAnyValue(body_names);




    while (true) {
        angles = motion2.call<std::vector<double>>("getAngles","Body",1);
        qi::AnyValue angles_qi = fromDoubleVectorToAnyValue(angles);
        //prepare the list of joint angles
        motion.call<void>("setAngles",names_qi,angles_qi,0.2f);
        std::cout << "running" << std::endl;
    }


    return 0;
}

