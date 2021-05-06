/*
 * Copyright (c) 2012-2019 SoftBank Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */
#include <iostream>
#include <alproxies/almemoryproxy.h>
#include <alproxies/almotionproxy.h>

#include <string>

int main(int argc, char *argv[])
{

    if(argc<3)
    {
        cerr << "You must specify: speed, IP robot controlled, IP robot to control" << endl;
        exit(2);
    }

	// argument 1 : speed, IP robot controlled, IP robot to control
	int nbr_guest_robot = argc-2;
	float speed= atof(argv[1]);
	std::string main_host=argv[2];
	std::string guest_host = argv[3];
	AL::ALMotionProxy host_motion(main_host);
	AL::ALMotionProxy guest_motion(guest_host);
	AL::ALValue angle;
	AL::ALValue name;
	std::string body = "Body";
	name=host_motion.getBodyNames(body);
	while (true){
		angle = host_motion.getAngles(body,false);
		guest_motion.setAngles(name,angle,speed);
	}
	return 0;
}
