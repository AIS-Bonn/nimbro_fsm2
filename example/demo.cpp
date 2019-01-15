// Demo FSM (main entry point)
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "fsm.h"

#include "idle.h"
#include "driving.h"
#include "crashed.h"

#include <thread>
#include <chrono>
#include <iostream>

#include <ros/init.h>
#include <ros/node_handle.h>

namespace my_fsm
{

void Driver::drive(double vx, double vy)
{
	std::cout << "Driving command: " << vx << ", " << vy << "\n";
}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "demo", ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;

	my_fsm::Driver driver;
	my_fsm::FSM fsm(driver);

	//[initialize]
	fsm.initialize<my_fsm::Idle>();
	//[initialize]

	std::cout << "Here is a dot graph of my states:\n";
	fsm.dumpDot<my_fsm::Idle>();

	//[StateName]
	std::cout << "Starting in state: " << my_fsm::Idle::Name.c_str() << "\n";
	//[StateName]

	//[start]
	fsm.start<my_fsm::Idle>();
	//[start]

	while(1)
	{
		using namespace std::chrono_literals;

		fsm.step();
		std::this_thread::sleep_for(1s);
	}
}
