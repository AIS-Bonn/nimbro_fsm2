// Demo FSM (main entry point)
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "fsm.h"

#include "idle.h"
#include "driving.h"
#include "crashed.h"

#include <ros/init.h>
#include <ros/node_handle.h>

namespace my_fsm
{

void Driver::drive(double vx, double vy)
{
	std::cout << "Driving command: " << vx << ", " << vy << "\n";
}

void Driver::reportCrash()
{
	m_numCrashes++;
}

//[Driver::toString]
std::string Driver::toString() const
{
	return fmt::format("Number of crashes: {}", m_numCrashes);
}
//[Driver::toString]

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

	//[StateName]
	ROSFMT_INFO("Starting in state: {}", std::string_view{my_fsm::Idle::Name});
	//[StateName]

	//[setState]
	fsm.setState<my_fsm::Idle>();
	//[setState]

	ros::Rate rate(1.0);
	while(ros::ok())
	{
		ros::spinOnce();
		fsm.step();
		rate.sleep();
	}

	return 0;
}
