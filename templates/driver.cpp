// Driver for @package_name
// Author: @maintainer <@email>
@{
state_include = state_name.lower()
if state_ns:
	state_include = state_ns + "/" + state_name.lower()

driver_lower = driver.lower()
if not state_ns:
	state_ns = ""
else:
	state_ns = state_ns + "::"
}
#include "@driver_lower@ .h"

#include <ros/init.h>
#include <ros/rate.h>
@[if create_state]#include "states/@state_include@ .h"
@[end if]
namespace @package_name
{

@driver@ ::@driver@ ()
 : m_fsm(*this) // give the FSM a reference to the driver object
{}

void @driver@ ::work()
{
	// Initialize the ROS interface. Here, we have to give a list of the
	// possible entry states as template parameters.
	@[if create_state]m_fsm.initialize<@state_ns@state_name>();@[else]//Example: m_fsm.initialize<ns::State>();@[end if]

	// Set the start state
	@[if create_state]m_fsm.setState<@state_ns@state_name>();@[else]//Example: m_fsm.setState<ns::State>();@[end if]

	// .. and go!
	ros::Rate rate(ros::Duration(1.0));
	while(ros::ok())
	{
		ros::spinOnce();
		m_fsm.step();
		rate.sleep();
	}
}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "@package_name");

	@package_name@ ::@driver @driver_lower@ ;
	@driver_lower@ .work();

	return 0;
}
