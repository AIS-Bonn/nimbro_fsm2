// Demo FSM (main entry point)
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "fsm.h"

#include "idle.h"
#include "driving.h"
#include "crashed.h"

#include <thread>
#include <chrono>
#include <iostream>

namespace my_fsm
{

void Driver::drive(double vx, double vy)
{
	std::cout << "Driving command: " << vx << ", " << vy << "\n";
}

}

int main(int argc, char** argv)
{
	my_fsm::Driver driver;
	my_fsm::FSM fsm(driver);

	//[StateName]
	std::cout << "Starting in state: " << my_fsm::Idle::Name.c_str() << "\n";
	//[StateName]

	fsm.start<my_fsm::Idle>();

	while(1)
	{
		using namespace std::chrono_literals;

		fsm.step();
		std::this_thread::sleep_for(1s);
	}
}
