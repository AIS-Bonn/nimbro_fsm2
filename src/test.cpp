// Example FSM
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "test.h"
#include "introspection.h"

#include <iostream>
#include <chrono>
#include <thread>

#include <boost/hana/for_each.hpp>

namespace my_fsm
{

void Driver::drive(double vx, double vy)
{
	std::cout << "Driving: " << vx << " " << vy << "\n";
}

// IDLE
Idle::Transition Idle::execute(Driver& driver)
{
	std::cout << "Let's go for a ride!\n";
	return transit<Driving>();
}

// DRIVING
void Driving::enter(Driver& driver)
{
	std::cout << "Wohoo, starting to drive!\n";
}

Driving::Transition Driving::execute(Driver& driver)
{
	driver.drive(0.0, 1.0);

	if(++m_count == 10)
	{
		std::cout << "This doesn't look good!\n";
		return transit<Crashed>();
	}

	return stay();
}

// CRASHED
Crashed::Transition Crashed::execute(Driver& driver)
{
	std::cout << "It hurts!\n";
	return stay();
}

}


int main(int argc, char** argv)
{
	my_fsm::Driver driver;
	my_fsm::FSM fsm(driver);

	fsm.start<my_fsm::Idle>();

	while(1)
	{
		using namespace std::chrono_literals;

		fsm.step();
		std::this_thread::sleep_for(1s);
	}
}
