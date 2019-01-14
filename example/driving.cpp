// Actually drive the car
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "driving.h"
#include "idle.h"
#include "crashed.h"

#include <iostream>

namespace my_fsm
{

void Driving::enter(Driver& driver)
{
	std::cout << "Wohoo, starting to drive!\n";

	std::random_device dev;
	m_randomGenerator.seed(dev());
}

//[execute]
Transition Driving::execute(Driver& driver)
{
	if(std::bernoulli_distribution{0.2}(m_randomGenerator))
	{
		std::cout << "Didn't see that tree coming :-(\n";
		return transit<Crashed>();
	}

	if(++m_count == 10)
	{
		std::cout << "Phew, made it!\n";
		return transit<Idle>();
	}

	// Driving along
	driver.drive(0.0, 1.0);
	return stay();
}
//[execute]

}
