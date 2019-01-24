// Car wrecked.
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "crashed.h"
#include "idle.h"

#include <iostream>

namespace my_fsm
{

void Crashed::enter(Driver& driver)
{
	std::cout << "crashed!\n";
	driver.reportCrash();
}

Transition Crashed::execute(Driver& driver)
{
	display("count: {}", m_count);

	if(++m_count == 4)
	{
		std::cout << "Got rescued.\n";
		return transit<Idle>();
	}

	std::cout << "It hurts!\n";
	return stay();
}

}
