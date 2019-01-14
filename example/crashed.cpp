// Car wrecked.
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "crashed.h"
#include "idle.h"

#include <iostream>

namespace my_fsm
{

Transition Crashed::execute(Driver& driver)
{
	if(++m_count == 4)
	{
		std::cout << "Got rescued.\n";
		return transit<Idle>();
	}

	std::cout << "It hurts!\n";
	return stay();
}

}
