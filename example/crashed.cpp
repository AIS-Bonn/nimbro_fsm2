// Car wrecked.
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "crashed.h"

#include <iostream>

namespace my_fsm
{

Transition Crashed::execute(Driver& driver)
{
	std::cout << "It hurts!\n";
	return stay();
}

}
