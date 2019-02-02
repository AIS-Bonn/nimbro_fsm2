// Idle state: wait for start
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "idle.h"
#include "driving.h"

#include <iostream>

namespace my_fsm
{

Transition Idle::execute()
{
	std::cout << "Let's go for a ride!\n";
	return transit<Driving>();
}

}
