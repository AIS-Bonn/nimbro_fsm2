// Example FSM
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef FSM_H
#define FSM_H

#include <nimbro_fsm2/fsm.h>

namespace my_fsm
{

class Driver
{
public:
	void drive(double vx, double vy);
};

using FSM = nimbro_fsm2::FSM<Driver>;
using Transition = FSM::Transition;

}

#endif
