// Idle state: wait for start
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef IDLE_H
#define IDLE_H

#include "fsm.h"

namespace my_fsm
{

class Driving;

class Idle : public FSM::State<Idle, FSM::Transitions<Driving>>
{
public:
	virtual Transition execute(Driver& driver) override;
};

}

#endif
