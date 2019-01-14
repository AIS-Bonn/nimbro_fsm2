// Car wrecked.
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef CRASHED_H
#define CRASHED_H

#include "fsm.h"

namespace my_fsm
{

class Crashed : public FSM::State<Crashed, FSM::Transitions<>>
{
public:
	virtual Transition execute(Driver& driver) override;
};

}

#endif
