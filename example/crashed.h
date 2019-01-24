// Car wrecked.
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef CRASHED_H
#define CRASHED_H

#include "fsm.h"

namespace my_fsm
{
class Idle;

class Crashed : public FSM::State<Crashed, FSM::Transitions<Idle>>
{
public:
	void enter(Driver& driver) override;
	virtual Transition execute(Driver& driver) override;
private:
	int m_count = 0;
};

}

#endif
