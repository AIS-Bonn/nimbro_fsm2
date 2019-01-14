// Actually drive the car
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef DRIVING_H
#define DRIVING_H

#include "fsm.h"

#include <random>

namespace my_fsm
{

//[Driving]
class Idle;
class Crashed;

class Driving : public FSM::State<Driving, FSM::Transitions<Idle, Crashed>>
{
public:
	virtual void enter(Driver& driver) override;
	virtual Transition execute(Driver& driver) override;
private:
	std::mt19937 m_randomGenerator;
	int m_count = 0;
};
//[Driving]

}

#endif
