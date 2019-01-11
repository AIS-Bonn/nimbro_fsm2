// Example FSM
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef TEST_H
#define TEST_H

#include "fsm.h"

namespace my_fsm
{

class Driver
{
public:
	void drive(double vx, double vy);
};

using FSM = nimbro_fsm2::FSM<Driver>;

class Idle;
class Driving;
class Crashed;

class Idle : public FSM::State<Idle, FSM::Transitions<Driving>>
{
public:
	virtual Transition execute(Driver& driver) override;
};

class Driving : public FSM::State<Driving, FSM::Transitions<Idle, Crashed>>
{
public:
	virtual void enter(Driver& driver) override;
	virtual Transition execute(Driver& driver) override;
private:
	int m_count = 0;
};

class Crashed : public FSM::State<Crashed, FSM::Transitions<>>
{
public:
	virtual Transition execute(Driver& driver) override;
};

}

#endif
