// @state_name state
// Author: @maintainer <@email>
@{
if state_ns:
	headerdef = state_ns.upper() + "_" + state_name.upper()
else:
	headerdef = state_name.upper()
}
#ifndef @headerdef@ _H
#define @headerdef@ _H

#include "@fsm@ .h"

namespace @package_name
{
@[if state_ns]namespace @state_ns
{@[end if]
// Forward declarations of successor states
// class SuccessorState;

class @state_name : public FSM::State<@state_name, FSM::Transitions</*SuccessorState*/>>
{
public:
	void enter() override;
	Transition execute() override;
	void leave() override;

private:
};

@[if state_ns]} // @state_ns@[end if]
} // @package_name

#endif
