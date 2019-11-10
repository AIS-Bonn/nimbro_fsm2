// @state_name state
// Author: @maintainer <@email>
@{file_name = state_name.lower()}
#include "@file_name@ .h"

namespace @package_name
{
@[if state_ns]namespace @state_ns
{
@[end if]
void @state_name@ ::enter()
{
	// Executed once when entering this state
}

Transition @state_name@ ::execute()
{
	// Execute: Determines the action on each cycle
	return stay();
}

void @state_name@ ::leave()
{
	// Executed once when leaving this state
}

@[if state_ns]} // @state_ns@[end if]
} // @package_name
