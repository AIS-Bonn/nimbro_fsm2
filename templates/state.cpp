// @state_name state
// Author: @maintainer <@email>

@{file_name = state_name.lower()}

#include "@file_name@ .h"

namespace @package_name
{

@[if state_ns]namespace @state_ns
{
@[end if]

//enter: code is executed once when entering this state
void @state_name@ ::enter()
{
}

//execute
Transition @state_name@ ::execute()
{

	return stay();
}

//leave: code is executed once when exiting this state
void @state_name@ ::leave()
{
}

@[if state_ns]}//State NS@[end if]

}//NS
