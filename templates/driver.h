// Driver for @package_name
// Author: @maintainer <@email>
@{headerdef = package_name.upper() + "_" + driver.upper()}
#ifndef @headerdef@ _H
#define @headerdef@ _H

#include "@fsm@ .h"

namespace @package_name
{

class @driver
{
public:
	@driver@ ();

	void work();
private:
	FSM m_fsm;
};

}

#endif
