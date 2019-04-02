// fsm header for @package_name
// Author: FSM2 Template <template@@ais.uni-bonn.de>

@{headerdef = package_name.upper()}

#ifndef @headerdef@ _H
#define @headerdef@ _H

#include <nimbro_fsm2/fsm.h>

namespace @package_name
{

class @driver@ ;

using FSM = nimbro_fsm2::FSM<@driver>;
using Transition = FSM::Transition;

}

#endif
