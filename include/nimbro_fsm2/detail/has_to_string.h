// Checks if a particular driver class offers the toString() method
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef NIMBRO_FSM2_DETAIL_HAS_TO_STRING_H
#define NIMBRO_FSM2_DETAIL_HAS_TO_STRING_H

#include <type_traits>

namespace nimbro_fsm2
{
namespace detail
{

template<class T>
constexpr auto hasToString(int) -> decltype(std::declval<T>().toString(), std::true_type{})
{ return {}; }

template<class T>
constexpr auto hasToString(...) -> std::false_type
{ return {}; }

}
}

#endif
