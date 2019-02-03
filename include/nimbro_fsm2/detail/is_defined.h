// Check if a type is fully defined instead of just forward-declared
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef NIMBRO_FSM2_DETAIL_IS_DEFINED_H
#define NIMBRO_FSM2_DETAIL_IS_DEFINED_H

namespace nimbro_fsm2
{
namespace detail
{

// from: https://stackoverflow.com/a/45594334
template <class T, class Enable = void>
struct is_defined
{
	static constexpr bool value = false;
};

template <class T>
struct is_defined<T, std::enable_if_t<(sizeof(T) > 0)>>
{
	static constexpr bool value = true;
};

template<class T>
static constexpr bool is_defined_v = is_defined<T>::value;

}
}

#endif
