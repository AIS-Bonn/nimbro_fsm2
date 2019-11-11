// Compile-time introspection into the state graph
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef NIMBRO_FSM2_INTROSPECTION_H
#define NIMBRO_FSM2_INTROSPECTION_H

#include <brigand/brigand.h>

#include "detail/is_defined.h"

namespace nimbro_fsm2
{

namespace detail
{
	template <bool>
	struct conditional;

	template <>
	struct conditional<true> {
		template <typename A, typename B>
		using f = typename A::type;
	};

	template <>
	struct conditional<false> {
		template <typename A, typename B>
		using f = typename B::type;
	};

	template<typename CurrentSet, typename State>
	struct Fold;

	template<typename CurrentSet, typename State>
	struct Recurse
	{
		using type = typename brigand::fold<typename State::Transitions::Set,
			brigand::insert<CurrentSet, State>,
			Fold<brigand::_state, brigand::_element>
		>;
	};

	template<typename CurrentSet, typename State>
	struct Fold
	{
		static_assert(detail::is_defined<State>::value,
			"For introspection, you need to include *all* state headers! "
			"See below for the state class that you need to include."
		);

		using alreadyKnown = brigand::contains<CurrentSet, State>;

		using type = typename conditional<alreadyKnown::value>::template f<
			brigand::type_<CurrentSet>,
			Recurse<CurrentSet, State>
		>;
	};
}

template<typename ... StartStates>
using ReachableStates = brigand::fold<
	brigand::set<StartStates...>,
	brigand::set<>,
	detail::Fold<brigand::_state, brigand::_element>
>;

}

#endif
