// Compile-time introspection into the state graph
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef NIMBRO_FSM2_INTROSPECTION_H
#define NIMBRO_FSM2_INTROSPECTION_H

#include <boost/hana/type.hpp>
#include <boost/hana/set.hpp>
#include <boost/hana/tuple.hpp>
#include <boost/hana/functional/fix.hpp>
#include <boost/hana/fold.hpp>
#include <boost/hana/difference.hpp>
#include <boost/hana/union.hpp>

#include "detail/is_defined.h"

namespace nimbro_fsm2
{

template<class ... StartStates>
constexpr auto reachableStates()
{
	namespace hana = boost::hana;

	auto startStates = hana::tuple_c<StartStates...>;
	auto startSet = hana::to_set(startStates);

	auto visitor = hana::fix([](auto self, const auto& visited, auto x) {
		// Find out which of the successor states we have not visited so far
		using State = typename decltype(x)::type;
		static_assert(detail::is_defined<State>::value,
			"For introspection, you need to include *all* state headers! "
			"See below for the state class that you need to include."
		);

		auto successorStates = State::Transitions::SuccessorStateSet;
		auto newStates = hana::difference(successorStates, visited);

		// Make them visited
		auto includingNewStates = hana::union_(successorStates, visited);

		auto result = hana::fold(newStates, includingNewStates,
			[&self](const auto& lvisited, const auto& state) {
				return self(lvisited, state);
			}
		);

		return result;
	});

	auto state_set = hana::fold(startSet, startSet, visitor);

	return hana::to_tuple(state_set);
}

}

#endif
