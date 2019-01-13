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

namespace hana = boost::hana;

template<class StartState>
constexpr auto reachableStates()
{
	auto startState = hana::type_c<StartState>;
	auto startSet = hana::make_set(startState);

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

	auto state_set = visitor(startSet, startState);

	return hana::to_tuple(state_set);
}

}

#endif
