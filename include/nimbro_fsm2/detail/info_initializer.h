// Initializes Info messages with static state information
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef NIMBRO_FSM2_INFO_INITIALIZER_H
#define NIMBRO_FSM2_INFO_INITIALIZER_H

#include <brigand/brigand.h>

#include <nimbro_fsm2/Info.h>

namespace nimbro_fsm2
{
namespace detail
{

template<class... T>
struct SuccessorInitializer {};

template<class... T>
struct SuccessorInitializer<brigand::detail::set_impl<T...>>
{
	std::vector<std::string> successors{
		std::string_view{T::Name}...
	};
};

template<class... T>
struct InfoInitializer {};

StateInfo createInfo(const std::string_view& name, std::vector<std::string>&& successors)
{
	StateInfo ret;
	ret.name = name;
	ret.successors = std::move(successors);

	return ret;
}

template<class... T>
struct InfoInitializer<brigand::detail::set_impl<T...>>
{
	std::vector<StateInfo> states{
		createInfo(std::string_view{T::Name}, SuccessorInitializer<typename T::Transitions::Set>{}.successors)...
	};
};

}
}

#endif
