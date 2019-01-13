// Finite state machine
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef NIMBRO_FSM2_FSM_H
#define NIMBRO_FSM2_FSM_H

#include <variant>
#include <memory>
#include <iostream>

#include <boost/hana/tuple.hpp>
#include <boost/hana/set.hpp>

#include "detail/type_name.h"

namespace nimbro_fsm2
{

namespace detail
{

template <typename T, typename Variant>
struct has_type;

template <typename T, typename... Us>
struct has_type<T, std::variant<Us...>> : std::disjunction<std::is_same<T, Us>...> {};

}

template<class DriverClass>
class FSM
{
public:

	class StateBase
	{
	public:
		virtual ~StateBase() {}

		virtual std::unique_ptr<StateBase> doExecute(DriverClass& driver) = 0;
		virtual void doEnter(DriverClass& driver) = 0;
		virtual void doLeave(DriverClass& driver) = 0;
	};

	class Stay
	{
	};

	template<class ... SuccessorStates>
	class Transitions
	{
	public:
		using Variant = std::variant<Stay, std::unique_ptr<SuccessorStates>...>;
		using Tuple = std::tuple<SuccessorStates...>;

		static constexpr auto SuccessorStateSet = boost::hana::to_set(boost::hana::tuple_t<SuccessorStates...>);

		// Constructor from specific state
		template<class T>
		constexpr Transitions(T&& state)
		 : m_data{std::move(state)}
		{}

		operator std::unique_ptr<StateBase>() &&
		{
			return std::visit([](auto&& arg) -> std::unique_ptr<StateBase> {
				if constexpr(std::is_same_v<std::decay_t<decltype(arg)>, Stay>)
					return {};
				else
					return std::unique_ptr<StateBase>{arg.release()};
			}, m_data);
		}

		operator bool() const
		{
			return m_data.index() != 0;
		}
	private:
		Variant m_data;
	};

	template<class Derived, class TransitionSpec>
	class State : public StateBase
	{
	public:
		using Transition = TransitionSpec;

		static constexpr auto Name = detail::type_name<Derived>();

		std::unique_ptr<StateBase> doExecute(DriverClass& driver) override
		{
			Derived& derived = static_cast<Derived&>(*this);
			return derived.execute(driver);
		}

		void doEnter(DriverClass& driver) override
		{
			std::cout << "Entering " << Name.c_str() << "\n";
			Derived& derived = static_cast<Derived&>(*this);
			derived.enter(driver);
		}

		void doLeave(DriverClass& driver) override
		{
			Derived& derived = static_cast<Derived&>(*this);
			derived.leave(driver);
		}

		template<class T, class ... Args>
		Transition transit(Args&&... args)
		{
			if constexpr (detail::has_type<std::unique_ptr<T>, typename Transition::Variant>::value)
			{
				return std::make_unique<T>(std::forward<Args>(args)...);
			}
			else
			{
				static_assert(detail::has_type<std::unique_ptr<T>, typename Transition::Variant>::value,
					"You tried to transit to a state not mentioned in your TransitionSpec"
				);
			}
		}

		virtual void enter(DriverClass& driver)
		{}

		virtual void leave(DriverClass& driver)
		{}

		virtual Transition execute(DriverClass& driver) = 0;
	};


	explicit FSM(DriverClass& driver)
	 : m_driver(driver)
	{}

	void setState(std::unique_ptr<StateBase>&& state)
	{
		m_state = std::move(state);
		m_state->doEnter(m_driver);
	}

	void step()
	{
		if(!m_state)
			return;

		std::unique_ptr<StateBase> nextState = m_state->doExecute(m_driver);

		if(nextState)
		{
			m_state->doLeave(m_driver);
			m_state = std::move(nextState);
			m_state->doEnter(m_driver);
		}
	}

private:
	DriverClass& m_driver;
	std::unique_ptr<StateBase> m_state;
};

}

#endif
