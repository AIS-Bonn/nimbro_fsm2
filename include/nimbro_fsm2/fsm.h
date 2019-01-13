// Finite state machine
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef NIMBRO_FSM2_FSM_H
#define NIMBRO_FSM2_FSM_H

#include <memory>
#include <iostream>
#include <variant>

#include <boost/hana/tuple.hpp>
#include <boost/hana/set.hpp>
#include <boost/hana/for_each.hpp>

#include <ros/time.h>

#include "detail/type_name.h"
#include "detail/is_defined.h"
#include "introspection.h"

namespace nimbro_fsm2
{

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
		static constexpr auto SuccessorStateSet = boost::hana::to_set(boost::hana::tuple_t<SuccessorStates...>);
	};

	class Transition
	{
	public:
		explicit Transition(const Stay& stay)
		 : m_data{stay}
		{}

		operator std::unique_ptr<StateBase>() &&
		{
			return std::move(std::get<1>(m_data));
		}

		operator bool() const
		{
			return m_data.index() != 0;
		}

		static Transition _unchecked(std::unique_ptr<StateBase>&& state)
		{ return Transition(std::move(state)); }
	private:
		explicit Transition(std::unique_ptr<StateBase>&& state)
		 : m_data{std::move(state)}
		{}

		std::variant<Stay, std::unique_ptr<StateBase>> m_data;
	};

	template<class Derived, class TransitionSpec>
	class State : public StateBase
	{
	public:
		using Transitions = TransitionSpec;

		static constexpr auto Name = detail::type_name<Derived>();

		std::unique_ptr<StateBase> doExecute(DriverClass& driver) override
		{
			Derived& derived = static_cast<Derived&>(*this);
			return derived.execute(driver);
		}

		void doEnter(DriverClass& driver) override
		{
			std::cout << "Entering " << Name.c_str() << "\n";
			m_enterTime = ros::Time::now();

			Derived& derived = static_cast<Derived&>(*this);
			derived.enter(driver);
		}

		void doLeave(DriverClass& driver) override
		{
			Derived& derived = static_cast<Derived&>(*this);
			derived.leave(driver);
		}

		ros::Duration elapsedTime() const
		{
			return ros::Time::now() - m_enterTime;
		}

		template<class T, class ... Args>
		Transition transit(Args&&... args)
		{
			namespace hana = boost::hana;

			if constexpr (detail::is_defined_v<T>)
			{
				if constexpr (hana::contains(TransitionSpec::SuccessorStateSet, hana::type_c<T>))
				{
					return Transition::_unchecked(std::make_unique<T>(std::forward<Args>(args)...));
				}
				else
				{
					static_assert(hana::contains(TransitionSpec::SuccessorStateSet, hana::type_c<T>),
						"You tried to transit to a state not mentioned in your TransitionSpec"
					);
				}
			}
			else
			{
				static_assert(detail::is_defined_v<T>,
					"You tried to transit to a forward-declared state. "
					"Please include the appropriate header in your .cpp file."
				);
			}
		}

		Transition stay()
		{
			return Transition{Stay{}};
		}

		virtual void enter(DriverClass&)
		{}

		virtual void leave(DriverClass&)
		{}

		virtual Transition execute(DriverClass& driver) = 0;
	private:
		ros::Time m_enterTime;
	};


	explicit FSM(DriverClass& driver)
	 : m_driver(driver)
	{}

	template<class StartState, class ... Args>
	void start(Args ... args)
	{
		std::cout << "Finite State Machine with states:\n";
		auto stateList = reachableStates<StartState>();
		boost::hana::for_each(stateList, [](auto state){
			using State = typename decltype(state)::type;
			std::cout << " - " << State::Name.c_str() << " trans [";

			auto successorStates = State::Transitions::SuccessorStateSet;
			boost::hana::for_each(successorStates, [](auto successorState) {
				using SuccessorState = typename decltype(successorState)::type;
				std::cout << SuccessorState::Name.c_str() << ", ";
			});
			std::cout << "]\n";
		});

		m_state = std::make_unique<StartState>(std::forward(args)...);
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
