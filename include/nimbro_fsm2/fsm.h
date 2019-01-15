// Finite state machine
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef NIMBRO_FSM2_FSM_H
#define NIMBRO_FSM2_FSM_H

#include <memory>
#include <iostream>
#include <variant>

#include <boost/hana/tuple.hpp>
#include <boost/hana/set.hpp>
#include <boost/hana/range.hpp>
#include <boost/hana/map.hpp>
#include <boost/hana/for_each.hpp>
#include <boost/hana/zip.hpp>
#include <boost/hana/at_key.hpp>

#include <ros/time.h>

#include <fmt/format.h>

#include "detail/type_name.h"
#include "detail/is_defined.h"
#include "introspection.h"

namespace nimbro_fsm2
{

/**
 * @brief Finite State Machine
 *
 * This class defines the necessary types and also contains the runtime
 * needed for Finite State Machines. It is templated on a @p DriverClass type,
 * which can be used by the FSM states to interact with the outside world.
 **/
template<class DriverClass>
class FSM
{
public:
	class Transition;

	/**
	 * @brief Abstract base class for states
	 *
	 * You should not directly use this class. Derive your class from State
	 * instead.
	 **/
	class StateBase
	{
	public:
		virtual ~StateBase() {}

		virtual Transition doExecute(DriverClass& driver) = 0;
		virtual void doEnter(DriverClass& driver) = 0;
		virtual void doLeave(DriverClass& driver) = 0;
	};

	class Stay
	{
	};

	/**
	 * @brief Transitions specifications
	 *
	 * This class is used in combination with the State class for specifying
	 * the successor states of your state. See the documentation of State for
	 * more details.
	 *
	 * @note The mentioned classes do not need to be defined, they can also be
	 *   forward-declared.
	 **/
	template<class ... SuccessorStates>
	class Transitions
	{
	public:
		static constexpr auto SuccessorStateSet = boost::hana::to_set(boost::hana::tuple_t<SuccessorStates...>);
	};

	/**
	 * @brief Runtime transition
	 *
	 * This is the return type of @ref State::execute and represents the
	 * transition to be executed. See @ref State::transit and @ref State::stay
	 * for how to create this type.
	 **/
	class [[nodiscard]] Transition
	{
	public:
		explicit Transition(const Stay& stay)
		 : m_data{stay}, m_label{"Stay"}
		{}

		/**
		 * @brief Cast operator
		 *
		 * Obtain the contained StateBase pointer. Caution: this throws if
		 * there is no successor state (e.g. @ref State::stay).
		 **/
		operator std::unique_ptr<StateBase>() &&
		{
			return std::move(std::get<1>(m_data));
		}

		/**
		 * @brief Check if there is a successor state
		 *
		 * This returns true iff this is a true transition i.e. not
		 * @ref State::stay.
		 **/
		constexpr operator bool() const
		{
			return m_data.index() != 0;
		}

		static Transition _unchecked(std::unique_ptr<StateBase>&& state, const char* label)
		{ return Transition(std::move(state), label); }

		constexpr const char* label() const
		{ return m_label; }
	private:
		explicit Transition(std::unique_ptr<StateBase>&& state, const char* label)
		 : m_data{std::move(state)}, m_label{label}
		{}

		std::variant<Stay, std::unique_ptr<StateBase>> m_data;
		const char* m_label;
	};

	/**
	 * @brief State
	 *
	 * Represents a single state in your FSM.
	 *
	 * @section FSM-State-Parameters Template parameters
	 *
	 * nimbro_fsm2 heavily uses compile-time verification and introspection.
	 * For this reason, you need to specify two template parameters:
	 *
	 * * @p Derived: This should be your state class.
	 * * @p TransitionSpec: Pass in @ref FSM::Transitions with the set of
	 *   allowed transitions from this state.
	 *
	 * @section FSM-State-usage Usage
	 *
	 * Common usage is to declare states in their own header file:
	 *
	 * @snippet driving.h Driving
	 **/
	template<class Derived, class TransitionSpec>
	class State : public StateBase
	{
	public:
		//! Transitions type for compile-time lookup
		using Transitions = TransitionSpec;

		/**
		 * @brief State name
		 *
		 * This state name is extracted at compile time and stored as a
		 * @p boost::hana::string. You can use `.c_str()` to get a runtime
		 * string:
		 *
		 * @snippet demo.cpp StateName
		 **/
		static constexpr auto Name = detail::type_name<Derived>();

		Transition doExecute(DriverClass& driver) override
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

		/**
		 * @brief Elapsed time
		 *
		 * This returns the elapsed time since the state was entered (i.e.
		 * since @ref State::enter was called).
		 **/
		ros::Duration elapsedTime() const
		{
			return ros::Time::now() - m_enterTime;
		}

		/**
		 * @brief Perform state transition
		 *
		 * This method is used to perform a state transition. The resulting
		 * @ref Transition should be returned from the @ref State::execute
		 * method.
		 *
		 * Example:
		 * @code
		 * Transition MyState::execute(Driver& driver)
		 * { return transit<NextState>(); }
		 * @endcode
		 *
		 * Calling @ref transit with a state not mentioned in your
		 * TransitionSpec will result in a compilation error.
		 *
		 * Any arguments are forwarded to the state constructor.
		 **/
		template<class T, class ... Args>
		Transition transit(Args&&... args)
		{
			namespace hana = boost::hana;

			if constexpr (detail::is_defined_v<T>)
			{
				if constexpr (hana::contains(TransitionSpec::SuccessorStateSet, hana::type_c<T>))
				{
					return Transition::_unchecked(std::make_unique<T>(std::forward<Args>(args)...), Name.c_str());
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

		/**
		 * @brief Stay in this state
		 *
		 * This method is used to stay in the current state. The resulting
		 * @ref Transition should be returned from the @ref State::execute
		 * method.
		 *
		 * Example:
		 * @code
		 * Transition MyState::execute(Driver& driver)
		 * { return stay(); }
		 * @endcode
		 **/
		Transition stay()
		{
			return Transition{Stay{}};
		}

		//! @name Hooks
		//@{
		//! Called after entering the state
		virtual void enter(DriverClass&)
		{}

		//! Called before leaving the state
		virtual void leave(DriverClass&)
		{}

		/**
		 * @brief State execution
		 *
		 * You have to implement this function. It is called once per FSM
		 * step while this state is active. On each iteration, the method
		 * either returns @ref stay to stay in this state, or uses @ref transit
		 * to cause a transition to another state:
		 *
		 * @snippet driving.cpp execute
		 **/
		virtual Transition execute(DriverClass& driver) = 0;
		//@}
	private:
		ros::Time m_enterTime;
	};


	explicit FSM(DriverClass& driver)
	 : m_driver(driver)
	{}

	template<class ... StartStates>
	void initialize()
	{
	}

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
		m_stateLabel = StartState::Name.c_str();
		m_state->doEnter(m_driver);
	}

	template<class StartState>
	void dumpDot()
	{
		namespace hana = boost::hana;
		using namespace hana::literals;

		constexpr auto stateList = hana::to_tuple(reachableStates<StartState>());
		constexpr auto statesWithIds = hana::zip_with(hana::make_pair,
			stateList,
			hana::to_tuple(hana::make_range(hana::int_c<0>, hana::length(stateList)))
		);
		constexpr auto stateIDMap = hana::to_map(statesWithIds);

		fmt::print("digraph {{\n");
		hana::for_each(statesWithIds, [](const auto& pair){
			auto state = hana::first(pair);
			using State = typename decltype(state)::type;
			int idx = hana::value(hana::second(pair));
			fmt::print("  v{} [ label=\"{}\" ];\n", idx, State::Name.c_str());
		});
		hana::for_each(statesWithIds, [](const auto& pair){
			auto state = hana::first(pair);
			using State = typename decltype(state)::type;
			int idx = hana::value(hana::second(pair));

			hana::for_each(State::Transitions::SuccessorStateSet, [&](auto suc){
				int idx2 = hana::value(stateIDMap[suc]);
				fmt::print("  v{} -> v{};", idx, idx2);
			});
			fmt::print("\n");
		});
		fmt::print("}}\n");
	}

	void step()
	{
		if(!m_state)
			return;

		Transition nextState = m_state->doExecute(m_driver);

		if(nextState)
		{
			m_state->doLeave(m_driver);
			m_stateLabel = nextState.label();
			m_state = std::move(nextState);
			m_state->doEnter(m_driver);
		}
	}

private:
	DriverClass& m_driver;
	std::unique_ptr<StateBase> m_state;
	const char* m_stateLabel;
};

}

#endif
