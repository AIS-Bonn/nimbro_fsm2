// Finite state machine
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef NIMBRO_FSM2_FSM_H
#define NIMBRO_FSM2_FSM_H

#include <memory>
#include <variant>

#include <brigand/brigand.h>
#include <type_name/type_name.hpp>

#include <ros/time.h>

#include <rosfmt/rosfmt.h>
#include <fmt/format.h>

#include "detail/ros_interface.h"
#include "detail/has_to_string.h"
#include "detail/is_defined.h"
#include "detail/pointer.h"
#include "detail/format.h"
#include "detail/watchdog.h"
#include "introspection.h"

#if !NIMBRO_FSM2_CHECKED
#warning You are not compiling with the nimbro_fsm2 compiler plugin.\
 You probably need to install clang and delete the build directories of\
 nimbro_fsm2 and your package.
#endif

namespace nimbro_fsm2
{

namespace detail
{
	// Used for the Clang plugin for state discovery without going through
	// template magic.
	class AbstractState
	{
	};

	template<class... T>
	class Collector
	{
	};

	template<class... T>
	struct toCollector {};

	template<class... T>
	struct toCollector<brigand::detail::set_impl<T...>>
	{ using type = Collector<T...>; };
}

/**
 * @brief Finite State Machine
 *
 * This class defines the necessary types and also contains the runtime
 * needed for Finite State Machines. It is templated on a @p DriverClass type,
 * which can be used by the FSM states to interact with the outside world.
 *
 * The FSM offers a ROS interface for introspection (information about the
 * current state and state history), as well as control (switching to a new
 * state).
 *
 * The @p DriverClass can offer a `std::string DriverClass::toString() const`
 * method. In that case, the information given by this method is reported
 * in the ROS status message for display in the GUI.
 **/
template<class DriverClass>
class FSM
{
public:
	static constexpr auto Namespace = type_name::namespace_of_v<DriverClass>;

	class Transition;

	/**
	 * @brief Abstract base class for states
	 *
	 * You should not directly use this class. Derive your class from State
	 * instead.
	 **/
	class StateBase : private detail::AbstractState
	{
	public:
		virtual ~StateBase() {}

		void setDriver(DriverClass* driver)
		{ m_driver = driver; }

		virtual Transition doExecute() = 0;
		virtual void doEnter() = 0;
		virtual void doLeave() = 0;
		virtual std::string collectDisplayMessages() = 0;

	protected:
		DriverClass* _driver()
		{ return m_driver; }
	private:
		DriverClass* m_driver = nullptr;
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
		/**
		 * @brief Set of possible successor states
		 *
		 * This is a brigand::set of all the possible successor states.
		 **/
		using Set = brigand::set<SuccessorStates...>;
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
		operator detail::Pointer<StateBase>() &&
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

		static Transition _unchecked(detail::Pointer<StateBase>&& state, const std::string_view& label)
		{ return Transition(std::move(state), label); }

		constexpr std::string_view label() const
		{ return m_label; }
	private:
		explicit Transition(detail::Pointer<StateBase>&& state, const std::string_view& label)
		 : m_data{std::move(state)}, m_label{label}
		{}

		std::variant<Stay, detail::Pointer<StateBase>> m_data;
		std::string_view m_label;
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

		//! @name Information
		//@{
		/**
		 * @brief State name
		 *
		 * This state name is extracted at compile time and stored either as
		 * @p std::string_view (GCC) or a custom static string type (clang).
		 * You can use `.c_str()` to get a runtime string.
		 *
		 * @snippet demo.cpp StateName
		 **/
		static constexpr auto Name = type_name::relative_name_v<
			Derived, DriverClass
		>;

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
		//@}

		//! @name Logging & visualization
		//@{
		/**
		 * @brief Display values during state execution
		 *
		 * With this method you can visualize high-frequency information from
		 * your state execution method. It uses the @p fmt syntax for format
		 * strings. All emitted display strings for one FSM cycle are aggregated
		 * and displayed to the user in the @p nimbro_fsm2 GUI. Additionally,
		 * the last collected messages are displayed at state exit, so that one
		 * can more easily reason about the conditions that led to the
		 * transition. Example:
		 *
		 * @snippet driving.cpp display
		 **/
		template<class ... Args>
		void display(const std::string& formatString, const Args& ... args)
		{
			fmt::format_arg_store<fmt::format_context, Args...> as{args...};
			detail::vformat_to_nl(m_displayBuffer, formatString, as);
		}
		//@}

		//! @name Actions
		//@{
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
			if constexpr (detail::is_defined_v<T>)
			{
				if constexpr (brigand::contains<typename TransitionSpec::Set, T>::value)
				{
					return Transition::_unchecked(
						detail::Pointer<T>{detail::InPlaceInit, std::forward<Args>(args)...},
						std::string_view{T::Name}
					);
				}
				else
				{
					static_assert(brigand::contains<typename TransitionSpec::Set, T>::value,
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
		//@}

		//! @name Hooks
		//@{
		//! Called after entering the state
		virtual void enter()
		{}

		//! Called before leaving the state
		virtual void leave()
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
		virtual Transition execute() = 0;
		//@}

		//!@name I/O
		//@{
		/**
		 * @brief Obtain a reference to the global driver object
		 *
		 * You can use this function to access the driver object, which may
		 * be used to store information (e.g. discovered objects/poses/etc).
		 *
		 * @warning You cannot use this function in the constructor of your
		 * state! The reference is set up *after* creation of the state object.
		 **/
		[[nodiscard]] DriverClass& driver()
		{
			auto* ptr = this->_driver();
			if(!ptr)
			{
				throw std::logic_error("You called driver() before the state "
					"was fully constructed!");
			}

			return *ptr;
		}
		//@}
	private:
		Transition doExecute() override
		{
			Derived& derived = static_cast<Derived&>(*this);
			return derived.execute();
		}

		void doEnter() override
		{
			m_enterTime = ros::Time::now();

			Derived& derived = static_cast<Derived&>(*this);
			derived.enter();
		}

		void doLeave() override
		{
			Derived& derived = static_cast<Derived&>(*this);
			derived.leave();
		}

		std::string collectDisplayMessages() override
		{
			std::string ret = fmt::to_string(m_displayBuffer);
			m_displayBuffer = fmt::memory_buffer{};
			return ret;
		}

		ros::Time m_enterTime;
		fmt::memory_buffer m_displayBuffer;
	};


	/**
	 * @brief Constructor
	 *
	 * @param driver Reference to an instance of your driver class
	 * @param nh ROS NodeHandle for the ROS API.
	 **/
	explicit FSM(DriverClass& driver, const ros::NodeHandle& nh)
	 : m_driver{driver}
	 , m_rosInterface{nh}
	{}

	/**
	 * @brief Constructor
	 *
	 * @param driver Reference to an instance of your driver class
	 **/
	explicit FSM(DriverClass& driver)
	 : m_driver{driver}
	{}

	/**
	 * @brief Initialize the type system
	 *
	 * This call is used to initialize the compile-time introspection of your
	 * states. It should be called with all possible entry states as template
	 * parameters.
	 *
	 * Example: @snippet demo.cpp initialize
	 *
	 * @note All reachable states need to be fully declared (i.e. not only
	 *   forward-declared) for this to work.
	 **/
	template<class ... StartStates>
	void initialize()
	{
		using StateList = ReachableStates<StartStates...>;

		// This is read out and checked by the clang plugin against the list
		// of all states
		using ReachableStates [[maybe_unused]] = typename detail::toCollector<StateList>::type;

		ROS_INFO("Finite State Machine with states:");
		brigand::for_each<StateList>([](auto state){
			using State = typename decltype(state)::type;
			std::stringstream ss;
			ss << fmt::format(" - {} trans [", State::Name.c_str());

			brigand::for_each<typename State::Transitions::Set>([&](auto successorState) {
				using SuccessorState = typename decltype(successorState)::type;
				ss << fmt::format(" {},", SuccessorState::Name.c_str());
			});
			ROS_INFO_STREAM(ss.str());
		});

		// Send out & latch Info message
		m_infoMsg.states.clear();

		brigand::for_each<StateList>([&](auto state){
			using State = typename decltype(state)::type;

			StateInfo stateInfo;
			stateInfo.name = State::Name.c_str();

			brigand::for_each<typename State::Transitions::Set>([&](auto suc){
				using Suc = typename decltype(suc)::type;

				stateInfo.successors.emplace_back(Suc::Name.c_str());
			});

			m_infoMsg.states.push_back(std::move(stateInfo));
		});

		m_rosInterface.publishInfo(m_infoMsg);

		// Setup factory functions
		m_factories.clear();
		brigand::for_each<StateList>([&](auto state){
			using State = typename decltype(state)::type;
			if constexpr (std::is_constructible_v<State>)
			{
				m_factories.emplace(State::Name.c_str(), [](){
					return StateWithName{
						detail::Pointer<State>{detail::InPlaceInit},
						std::string_view{State::Name}
					};
				});
			}
		});
	}

	/**
	 * @brief Set FSM state
	 *
	 * This method is used to enter a specific state, continuing execution from
	 * there. Any arguments are forwarded to the constructor of the state.
	 *
	 * Example: @snippet demo.cpp setState
	 **/
	template<class State, class ... Args>
	void setState(Args && ... args)
	{
		switchState(
			detail::Pointer<State>{detail::InPlaceInit, std::forward<Args>(args)...},
			State::Name.c_str()
		);
	}

	/**
	 * @brief Step the FSM
	 *
	 * This will call the @ref State::execute() method on the currently active
	 * state and handle the returned Transition object appropriately, either
	 * staying in the same or transitioning to another state.
	 *
	 * This method will also publish a ROS message on the `status` topic with
	 * the current FSM state and the state history.
	 **/
	void step()
	{
		if(!m_state)
			return;

		// Handle change state requests
		if(m_rosInterface.changeRequested())
		{
			auto stateName = m_rosInterface.requestedState();
			auto it = m_factories.find(stateName);
			if(it != m_factories.end())
			{
				ROSFMT_INFO("ROS interface: requested state '{}'", stateName);
				auto stateWithName = it->second();
				switchState(std::move(stateWithName.first), stateWithName.second);
				m_rosInterface.report(detail::ROSInterface::Result::Success);
			}
			else
			{
				ROSFMT_ERROR("ROS interface: requested transition to state '{}', "
					"which is not known to me. Maybe it cannot be found by "
					"FSM::initialize<>()?",
					stateName
				);
				m_rosInterface.report(detail::ROSInterface::Result::NoSuchState);
			}
		}

		m_rosInterface.updateCurrentState();

		Transition nextState = m_watchdog.call(m_stateLabel, "execute", [&](){
			return m_state->doExecute();
		});
		std::string messages = m_state->collectDisplayMessages();

		if(!messages.empty() && messages.back() == '\n')
			messages.pop_back();

		// Publish status msg
		sendROSStatus(messages);

		if(nextState)
		{
			if(!messages.empty())
			{
				ROSFMT_INFO("Last display messages of state {}:", m_stateLabel);
				ROSFMT_INFO("{}", messages);
			}

			auto label = nextState.label();
			switchState(std::move(nextState), label);
		}
	}

	/**
	 * @brief Retrieve class name of currently active state
	 *
	 * If the FSM is not running (i.e. no state has been set with
	 * @ref setState), this will return `std::string{}`.
	 **/
	[[nodiscard]] std::string currentStateName() const
	{
		if(!m_stateLabel.empty())
			return std::string{m_stateLabel};
		else
			return {};
	}

	/**
	 * @brief Information on states & transitions
	 *
	 * This returns a ROS message instance with the list of states and possible
	 * transitions.
	 *
	 * @note You will need to call @ref initialize() first, otherwise the result
	 *   will be empty.
	 **/
	[[nodiscard]] const Info& stateInfo() const
	{
		return m_infoMsg;
	}

private:
	void switchState(detail::Pointer<StateBase>&& state, const std::string_view& label)
	{
		if(m_state)
		{
			ROSFMT_INFO("Leaving state {}", m_stateLabel);
			m_watchdog.call(m_stateLabel, "leave", [&](){
				m_state->doLeave();
			});
		}

		m_state = std::move(state);
		m_stateLabel = label;

		m_state->setDriver(&m_driver);

		ROSFMT_INFO("Entering state {}", m_stateLabel);
		m_watchdog.call(m_stateLabel, "enter", [&](){
			m_state->doEnter();
		});

		m_rosInterface.pushStateHistory(std::string{m_stateLabel});
	}

	void sendROSStatus(const std::string& messages)
	{
		std::string driverInfo;

		if constexpr(detail::hasToString<DriverClass>(0))
			driverInfo = m_driver.toString();

		m_rosInterface.publishStatus(std::string{m_stateLabel}, driverInfo, messages);
	}

	DriverClass& m_driver;
	detail::Pointer<StateBase> m_state;
	std::string_view m_stateLabel = nullptr;

	using StateWithName = std::pair<detail::Pointer<StateBase>, std::string_view>;
	using StateFactory = std::function<StateWithName()>;
	std::map<std::string, StateFactory> m_factories;

	Info m_infoMsg;

	detail::Watchdog m_watchdog;

	detail::ROSInterface m_rosInterface;

};

}

#endif
