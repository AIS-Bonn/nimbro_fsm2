// Finite state machine
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef NIMBRO_FSM2_FSM_H
#define NIMBRO_FSM2_FSM_H

#include <memory>
#include <variant>

#include <boost/hana/tuple.hpp>
#include <boost/hana/set.hpp>
#include <boost/hana/for_each.hpp>

#include <boost/circular_buffer.hpp>

#include <ros/time.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <nimbro_fsm2/Info.h>
#include <nimbro_fsm2/Status.h>

#include <rosfmt/rosfmt.h>
#include <fmt/format.h>

#include "detail/action_interface.h"
#include "detail/has_to_string.h"
#include "detail/type_name.h"
#include "detail/is_defined.h"
#include "detail/format.h"
#include "detail/watchdog.h"
#include "introspection.h"

namespace nimbro_fsm2
{

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
	static constexpr auto Namespace = detail::namespace_of(detail::type_name<DriverClass>());

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
		virtual std::string collectDisplayMessages() = 0;
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
		 * This is a boost::hana set of all the possible successor states.
		 **/
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

		//! @name Information
		//@{
		/**
		 * @brief State name
		 *
		 * This state name is extracted at compile time and stored as a
		 * @p boost::hana::string. You can use `.c_str()` to get a runtime
		 * string:
		 *
		 * @snippet demo.cpp StateName
		 **/
		static constexpr auto Name = detail::relative_name<Derived>(Namespace);

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
			namespace hana = boost::hana;

			if constexpr (detail::is_defined_v<T>)
			{
				if constexpr (hana::contains(TransitionSpec::SuccessorStateSet, hana::type_c<T>))
				{
					return Transition::_unchecked(std::make_unique<T>(std::forward<Args>(args)...), T::Name.c_str());
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
		//@}

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
		Transition doExecute(DriverClass& driver) override
		{
			Derived& derived = static_cast<Derived&>(*this);
			return derived.execute(driver);
		}

		void doEnter(DriverClass& driver) override
		{
			m_enterTime = ros::Time::now();

			Derived& derived = static_cast<Derived&>(*this);
			derived.enter(driver);
		}

		void doLeave(DriverClass& driver) override
		{
			Derived& derived = static_cast<Derived&>(*this);
			derived.leave(driver);
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
	 * @param nh ROS NodeHandle for the ROS API. The default uses your node's
	 *           private scope.
	 **/
	explicit FSM(DriverClass& driver, const ros::NodeHandle& nh = {"~"})
	 : m_driver{driver}
	 , m_nh{nh}
	 , m_actionInterface{nh}
	{
		m_pub_info = m_nh.advertise<Info>("info", 1, true);
		m_pub_status = m_nh.advertise<Status>("status", 5);
	}

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
		namespace hana = boost::hana;

		auto stateList = reachableStates<StartStates...>();

		ROS_INFO("Finite State Machine with states:");
		boost::hana::for_each(stateList, [](auto state){
			using State = typename decltype(state)::type;
			std::stringstream ss;
			ss << fmt::format(" - {} trans [", State::Name.c_str());

			auto successorStates = State::Transitions::SuccessorStateSet;
			boost::hana::for_each(successorStates, [&](auto successorState) {
				using SuccessorState = typename decltype(successorState)::type;
				ss << fmt::format(" {},", SuccessorState::Name.c_str());
			});
			ROS_INFO_STREAM(ss.str());
		});

		// Send out & latch Info message
		m_infoMsg.states.clear();

		hana::for_each(stateList, [&](auto state){
			using State = typename decltype(state)::type;

			StateInfo stateInfo;
			stateInfo.name = State::Name.c_str();

			hana::for_each(State::Transitions::SuccessorStateSet, [&](auto suc){
				using Suc = typename decltype(suc)::type;

				stateInfo.successors.emplace_back(Suc::Name.c_str());
			});

			m_infoMsg.states.push_back(std::move(stateInfo));
		});

		m_pub_info.publish(m_infoMsg);

		// Setup factory functions
		m_factories.clear();
		boost::hana::for_each(stateList, [this](auto state){
			using State = typename decltype(state)::type;
			if constexpr (std::is_constructible_v<State>)
			{
				m_factories.emplace(State::Name.c_str(), [](){
					return StateWithName{
						std::make_unique<State>(),
						State::Name.c_str()
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
			std::make_unique<State>(std::forward<Args>(args)...),
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
		if(m_actionInterface.changeRequested())
		{
			auto stateName = m_actionInterface.requestedState();
			auto it = m_factories.find(stateName);
			if(it != m_factories.end())
			{
				ROSFMT_INFO("ROS interface: requested state '{}'", stateName);
				auto stateWithName = it->second();
				switchState(std::move(stateWithName.first), stateWithName.second);
				m_actionInterface.report(detail::ActionInterface::Result::Success);
			}
			else
			{
				ROSFMT_ERROR("ROS interface: requested transition to state '{}', "
					"which is not known to me. Maybe it cannot be found by "
					"FSM::initialize<>()?",
					stateName
				);
				m_actionInterface.report(detail::ActionInterface::Result::NoSuchState);
			}
		}

		m_history.back().end = ros::Time::now();

		Transition nextState = m_watchdog.call(m_stateLabel, "execute", [&](){
			return m_state->doExecute(m_driver);
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

			const char* label = nextState.label();
			switchState(std::move(nextState), label);
		}
	}

	/**
	 * @brief Retrieve class name of currently active state
	 *
	 * If the FSM is not running (i.e. no state has been set with
	 * @ref setState), this will return `std::string{}`.
	 **/
	[[nodiscard]] constexpr std::string currentStateName() const
	{
		if(m_stateLabel)
			return m_stateLabel;
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
	[[nodiscard]] constexpr const Info& stateInfo() const
	{
		return m_infoMsg;
	}

private:
	void switchState(std::unique_ptr<StateBase>&& state, const char* label)
	{
		if(m_state)
		{
			ROSFMT_INFO("Leaving state {}", m_stateLabel);
			m_watchdog.call(m_stateLabel, "leave", [&](){
				m_state->doLeave(m_driver);
			});
		}

		m_state = std::move(state);
		m_stateLabel = label;

		ROSFMT_INFO("Entering state {}", m_stateLabel);
		m_watchdog.call(m_stateLabel, "enter", [&](){
			m_state->doEnter(m_driver);
		});

		StateStatus stateStatus;
		stateStatus.name = m_stateLabel;
		stateStatus.start = stateStatus.end = ros::Time::now();
		m_history.push_back(std::move(stateStatus));
	}

	void sendROSStatus(const std::string& messages)
	{
		Status status;
		status.current_state = m_stateLabel ? m_stateLabel : "<not running>";
		status.display_messages = messages;

		if constexpr(detail::hasToString<DriverClass>(0))
		{
			status.driver_info = m_driver.toString();
		}

		status.paused = false;
		status.history.reserve(m_history.size());
		std::copy(m_history.begin(), m_history.end(),
			std::back_inserter(status.history)
		);

		m_pub_status.publish(status);
	}

	DriverClass& m_driver;
	std::unique_ptr<StateBase> m_state;
	const char* m_stateLabel = nullptr;

	ros::NodeHandle m_nh;
	ros::Publisher m_pub_info;
	ros::Publisher m_pub_status;

	Info m_infoMsg;
	boost::circular_buffer<StateStatus> m_history{100};

	using StateWithName = std::pair<std::unique_ptr<StateBase>, const char*>;
	using StateFactory = std::function<StateWithName()>;
	std::map<std::string, StateFactory> m_factories;

	detail::Watchdog m_watchdog;

	detail::ActionInterface m_actionInterface;
};

}

#endif
