// Finite state machine
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef NIMBRO_FSM2_FSM_H
#define NIMBRO_FSM2_FSM_H

#include <variant>
#include <memory>
#include <iostream>
#include <typeinfo>

#include "ctti/nameof.hpp"

namespace nimbro_fsm2
{

namespace detail
{
template <typename T, typename Variant>
struct has_type;

template <typename T, typename... Us>
struct has_type<T, std::variant<Us...>> : std::disjunction<std::is_same<T, Us>...> {};

struct compile_time_string
{
	constexpr compile_time_string(const char* start, const char* end)
	 : m_data{}
	{
		int i = 0;
		for(; i < sizeof(m_data)-1 && start != end; ++i)
			m_data[i] = *(start++);
		m_data[i] = 0;
	}

	char m_data[256];
};

inline std::ostream& operator<<(std::ostream& o, const compile_time_string& s)
{
	o << s.m_data;
	return o;
}

constexpr const char* UNKNOWN_NAME = "unknown";

template<class T>
constexpr compile_time_string get_name()
{
	// ... get_name<T>() [with T = MyType]
	const char* p = __PRETTY_FUNCTION__;

	// Skip to "= "
	while(*p && *p++ != '=');
	while(*p == ' ')
		*p++;

	// Search for end
	if(*p)
	{
		const char* p2 = p;
		int count = 1;
		for (;;++p2)
		{
			switch (*p2)
			{
			case '[':
				++count;
				break;
			case ']':
				--count;
				if (!count)
					return compile_time_string{p, p2};
			case 0:
				return {UNKNOWN_NAME, UNKNOWN_NAME+7};
			}
		}
	}

	return {UNKNOWN_NAME, UNKNOWN_NAME+7};
}

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

		static constexpr detail::compile_time_string Name = detail::get_name<Derived>();

		std::unique_ptr<StateBase> doExecute(DriverClass& driver) override
		{
			Derived& derived = static_cast<Derived&>(*this);
			return derived.execute(driver);
		}

		void doEnter(DriverClass& driver) override
		{
			std::cout << "Entering " << Name << "\n";
// 			std::cout << "Entering " << typeid(*this).name() << "\n";
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
