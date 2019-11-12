// Warns on functions that block
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef NIMBRO_FSM2_DETAIL_WATCHDOG_H
#define NIMBRO_FSM2_DETAIL_WATCHDOG_H

#include <thread>
#include <mutex>

#include <ros/time.h>

namespace nimbro_fsm2
{
namespace detail
{

class Watchdog
{
public:
	Watchdog();
	~Watchdog();

	template<class F>
	auto call(const std::string_view& className, const std::string& methodName, F f)
	{
		Scope scope(*this, className, methodName);

		return f();
	}
private:
	class Scope
	{
	public:
		Scope(Watchdog& wd, const std::string_view& className, const std::string& method);
		~Scope();
	private:
		Watchdog& m_wd;
	};

	void enter(const std::string_view& className, const std::string& method);
	void leave();

	void thread();
	void check();

	std::thread m_thread;

	std::mutex m_mutex;
	ros::Time m_startTime = ros::Time(0);
	std::string_view m_className;
	std::string m_methodName;

	bool m_shouldExit = false;
};

}
}

#endif
