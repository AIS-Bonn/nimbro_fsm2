// Warns on functions that block
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <nimbro_fsm2/detail/watchdog.h>

#include <functional>

#include <ros/rate.h>

#include <rosfmt/rosfmt.h>

namespace nimbro_fsm2
{
namespace detail
{

Watchdog::Watchdog()
 : m_thread{std::bind(&Watchdog::thread, this)}
{
}

Watchdog::~Watchdog()
{
	{
		std::unique_lock<std::mutex> lock(m_mutex);
		m_shouldExit = true;
	}

	m_thread.join();
}

Watchdog::Scope::Scope(Watchdog& wd, const std::string& className, const std::string& methodName)
 : m_wd(wd)
{
	m_wd.enter(className, methodName);
}

Watchdog::Scope::~Scope()
{
	m_wd.leave();
}

void Watchdog::enter(const std::string& className, const std::string& methodName)
{
	std::unique_lock<std::mutex> lock(m_mutex);
	m_startTime = ros::Time::now();
	m_className = className;
	m_methodName = methodName;
}

void Watchdog::leave()
{
	std::unique_lock<std::mutex> lock(m_mutex);
	m_startTime = ros::Time(0);
}

void Watchdog::check()
{
	std::unique_lock<std::mutex> lock(m_mutex);

	if(m_startTime == ros::Time(0))
		return;

	// can't make these constexpr :-(
	const ros::Duration WARN_DURATION(1.0);
	const ros::Duration ERROR_DURATION(4.0);

	ros::Time now = ros::Time::now();

	if(now - m_startTime > ERROR_DURATION)
	{
		ROSFMT_ERROR_THROTTLE(1.0,
			"Watchdog: {}::{}() is taking longer than {}s. This is probably a bug.",
			m_className, m_methodName, ERROR_DURATION.toSec()
		);
		return;
	}

	if(now - m_startTime > WARN_DURATION)
	{
		ROSFMT_WARN_THROTTLE(1.0,
			"Watchdog: {}::{}() is taking longer than {}s. This is probably a bug.",
			m_className, m_methodName, WARN_DURATION.toSec()
		);
		return;
	}
}

void Watchdog::thread()
{
	ros::WallRate rate(ros::Duration(0.5));
	while(!m_shouldExit)
	{
		check();
		rate.sleep();
	}
}

}
}
