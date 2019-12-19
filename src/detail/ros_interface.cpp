// ROS interface
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <nimbro_fsm2/detail/ros_interface.h>

#include <actionlib/server/action_server.h>

#include <nimbro_fsm2/ChangeStateAction.h>

#include <nimbro_fsm2/Info.h>
#include <nimbro_fsm2/Status.h>

#include <boost/circular_buffer.hpp>

namespace nimbro_fsm2
{
namespace detail
{

namespace
{
	using ActionServer = actionlib::ActionServer<ChangeStateAction>;
}

class ROSInterface::Private
{
public:
	Private(const ros::NodeHandle& nhIn)
	 : nh{nhIn}
	 , server(nh, "change_state", false)
	{
		server.registerGoalCallback(boost::bind(&Private::handleGoal, this, _1));
		server.start();

		pub_info = nh.advertise<Info>("info", 1, true);
		pub_status = nh.advertise<Status>("status", 5);
	}

	void handleGoal(ActionServer::GoalHandle goal)
	{
		if(currentGoal.isValid())
		{
			goal.setRejected({}, "Already handling a change request, rejecting new ones.");
			return;
		}

		currentGoal = goal;
		currentGoal.setAccepted();
	}

	ros::NodeHandle nh;
	ActionServer server;
	ActionServer::GoalHandle currentGoal;

	ros::Publisher pub_info;
	ros::Publisher pub_status;

	boost::circular_buffer<StateStatus> history{100};
};

ROSInterface::ROSInterface()
 : m_d{std::make_unique<Private>(ros::NodeHandle("~"))}
{}

ROSInterface::ROSInterface(const ros::NodeHandle& nh)
 : m_d{std::make_unique<Private>(nh)}
{}

ROSInterface::~ROSInterface() = default;

bool ROSInterface::changeRequested() const
{
	return m_d->currentGoal.isValid();
}

std::string ROSInterface::requestedState() const
{
	if(!m_d->currentGoal.isValid())
		return {};

	auto goal = m_d->currentGoal.getGoal();
	if(!goal)
		return {};

	return goal->state;
}

void ROSInterface::report(Result result)
{
	if(!m_d->currentGoal.isValid())
	{
		ROS_WARN("ROSInterface::report() called without valid goal");
		return;
	}

	switch(result)
	{
		case Result::Success:
			m_d->currentGoal.setSucceeded({}, "State changed.");
			break;
		case Result::NoSuchState:
			m_d->currentGoal.setAborted({}, "Could not find state.");
			break;
		case Result::NotConstructible:
			m_d->currentGoal.setAborted({}, "Could not construct state because it requires constructor arguments.");
			break;
		case Result::TransitionDenied:
			m_d->currentGoal.setAborted({}, "Transition was denied by the FSM");
			break;
	}

	// All reports are final for the given goal.
	m_d->currentGoal = {};
}

void ROSInterface::updateCurrentState()
{
	m_d->history.back().end = ros::Time::now();
}

void ROSInterface::pushStateHistory(const std::string& state)
{
	StateStatus stateStatus;
	stateStatus.name = state;
	stateStatus.start = stateStatus.end = ros::Time::now();
	m_d->history.push_back(std::move(stateStatus));
}

void ROSInterface::publishStatus(const std::string_view& currentState, const std::string& driverInfo, const std::string& messages)
{
	Status status;
	status.current_state = currentState.empty() ? "<not running>" : currentState;
	status.driver_info = driverInfo;
	status.display_messages = messages;

	status.paused = false;
	status.history.reserve(m_d->history.size());
	std::copy(m_d->history.begin(), m_d->history.end(),
		std::back_inserter(status.history)
	);

	m_d->pub_status.publish(status);
}

void ROSInterface::publishInfo(const Info& msg)
{
	m_d->pub_info.publish(msg);
}

}
}
