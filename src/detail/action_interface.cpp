// ROS Action interface
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <nimbro_fsm2/detail/action_interface.h>

#include <actionlib/server/action_server.h>

#include <nimbro_fsm2/ChangeStateAction.h>

namespace nimbro_fsm2
{
namespace detail
{

namespace
{
	using ActionServer = actionlib::ActionServer<ChangeStateAction>;
}

class ActionInterface::Private
{
public:
	Private(const ros::NodeHandle& nh)
	 : server(nh, "change_state", false)
	{
		server.registerGoalCallback(boost::bind(&Private::handleGoal, this, _1));
		server.start();
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

	ActionServer server;
	ActionServer::GoalHandle currentGoal;
};

ActionInterface::ActionInterface(const ros::NodeHandle& nh)
 : m_d{std::make_unique<Private>(nh)}
{
}

ActionInterface::~ActionInterface() = default;

bool ActionInterface::changeRequested() const
{
	return m_d->currentGoal.isValid();
}

std::string ActionInterface::requestedState() const
{
	if(!m_d->currentGoal.isValid())
		return {};

	auto goal = m_d->currentGoal.getGoal();
	if(!goal)
		return {};

	return goal->state;
}

void ActionInterface::report(Result result)
{
	if(!m_d->currentGoal.isValid())
	{
		ROS_WARN("ActionInterface::report() called without valid goal");
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
	}

	// All reports are final for the given goal.
	m_d->currentGoal = {};
}

}
}
