// ActionClient interface suitable for FSMs
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef NIMBRO_FSM2_HELPERS_ACTION_CLIENT_H
#define NIMBRO_FSM2_HELPERS_ACTION_CLIENT_H

#include <actionlib/client/simple_action_client.h>

#include <optional>

namespace nimbro_fsm2
{
namespace helpers
{

enum class ActionState
{
	Connecting,
	Idle,
	InProcess,
	Succeeded,
	Failed,
};

template<class ActionSpec>
class ActionClient
{
private:
	ACTION_DEFINITION(ActionSpec);
	using Client = actionlib::SimpleActionClient<ActionSpec>;

public:
	using ActionState = nimbro_fsm2::helpers::ActionState;

	explicit ActionClient(const std::string& topic);
	explicit ActionClient(ros::NodeHandle& nh, const std::string& topic);

	~ActionClient();

	void setGoal(const Goal& goal);
	void resend();

	[[nodiscard]] ActionState step();

	[[nodiscard]] actionlib::SimpleClientGoalState state()
	{ return m_ac.getState(); }

	[[nodiscard]] ResultConstPtr result()
	{ return m_ac.getResult(); }

	[[nodiscard]] FeedbackConstPtr feedback()
	{ return m_feedback; }
private:
	void handleFeedback(const FeedbackConstPtr& feedback)
	{ m_feedback = feedback; }

	std::string m_topic;
	Client m_ac;
	ActionState m_state{ActionState::Connecting};
	std::optional<Goal> m_goal;

	ros::Time m_startTime{ros::Time::now()};

	ros::Duration m_connectTimeout{5.0};

	FeedbackConstPtr m_feedback;
};

// IMPLEMENTATION
template<typename Action>
ActionClient<Action>::ActionClient(const std::string& topic)
 : m_topic{topic}
 , m_ac{topic, false}
{
}

template<typename Action>
ActionClient<Action>::ActionClient(ros::NodeHandle& nh, const std::string& topic)
 : m_topic{topic}
 , m_ac{nh, topic, false}
{
}

template<typename Action>
ActionClient<Action>::~ActionClient()
{
	if(m_state == ActionState::InProcess)
	{
		ROSFMT_WARN("Canceling active goal on action topic {}", m_topic);
		m_ac.cancelGoal();
	}
}

template<typename Action>
void ActionClient<Action>::setGoal(const ActionClient::Goal& goal)
{
	if(m_goal)
		throw std::logic_error("You are calling ActionClient::setGoal() twice");

	m_goal = goal;
}

template<typename Action>
void ActionClient<Action>::resend()
{
	if(m_state != ActionState::Failed && m_state != ActionState::Succeeded)
	{
		throw std::logic_error("You called resend() without being in a terminal state (Succeeded/Failed)");
	}

	m_state = ActionState::Idle;
}

template<typename Action>
ActionState ActionClient<Action>::step()
{
	switch(m_state)
	{
		case ActionState::Connecting:
			if(m_ac.isServerConnected())
			{
				m_state = ActionState::Idle;
			}

			if(ros::Time::now() - m_startTime > m_connectTimeout)
			{
				ROSFMT_ERROR("Could not connect to action server on topic {}. "
					"Reporting action failure.",
					m_topic
				);
				m_state = ActionState::Failed;
			}
			break;
		case ActionState::Idle:
			if(!m_ac.isServerConnected())
			{
				m_state = ActionState::Connecting;
			}
			else
			{
				if(m_goal)
				{
					m_ac.sendGoal(*m_goal,
						typename Client::SimpleDoneCallback{},
						typename Client::SimpleActiveCallback{},
						std::bind(&ActionClient<Action>::handleFeedback, this, std::placeholders::_1)
					);
					m_state = ActionState::InProcess;
				}
			}
			break;

		case ActionState::InProcess:
		{
			if(!m_ac.isServerConnected())
			{
				ROSFMT_ERROR("Lost connection to action server {}", m_topic);
				m_ac.cancelGoal();
				m_state = ActionState::Failed;
				break;
			}

			auto acState = m_ac.getState();
			if(acState.isDone())
			{
				if(acState == actionlib::SimpleClientGoalState::SUCCEEDED)
					m_state = ActionState::Succeeded;
				else
				{
					ROSFMT_WARN("Action {} failed with terminal state {}: {}",
						m_topic, acState.toString(), acState.getText()
					);
					m_state = ActionState::Failed;
				}
			}
			break;
		}
		case ActionState::Succeeded:
		case ActionState::Failed:
			break;
	}

	return m_state;
}

}
}

inline std::ostream& operator<<(std::ostream& stream, nimbro_fsm2::helpers::ActionState state)
{
	using namespace nimbro_fsm2::helpers;

	switch(state)
	{
		case ActionState::Connecting: stream << "Connecting"; return stream;
		case ActionState::Idle:       stream << "Idle";       return stream;
		case ActionState::InProcess:  stream << "InProcess";  return stream;
		case ActionState::Succeeded:  stream << "Succeeded";  return stream;
		case ActionState::Failed:     stream << "Failed";     return stream;
	}

	stream << "INVALID";
	return stream;
}

#endif
