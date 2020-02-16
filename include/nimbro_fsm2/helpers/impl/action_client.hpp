// ActionClient interface suitable for FSMs
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef NIMBRO_FSM2_HELPERS_ACTIONCLIENT_IMPL_H
#define NIMBRO_FSM2_HELPERS_ACTIONCLIENT_IMPL_H

#include "../action_client.h"

#include <rosfmt/rosfmt.h>

#include <actionlib/client/simple_action_client.h>

namespace nimbro_fsm2
{
namespace helpers
{

template<typename ActionSpec>
class ActionClient<ActionSpec>::Private
{
public:
	using Public = ActionClient<ActionSpec>;
	using Client = actionlib::SimpleActionClient<ActionSpec>;

	explicit Private(const std::string& topic)
	 : ac{topic, false}
	{
		topicVis = fmt::format("{} (resolved to {})", topic, ros::names::resolve(topic));
	}

	explicit Private(ros::NodeHandle& nh, const std::string& topic)
	 : ac{nh, topic, false}
	{
		topicVis = fmt::format("{} (resolved to {})", topic, nh.resolveName(topic));
	}

	std::string topicVis;
	Client ac;

	ActionState state{ActionState::Connecting};
	std::optional<Goal> goal;

	ros::Time startTime{0};

	ros::Duration connectTimeout{5.0};

	Public::FeedbackConstPtr feedback;
};

// IMPLEMENTATION
template<typename Action>
ActionClient<Action>::ActionClient(const std::string& topic)
 : m_d{std::make_unique<Private>(topic)}
{}

template<typename Action>
ActionClient<Action>::ActionClient(ros::NodeHandle& nh, const std::string& topic)
 : m_d{std::make_unique<Private>(nh, topic)}
{}

template<typename Action>
ActionClient<Action>::~ActionClient()
{
	if(m_d->state == ActionState::InProcess)
	{
		ROSFMT_WARN("Canceling active goal on action topic {}", m_d->topicVis);
		m_d->ac.cancelGoal();
	}
}

template<typename Action>
void ActionClient<Action>::setGoal(const ActionClient::Goal& goal)
{
	if(m_d->goal)
		throw std::logic_error("You are calling ActionClient::setGoal() twice");

	m_d->goal = goal;
}

template<typename Action>
void ActionClient<Action>::setNewGoal(const ActionClient::Goal& goal)
{
	m_d->goal = goal;
	m_d->state = ActionState::Succeeded; // set so new goal can be resend
	resend();
}

template<typename Action>
void ActionClient<Action>::resend()
{
	if(m_d->state != ActionState::Failed && m_d->state != ActionState::Succeeded)
	{
		throw std::logic_error("You called resend() without being in a terminal state (Succeeded/Failed)");
	}

	m_d->state = ActionState::Idle;
}

template<typename Action>
ActionState ActionClient<Action>::step()
{
	switch(m_d->state)
	{
		case ActionState::Connecting:
			if(m_d->startTime == ros::Time(0))
				m_d->startTime = ros::Time::now();

			if(m_d->ac.isServerConnected())
			{
				m_d->state = ActionState::Idle;
			}
			else if(ros::Time::now() - m_d->startTime > m_d->connectTimeout)
			{
				ROSFMT_ERROR("Could not connect to action server on topic {}. "
					"Reporting action failure.",
					m_d->topicVis
				);
				m_d->state = ActionState::Failed;
			}
			break;
		case ActionState::Idle:
			if(!m_d->ac.isServerConnected())
			{
				m_d->state = ActionState::Connecting;
			}
			else
			{
				if(m_d->goal)
				{
					m_d->ac.sendGoal(*m_d->goal,
						typename Private::Client::SimpleDoneCallback{},
						typename Private::Client::SimpleActiveCallback{},
						std::bind(&ActionClient<Action>::handleFeedback, this, std::placeholders::_1)
					);
					m_d->state = ActionState::InProcess;
				}
			}
			break;

		case ActionState::InProcess:
		{
			if(!m_d->ac.isServerConnected())
			{
				ROSFMT_ERROR("Lost connection to action server {}", m_d->topicVis);
				m_d->ac.cancelGoal();
				m_d->state = ActionState::Failed;
				break;
			}

			auto acState = m_d->ac.getState();
			if(acState.isDone())
			{
				if(acState == actionlib::SimpleClientGoalState::SUCCEEDED)
					m_d->state = ActionState::Succeeded;
				else
				{
					ROSFMT_WARN("Action {} failed with terminal state {}: {}",
						m_d->topicVis, acState.toString(), acState.getText()
					);
					m_d->state = ActionState::Failed;
				}
			}
			break;
		}
		case ActionState::Succeeded:
		case ActionState::Failed:
			break;
	}

	return m_d->state;
}

template<class ActionSpec>
actionlib::SimpleClientGoalState ActionClient<ActionSpec>::state()
{
	return m_d->ac.getState();
}

template<class ActionSpec>
typename ActionClient<ActionSpec>::ResultConstPtr ActionClient<ActionSpec>::result()
{
	return m_d->ac.getResult();
}

template<class ActionSpec>
typename ActionClient<ActionSpec>::FeedbackConstPtr ActionClient<ActionSpec>::feedback()
{
	return m_d->feedback;
}

template<class ActionSpec>
void ActionClient<ActionSpec>::handleFeedback(const FeedbackConstPtr& feedback)
{
	m_d->feedback = feedback;
}

}
}

#endif
