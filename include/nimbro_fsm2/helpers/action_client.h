// ActionClient interface suitable for FSMs
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef NIMBRO_FSM2_HELPERS_ACTION_CLIENT_H
#define NIMBRO_FSM2_HELPERS_ACTION_CLIENT_H

#include <actionlib/action_definition.h>
#include <actionlib/client/simple_client_goal_state.h>

#include <memory>
#include <optional>

namespace ros { class NodeHandle; }

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

public:
	using ActionState = nimbro_fsm2::helpers::ActionState;

	explicit ActionClient(const std::string& topic);
	explicit ActionClient(ros::NodeHandle& nh, const std::string& topic);

	~ActionClient();

	void setGoal(const Goal& goal);
	void resend();

	[[nodiscard]] ActionState step();

	[[nodiscard]] actionlib::SimpleClientGoalState state();

	[[nodiscard]] ResultConstPtr result();

	[[nodiscard]] FeedbackConstPtr feedback();
private:
	void handleFeedback(const FeedbackConstPtr& feedback);

	class Private;
	std::unique_ptr<Private> m_d;
};

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
