// ROS Action interface
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef NIMBRO_FSM2_DETAIL_ACTION_INTERFACE_H
#define NIMBRO_FSM2_DETAIL_ACTION_INTERFACE_H

#include <memory>

namespace ros { class NodeHandle; }

namespace nimbro_fsm2
{
namespace detail
{

// NOTE: Implemented using pimpl to keep actionlib dependency hidden
class ActionInterface
{
public:
	enum class Result
	{
		Success,

		NoSuchState,
		NotConstructible,
	};

	explicit ActionInterface(const ros::NodeHandle& nh);
	~ActionInterface();

	bool changeRequested() const;
	std::string requestedState() const;

	void report(Result result);
private:
	class Private;

	std::unique_ptr<Private> m_d;
};

}
}

#endif
