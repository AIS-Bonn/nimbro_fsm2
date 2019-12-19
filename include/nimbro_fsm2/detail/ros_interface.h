// ROS Action interface
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef NIMBRO_FSM2_DETAIL_ACTION_INTERFACE_H
#define NIMBRO_FSM2_DETAIL_ACTION_INTERFACE_H

#include <memory>
#include <vector>

#include <nimbro_fsm2/Info.h>

namespace ros { class NodeHandle; }

namespace nimbro_fsm2
{
namespace detail
{

// NOTE: Implemented using pimpl to keep actionlib dependency hidden
class ROSInterface
{
public:
	enum class Result
	{
		Success,

		NoSuchState,
		NotConstructible,
		TransitionDenied,
	};

	ROSInterface();
	explicit ROSInterface(const ros::NodeHandle& nh);
	~ROSInterface();

	bool changeRequested() const;
	std::string requestedState() const;

	void report(Result result);

	void pushStateHistory(const std::string& state);
	void updateCurrentState();
	void publishStatus(const std::string_view& state, const std::string& driverInfo, const std::string& messages);
	void publishInfo(const Info& msg);
private:
	class Private;

	std::unique_ptr<Private> m_d;
};

}
}

#endif
