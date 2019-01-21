// rqt plugin for debugging/controlling a nimbro_fsm2 instance
// Author: Max Schwarz <max.schwarz@uni-bonn.de>
// Author: Christian Lenz <lenz@ais.uni-bonn.de>

#ifndef NIMBRO_FSM_STATEMACHINE_GUI_H
#define NIMBRO_FSM_STATEMACHINE_GUI_H

#include <rqt_gui_cpp/plugin.h>

#include <ros/subscriber.h>
#include <ros/service_client.h>

#include <QtCore/QObject>
#include <QtCore/QMutex>

#include <nimbro_fsm2/Status.h>
#include <nimbro_fsm2/Info.h>

class Ui_StateMachineGUI;

namespace nimbro_fsm2
{

class StateMachineGUI : public rqt_gui_cpp::Plugin
{
Q_OBJECT
public:
	StateMachineGUI();
	virtual ~StateMachineGUI();

	virtual void initPlugin(qt_gui_cpp::PluginContext& ctx);

	virtual void shutdownPlugin();
	virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
	virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

Q_SIGNALS:
	void statusReceived(const nimbro_fsm2::StatusConstPtr& msg);

private Q_SLOTS:
	void refreshTopicList();
	void subscribe();
	void changeState();
	void changeStateTo(std::string state);

	void processStatus(const nimbro_fsm2::StatusConstPtr& msg);
private:
	void handleInfo(const nimbro_fsm2::InfoConstPtr& msg);

	QWidget* m_w;
	Ui_StateMachineGUI* m_ui;

	QMutex m_stateListMutex;
	nimbro_fsm2::InfoConstPtr m_stateList;

	ros::Subscriber m_sub_status;
	ros::Subscriber m_sub_info;
	ros::ServiceClient m_srv_changeState;

	std::string m_prefix;

	bool m_changeState = false;

	bool m_shuttingDown;
};

}

#endif
