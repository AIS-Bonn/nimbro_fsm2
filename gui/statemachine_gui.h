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

#include <actionlib/client/simple_action_client.h>
#include <nimbro_fsm2/ChangeStateAction.h>

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
	void infoReceived(const nimbro_fsm2::InfoConstPtr& msg);
	void actionClientFinished();

private Q_SLOTS:
	void refreshTopicList();
	void subscribe();
	void changeState();
	void changeStateTo(std::string state);
	void timerCB();
	void checkActionClient();

	void processStatus(const nimbro_fsm2::StatusConstPtr& msg);
	void handleInfo(const nimbro_fsm2::InfoConstPtr& msg);
private:

	QWidget* m_w;
	Ui_StateMachineGUI* m_ui;

	QTimer* m_timer;

	nimbro_fsm2::InfoConstPtr m_stateList;

	ros::Subscriber m_sub_status;
	ros::Subscriber m_sub_info;

	typedef actionlib::SimpleActionClient<nimbro_fsm2::ChangeStateAction> ActionClient;
	boost::shared_ptr<ActionClient> m_ac;
	bool m_actionActive = false;

	std::string m_prefix;

	bool m_changeState = false;
	ros::Time m_lastUpdate = ros::Time(0);

	bool m_shuttingDown;
};

}

#endif
