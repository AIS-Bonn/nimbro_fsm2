// rqt plugin for debugging/controlling a nimbro_fsm2 instance
// Author: Max Schwarz <max.schwarz@uni-bonn.de>
// Author: Christian Lenz <lenz@ais.uni-bonn.de>

#include "statemachine_gui.h"

#include <pluginlib/class_list_macros.h>

#include <ros/master.h>
#include <ros/node_handle.h>

#include <boost/foreach.hpp>
#include <QMessageBox>
#include <QTimer>

#include "ui_statemachine_gui.h"

Q_DECLARE_METATYPE(nimbro_fsm2::StatusConstPtr);

namespace nimbro_fsm2
{

StateMachineGUI::StateMachineGUI()
 : m_ui(0)
 , m_shuttingDown(false)
{
	qRegisterMetaType<nimbro_fsm2::StatusConstPtr>();
	QObject::connect(
		this, SIGNAL(statusReceived(nimbro_fsm2::StatusConstPtr)),
		SLOT(processStatus(nimbro_fsm2::StatusConstPtr)),
		Qt::QueuedConnection
	);
}

StateMachineGUI::~StateMachineGUI()
{
	if(m_ui)
		delete m_ui;
}

void StateMachineGUI::initPlugin(qt_gui_cpp::PluginContext& ctx)
{
	qt_gui_cpp::Plugin::initPlugin(ctx);

	m_w = new QWidget;
	m_ui = new Ui_StateMachineGUI;
	m_ui->setupUi(m_w);

	ctx.addWidget(m_w);

	QObject::connect(m_ui->prefixComboBox, SIGNAL(activated(QString)), SLOT(subscribe()));
	QObject::connect(m_ui->refreshButton, SIGNAL(clicked(bool)), SLOT(refreshTopicList()));
	QObject::connect(m_ui->btn_changeState, SIGNAL(clicked(bool)), SLOT(changeState()));

	QObject::connect(m_ui->nodeGraph, SIGNAL(changeStateActive()), SLOT(changeState()));

// 	QTimer* timer = new QTimer(this);
// 	QObject::connect(timer, SIGNAL(timeout()), SLOT(refreshTopicList()));
// 	timer->start(1000);
}

void StateMachineGUI::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
	qt_gui_cpp::Plugin::saveSettings(plugin_settings, instance_settings);

	instance_settings.setValue("prefix", QString::fromStdString(m_prefix));
}

void StateMachineGUI::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
	qt_gui_cpp::Plugin::restoreSettings(plugin_settings, instance_settings);

	m_prefix = instance_settings.value("prefix").toString().toStdString();
}

void StateMachineGUI::refreshTopicList()
{
	if(m_shuttingDown)
		return;

	ros::master::V_TopicInfo topics;
	ros::master::getTopics(topics);

	m_ui->prefixComboBox->clear();

	int idx = 0;
	BOOST_FOREACH(const ros::master::TopicInfo topic, topics)
	{
		if(topic.datatype != "nimbro_fsm2/Info")
			continue;

		int pos = topic.name.rfind("/info");
		if(pos < 0)
			continue;

		std::string prefix = topic.name.substr(0, pos);

		m_ui->prefixComboBox->addItem(QString::fromStdString(prefix));
		if(prefix == m_prefix)
		{
			m_ui->prefixComboBox->setCurrentIndex(idx);
			subscribe();
		}
		++idx;
	}
}

void StateMachineGUI::subscribe()
{
	ros::NodeHandle nh = getPrivateNodeHandle();
	std::string prefix = m_ui->prefixComboBox->currentText().toStdString();
	m_prefix = prefix;

	m_sub_status = nh.subscribe(prefix + "/status", 1, &StateMachineGUI::statusReceived, this);
	m_sub_info = nh.subscribe(prefix + "/info", 1, &StateMachineGUI::handleInfo, this);
// 	m_srv_changeState = nh.serviceClient<nimbro_fsm::ChangeState>(prefix + "/change_state");

	m_w->setWindowTitle(QString::fromStdString(m_prefix));
}

void StateMachineGUI::processStatus(const StatusConstPtr& msg)
{
	m_ui->stateLabel->setText(QString::fromStdString(msg->current_state));

	m_ui->timeline->updateTimeline(msg);
	m_ui->nodeGraph->updateStatus(msg);

// 	m_ui->debugTextEdit->setText(QString::fromStdString(msg->debug_information));

// 	QStringList history;
// 	BOOST_FOREACH(const std::string& state, msg->state_history)
// 		history << QString::fromStdString(state);

// 	m_ui->historyLabel->setText(history.join("\n"));
}

void StateMachineGUI::shutdownPlugin()
{
	rqt_gui_cpp::Plugin::shutdownPlugin();
	m_sub_status.shutdown();
	m_sub_info.shutdown();
	m_shuttingDown = true;
}

void StateMachineGUI::handleInfo(const InfoConstPtr& msg)
{
	QMutexLocker locker(&m_stateListMutex);
	m_stateList = msg;
	m_ui->timeline->updateStateList(msg);
	m_ui->nodeGraph->updateInfo(msg);
}


void StateMachineGUI::changeState()
{
	m_changeState = !m_changeState;

	if(m_changeState)
		m_ui->btn_changeState->setStyleSheet("background-color : orange");
	else
		m_ui->btn_changeState->setStyleSheet("background-color : grey");

	m_ui->nodeGraph->setChangeStateActive(m_changeState);

// 	nimbro_fsm::ChangeStateRequest req;
// 	nimbro_fsm::ChangeStateResponse resp;
//
// 	req.state = state.toStdString();

// 	if(!m_srv_changeState.call(req, resp))
// 		QMessageBox::critical(m_w, "Error", "Could not switch state");
}

}

PLUGINLIB_EXPORT_CLASS(nimbro_fsm2::StateMachineGUI, rqt_gui_cpp::Plugin)
