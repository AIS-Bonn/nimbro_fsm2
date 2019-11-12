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
Q_DECLARE_METATYPE(nimbro_fsm2::InfoConstPtr);

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
	qRegisterMetaType<nimbro_fsm2::InfoConstPtr>();
	QObject::connect(
		this, SIGNAL(infoReceived(nimbro_fsm2::InfoConstPtr)),
		SLOT(handleInfo(nimbro_fsm2::InfoConstPtr)),
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
	ros::NodeHandle nh("~");

	ctx.addWidget(m_w);

	m_timer = new QTimer(this);
	QObject::connect(m_timer , SIGNAL(timeout()), SLOT(timerCB()));
	m_timer ->start(100);

	QObject::connect(m_ui->prefixComboBox, SIGNAL(activated(QString)), SLOT(subscribe()));
	QObject::connect(m_ui->refreshButton, SIGNAL(clicked(bool)), SLOT(refreshTopicList()));
	QObject::connect(m_ui->btn_changeState, SIGNAL(clicked(bool)), SLOT(changeState()));
	m_ui->btn_changeState->setStyleSheet("background-color : lightgrey");

	QObject::connect(m_ui->nodeGraph, SIGNAL(changeStateTo(std::string)), SLOT(changeStateTo(std::string)));

	QObject::connect(m_ui->check_ratio, SIGNAL(clicked(bool)), m_ui->nodeGraph, SLOT(checkboxGraphRatio(bool)));
	QObject::connect(m_ui->check_transpose, SIGNAL(clicked(bool)), m_ui->nodeGraph, SLOT(checkboxGraphTranspose(bool)));
	QObject::connect(m_ui->check_subgraph, SIGNAL(clicked(bool)), m_ui->nodeGraph, SLOT(checkboxSubgraph(bool)));
	QObject::connect(m_ui->check_subgraph, SIGNAL(clicked(bool)), m_ui->timeline->getTimeLineWidget(), SLOT(checkboxSubgraph(bool)));

	connect(this, SIGNAL(actionClientFinished()), SLOT(checkActionClient()), Qt::QueuedConnection);

}

void StateMachineGUI::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
	qt_gui_cpp::Plugin::saveSettings(plugin_settings, instance_settings);

	instance_settings.setValue("prefix", QString::fromStdString(m_prefix));

	instance_settings.setValue("check_ratio", m_ui->check_ratio->isChecked());
	instance_settings.setValue("check_transpose", m_ui->check_transpose->isChecked());
	instance_settings.setValue("check_subgraph", m_ui->check_subgraph->isChecked());

	instance_settings.setValue("splitter_h", m_ui->splitter_h->saveState());
	instance_settings.setValue("splitter_v", m_ui->splitter_v->saveState());
}

void StateMachineGUI::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
	qt_gui_cpp::Plugin::restoreSettings(plugin_settings, instance_settings);

	m_prefix = instance_settings.value("prefix").toString().toStdString();
	refreshTopicList();

	bool val_ratio = instance_settings.value("check_ratio").toBool();
	m_ui->check_ratio->setChecked(val_ratio);
	m_ui->nodeGraph->checkboxGraphRatio(val_ratio);

	bool val_transpose = instance_settings.value("check_transpose").toBool();
	m_ui->check_transpose->setChecked(val_transpose);
	m_ui->nodeGraph->checkboxGraphTranspose(val_transpose);

	bool val_subgraph = instance_settings.value("check_subgraph").toBool();
	m_ui->check_subgraph->setChecked(val_subgraph);
	m_ui->nodeGraph->checkboxSubgraph(val_subgraph);
	m_ui->timeline->getTimeLineWidget()->checkboxSubgraph(val_subgraph);

	m_ui->splitter_h->restoreState(instance_settings.value("splitter_h").toByteArray());
	m_ui->splitter_v->restoreState(instance_settings.value("splitter_v").toByteArray());
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
	m_sub_info = nh.subscribe(prefix + "/info", 1, &StateMachineGUI::infoReceived, this);

	m_ac.reset(new actionlib::SimpleActionClient<nimbro_fsm2::ChangeStateAction>(
		nh, prefix + "/change_state", false)
	);

	m_w->setWindowTitle(QString::fromStdString(m_prefix));
}

void StateMachineGUI::processStatus(const StatusConstPtr& msg)
{
	m_ui->stateLabel->setText(QString::fromStdString(msg->current_state));

	m_ui->timeline->updateTimeline(msg);
	m_ui->nodeGraph->receiveStatus(msg);
	m_lastUpdate = ros::Time::now();

	QString data = "no data for this state...";
	if(msg->display_messages != "")
		data = QString::fromStdString(msg->display_messages);
	m_ui->label_state_info->setText(data);

	data = "no driver information...";
	if(msg->driver_info != "")
		data = QString::fromStdString(msg->driver_info);
	m_ui->label_driver_info->setText(data);

}

void StateMachineGUI::shutdownPlugin()
{
	m_timer->stop();
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
		m_ui->btn_changeState->setStyleSheet("background-color : lightgrey");

	m_ui->nodeGraph->setChangeStateActive(m_changeState);

}

void StateMachineGUI::changeStateTo(std::string state)
{
	m_changeState = false;
	m_ui->btn_changeState->setStyleSheet("background-color : lightgrey");

	if(!m_actionActive)
	{
		m_actionActive = true;
		nimbro_fsm2::ChangeStateGoal goal;
		goal.state = state;
		m_ac->sendGoal(goal,
			boost::bind(&StateMachineGUI::actionClientFinished, this)
		);
	}
	else
	{
		ROS_ERROR("Action Server is not free. Can't send new state change request");
	}
}

void StateMachineGUI::timerCB()
{
	float updateElapsed = (ros::Time::now() - m_lastUpdate).toSec();
	m_ui->label_status_update->setText(QString::number(updateElapsed, 'f', 1) + "sec");
	m_ui->label_status_update->setStyleSheet("background-color : lime");
	if(updateElapsed > 1.5)
		m_ui->label_status_update->setStyleSheet("background-color : yellow");
	if(updateElapsed > 3.0)
		m_ui->label_status_update->setStyleSheet("background-color : red");

	if(!m_ac || !m_ac->isServerConnected())
	{
		m_ui->btn_changeState->setText("[not connected]");
		m_ui->btn_changeState->setEnabled(false);
		m_actionActive = false;
	}
	else if(m_actionActive)
	{
		m_ui->btn_changeState->setText("...changing state...");
		m_ui->btn_changeState->setEnabled(false);
	}
	else
	{
		m_ui->btn_changeState->setText("Change State ->");
		m_ui->btn_changeState->setEnabled(true);
	}
}

void StateMachineGUI::checkActionClient()
{
	if(m_actionActive)
	{
		auto state = m_ac->getState();
		if(state.isDone())
		{
			m_actionActive = false;
			if(state != state.SUCCEEDED)
			{
				QMessageBox::critical(m_w, "Error",
					QString("Could not change state: %1").arg(
						QString::fromStdString(state.getText())
					)
				);
			}
		}
	}
}


}//NS
PLUGINLIB_EXPORT_CLASS(nimbro_fsm2::StateMachineGUI, rqt_gui_cpp::Plugin)
