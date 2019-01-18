// Nimbro FSM2 Timeline widget
// Author: Christian Lenz <lenz@ais.uni-bonn.de>

#ifndef NIMBRO_FSM_TIMELINE_H
#define NIMBRO_FSM_TIMELINE_H

#include <QWidget>
#include <ros/ros.h>
#include <QTimer>
#include <QVBoxLayout>
#include <QScrollBar>

#include <nimbro_fsm2/Status.h>
#include <nimbro_fsm2/Info.h>

namespace nimbro_fsm2_timeline
{

class TimeLine : public QWidget
{
Q_OBJECT
public:
	TimeLine(QScrollBar* scrollbar, QWidget* parent = 0);
	virtual ~TimeLine();

	void updateTimeline(const nimbro_fsm2::StatusConstPtr& msg);
	void updateStateList(const nimbro_fsm2::InfoConstPtr& msg);

	virtual void paintEvent(QPaintEvent *) override;
	virtual QSize sizeHint() const override;

private Q_SLOTS:
	void handleScrollbar(int value);

private:
	QTimer* m_timer;
	QScrollBar* m_scrollbar;
	nimbro_fsm2::Status m_data;
	nimbro_fsm2::Info m_stateList;
	QString durationToString(ros::Duration d);
	QString durationIntToStr(int sec);
	int m_scrollbar_value;
};


class TimeLineWidget : public QWidget
{
Q_OBJECT
public:
	TimeLineWidget(QWidget* parent);
	virtual ~TimeLineWidget();
	void updateTimeline(const nimbro_fsm2::StatusConstPtr& msg);
	void updateStateList(const nimbro_fsm2::InfoConstPtr& msg);

private:
	QWidget* m_window;
	QVBoxLayout* m_vbox;
	QScrollBar* m_scrollbar;
	TimeLine* m_timeline;

};


}//NS

#endif
