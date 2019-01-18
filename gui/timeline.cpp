// Display the time line for ARC
// Author: Christian Lenz<lenz@ais.uni-bonn.de>

#include "timeline.h"
#include <QPainter>

namespace nimbro_fsm2_timeline
{

TimeLine::TimeLine(QScrollBar* scrollbar, QWidget* parent)
: QWidget(parent)
, m_scrollbar(scrollbar)
{
	m_timer = new QTimer();
	m_timer->setInterval(10);
	m_timer->start();

	connect(m_timer, SIGNAL(timeout()), this, SLOT(update()));
	connect(m_scrollbar, SIGNAL(valueChanged(int)),
            this, SLOT(handleScrollbar(int)));
	m_scrollbar_value = 0;
}

TimeLine::~TimeLine()
{
}

void TimeLine::handleScrollbar(int value)
{
	m_scrollbar_value = value;
	update();
}

void TimeLine::paintEvent(QPaintEvent*)
{
	QPainter painter(this);
	painter.fillRect(rect(), Qt::white);

	painter.setRenderHint(QPainter::SmoothPixmapTransform);

	int num_states = m_stateList.states.size();

	if(m_data.history.size() < 1 || num_states < 1)
	{
		ROS_WARN_THROTTLE(2.0, "Waiting... (history size: %i, stateList size:  %i", (int)m_data.history.size(), num_states);
		return;
	}

	float num_of_secs = (ros::Time::now() - m_data.history[0].start).toSec();
	float num_of_secs_displayed = 180;
	int text_offset = 180;
	int w = width() - text_offset;
	int h_line = std::min(40, height());
	int h_tasks = std::max(0, (height() - h_line) / num_states);

	int time_width = w / num_of_secs_displayed;

	if(w<=0)
		return;

	bool slide = false;
	if(m_scrollbar->sliderPosition() == m_scrollbar->maximum() && num_of_secs - num_of_secs_displayed > 0)
	{
		slide = true;
	}

	m_scrollbar->setMaximum(std::max(0.0f,num_of_secs - num_of_secs_displayed) * 100);

	if(slide)
	{
		m_scrollbar_value = (num_of_secs - num_of_secs_displayed) * 100;
		m_scrollbar->setSliderPosition(m_scrollbar_value);
	}


	//Task names

	//sort
	std::map<std::string, std::vector<std::string>> sort_map;
	std::map<QString, int> task_name_id;

	for(auto& state: m_stateList.states)
	{
		std::string name = state.name;
		std::size_t pos_ns = name.find_last_of("::");

		std::string ns = name.substr(0,pos_ns);
		sort_map[ns].push_back(name);
	}

	painter.setBrush(QBrush(QColor(255,255,255)));
	int idx = 0;
	for(auto& [ns, list] : sort_map)
	{
		for(auto& state: list)
		{
			task_name_id[QString::fromStdString(state)] = idx;
			painter.setPen(QPen(QColor(255,255,255)));
			painter.drawRect(QRect(0, idx * h_tasks, text_offset - 5, h_tasks));

			std::size_t pos_name = state.find("::");
			std::string name = state.substr(pos_name + 2);

			painter.setPen(QPen(QColor(0,0,0)));
			painter.drawText(QRect(0, idx * h_tasks, text_offset - 5, h_tasks), Qt::AlignVCenter, QString::fromStdString(name));

			idx++;
		}
	}

	//History
	ros::Time start_time = m_data.history[0].start;
	int pos_now = text_offset + (ros::Time::now() - start_time).toSec() * time_width - m_scrollbar_value * time_width / 100;
	for(auto& state : m_data.history)
	{
		painter.setBrush(QBrush(QColor(250,120,0)));
		painter.setPen(QPen(QColor(220,100,0)));
		int x,y,wi,he;
		x = pos_now - (ros::Time::now() - state.start).toSec() * time_width;
		wi = (state.end - state.start).toSec() * time_width;
		y = task_name_id[QString::fromStdString(state.name)] * h_tasks;
		he = h_tasks;

		if(x < text_offset)
		{
			wi = std::max(0, wi - text_offset + x);
			x= text_offset;
		}

		if(wi > 0 && x > 0)
		{
			painter.setBrush(QBrush(QColor(250,120,0)));
			painter.setPen(QPen(QColor(220,100,0)));
			QRect rect_line(x,y,wi,he);
			painter.drawRect(rect_line);
			painter.setPen(QPen(QColor(0,0,0)));
			rect_line.setWidth(std::max(rect_line.width(), 60));
			QString name = "(" + durationToString(state.end - state.start) + "s)";
			painter.drawText(rect_line, Qt::AlignVCenter, name);
		}
	}

	//Red line at current time
	{
		painter.setPen(QPen(QColor(255,0,0)));
		int x = text_offset + (ros::Time::now() - start_time).toSec() * time_width - m_scrollbar_value * time_width / 100;
		if(x < width())
			painter.drawLine(x, 0, x, height());
	}

	//Time stamps
	{
		int pos_now = text_offset + (ros::Time::now() - start_time).toSec() * time_width - m_scrollbar_value * time_width / 100;

		int y_line = num_states * h_tasks;
		int time_height_up = y_line;
		int time_height_down = y_line + h_line / 2;


		painter.setPen(QPen(QColor(0,0,0)));
		painter.drawLine(text_offset, y_line, width() - 20, y_line);

		for(int i=0 ; i < num_of_secs_displayed + num_of_secs; i++)
		{
			int x = pos_now - i * time_width;
			if(x < text_offset)
				continue;
			if(i%10==0)
			{
				painter.drawLine(x, time_height_down, x, time_height_up);
				painter.drawText(QRect(x- 25, time_height_down, 50, h_line/2), Qt::AlignCenter, durationIntToStr(-i));
			}
			else if(i%5==0)
				painter.drawLine(x, time_height_down - h_line / 8, x, time_height_up);
			else
				painter.drawLine(x, time_height_down - h_line / 3, x, time_height_up);
		}
	}

}

QSize TimeLine::sizeHint() const
{
	float time = 100;
	if(m_data.history.size() > 0)
		time = (ros::Time::now() - m_data.history[0].start).toSec();
	QSize size(400 * time, 200);
	return size;
}


void TimeLine::updateTimeline(const nimbro_fsm2::StatusConstPtr& msg)
{
	m_data = *msg;
	update();
}

void TimeLine::updateStateList(const nimbro_fsm2::InfoConstPtr& msg)
{
	m_stateList = *msg;
	update();
}


QString TimeLine::durationToString(ros::Duration d)
{
	int run_seconds = d.toSec() * 10;
// 	int tenths = run_seconds % 10;
	int secs = run_seconds / 10;
	int mins = secs / 60;
	mins = mins % 60;
	secs = secs % 60;
	return QString::number(secs);
}

QString TimeLine::durationIntToStr(int sec)
{
	QString t;
	if(sec < 0)
		t = "-";
	sec = std::abs(sec);
	if(sec / 60 > 0)
		t += QString::number(sec / 60);
	else
		t += "0";
	t += ":";
	t += QString::number(sec % 60);
	if(sec%60 == 0)
		t+= "0";
	return t;
}



TimeLineWidget::TimeLineWidget(QWidget* parent)
: QWidget(parent)
{
	m_vbox = new QVBoxLayout(this);

	m_scrollbar = new QScrollBar(Qt::Horizontal, this);
	m_timeline = new TimeLine(m_scrollbar, this);

	m_vbox->addWidget(m_timeline);
	m_vbox->addWidget(m_scrollbar);
	setLayout(m_vbox);
}

TimeLineWidget::~TimeLineWidget()
{
}


void TimeLineWidget::updateTimeline(const nimbro_fsm2::StatusConstPtr& msg)
{
	m_timeline->updateTimeline(msg);
}

void TimeLineWidget::updateStateList(const nimbro_fsm2::InfoConstPtr& msg)
{
	m_timeline->updateStateList(msg);
}


}//NS
