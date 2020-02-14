// Display the time line for ARC
// Author: Christian Lenz<lenz@ais.uni-bonn.de>

#include "timeline.h"
#include <QPainter>

static const std::vector<QColor> static_colors = {
    QColor(255, 179, 0),
    QColor(128, 62, 117),
    QColor(255, 104, 0),
    QColor(166, 189, 215),
    QColor(193, 0, 32),
    QColor(206, 162, 98),
    QColor(129, 112, 102),
    QColor(0, 125, 52),
    QColor(246, 118, 142),
    QColor(0, 83, 138),
    QColor(255, 122, 92),
    QColor(83, 55, 122),
    QColor(255, 142, 0),
    QColor(179, 40, 81),
    QColor(244, 200, 0),
    QColor(127, 24, 13),
    QColor(147, 170, 0),
    QColor(89, 51, 21),
    QColor(241, 58, 19),
    QColor(35, 44, 22)
};

namespace nimbro_fsm2_timeline
{

TimeLine::TimeLine(QScrollBar* scrollbar, QWidget* parent)
: QWidget(parent)
, m_scrollbar(scrollbar)
{
	m_timer = new QTimer(this);
	m_timer->setInterval(100);
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

void TimeLine::wheelEvent(QWheelEvent* event)
{
	event->accept();
	float wheelDelta = event->angleDelta().y() / 120;

	if(m_mouseWheelPos > 10 && wheelDelta > 0)
		return;

	if(m_mouseWheelPos < 0.005 && wheelDelta < 0)
		return;

	m_mouseWheelPos *= std::pow(1.3, wheelDelta);
}

void TimeLine::paintEvent(QPaintEvent*)
{
	QPainter painter(this);
	painter.fillRect(rect(), Qt::white);

	painter.setRenderHint(QPainter::SmoothPixmapTransform);

	int num_states = m_stateList.states.size();

	if(m_data.history.size() < 1 || num_states < 1)
		return;

	float right_offset = 20;
	float num_of_secs = (ros::Time::now() - m_data.history[0].start).toSec();
	float text_offset = std::min(250, width() / 4);
	float w = width() - text_offset - right_offset;
	float h_line = std::min(40, height());
	float h_tasks = std::max(0.f, (height() - h_line) / num_states);

	float num_of_secs_displayed = 180 * m_mouseWheelPos;
	float time_width = w / num_of_secs_displayed;


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
	float pixelsize = 1000;
	QFont normal_font = painter.font();
	//sort
	std::map<QString, QVector<QString>> sort_map;
	std::map<QString, int> task_name_id;
	std::map<QString, QColor> task_color;

	for(auto& state: m_stateList.states)
	{
		std::string name = state.name;
		std::size_t pos_ns = name.find("::");

		std::string ns = "";
		if(pos_ns != std::string::npos)
			ns = name.substr(0,pos_ns);
		sort_map[QString::fromStdString(ns)].push_back(QString::fromStdString(name));

		float name_length = name.size();
// 		if(m_showSubgraph)
// 			name_length -= ns.length();

		pixelsize = std::min(pixelsize,(float)text_offset / name_length * 1.7f);
	}

	QFont font;
	pixelsize = std::min(pixelsize,(float)h_tasks * 0.8f);
	font.setPixelSize(std::max(pixelsize, 1.0f));

	//Paint names
	int idx = 0;
	int ns_idx = 0;
	for(auto& [ns, list] : sort_map)
	{
		QColor color = QColor(250,120,0);
		painter.setBrush(QBrush(QColor(255,255,255)));
		if(m_showSubgraph && ns != "")
		{
			color = static_colors[ns_idx % sort_map.size()];
			color.setAlpha(60);
			painter.setBrush(color);
			painter.setBrush(QBrush(color));
			ns_idx++;
		}
		for(auto& state: list)
		{
			task_name_id[state] = idx;
			color.setAlpha(100);
			task_color[state] = color;
			painter.setPen(QPen(QColor(255,255,255)));
			painter.drawRect(QRect(0, idx * h_tasks, text_offset - 5, h_tasks));

			QString label = state;
// 			if(m_showSubgraph)
// 			{
// 				std::size_t pos_name = state.toStdString().find("::");
// 				if(pos_name != std::string::npos)
// 					label = QString::fromStdString(state.toStdString().substr(pos_name + 2));
// 			}

			painter.setFont(font);
			painter.setPen(QPen(QColor(0,0,0)));
			painter.drawText(QRect(0, idx * h_tasks, text_offset - 5, h_tasks), Qt::AlignVCenter, label);

			idx++;
		}
	}

	painter.setFont(normal_font);

	//History
	ros::Time start_time = m_data.history[0].start;
	int pos_now = text_offset + (ros::Time::now() - start_time).toSec() * time_width - m_scrollbar_value * time_width / 100;
	for(auto& state : m_data.history)
	{
		painter.setBrush(QBrush(QColor(250,120,0)));
		painter.setPen(QPen(QColor(220,100,0)));
		float x,y,wi,he, end;
		x = pos_now - (ros::Time::now() - state.start).toSec() * time_width;
		wi = (state.end - state.start).toSec() * time_width;
		y = task_name_id[QString::fromStdString(state.name)] * h_tasks;
		he = h_tasks;
		end = x + wi;


		if(x < text_offset)
		{
			wi = std::max(1.f, wi - text_offset + x);
			x= text_offset;
		}

		if(end < text_offset)
			wi = 0;

		if(wi > 0 && x > 0)
		{

			painter.setBrush(QBrush(task_color[QString::fromStdString(state.name)]));
			painter.setPen(QPen(task_color[QString::fromStdString(state.name)]));
			QRect rect_line(x,y,wi,he);
			painter.drawRect(rect_line);
			painter.setPen(QPen(QColor(0,0,0)));
			rect_line.setWidth(std::max(rect_line.width(), 60));
			QString name = "(" + durationIntToStr((state.end - state.start).toSec(), true) + "s)";
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


		int loopSize = num_of_secs_displayed + num_of_secs;
		int sec_mainline = 10, sec_2ndLine = 5, sec_4thLine = 1, line_shift = 0;


		if(time_width <= 0.8)
		{
			sec_mainline = 120;
			sec_2ndLine = 60;
			sec_4thLine = loopSize + 1;
		}
		else if(time_width > 0.8 && time_width <= 1.6)
		{
			sec_mainline = 60;
			sec_2ndLine = 30;
			sec_4thLine = loopSize + 1;
		}
		else if(time_width > 1.6 && time_width <= 2.5)
		{
			sec_mainline = 30;
			sec_2ndLine = 15;
			sec_4thLine = loopSize + 1;
		}
		else if(time_width > 2.5 && time_width <= 8)
		{
			sec_mainline = 20;
			sec_2ndLine = 10;
			sec_4thLine = 5;
		}
		else if(time_width > 8 && time_width <= 30)
		{
			sec_mainline = 10;
			sec_2ndLine = 5;
			sec_4thLine = 1;
		}
		else if(time_width > 30 && time_width <= 150)
		{
			sec_mainline = 5;
			sec_2ndLine = 1;
			sec_4thLine = loopSize +1;
		}
		else if(time_width > 150)
		{
			sec_mainline = sec_2ndLine = 1;
			sec_4thLine = loopSize + 1;
			line_shift = time_width / 2;
		}

		for(int i=0 ; i < loopSize; i++)
		{
			int x = pos_now - i * time_width;
			if(x < text_offset)
				continue;
			//Main lines with numbers
			if(i% sec_mainline == 0)
			{
				painter.drawLine(x, time_height_down, x, time_height_up);
				painter.drawText(QRect(x- 25, time_height_down, 50, h_line/2), Qt::AlignCenter, durationIntToStr(-i));
			}
			//2nd lines
			if(i% sec_2ndLine == 0){
				if(x - line_shift < text_offset)
					continue;
				painter.drawLine(x - line_shift, time_height_down - h_line / 8, x - line_shift, time_height_up);
			}
			//4th lines
			if(i % sec_4thLine == 0)
				painter.drawLine(x, time_height_down - h_line / 3, x, time_height_up);
		}
	}

}

void TimeLine::checkboxSubgraph(bool checked)
{
	m_showSubgraph = checked;
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
// 	update();
}

void TimeLine::updateStateList(const nimbro_fsm2::InfoConstPtr& msg)
{
	m_stateList = *msg;
// 	update();
}

QString TimeLine::durationIntToStr(int sec, bool onlySec)
{
	QString t;
	if(sec < 0)
		t = "-";
	sec = std::abs(sec);
	//mins
	if(sec / 60 > 0)
		t += QString::number(sec / 60) + ":";
	else if(!onlySec)
		t += "0:";

	if(sec%60 < 10 && (!onlySec || sec / 60 > 0))
		t+= "0";
	t += QString::number(sec % 60);
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
