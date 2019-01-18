// Nimbro FSM2 Node Graph widget
// Author: Christian Lenz <lenz@ais.uni-bonn.de>

#include "node_graph.h"

#include <QPainter>
#include <fmt/format.h>

#include <fcntl.h>
#include <stdexcept>
#include <sys/wait.h>
#include <QJsonDocument>
#include <QJsonObject>
#include <QVariant>

static void runDot(const std::string& dot, char* dotOut, size_t dotOutSize)
{
	int inputPipe[2];
	int outputPipe[2];

	if(pipe(inputPipe) != 0 || pipe(outputPipe) != 0)
		throw std::runtime_error("Could not create pipe");

	int pid = fork();
	if(pid == 0)
	{
		// Child
		close(inputPipe[1]);
		close(outputPipe[0]);

		dup2(inputPipe[0], STDIN_FILENO);
		dup2(outputPipe[1], STDOUT_FILENO);

		if(execlp("dot", "dot", "-Tjson", "-Gdpi=200","/dev/stdin", (char*)NULL) != 0)
		{
			perror("execlp() failed");
			throw std::runtime_error("Could not execute dot");
			exit(1);
		}

		// Cannot reach this
		return;
	}
	else
	{
		// Parent
		close(inputPipe[0]);
		close(outputPipe[1]);

		size_t written = 0;
		size_t len = dot.length();
		while(written != len)
		{
			int ret = write(inputPipe[1], dot.c_str() + written, len - written);
			if(ret <= 0)
				perror("Could not write");

			written += ret;
		}
		close(inputPipe[1]);

		size_t readBytes = 0;
		while(1)
		{
			if(dotOutSize == readBytes)
			{
				std::cout << "dotOutSize: " << dotOutSize << std::endl;
				throw std::runtime_error("dot buffer is to small");
			}

			int ret = read(outputPipe[0], dotOut + readBytes, dotOutSize - readBytes);
			if(ret == 0)
				break;
			else if(ret < 0)
			{
				perror("Could not read");
				throw std::runtime_error("Could not read");
			}

			readBytes += ret;
		}
		close(outputPipe[0]);

		if(dotOutSize == readBytes)
		{
			std::cout << "dotOutSize: " << dotOutSize << std::endl;
			throw std::runtime_error("dot buffer is to small");
		}

		dotOut[readBytes] = 0;

		int status;
		waitpid(pid, &status, 0);
		if(status != 0)
			fprintf(stderr, "dot exited with error status %d\n", status);
	}
}


namespace nimbro_fsm2_node_graph
{

NodeGraph::NodeGraph(QWidget* parent)
: QWidget(parent)
{
	m_timer = new QTimer();
	m_timer->setInterval(100);
	m_timer->start();

	connect(m_timer, SIGNAL(timeout()), this, SLOT(update()));
}

NodeGraph::~NodeGraph()
{
}

void NodeGraph::paintEvent(QPaintEvent*)
{
	QPainter painter(this);
	painter.fillRect(rect(), Qt::white);
	painter.setRenderHint(QPainter::SmoothPixmapTransform);

	//graph initialized?
	if(!m_graph.init)
		return;

	m_graph.scale = std::min((float)width() / m_graph.bb.width(), (float)height() / m_graph.bb.height());

	painter.setBrush(QBrush(QColor(255,255,255)));
	painter.drawRect(scaleRect(m_graph.bb));



	for(auto& e : m_graph.edges)
	{
		//Line
		if(e.line.size() < 2)
		{
			ROS_WARN("Edge path has not enough points.");
			continue;
		}

		painter.setPen(QPen(e.color));
		painter.setBrush(QBrush(Qt::NoBrush));

		painter.drawPolyline(scalePolygon(e.line));


		//Arrow
		if(e.arrow.size() != 3)
		{
			ROS_WARN("Edge arrow has != 3 points.");
			continue;
		}

		painter.setBrush(QBrush(e.color));
		painter.drawPolygon(scalePolygon(e.arrow));
	}

	for(auto& n : m_graph.nodes)
	{
		painter.setBrush(QBrush(n.bColor));
		painter.drawRect(scaleRect(n.bb));
		painter.setPen(QPen(n.tColor));
		painter.drawText(scaleRect(n.bb), Qt::AlignCenter, n.label);
	}

}


void NodeGraph::updateStatus(const nimbro_fsm2::StatusConstPtr& msg)
{
	if(m_graph.init)
	{
		int max_hist = std::min((int)msg->history.size(), 2);
		int last_idx = msg->history.size();

		for(auto& n : m_graph.nodes)
		{
			n.bColor = QColor(255,255,255);
			for(int i=0; i < max_hist; i++)
			{
				int idx = last_idx - max_hist + i;
				if(n.label.toStdString() == msg->history[idx].name)
				{
					n.bColor = QColor((max_hist - i -1) * 200, 255, (max_hist - i-1) * 200);
				}
			}
		}
	}

	update();
}

void NodeGraph::updateInfo(const nimbro_fsm2::InfoConstPtr& msg)
{
	m_stateList = *msg;

	generateGraph();

	update();
}


void NodeGraph::generateGraph()
{
	//create stringstream for dot graph
	std::stringstream ss;
	std::map<std::string, int> map_node_id;
	size_t dotOutSize = 1000;
	ss << "digraph {\n";

	//Nodes
	for(int i=0;i<m_stateList.states.size();i++)
	{
		auto& state = m_stateList.states[i];
		ss << fmt::format("  v{} [ label=\"{}\" shape=box];\n",i, state.name);
		map_node_id[state.name] = i;
		dotOutSize+=1000;
	}
	//Edges
	for(int i=0;i<m_stateList.states.size();i++)
	{
		auto& state = m_stateList.states[i];
		for(auto succ : state.successors)
		{
			ss << fmt::format("  v{} -> v{};",i, map_node_id[succ]);
			dotOutSize+=1000;
		}
		if(state.successors.size() > 0)
			ss << "\n";
	}
	ss << "}\n";

// 	std::cout << "-----------" << std::endl << ss.str() << std::endl << "--------_" << std::endl;

	//Run dot
	char dotOut[dotOutSize];
	runDot(ss.str(), dotOut, dotOutSize);

	//Print Json//TODO
	std::cout << dotOut <<std::endl;
// 	std::cout << "----------" << std::endl;

	//Create Json object
	QJsonDocument dotJson = QJsonDocument::fromJson(QString(dotOut).toUtf8());
	QJsonObject dotObj;

	if(!dotJson.isNull())
	{
		if(dotJson.isObject())
			dotObj = dotJson.object();
		else
			throw std::runtime_error("no Json Object");
	}
	else
		throw std::runtime_error("no Json Document");

	QVariantMap dot_map = dotObj.toVariantMap();

	//Find bounding box corners
	std::string bb = dot_map["bb"].toString().toStdString();

	size_t idx[3];
	idx[0] = bb.find(",");
	idx[1] = bb.find(",", idx[0] + 1);
	idx[2] = bb.find(",", idx[1] + 1);

	float x = boost::lexical_cast<float>(bb.substr(0,idx[0]++));
	float y = boost::lexical_cast<float>(bb.substr(idx[0], idx[1]++ - idx[0]));
	float xm = boost::lexical_cast<float>(bb.substr(idx[1], idx[2]++ - idx[1]));
	float ym = boost::lexical_cast<float>(bb.substr(idx[2]));
	float w = xm-x;
	float h = ym-y;

	m_graph.bb = QRect(x,y,w,h);

	//Nodes
	QVariantList node_list = dot_map["objects"].toList();

	for(auto& nodes: node_list)
	{
		QVariantMap node_map = nodes.toMap();
		Node n;
		n.name = node_map["name"].toString();
		n.label = node_map["label"].toString();
		n.tColor = QColor(0,0,0);
		n.bColor = QColor(255,255,255);

		//bb
		QVariantList draw_list = node_map["_draw_"].toList();
		for(auto& draw : draw_list)
		{
			QVariantMap draw_map = draw.toMap();
			QString op = draw_map["op"].toString();
			if(op != "p")
				continue;

			QVariantList point_list = draw_map["points"].toList();
			float x_array[4];
			float y_array[4];
			int idx = 0;

			if(point_list.size() != 4)
				throw std::runtime_error("Node bounding box has != 4 coordinates");
			for(auto& p : point_list)
			{
				QVariantList points = p.toList();
				if(points.size() != 2)
					throw std::runtime_error("Node bounding box coordinates has != 2 values");

				x_array[idx] = points[0].toFloat();
				y_array[idx] = points[1].toFloat();
				idx++;
			}

			float x = *std::min_element(x_array, x_array+4);
			float y = *std::min_element(y_array, y_array+4);
			float xm = *std::max_element(x_array, x_array+4);
			float ym = *std::max_element(y_array, y_array+4);
			float w = xm - x;
			float h = ym - y;

			//Flip y-axis. Origin in bottom left in dot and top left in Qt
			y = m_graph.bb.height() - ym;

			n.bb = QRect(x,y,w,h);
			break;
		}

		m_graph.nodes.push_back(n);
	}

	//Edges
	QVariantList edge_list = dot_map["edges"].toList();
	for(auto& edge : edge_list)
	{
		Edge e;
		e.color = QColor(0,0,0);

		QVariantMap edge_map = edge.toMap();

		//Line points
		QVariantList draw_list = edge_map["_draw_"].toList();
		for(auto& draw : draw_list)
		{
			QVariantMap draw_map = draw.toMap();
			QString op = draw_map["op"].toString();
			if(op != "b")
				continue;

			QVariantList point_list = draw_map["points"].toList();
			for(auto& p : point_list)
			{
				QVariantList points = p.toList();
				if(points.size() != 2)
					throw std::runtime_error("Edge points coordinates has != 2 values");
				QPointF p2d;
				p2d.setX(points[0].toFloat());
				p2d.setY(m_graph.bb.height() - points[1].toFloat());
				e.supportPoints.push_back(p2d);
			}
		}

		//Arrow Points
		draw_list = edge_map["_hdraw_"].toList();
		for(auto& draw : draw_list)
		{
			QVariantMap draw_map = draw.toMap();
			QString op = draw_map["op"].toString();
			if(op != "P")
				continue;

			QVariantList point_list = draw_map["points"].toList();
			for(auto& p : point_list)
			{
				QVariantList points = p.toList();
				if(points.size() != 2)
					throw std::runtime_error("Edge points coordinates has != 2 values");
				QPointF p2d;
				p2d.setX(points[0].toFloat());
				p2d.setY(m_graph.bb.height() - points[1].toFloat());
				e.arrow.push_back(p2d);
			}
		}

		m_bSpline.compute(&e.supportPoints, &e.line);

		m_graph.edges.push_back(e);
	}

	m_graph.init = true;
}


QRect NodeGraph::scaleRect(QRect rect)
{
	float scale = m_graph.scale;
	return QRect(rect.x() * scale, rect.y() * scale, rect.width() * scale, rect.height() * scale);
}

QPoint NodeGraph::scalePoint(Point2D p)
{
	return QPoint(p.x * m_graph.scale, p.y * m_graph.scale);
}

QPolygonF NodeGraph::scalePolygon(QPolygonF poly)
{
	QPolygonF scaledPoly;
	for(int i=0; i< poly.size(); i++)
	{
		scaledPoly.push_back(QPointF(poly[i].x() * m_graph.scale, poly[i].y() * m_graph.scale));
	}
	return scaledPoly;
}

QString NodeGraph::durationToString(ros::Duration d)
{
	int run_seconds = d.toSec() * 10;
// 	int tenths = run_seconds % 10;
	int secs = run_seconds / 10;
	int mins = secs / 60;
	mins = mins % 60;
	secs = secs % 60;
	return QString::number(secs);
}

QString NodeGraph::durationIntToStr(int sec)
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



}//NS
