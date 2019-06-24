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
                ROS_WARN("dotOutSize: %i",(int)dotOutSize);
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
            ROS_WARN("dotOutSize: %i",(int)dotOutSize);
            throw std::runtime_error("dot buffer is to small");
        }

        dotOut[readBytes] = 0;

        int status;
        waitpid(pid, &status, 0);
        if(status != 0)
            fprintf(stderr, "dot exited with error status %d\n", status);
    }
}

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

namespace nimbro_fsm2_node_graph
{

NodeGraph::NodeGraph(QWidget* parent)
    : QWidget(parent)
{
    m_timer = new QTimer();
    m_timer->setInterval(100);
    m_timer->start();

	m_regEx.setPattern(R"EOS(((?P<ns>\w[\w0-9]*)::)?(?P<name>.*))EOS");

    connect(m_timer, SIGNAL(timeout()), this, SLOT(update()));
    setMouseTracking(true);

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

// 	painter.setBrush(QBrush(QColor(255,5,255)));
// 	painter.drawRect(scaleRect(m_graph.bb));

    for(auto& em : m_graph.edges)
    {
        auto& e = em.second;
        //Line
        if(e.line.size() < 2)
        {
            ROS_WARN("Edge path has not enough points.");
            continue;
        }

        painter.setPen(QPen(QColor(0,0,0)));
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

	for(unsigned int i=0;i< m_graph.subgraphs.size(); i++)
    {
		auto& sg = m_graph.subgraphs[i];

		QRect sg_rect = scaleRect(sg.bb);
        painter.setPen(QPen(QColor(0,0,0)));
		QColor color = static_colors[i % m_graph.subgraphs.size()];
		color.setAlpha(40);
		painter.setBrush(color);
		painter.drawRect(sg_rect);

		//serch highest node in subgraph
		int y = sg_rect.y() + sg_rect.height();
		QRect top_bb;
		for(int n_id : sg.subgraph_content)
		{
			QRect n_bb = scaleRect(m_graph.nodes[n_id].bb);
			if(n_bb.y() < y)
			{
				y = n_bb.y();
				top_bb = n_bb;
			}
		}

		int dist_l = std::abs(sg_rect.x() - top_bb.x());
		int dist_r = std::abs(sg_rect.x() + sg_rect.width() - top_bb.x() - top_bb.width());


		QFont font;
		int pixelsize = std::min(sg_rect.width() / sg.label.size(), sg_rect.height());
		font.setPixelSize(pixelsize / 2);
		painter.setFont(font);

		color.setAlpha(200);
		painter.setPen(color);
		QString text = " " + sg.label + " ";
		if(dist_l >= dist_r)
			painter.drawText(sg_rect, Qt::AlignLeft, text);
		else
			painter.drawText(sg_rect, Qt::AlignRight, text);

    }

    for(auto& nm : m_graph.nodes)
    {
        auto& n = nm.second;
		QRect n_rect = scaleRect(n.bb);
        painter.setBrush(QBrush(n.bColor));
        painter.setPen(QPen(n.tColor));
        if(n.selected)
        {
            painter.setBrush(QBrush(QColor(255,150,0)));
            painter.setPen(QPen(QColor(0,0,0)));
        }
        painter.drawRect(n_rect);

		QFont font;

		int pixelsize = std::min((float)n_rect.height(),(float)n_rect.width() / n.label.size() * 1.7f);
		font.setPixelSize(pixelsize);
		painter.setFont(font);

        painter.drawText(n_rect, Qt::AlignCenter, n.label);
    }



}

void NodeGraph::resizeEvent(QResizeEvent* event)
{
	if(m_graph_ratio)
		generateGraph();
}


void NodeGraph::receiveStatus(const nimbro_fsm2::StatusConstPtr& msg)
{
	m_statusMsg = msg;
	updateStatus();
}

void NodeGraph::updateStatus()
{
    if(!m_graph.init || !m_statusMsg)
        return;

    //Edges black
    for(auto& e : m_graph.edges)
        e.second.color = QColor(0,0,0);

    //Nodes white
    for(auto& nm : m_graph.nodes)
        nm.second.bColor = QColor(255,255,255);

    //get history information
    int max_hist = std::min((int)m_statusMsg->history.size(), 3);
    int last_idx = m_statusMsg->history.size();

    int history_idx[max_hist];

    for(int i=0; i< max_hist; i++)
    {
        for(auto& nm: m_graph.nodes)
        {
            auto& n = nm.second;
            if(n.full_name.toStdString() == m_statusMsg->history[last_idx - max_hist + i].name)
            {
                history_idx[i] = n.id;
            }
        }
    }

    for(int i=0; i<max_hist; i++)
    {
        int cv = -250 * std::pow(2,-1.8 * (max_hist - i - 1))+255;
        QColor color(cv, 255, cv);

        //Color nodes
        int idx = history_idx[i];
        m_graph.nodes[idx].bColor = color;


        if(i==max_hist - 1)
            continue;

        //Color edges
        int idx_succ = history_idx[i + 1];



        //Check if edge exist (not the case for state jump by the user)
        if(m_graph.nodes[idx].succ.count(idx_succ) > 0)
            m_graph.edges[m_graph.nodes[idx].succ[idx_succ]].color = color;

    }
    update();
}

void NodeGraph::mouseMoveEvent(QMouseEvent* event)
{
    if(!m_changeStateActive)
        return;

    for(auto& nm : m_graph.nodes)
    {
        auto& n = nm.second;
        n.selected = false;

        if(scaleRect(n.bb).contains(event->pos()))
            n.selected = true;
    }

    return;
}

void NodeGraph::mousePressEvent(QMouseEvent *event)
{
    return;
}

void NodeGraph::mouseReleaseEvent(QMouseEvent *event)
{
    if(!m_changeStateActive)
        return;

    for(auto& nm : m_graph.nodes)
    {
        auto& n = nm.second;
        n.selected = false;
        if(scaleRect(n.bb).contains(event->pos()))
        {
            setChangeStateActive(false);
            if(parent() != 0)
                emit changeStateTo(n.full_name.toStdString());
        }
    }
}

void NodeGraph::setChangeStateActive(bool active)
{
    m_changeStateActive = active;
}


void NodeGraph::checkboxGraphRatio(bool checked)
{
    m_graph_ratio = checked;
    generateGraph();
}

void NodeGraph::checkboxGraphTranspose(bool checked)
{
    m_graph_transpose = checked;
    generateGraph();
}

void NodeGraph::checkboxSubgraph(bool checked)
{
    m_graph_subgraph = checked;
    generateGraph();
}


void NodeGraph::updateInfo(const nimbro_fsm2::InfoConstPtr& msg)
{
    m_stateList = msg;
    generateGraph();
    update();
}


void NodeGraph::generateGraph()
{
	if(!m_stateList)
		return;
    //clear old content
    m_graph.nodes.clear();
    m_graph.edges.clear();
    m_graph.subgraphs.clear();

    //create stringstream for dot graph
    std::stringstream ss;
	ss.imbue(std::locale::classic());

    //remember node ids for creating edges
    std::map<std::string, int> map_node_id;


    //sort nodes according to namespace
    std::map<std::string, std::vector<std::string>> subgraph_list;
    size_t dotOutSize = 1000;
    ss << "digraph {\n";

	//widget size in inch for the dot size parameter:
	//(float)width() / logicalDpiX(), (float)height() / logicalDpiY()

	if(m_graph_ratio)
    {
        float ratio = height() / (float)width();
        ss << "ratio=" << ratio << ";\n";
    }
    if(m_graph_transpose)
        ss << "rankdir=LR;\n";

    for(int i=0; i < (int)m_stateList->states.size(); i++)
    {
        auto& state = m_stateList->states[i];


		if(m_graph_subgraph)
		{
			auto match = m_regEx.match(QString::fromStdString(state.name));
			std::string name;
			std::string ns;

			if(match.hasMatch())
			{
				name = match.captured("name").toStdString();
				ns = match.captured("ns").toStdString();
			}
			else
			{
				ROS_WARN("Could not match class name '%s'", state.name.c_str());
				name = state.name;
			}

			subgraph_list[ns].push_back(name);
		}
		else
			subgraph_list[""].push_back(state.name);

		map_node_id[state.name] = i;
    }

    //Nodes
    int subgraph_id = 0;
    for(auto& [ns , nodes] : subgraph_list)
    {
        std::string indent= "  ";
        if(m_graph_subgraph && ns != "")
        {
            ss << fmt::format("  subgraph cluster_{} ", subgraph_id++) << "{\n";
            ss << fmt::format("{}label=\"{}\";\n", indent, ns);
            indent = "    ";
        }

        for(auto& node : nodes)
        {
			std::string full_name = node;
			if(ns != "")
				full_name = ns + "::" + node;

            ss << fmt::format("{}v{}  [ label=\"{}\" shape=box];\n",indent, map_node_id[full_name], node);
            dotOutSize+=2000;
        }

        if(m_graph_subgraph && ns != "")
            ss << "  }\n";
    }


    //Edges
    for(int i=0; i<(int)m_stateList->states.size(); i++)
    {
        auto& state = m_stateList->states[i];
        for(auto succ : state.successors)
        {
		    ss << fmt::format("  v{} -> v{};",i, map_node_id[succ]);
            dotOutSize+=1000;
        }
        if(state.successors.size() > 0)
            ss << "\n";
    }
    ss << "}\n";

	std::cout << ss.str() << std::endl;

    //Run dot
    char dotOut[dotOutSize];
    runDot(ss.str(), dotOut, dotOutSize);

    //Create Json object
    QJsonDocument qd = QJsonDocument::fromJson(QString(dotOut).toUtf8());
    interpreteJsonGraph(qd);


    m_graph.init = true;
	updateStatus();
}

void NodeGraph::interpreteJsonGraph(const QJsonDocument qd)
{
    QJsonObject dotObj;

    if(!qd.isNull())
    {
        if(qd.isObject())
            dotObj = qd.object();
        else
            throw std::runtime_error("no Json Object");
    }
    else
        throw std::runtime_error("no Json Document");

    QVariantMap dot_map = dotObj.toVariantMap();

    //Find bounding box corners

	m_graph.bb = jsonBB(&dot_map,"P", true);

    //Nodes / Subgraphs
    QVariantList node_list = dot_map["objects"].toList();
    for(auto& nodes: node_list)
    {
        QVariantMap node_map = nodes.toMap();
        Node n;
		n.id = node_map["_gvid"].toInt();
        n.label = node_map["label"].toString();
		n.full_name = n.label;
        n.tColor = QColor(0,0,0);
        n.bColor = QColor(255,255,255);
		n.bb = jsonBB(&node_map, "p");

		std::string object_name = node_map["name"].toString().toStdString();

        if(object_name.find("cluster_") != std::string::npos)
        {
            //Cluster
			QVariantList nl = node_map["nodes"].toList();
			for(auto& node : nl)
				n.subgraph_content.push_back(node.toInt());

            m_graph.subgraphs.push_back(n);
        }
        else
        {
            //Normal node
			m_graph.nodes[n.id] = n;
        }
    }

    for(auto& sg : m_graph.subgraphs)
	{
		for(auto& node_id : sg.subgraph_content)
		{
			auto& node = m_graph.nodes[node_id];
			node.full_name = sg.label + "::" + node.label;
		}
	}

    //Edges
    QVariantList edge_list = dot_map["edges"].toList();
    for(auto& edge : edge_list)
    {
        Edge e;
        e.id = m_graph.edges.size();
        e.color = QColor(0,0,0);

        QVariantMap edge_map = edge.toMap();

        e.tail = edge_map["tail"].toInt();
        e.head = edge_map["head"].toInt();

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

        m_graph.nodes[e.tail].succ[e.head] = e.id;
        m_graph.edges[e.id] = e;
    }
}

QRect NodeGraph::jsonBB(QVariantMap* node_map, QString key, bool window)
{
	QVariantList draw_list = (*node_map)["_draw_"].toList();
	for(auto& draw : draw_list)
	{
		QVariantMap draw_map = draw.toMap();
		QString op = draw_map["op"].toString();
		if(op != key)
			continue;

		QVariantList point_list = draw_map["points"].toList();
		float x_array[4];
		float y_array[4];
		int idx = 0;

		if(point_list.size() != 4)
			throw std::runtime_error("Bounding box has != 4 coordinates");
		for(auto& p : point_list)
		{
			QVariantList points = p.toList();
			if(points.size() != 2)
				throw std::runtime_error("Bounding box coordinates has != 2 values");

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

		//Flip y-axis. Origin is bottom left in dot and top left in Qt
		if(!window)
			y = m_graph.bb.height() - ym;

		return QRect(x,y,w,h);
	}
	ROS_ERROR("could not find bounding box in json data :(");
	return QRect(0,0,0,0);
}

QRect NodeGraph::scaleRect(QRect rect)
{
    float scale = m_graph.scale;
    return QRect(rect.x() * scale, rect.y() * scale, rect.width() * scale, rect.height() * scale);
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


}//NS
