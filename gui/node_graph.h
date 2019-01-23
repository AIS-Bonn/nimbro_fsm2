// Nimbro FSM2 Node Graph widget
// Author: Christian Lenz <lenz@ais.uni-bonn.de>

#ifndef NIMBRO_FSM_NODE_GRAPH_H
#define NIMBRO_FSM_NODE_GRAPH_H

#include <QWidget>
#include <ros/ros.h>
#include <QTimer>
#include <QMouseEvent>
#include <QResizeEvent>

#include <nimbro_fsm2/Status.h>
#include <nimbro_fsm2/Info.h>
#include "b_splines.h"

namespace nimbro_fsm2_node_graph
{

struct Edge
{
	int id;
	int tail;
	int head;
	QColor color;
	QVector<QPointF> supportPoints;

	QPolygonF line;
	QPolygonF arrow;
};

struct Node
{
	int id; //_gvid
	QString full_name; //ns + label
	QString label;
	QColor bColor;
	QColor tColor;

	QRect bb;

	bool selected = false;
	bool in_subgraph = false;

	//in case of a node
	std::map<int, int> succ; //succ node-id -> edge id
	//in case of a subgraph
	std::vector<int> subgraph_content; //nodes in this subgraph

	std::string getLabel();
};


struct Graph
{
	QRect bb;
	std::map<int ,Node> nodes;
	std::map<int ,Edge> edges;

	std::vector<Node> subgraphs;

	bool init = false;

	float scale = 1;
};



class NodeGraph : public QWidget
{
Q_OBJECT
public:
	NodeGraph(QWidget* parent = 0);
	virtual ~NodeGraph();

	void receiveStatus(const nimbro_fsm2::StatusConstPtr& msg);
	void updateInfo(const nimbro_fsm2::InfoConstPtr& msg);

	virtual void paintEvent(QPaintEvent*) override;
	void mousePressEvent(QMouseEvent* event) override;
	void mouseMoveEvent(QMouseEvent* event) override;
	void mouseReleaseEvent(QMouseEvent* event) override;
	void resizeEvent(QResizeEvent* event) override;

	void setChangeStateActive(bool active);

Q_SIGNALS:
	void changeStateTo(std::string state);

public Q_SLOTS:
	void checkboxGraphRatio(bool checked);
	void checkboxGraphTranspose(bool checked);
	void checkboxSubgraph(bool checked);
private:
	QTimer* m_timer;
	nimbro_fsm2::InfoConstPtr m_stateList;
	QString durationToString(ros::Duration d);
	QString durationIntToStr(int sec);

	void generateGraph();
	void interpreteJsonGraph(const QJsonDocument qd);

	Graph m_graph;
	bool m_graph_ratio = false;
	bool m_graph_transpose = false;
	bool m_graph_subgraph = false;
	void updateStatus();
	nimbro_fsm2::StatusConstPtr m_statusMsg;

	void debugGraph();

// 	QString getNodeNs(Node node);
	QRect jsonBB(QVariantMap* node_map, QString key, bool window = false);


	b_spline::BSpline m_bSpline;

	QRect scaleRect(QRect rect);
	QPolygonF scalePolygon(QPolygonF poly);

	bool m_changeStateActive = false;
};

}//NS

#endif
