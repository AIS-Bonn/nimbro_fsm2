// Nimbro FSM2 Node Graph widget
// Author: Christian Lenz <lenz@ais.uni-bonn.de>

#ifndef NIMBRO_FSM_NODE_GRAPH_H
#define NIMBRO_FSM_NODE_GRAPH_H

#include <QWidget>
#include <ros/ros.h>
#include <QTimer>

#include <nimbro_fsm2/Status.h>
#include <nimbro_fsm2/Info.h>
#include "b_splines.h"

namespace nimbro_fsm2_node_graph
{

struct Point2D
{
	float x;
	float y;
};

struct Edge
{
	int parent;
	int child;
	QColor color;

	QVector<QPointF> supportPoints;

	QPolygonF line;
	QPolygonF arrow;
};

struct Node
{
	QString name;
	QString label;
	QColor bColor;
	QColor tColor;

	QRect bb;

};


struct Graph
{
	QRect bb;
	std::vector<Node> nodes;
	std::vector<Edge> edges;

	bool init = false;

	float scale = 1;
};



class NodeGraph : public QWidget
{
Q_OBJECT
public:
	NodeGraph(QWidget* parent = 0);
	virtual ~NodeGraph();

	void updateStatus(const nimbro_fsm2::StatusConstPtr& msg);
	void updateInfo(const nimbro_fsm2::InfoConstPtr& msg);

	virtual void paintEvent(QPaintEvent *) override;
// 	virtual QSize sizeHint() const override;

private Q_SLOTS:

private:
	QTimer* m_timer;
	nimbro_fsm2::Info m_stateList;
	QString durationToString(ros::Duration d);
	QString durationIntToStr(int sec);

	void generateGraph();
	Graph m_graph;

	b_spline::BSpline m_bSpline;

	QRect scaleRect(QRect rect);
	QPoint scalePoint(Point2D p);
	QPolygonF scalePolygon(QPolygonF poly);
};

}//NS

#endif
