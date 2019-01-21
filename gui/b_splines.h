//B-Spline interpolation for graph view
//Author: Christian Lenz <lenz@ais.uni-bonn.de>

// Code based on https://github.com/vkorchagin/animated-b-spline

// InterpolateBezier - interpolates points with bezier curve.
// Algorithm is based on article "Adaptive Subdivision of Bezier Curves" by
// Maxim Shemanarev.
// http://www.antigrain.com/research/adaptive_bezier/index.html

// CalculateBoorNet - inserts new control points with de Boor algorithm for
// transformation of B-spline into composite Bezier curve.


#ifndef NIMBRO_FSM2_BSPLINES_H
#define NIMBRO_FSM2_BSPLINES_H

#include <QPolygonF>
#include <QPointF>

namespace b_spline
{

class BSpline
{

public:
	BSpline();
	~BSpline();

	void compute(QVector<QPointF>* controlPoints, QPolygonF* interpolatedPoints);

private:

	void calculateBoorNet();
	void interpolateBezier(double x1, double y1,double x2, double y2,double x3, double y3,double x4, double y4,unsigned level);
	void fillKnotVector();

	QVector<qreal> m_knotVector;
	QVector<QPointF>* m_controlPoints;
	QPolygonF* m_interpolatedPoints;
	QPolygonF m_boorNetPoints;

	double m_distanceTolerance = 0.5;
};

}//NS


#endif

