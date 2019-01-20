//B-Spline interpolation for graph view
//Author: Christian Lenz <lenz@ais.uni-bonn.de>

#include "b_splines.h"
#include <iostream>
#include <QHash>
#include <QtCore/qmath.h>

namespace b_spline
{

BSpline::BSpline()
{
}

BSpline::~BSpline()
{
}

void BSpline::compute(QVector<QPointF>* controlPoints, QPolygonF* interpolatedPoints)
{
	m_controlPoints = controlPoints;
	m_interpolatedPoints = interpolatedPoints;
	fillKnotVector();


	m_interpolatedPoints->clear();

	calculateBoorNet();


	m_interpolatedPoints->push_back(controlPoints->first());

	for(int i = 0; i < m_boorNetPoints.size() - 3; i += 3)
	{
		double x1 = m_boorNetPoints[i].x();
		double x2 = m_boorNetPoints[i+1].x();
		double x3 = m_boorNetPoints[i+2].x();
		double x4 = m_boorNetPoints[i+3].x();
		double y1 = m_boorNetPoints[i].y();
		double y2 = m_boorNetPoints[i+1].y();
		double y3 = m_boorNetPoints[i+2].y();
		double y4 = m_boorNetPoints[i+3].y();
		interpolateBezier(x1,y1,x2,y2,x3,y3,x4,y4,0);
	}

	m_interpolatedPoints->push_back(controlPoints->last());



}

void BSpline::calculateBoorNet()
{
	if(m_controlPoints->size() < 3)
		throw std::runtime_error("calculateBoorNet: not enough controll Points.");

	if(m_knotVector.size() < 5)
		throw std::runtime_error("calculateBoorNet: not enough knot Points.");

  // We draw uniform cubic B-spline that passes through endpoints, so we assume
  // that multiplicity of first and last knot is 4 and 1 for knots between.

	QVector<qreal> newKnotVector = m_knotVector;
	m_boorNetPoints.clear();

	for(int i = 0; i < m_controlPoints->size(); i++)
		m_boorNetPoints.push_back(m_controlPoints->at(i));

	// Insert every middle knot 2 times to increase its multiplicity from 1 to 3.
	const int curveDegree = 3;
	const int increaseMultiplicity = 2;

	for (int knotCounter = 4; knotCounter < newKnotVector.size() - 4; knotCounter += 3)
	{

		QHash< int, QHash<int, QPointF> > tempPoints;
		for (int i = knotCounter - curveDegree; i <= knotCounter; i++)
			tempPoints[i][0] = m_boorNetPoints[i];

		//insert counter
		for (int insertCounter = 1; insertCounter <= increaseMultiplicity; ++insertCounter)
		{
			for (int i = knotCounter - curveDegree + insertCounter; i < knotCounter;i++)
			{
				double coeff = (newKnotVector[knotCounter] - newKnotVector[i]) / (newKnotVector[i + curveDegree - insertCounter + 1] - newKnotVector[i]);

				QPointF newPoint = (1.0 - coeff) * tempPoints[i - 1][insertCounter - 1] + coeff * tempPoints[i][insertCounter - 1];
				tempPoints[i][insertCounter] = newPoint;
			}
		}


		for (int i = 0; i < increaseMultiplicity; i++)
			newKnotVector.insert(i, newKnotVector[i]);

		// Fill new control points.
		QPolygonF newBoorNetPoints;
		for (int i = 0; i <= knotCounter - curveDegree; i++)
			newBoorNetPoints.push_back(m_boorNetPoints[i]);

		for (int i = 1; i <= increaseMultiplicity; ++i)
		{
			QPointF &newP = tempPoints[knotCounter - curveDegree + i][i];
			newBoorNetPoints.push_back(newP);
		}

		for (int i = -curveDegree + increaseMultiplicity + 1; i <= -1; ++i)
		{
			QPointF &newP = tempPoints[knotCounter + i][increaseMultiplicity];
			newBoorNetPoints.push_back(newP);
		}

		for (int i = increaseMultiplicity - 1; i >= 1; --i)
			newBoorNetPoints.push_back(tempPoints[knotCounter - 1][i]);

		for (int i = knotCounter - 1; i < m_boorNetPoints.size(); ++i)
			newBoorNetPoints.push_back(m_boorNetPoints[i]);

		m_boorNetPoints = newBoorNetPoints;
	}
}


void BSpline::interpolateBezier(double x1, double y1,double x2, double y2,double x3, double y3,double x4, double y4,unsigned level)
{
	const unsigned curveRecursionLimit = 32;
	const double curveCollinearityEpsilon = 1e-30;
	const double curveangleToleranceEpsilon = 0.01;
	const double angleTolerance = 0.0;
	const double cuspLimit = 0.0;

	if(level > curveRecursionLimit) {
		return;
	}

	// Calculate all the mid-points of the line segments
	double x12   = (x1 + x2) / 2;
	double y12   = (y1 + y2) / 2;
	double x23   = (x2 + x3) / 2;
	double y23   = (y2 + y3) / 2;
	double x34   = (x3 + x4) / 2;
	double y34   = (y3 + y4) / 2;
	double x123  = (x12 + x23) / 2;
	double y123  = (y12 + y23) / 2;
	double x234  = (x23 + x34) / 2;
	double y234  = (y23 + y34) / 2;
	double x1234 = (x123 + x234) / 2;
	double y1234 = (y123 + y234) / 2;

	if(level > 0) {
	// Enforce subdivision first time

	// Try to approximate the full cubic curve by a single straight line
	double dx = x4-x1;
	double dy = y4-y1;

	double d2 = qAbs(((x2 - x4) * dy - (y2 - y4) * dx));
	double d3 = qAbs(((x3 - x4) * dy - (y3 - y4) * dx));

	double da1, da2;

	if(d2 > curveCollinearityEpsilon && d3 > curveCollinearityEpsilon) {
		// Regular care
		if((d2 + d3)*(d2 + d3) <= m_distanceTolerance * (dx*dx + dy*dy)) {
		// If the curvature doesn't exceed the distance_tolerance value
		// we tend to finish subdivisions.
		if(angleTolerance < curveangleToleranceEpsilon) {
			m_interpolatedPoints->push_back(QPointF(x1234, y1234));
			return;
		}

		// Angle & Cusp Condition
		double a23 = qAtan2(y3 - y2, x3 - x2);
		da1 = fabs(a23 - qAtan2(y2 - y1, x2 - x1));
		da2 = fabs(qAtan2(y4 - y3, x4 - x3) - a23);
		if(da1 >= M_PI) da1 = 2*M_PI - da1;
		if(da2 >= M_PI) da2 = 2*M_PI - da2;

		if(da1 + da2 < angleTolerance) {
			// Finally we can stop the recursion
			m_interpolatedPoints->push_back(QPointF(x1234, y1234));
			return;
		}

		if(cuspLimit != 0.0) {
			if(da1 > cuspLimit) {
			m_interpolatedPoints->push_back(QPointF(x2, y2));
			return;
			}

			if(da2 > cuspLimit) {
			m_interpolatedPoints->push_back(QPointF(x3, y3));
			return;
			}
		}
		}
	} else {
		if(d2 > curveCollinearityEpsilon) {
		// p1,p3,p4 are collinear, p2 is considerable
		if(d2 * d2 <= m_distanceTolerance * (dx*dx + dy*dy)) {
			if(angleTolerance < curveangleToleranceEpsilon) {
			m_interpolatedPoints->push_back(QPointF(x1234, y1234));
			return;
			}

			// Angle Condition
			da1 = fabs(qAtan2(y3 - y2, x3 - x2) - qAtan2(y2 - y1, x2 - x1));
			if(da1 >= M_PI)
			da1 = 2*M_PI - da1;

			if(da1 < angleTolerance) {
			m_interpolatedPoints->push_back(QPointF(x2, y2));
			m_interpolatedPoints->push_back(QPointF(x3, y3));
			return;
			}

			if(cuspLimit != 0.0) {
			if(da1 > cuspLimit) {
				m_interpolatedPoints->push_back(QPointF(x2, y2));
				return;
			}
			}
		}
		} else if(d3 > curveCollinearityEpsilon) {
		// p1,p2,p4 are collinear, p3 is considerable
		if(d3 * d3 <= m_distanceTolerance * (dx*dx + dy*dy)) {
			if(angleTolerance < curveangleToleranceEpsilon) {
			m_interpolatedPoints->push_back(QPointF(x1234, y1234));
			return;
			}

			// Angle Condition
			da1 = fabs(qAtan2(y4 - y3, x4 - x3) - qAtan2(y3 - y2, x3 - x2));
			if(da1 >= M_PI) da1 = 2*M_PI - da1;

			if(da1 < angleTolerance) {
			m_interpolatedPoints->push_back(QPointF(x2, y2));
			m_interpolatedPoints->push_back(QPointF(x3, y3));
			return;
			}

			if(cuspLimit != 0.0) {
			if(da1 > cuspLimit) {
				m_interpolatedPoints->push_back(QPointF(x3, y3));
				return;
			}
			}
		}
		} else {
		// Collinear case
		dx = x1234 - (x1 + x4) / 2;
		dy = y1234 - (y1 + y4) / 2;
		if(dx*dx + dy*dy <= m_distanceTolerance) {
			m_interpolatedPoints->push_back(QPointF(x1234, y1234));
			return;
		}
		}
	}
	}

	// Continue subdivision
	interpolateBezier(x1, y1, x12, y12, x123, y123, x1234, y1234,level + 1);
	interpolateBezier(x1234, y1234, x234, y234, x34, y34, x4, y4,level + 1);

}

void BSpline::fillKnotVector()
{

	int middleKnotNumber = m_controlPoints->size() - 4;
	m_knotVector.clear();
	for(int i = 0; i < 4; i++)
		m_knotVector.push_back(0.0);

	for(int i = 1; i <= middleKnotNumber; i++)
		m_knotVector.push_back(1.0 / (middleKnotNumber + 1) * i);

	for(int i = 0; i < 4; i++)
		m_knotVector.push_back(1.0);
}



}//NS
