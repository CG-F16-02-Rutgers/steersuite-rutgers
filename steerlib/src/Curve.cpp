//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
// Copyright (c) 2015 Mahyar Khayatkhoei
//

#include <algorithm>
#include <vector>
#include <util/Geometry.h>
#include <util/Curve.h>
#include <util/Color.h>
#include <util/DrawLib.h>
#include "Globals.h"

using namespace Util;

Curve::Curve(const CurvePoint& startPoint, int curveType) : type(curveType)
{
	controlPoints.push_back(startPoint);
}

Curve::Curve(const std::vector<CurvePoint>& inputPoints, int curveType) : type(curveType)
{
	controlPoints = inputPoints;
	sortControlPoints();
}

// Add one control point to the vector controlPoints
void Curve::addControlPoint(const CurvePoint& inputPoint)
{
	controlPoints.push_back(inputPoint);
	sortControlPoints();
}

// Add a vector of control points to the vector controlPoints
void Curve::addControlPoints(const std::vector<CurvePoint>& inputPoints)
{
	for (int i = 0; i < inputPoints.size(); i++)
		controlPoints.push_back(inputPoints[i]);
	sortControlPoints();
}

// Draw the curve shape on screen, usign window as step size (bigger window: less accurate shape)
void Curve::drawCurve(Color curveColor, float curveThickness, int window)
{
#ifdef ENABLE_GUI

	/*
	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
	std::cerr << "ERROR>>>>Member function drawCurve is not implemented!" << std::endl;
	flag = true;
	}
	//=========================================================================
	*/
	if (!checkRobust()) {
		std::cerr << "There should be at least two control points" << std::endl;
		return;
	}
	if (type == hermiteCurve) {
		for (int i = 0;i < controlPoints.size() - 1;i++) {

			Point startPoint = controlPoints[i].position;
			Point endPoint = startPoint;
			int interval = controlPoints[i + 1].time - controlPoints[i].time;
			for (float t = 0; t < 1; t += 0.005*window) {
				float x = controlPoints[i].position.x *(2 * t*t*t - 3 * t*t + 1) + controlPoints[i + 1].position.x*((-2)*t*t*t + 3 * t*t) + controlPoints[i].tangent.x*(t*t*t - 2 * t*t + t)*interval + controlPoints[i + 1].tangent.x*(t*t*t - t*t)*interval;
				float y = controlPoints[i].position.y *(2 * t*t*t - 3 * t*t + 1) + controlPoints[i + 1].position.y*((-2)*t*t*t + 3 * t*t) + controlPoints[i].tangent.y*(t*t*t - 2 * t*t + t)*interval + controlPoints[i + 1].tangent.y*(t*t*t - t*t)*interval;
				float z = controlPoints[i].position.z *(2 * t*t*t - 3 * t*t + 1) + controlPoints[i + 1].position.z*((-2)*t*t*t + 3 * t*t) + controlPoints[i].tangent.z*(t*t*t - 2 * t*t + t)*interval + controlPoints[i + 1].tangent.z*(t*t*t - t*t)*interval;
				endPoint = Point(x, y, z);
				DrawLib::drawLine(startPoint, endPoint, curveColor, curveThickness);
				startPoint = endPoint;
			}
		}
		return;
	}
	else if (type == catmullCurve) {
		for (int i = 0;i < controlPoints.size() - 1;i++) {

			Point startPoint = controlPoints[i].position;
			Point endPoint = startPoint;
			Point startTagent = calculateTangent(i);
			Point endTangent = calculateTangent(i + 1);
			int interval = controlPoints[i + 1].time - controlPoints[i].time;
			for (float t = 0; t < 1; t += 0.005*window) {
				float x = controlPoints[i].position.x *(2 * t*t*t - 3 * t*t + 1) + controlPoints[i + 1].position.x*((-2)*t*t*t + 3 * t*t) + startTagent.x*(t*t*t - 2 * t*t + t)*interval + endTangent.x*(t*t*t - t*t)*interval;
				float y = controlPoints[i].position.y *(2 * t*t*t - 3 * t*t + 1) + controlPoints[i + 1].position.y*((-2)*t*t*t + 3 * t*t) + startTagent.y*(t*t*t - 2 * t*t + t)*interval + endTangent.y*(t*t*t - t*t)*interval;
				float z = controlPoints[i].position.z *(2 * t*t*t - 3 * t*t + 1) + controlPoints[i + 1].position.z*((-2)*t*t*t + 3 * t*t) + startTagent.z*(t*t*t - 2 * t*t + t)*interval + endTangent.z*(t*t*t - t*t)*interval;
				endPoint = Point(x, y, z);
				DrawLib::drawLine(startPoint, endPoint, curveColor, curveThickness);
				startPoint = endPoint;
			}
		}
		return;
	}
	// Robustness: make sure there is at least two control point: start and end points
	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	// Note that you must draw the whole curve at each frame, that means connecting line segments between each two points on the curve

#endif
}

// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{
	/*
	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
	std::cerr << "ERROR>>>>Member function sortControlPoints is not implemented!" << std::endl;
	flag = true;
	}
	//=========================================================================
	*/
	int size = controlPoints.size();
	while (size--) {
		for (int i = 0;i < size;i++) {
			if (controlPoints[i].time > controlPoints[i + 1].time) {
				std::swap(controlPoints[i], controlPoints[i + 1]);
			}
		}
	}
	for (int i = 0; i < controlPoints.size() - 1;i++) {
		if (controlPoints[i].time == controlPoints[i + 1].time)
			controlPoints.erase(controlPoints.begin() + i +1);
	}
	return;
}

// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
// Note that this function should return false if the end of the curve is reached, or no next point can be found
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
		return false;

	// Define temporary parameters for calculation
	unsigned int nextPoint;

	// Find the current interval in time, supposing that controlPoints is sorted (sorting is done whenever control points are added)
	// Note that nextPoint is an integer containing the index of the next control point
	if (!findTimeInterval(nextPoint, time))
		return false;
	// Calculate position at t = time on curve given the next control point (nextPoint)
	if (type == hermiteCurve)
	{
		outputPoint = useHermiteCurve(nextPoint, time);
	}
	else if (type == catmullCurve)
	{
		outputPoint = useCatmullCurve(nextPoint, time);
	}

	// Return
	return true;
}

// Check Roboustness
bool Curve::checkRobust()
{
	/*
	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
	std::cerr << "ERROR>>>>Member function checkRobust is not implemented!" << std::endl;
	flag = true;
	}
	//=========================================================================
	*/
	if (type == hermiteCurve) {
		if (controlPoints.size() >= 2)
			return true;
		else
			return false;
	}
	else if (type == catmullCurve) {
		if (controlPoints.size() >= 3)
			return true;
		else
			return false;
	}
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
	/*
	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
	std::cerr << "ERROR>>>>Member function findTimeInterval is not implemented!" << std::endl;
	flag = true;
	}
	//=========================================================================
	*/
	for (int i = 0;i < controlPoints.size();i++) {
		if (controlPoints[i].time > time) {
			nextPoint = i;
			return true;
		}
	}
	return false;
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{

	Point newPosition;
	float normalTime, interval;
	/*
	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function useHermiteCurve is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================

	// Calculate position at t = time on Hermite curve
	*/
	interval = controlPoints[nextPoint].time - controlPoints[nextPoint - 1].time;
	normalTime = (time - controlPoints[nextPoint - 1].time) / interval;
	float t = normalTime;

	float x = controlPoints[nextPoint - 1].position.x *(2 * t*t*t - 3 * t*t + 1) + controlPoints[nextPoint].position.x*((-2)*t*t*t + 3 * t*t) + controlPoints[nextPoint-1].tangent.x*(t*t*t - 2 * t*t + t)*interval + controlPoints[nextPoint].tangent.x*(t*t*t - t*t)*interval;
	float y = controlPoints[nextPoint - 1].position.y *(2 * t*t*t - 3 * t*t + 1) + controlPoints[nextPoint].position.y*((-2)*t*t*t + 3 * t*t) + controlPoints[nextPoint-1].tangent.y*(t*t*t - 2 * t*t + t)*interval + controlPoints[nextPoint].tangent.y*(t*t*t - t*t)*interval;
	float z = controlPoints[nextPoint - 1].position.z *(2 * t*t*t - 3 * t*t + 1) + controlPoints[nextPoint].position.z*((-2)*t*t*t + 3 * t*t) + controlPoints[nextPoint-1].tangent.z*(t*t*t - 2 * t*t + t)*interval + controlPoints[nextPoint].tangent.z*(t*t*t - t*t)*interval;
	newPosition = Point(x, y, z);

	// Return result
	return newPosition;

}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;

	/*
	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function useCatmullCurve is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================

	// Calculate position at t = time on Catmull-Rom curve
	*/
	//Point startPoint = controlPoints[i].position;
	//Point endPoint = startPoint;
	Point startTagent = calculateTangent(nextPoint - 1);
	Point endTangent = calculateTangent(nextPoint);
	float interval = controlPoints[nextPoint].time - controlPoints[nextPoint - 1].time;
	float normalTime = (time - controlPoints[nextPoint - 1].time) / interval;
	float t = normalTime;

	float x = controlPoints[nextPoint - 1].position.x *(2 * t*t*t - 3 * t*t + 1) + controlPoints[nextPoint].position.x*((-2)*t*t*t + 3 * t*t) + startTagent.x*(t*t*t - 2 * t*t + t)*interval + endTangent.x*(t*t*t - t*t)*interval;
	float y = controlPoints[nextPoint - 1].position.y *(2 * t*t*t - 3 * t*t + 1) + controlPoints[nextPoint].position.y*((-2)*t*t*t + 3 * t*t) + startTagent.y*(t*t*t - 2 * t*t + t)*interval + endTangent.y*(t*t*t - t*t)*interval;
	float z = controlPoints[nextPoint - 1].position.z *(2 * t*t*t - 3 * t*t + 1) + controlPoints[nextPoint].position.z*((-2)*t*t*t + 3 * t*t) + startTagent.z*(t*t*t - 2 * t*t + t)*interval + endTangent.z*(t*t*t - t*t)*interval;
	newPosition = Point(x, y, z);

	// Return result
	return newPosition;
}

Point Curve::calculateTangent(int index) 
{	
	Point tangent;
	if (index == 0) {
		float t0 = controlPoints[0].time;
		float t1 = controlPoints[1].time;
		float t2 = controlPoints[2].time;
		Point p0 = controlPoints[0].position;
		Point p1 = controlPoints[1].position;
		Point p2 = controlPoints[2].position;
		tangent.x = (((t2 - t0) / (t2 - t1))*((p1.x - p0.x) / (t1 - t0))) - (((t1 - t0) / (t2 - t1))*((p2.x - p0.x) / (t2 - t0)));
		tangent.y = (((t2 - t0) / (t2 - t1))*((p1.y - p0.y) / (t1 - t0))) - (((t1 - t0) / (t2 - t1))*((p2.y - p0.y) / (t2 - t0)));
		tangent.z = (((t2 - t0) / (t2 - t1))*((p1.z - p0.z) / (t1 - t0))) - (((t1 - t0) / (t2 - t1))*((p2.z - p0.z) / (t2 - t0)));
	}
	else if (index == controlPoints.size() - 1) 
	{
		float t0 = controlPoints[index-2].time;
		float t1 = controlPoints[index-1].time;
		float t2 = controlPoints[index].time;
		Point p0 = controlPoints[index-2].position;
		Point p1 = controlPoints[index-1].position;
		Point p2 = controlPoints[index].position;
		tangent.x = (((t2 - t0) / (t1 - t0))*((p2.x - p1.x) / (t2 - t1))) - (((t2 - t1) / (t1 - t0))*((p2.x - p0.x) / (t2 - t0)));
		tangent.y = (((t2 - t0) / (t1 - t0))*((p2.y - p1.y) / (t2 - t1))) - (((t2 - t1) / (t1 - t0))*((p2.y - p0.y) / (t2 - t0)));
		tangent.z = (((t2 - t0) / (t1 - t0))*((p2.z - p1.z) / (t2 - t1))) - (((t2 - t1) / (t1 - t0))*((p2.z - p0.z) / (t2 - t0)));
	}
	else {
		float t0 = controlPoints[index - 1].time;
		float t1 = controlPoints[index].time;
		float t2 = controlPoints[index + 1].time;
		Point p0 = controlPoints[index - 1].position;
		Point p1 = controlPoints[index].position;
		Point p2 = controlPoints[index + 1].position;
		tangent.x = ((t1 - t0) / (t2 - t0))*((p2.x - p1.x) / (t2 - t1)) + ((t2 - t1) / (t2 - t0))*((p1.x - p0.x) / (t1 - t0));
		tangent.y = ((t1 - t0) / (t2 - t0))*((p2.y - p1.y) / (t2 - t1)) + ((t2 - t1) / (t2 - t0))*((p1.y - p0.y) / (t1 - t0));
		tangent.z = ((t1 - t0) / (t2 - t0))*((p2.z - p1.z) / (t2 - t1)) + ((t2 - t1) / (t2 - t0))*((p1.z - p0.z) / (t1 - t0));
	}
	return 
		tangent;
}