#pragma once

#ifndef STANDARDLINE_H_
#define STANDARDLINE_H_

#include <opencv2/opencv.hpp>

using namespace cv;

const int max_slope = 1000;
const double kp = 0.1;
const double ki=0.001;	

class StandardLine {
public:
	Point begin_point, end_point;
	double slope;
	double intercept;
public:
	StandardLine();
	StandardLine(Point _first_point, Point _second_point);
	~StandardLine();
	void initSlopeIntercept(Point _first_point,Point _second_point);
	void initSlopeIntercept(double _input_slope, double _input_intercept);
	double getDisToPoint(Point _input_point);
	int findEndX(void) 
	{
		return end_point.x;
	}
	void setLine(double _input_slope, double _input_intercept);
	void setLine(Point x, Point y);
private:
	



private:
	void standardPoint();

	friend bool operator <(const StandardLine &_first_point, const StandardLine &_second_point)
	{
		return _first_point.end_point.x < _second_point.end_point.x;
	}
	friend bool operator >(const StandardLine &_first_point, const StandardLine &_second_point)
	{
		return _first_point.end_point.x > _second_point.end_point.x;
	}
	friend bool operator ==(const StandardLine &_first_point, const StandardLine &_second_point)
	{
		return _first_point.end_point.x == _second_point.end_point.x;
	}
};
#endif // !STANDARDLINE_H_
