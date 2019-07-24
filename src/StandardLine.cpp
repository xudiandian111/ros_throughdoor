#include "StandardLine.h"

StandardLine::StandardLine()
{
   
}

StandardLine::StandardLine(Point _first_point,Point _second_point)
{
	initSlopeIntercept(_first_point, _second_point);
}

StandardLine::~StandardLine()
{

}

void StandardLine::initSlopeIntercept(double _input_slope, double _input_intercept)
{
	slope = _input_slope;
	intercept = _input_intercept; 
	standardPoint();
}
void StandardLine::initSlopeIntercept(Point _first_point, Point _second_point)
{
	begin_point = _first_point;
	end_point = _second_point;
	if (abs(begin_point.x - end_point.x) <= 1)
	{
		slope = max_slope;
	}
	else
	{
		slope = ((begin_point.y - end_point.y)*1.0 / (begin_point.x - end_point.x));
	}

	intercept = begin_point.y - slope * begin_point.x;
	standardPoint();
}

void StandardLine::standardPoint()
{
	begin_point.y = 1;
	begin_point.x = (int)((1 - intercept) / slope);
	end_point.y = 480;
	end_point.x = (int)((480 - intercept) / slope);

}

double StandardLine::getDisToPoint(Point _input_point)
{
	double temp=fabs(slope*_input_point.x - 1.0 * _input_point.y + intercept) / sqrt(slope*slope + 1*1);
	
	if((slope > 0 && _input_point.y > (_input_point.x*slope+intercept)) || (slope < 0 && _input_point.y < (_input_point.x*slope+intercept))){
        temp=-temp;
	}
	return temp;
    
	// std::cout<<"slope "<<slope<<std::endl;
	// std::cout<<"intercept "<<intercept<<std::endl;

}

void StandardLine::setLine(double _input_slope, double _input_intercept)
{
	slope = _input_slope;
	intercept = _input_intercept;
	standardPoint();
}
void StandardLine::setLine(Point _input_fir_point, Point _input_sec_point)
{
	begin_point = _input_fir_point;
	end_point = _input_sec_point;
	if (abs(begin_point.x - end_point.x) <= 1)
	{
		slope = max_slope;
	}
	else
	{
		slope = ((_input_fir_point.y - _input_sec_point.y)*1.0 / (_input_fir_point.x - _input_sec_point.x));
	}

	intercept = _input_fir_point.y - slope * _input_fir_point.x;
	standardPoint();

}