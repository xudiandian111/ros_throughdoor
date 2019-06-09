#ifndef FIND_CIRCLE_H_
#define FIND_CIRCLE_H_

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;
using namespace cv;

class FindCircle{
public:
 FindCircle();
 ~FindCircle();
 void circleMain(Mat input_img);
 bool getFlag();
 Point getPoint();
private:
 void keyPoint(Mat &input_img);
 void writeFileInit();
 void writeFile();
 void readFile();
 void preProcessImg(Mat &input_img);
 void getTheParameter(string _window_name);
 bool circleLeastFit(vector<Point> &m_Points);
private:
 int red_low;
 int green_low;
 int blue_low;
 int red_high;
 int green_high;
 int blue_high;
 bool flag_circle;
 Point res_circle;
 double height;
 int thre;
 vector<Point> points;
};

#endif