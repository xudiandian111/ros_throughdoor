#ifndef FIND_LINES_H_
#define FIND_LINES_H_

#include <opencv2/opencv.hpp>
#include "StandardLine.h"
#include <cmath>
#include <string>
#include <vector>

using namespace std;
using namespace cv;


class FindLines {
 public:
   FindLines(void);
   ~FindLines(void);
   void lineMain(Mat input_img,Mat out_img);
   double Angle();
   double Dis();
   bool Flag();
 private:
   void preProcessImg(Mat& input_img);
   void getAngle(Mat& input_img);
   void getDis(Mat& input_img);
   /*
   read and write the file which contain the paremeters
   */
   void writeFileInit();
   void writeFile();
   void readFile();
   vector<Point> Vec2f2point(cv::Vec2f input_data);
   void midLine(Mat& input_img);
   void judgeSize(vector<cv::Vec2f> lines);
   void getTheParameter(string _window_name);
   //static void onTrackbar(int,void* );
   bool inRect(Rect maxRect,Point a);
   float GetCross(Point& p1, Point& p2,Point& p);
 private:
   int oversize;
   bool flag_line;
   double angle;
   double dis;
   vector<Point> midline;
   cv::Vec2f img_midline;
   int red_high;
   int green_high;
   int blue_high;
   int red_low;
   int green_low;
   int blue_low;
   Mat hsv_img;
   Mat img;
   int line_length;
   Point p1,p3;
};

#endif // !FIND_LINES_H_