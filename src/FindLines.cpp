#include "FindLines.h"

#define PI 3.14159265

FindLines::FindLines(void)
{
    writeFileInit();
    readFile();
    oversize=-1;
    line_length=110;
    img_midline=((320,1),(320,480));
}

FindLines::~FindLines(void)
{

}

void FindLines::lineMain(Mat input_img,Mat out_img)
{
    //Mat pro_img;
    //pro_img = Mat::zeros((Size(input_img.rows, input_img.cols), CV_8UC1);
    // cout<<"processedimg"<<endl;
    input_img.copyTo(img);
    cvtColor(input_img,input_img,COLOR_BGR2HSV);
    input_img.copyTo(hsv_img);
    
    /*滑动条*/
    namedWindow("colorpara",WINDOW_NORMAL);
    getTheParameter("colorpara");
    imshow("colorpara",hsv_img);
    waitKey(10);
    //预处理后单通道
    preProcessImg(input_img);

    //获得应走的中线
    midLine(input_img);

    getAngle(input_img);
    // cout<<"processedimg"<<endl;
    //应走中线与图像中线之间的距离
    getDis(input_img);    
    namedWindow("processedimg",1);
    imshow("processedimg",input_img);
    waitKey(10);
    namedWindow("img",1);
    imshow("img",img);
    waitKey(10);
}

/*
初步的想法是图像中心与首尾两个点过所围凸集的中心的距离,以此调整与线的偏移
*/

void FindLines::getDis(Mat& input_img)
{
   //得到过最远点的切线
   StandardLine stdmidline(midline[0],midline[1]);
   dis=stdmidline.getDisToPoint(Point(input_img.cols/2,input_img.rows/2));
}

void FindLines::getAngle(Mat& input_img)
{
 //angle=atan(sin/cos);
 //line(img,midline[0],midline[1],Scalar(0,0,255),5,LINE_8,0);
 StandardLine stdmidline(midline[0],midline[1]);
 line(img,stdmidline.begin_point,stdmidline.end_point,Scalar(255,0,0),5,LINE_8,0);
 line(input_img,midline[0],midline[1],Scalar(255,255,255),10,LINE_8,0);
 if(stdmidline.slope > 0){
   angle=atan((320-stdmidline.begin_point.x) / (480-(stdmidline.slope*320+stdmidline.intercept)));
 }
 else{
   angle=atan((320-stdmidline.begin_point.x) / ((stdmidline.slope*320+stdmidline.intercept)));
 }
 if(stdmidline.slope==-0){
     angle = 0.0;
 }
}

double FindLines::Angle(){
    //cout<<"angle"<<angle<<endl;
    return angle;
}

double FindLines::Dis()
{
    //cout<<"dis"<<dis<<endl;
    return dis;
}

bool FindLines::Flag(){
   return flag_line;
}

void FindLines::writeFileInit()
{
   FileStorage write_file("FindLines.xml",FileStorage::WRITE);

   write_file<<"red_high"<<0;
   write_file<<"green_high"<<0;
   write_file<<"blue_high"<<0;
   write_file<<"red_low"<<84;
   write_file<<"green_low"<<104;
   write_file<<"blue_low"<<0;

   write_file.release();

}


void FindLines::writeFile()
{
   FileStorage write_file("FindLines.xml",FileStorage::WRITE);

   write_file<<"red_high"<<red_high;
   write_file<<"green_high"<<green_high;
   write_file<<"blue_high"<<blue_high;
   write_file<<"red_low"<<red_low;
   write_file<<"green_low"<<green_low;
   write_file<<"blue_low"<<blue_low;
   
   write_file.release();

}

void FindLines::readFile()
{
    FileStorage read_file;
    read_file.open("FindLines.xml",FileStorage::READ);
    
	if( !read_file.isOpened() ) // failed
    {
       cout<<"Save File Failed!"<<endl;
       return ;
    }

    read_file["red_high"] >> red_high;
    read_file["blue_high"] >> blue_high;
	  read_file["green_high"] >> green_high;
    read_file["red_low"] >> red_low;
    read_file["blue_low"] >> blue_low;
	  read_file["green_low"] >> green_low;

    read_file.release();
}

vector<Point> FindLines::Vec2f2point(cv::Vec2f input_data)
{
  float rho = input_data[0], theta = input_data[1];
  Point pt1, pt2;
  double a = cos(theta), b = sin(theta);
  double x0 = a*rho, y0 = b*rho;
  pt1.x = cvRound(x0 + 1000*(-b));
  pt1.y = cvRound(y0 + 1000*(a));
  pt2.x = cvRound(x0 - 1000*(-b));
  pt2.y = cvRound(y0 - 1000*(a));
  vector<Point> pt;
  pt.push_back(pt1);
  pt.push_back(pt2);
  return pt;
}

void FindLines::midLine(Mat& input_img)
{
   vector<cv::Vec2f> lines;
   vector<cv::Vec2f> res_lines;
   midline.clear();
   HoughLines(input_img,lines,1,PI/180,line_length,0,0);
   vector<cv::Vec2f>::iterator it;
   
   for(it=lines.begin();it!=lines.end();it++){
      vector<Point> cur_line=Vec2f2point(*it);
      //line(input_img,cur_line[0],cur_line[1],Scalar(255,0,0),1,LINE_8,0);
        double slope = ((cur_line[0].y - cur_line[1].y) / ((cur_line[0].x - cur_line[1].x)==0?100:(cur_line[0].x - cur_line[1].x)));

        if(fabs(slope)<0.2){
           
        }
        else{
          res_lines.push_back(*it);
          line(img,cur_line[0],cur_line[1],Scalar(0,255,0),1,LINE_8,0);
        }
   }
   lines=res_lines;
   if(lines.size()==0){
       flag_line=false;
       //处理图像得到midline
      //  Point left_point=Point(input_img.cols/2,input_img.rows/2),right_point=Point(input_img.cols/2,input_img.rows/2);
      //  for(int i=0;i<input_img.rows;i++)
      //  {
      //    for(int j=0;j<input_img.cols;j++)
      //    {
      //       if(input_img.at<uchar>(i,j)==255){
      //           if(i<left_point.y){
      //               left_point.x=j;
      //               left_point.y=i;
      //           }
      //           if(i>right_point.y){
      //               right_point.x=j;
      //               right_point.y=i;
      //           }
      //       }
      //    }
      //  }
      //midline.push_back(left_point);
      //midline.push_back(right_point);
      //midline.push_back(p1);
      Point left_point = Point(input_img.cols,0);
      Point right_point = Point(0,0);
      for(int i=0;i<input_img.rows;i++)
       {
         for(int j=0;j<input_img.cols;j++)
         {
            if(input_img.at<uchar>(i,j)==255){
                if(j<left_point.x){
                    left_point.x=j;
                    left_point.y=i;
                }
                if(j>right_point.x){
                    right_point.x=j;
                    right_point.y=i;
                }
            }
         }
       }
       midline.push_back(left_point);
       midline.push_back(right_point);

       circle(img,left_point,20,Scalar(255,255,0),-1);
       circle(img,right_point,20,Scalar(255,0,255),-1);
   }
   else{
      flag_line=true;
      //vector<cv::Vec2f>::iterator it;
      // for(it=lines.begin();it!=lines.end();it++){
      //   vector<Point> cur_line=Vec2f2point(*it);
      //   //line(input_img,cur_line[0],cur_line[1],Scalar(255,0,0),1,LINE_8,0);
      //   //StandardLine stdcurline(cur_line[0],cur_line[1]);
      //   double slope = ((cur_line[0].y - cur_line[1].y) / ((cur_line[0].x - cur_line[1].x)==0?100:(cur_line[0].x - cur_line[1].x)));

      //   if(-0.2<slope<0.2){
      //       //lines.erase(it);
      //   }
      //   else{
      //     line(img,cur_line[0],cur_line[1],Scalar(0,255,0),1,LINE_8,0);
      //   }
        
      // }
      try{
        judgeSize(lines);
      }
      catch(int oversize)
      {
          // cout<<oversize<<"lines oversize!!!"<<endl; 
      }
   }
  //  cout<<midline[0].x<<midline[0].y<<endl;
  //  cout<<midline[1].x<<midline[1].y<<endl;
   
}

void FindLines::judgeSize(vector<cv::Vec2f> lines)
{
  if(lines.size()==2)
  {
    vector<Point> points1=Vec2f2point(lines[0]);
    vector<Point> points2=Vec2f2point(lines[1]);
    midline[0].x=(points1[0].x+points2[0].x)/2;
    midline[0].y=(points1[0].y+points2[0].y)/2;
    midline[1].x=(points1[1].x+points2[1].x)/2;
    midline[1].y=(points1[1].y+points2[1].y)/2;
  }
  else if(lines.size()==1) {
    vector<Point> points=Vec2f2point(lines[0]);
    midline[0].x=(points[0].x)/1.;
    midline[0].y=(points[0].y)/1.;
    midline[1].x=(points[1].x)/1.;
    midline[1].y=(points[1].y)/1.;
  }
  if(lines.size()>2) {
    vector<Point> points;
    points.push_back(Point(0,0));
    points.push_back(Point(0,0));
    vector<cv::Vec2f>::iterator it;
    int num=0;
    for(it=lines.begin();it!=lines.end();it++){
        num+=1;
        vector<Point> cur_line=Vec2f2point(*it);
        points[0].x+=cur_line[0].x;
        points[0].y+=cur_line[0].y;
        points[1].x+=cur_line[1].x;
        points[1].y+=cur_line[1].y;
    }
    midline[0].x=(points[0].x)/num;
    midline[0].y=(points[0].y)/num;
    midline[1].x=(points[1].x)/num;
    midline[1].y=(points[1].y)/num;
    throw oversize;
  }
}

/*
input:rgb image;
output:gray_image which contain line
*/
void FindLines::preProcessImg(Mat& input_img)
{
    Mat result_img = Mat::zeros(Size(input_img.cols, input_img.rows), CV_8UC3);
    inRange(input_img,Scalar(red_low,green_low,blue_low),Scalar(red_high,green_high,blue_high),input_img);
    int g_nStructElementSize = 3; //结构元素(内核矩阵)的尺寸
    //获取自定义核
    Mat element = getStructuringElement(MORPH_RECT,Size(2*g_nStructElementSize+1,2*g_nStructElementSize+1));
    //(input_img,input_img,element);
    // erode(input_img,input_img, element);
    Rect maxRect = Rect(0,0,480,640);
    maxRect = boundingRect(input_img);
    //绘制外包矩阵
    rectangle(img, maxRect,Scalar(0, 255, 255));
    for(int i=0;i<input_img.rows;i++)
    {
         for(int j=0;j<input_img.cols;j++)
         {
            if(input_img.at<uchar>(i,j)==255 &&(!maxRect.contains(Point(j,i)))){
                input_img.at<uchar>(i,j) = 0;
            }
         }
    }
    //rectangle(input_img,maxRect,Scalar(255,255,255));
    namedWindow("line_img",WINDOW_AUTOSIZE);
    imshow("line_img",input_img);
    waitKey(10);
    Canny(input_img,input_img,50,100,3,false);
}

bool FindLines::inRect(Rect maxRect,Point p){
  Point p1 = maxRect.tl();
	//Point p2 = Point(maxRect.tl().x,maxRect.br().y);
	Point p3 = maxRect.br();
	//Point p4 = Point(maxRect.br().x,maxRect.tl().y);
  // return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

float FindLines::GetCross(Point& p1, Point& p2,Point& p)
{
	return (p2.x - p1.x) * (p.y - p1.y) -(p.x - p1.x) * (p2.y - p1.y);
}
void FindLines::getTheParameter(string _window_name)
{
   
    createTrackbar("redl change",_window_name,&red_low,180,0,0);
    createTrackbar("greenl change",_window_name,&green_low,255,0,0);
    createTrackbar("bluel change",_window_name,&blue_low,255,0,0);
    createTrackbar("redh change",_window_name,&red_high,180,0,0);
    createTrackbar("greenh change",_window_name,&green_high,255,0,0);
    createTrackbar("blueh change",_window_name,&blue_high,255,0,0);
    createTrackbar("llen change",_window_name,&line_length,150,0,0);
    writeFile();

}

