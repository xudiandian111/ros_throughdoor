#include "FindCircle.h"

FindCircle::FindCircle(){
    writeFileInit();
    readFile();
    flag_circle=false;
    thre=50;
}

FindCircle::~FindCircle(){

}

bool compare(Vec3f a, Vec3f b){
 return (a[2]<b[2]);
}

void FindCircle::circleMain(Mat input_img)
{
    Mat img;
    cvtColor(input_img,input_img,COLOR_BGR2HSV);
    input_img.copyTo(img);
    /*滑动条*/
    namedWindow("colorpara_c",WINDOW_NORMAL);
    getTheParameter("colorpara_c");
    imshow("colorpara_c",img);
    waitKey(10);
    //预处理后单通道
    preProcessImg(input_img);
    keyPoint(input_img);  
    namedWindow("processedimg_c",1);
    imshow("processedimg_c",input_img);
    waitKey(10);
}

bool FindCircle::circleLeastFit(vector<Point> &m_Points)
{
	if (!m_Points.empty())
	{
		int iNum = (int)m_Points.size();
		if (iNum < 3)	return false;
		double X1 = 0.0;
		double Y1 = 0.0;
		double X2 = 0.0;
		double Y2 = 0.0;
		double X3 = 0.0;
		double Y3 = 0.0;
		double X1Y1 = 0.0;
		double X1Y2 = 0.0;
		double X2Y1 = 0.0;
		vector<Point>::iterator iter;
		vector<Point>::iterator end = m_Points.end();
		for (iter = m_Points.begin(); iter != end; ++iter)
		{
			X1 = X1 + (*iter).x;
			Y1 = Y1 + (*iter).y;
			X2 = X2 + (*iter).x * (*iter).x;
			Y2 = Y2 + (*iter).y * (*iter).y;
			X3 = X3 + (*iter).x * (*iter).x * (*iter).x;
			Y3 = Y3 + (*iter).y * (*iter).y * (*iter).y;
			X1Y1 = X1Y1 + (*iter).x * (*iter).y;
			X1Y2 = X1Y2 + (*iter).x * (*iter).y * (*iter).y;
			X2Y1 = X2Y1 + (*iter).x * (*iter).x * (*iter).y;
		}
		double C = 0.0;
		double D = 0.0;
		double E = 0.0;
		double G = 0.0;
		double H = 0.0;
		double a = 0.0;
		double b = 0.0;
		double c = 0.0;
		C = iNum * X2 - X1 * X1;
		D = iNum * X1Y1 - X1 * Y1;
		E = iNum * X3 + iNum * X1Y2 - (X2 + Y2) * X1;
		G = iNum * Y2 - Y1 * Y1;
		H = iNum * X2Y1 + iNum * Y3 - (X2 + Y2) * Y1;
		a = (H * D - E * G) / (C * G - D * D);
		b = (H * C - E * D) / (D * D - G * C);
		c = -(a * X1 + b * Y1 + X2 + Y2) / iNum;
		double A = 0.0;
		double B = 0.0;
		double R = 0.0;
		A = a / (-2);
		B = b / (-2);
		R = double(sqrt(a * a + b * b - 4 * c) / 2);
		//Centroid.x = A;
		//Centroid.y = B;
        double center_x = A;
        double center_y = B;
        if(center_x >= 480) center_x=479;
        if(center_x < 0 ) center_x =0;
        if(center_y >= 640) center_y = 640;
        if(center_y < 0) center_y = 0;
        res_circle = Point(center_x,center_y);
		//dRadius = R;
		return true;
	}
	else
		return false;
	return true;

}


// bool FindCircle::circleLeastFit(const std::vector<Point> &points)
// {
//      double center_x = 0.0f;
//      double center_y = 0.0f;
//      double radius = 0.0f;
//      if (points.size() < 3)
//      {
//          return false;
//      }

//      double sum_x = 0.0f, sum_y = 0.0f;
//      double sum_x2 = 0.0f, sum_y2 = 0.0f;
//      double sum_x3 = 0.0f, sum_y3 = 0.0f;
//      double sum_xy = 0.0f, sum_x1y2 = 0.0f, sum_x2y1 = 0.0f;

//      int N = points.size();
//      for (int i = 0; i < N; i++)
//      {
//          double x = points[i].real();
//          double y = points[i].imag();
//          double x2 = x * x;
//          double y2 = y * y;
//          sum_x += x;
//          sum_y += y;
//          sum_x2 += x2;
//          sum_y2 += y2;
//          sum_x3 += x2 * x;
//          sum_y3 += y2 * y;
//          sum_xy += x * y;
//          sum_x1y2 += x * y2;
//          sum_x2y1 += x2 * y;
//      }

//      double C, D, E, G, H;
//      double a, b, c;

//      C = N * sum_x2 - sum_x * sum_x;
//      D = N * sum_xy - sum_x * sum_y;
//      E = N * sum_x3 + N * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
//      G = N * sum_y2 - sum_y * sum_y;
//      H = N * sum_x2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;
//      a = (H * D - E * G) / (C * G - D * D);
//      b = (H * C - E * D) / (D * D - G * C);
//      c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N;

//      center_x = a / (-2);
//      center_y = b / (-2);
//      if(center_x >= 480) center_x=479;
//      if(center_x < 0 ) center_x =0;
//      if(center_y >= 640) center_y = 640;
//      if(center_y < 0) center_y = 0;
//      res_circle = Point(center_x,center_y);
//      radius = sqrt(a * a + b * b - 4 * c) / 2;
//      return true;
// }

void FindCircle::keyPoint(Mat &input_img)
{
  	// Point p(0,0);
    // int num=0;
    //     for(int i=0;i<input_img.rows;i++)
    //     {
    //         for(int j=0;j<input_img.cols;j++)
    //         {
    //             if(input_img.at<uchar>(i,j)==255){
    //                 p.x+=j;
    //                 p.y+=i;
    //                 num+=1;
    //             }
    //         }
    //     }
    // if(num!=0){
    //     if(num>30){
    //         p.x/=num;
    //         p.y/=num; 
    //         flag_circle=true;
    //         res_circle=p;
    //         circle(input_img,res_circle,3,Scalar(255,255,255),-1);
    //     }
    //     flag_circle=false;
    // }
    // else{
    //   flag_circle=false;
    // }

    if(circleLeastFit(points)){
        flag_circle=true;
        circle(input_img,res_circle,5,Scalar(255,255,255),-1);
    }
    else{
        flag_circle=false;
    }

    // vector<Vec3f> circles;
	// HoughCircles(input_img,circles,CV_HOUGH_GRADIENT,1,1,200,thre,0);
    // Mat test_img;
    // input_img.copyTo(test_img);
    // Canny(test_img,test_img,100,200,3);
    // namedWindow("test_img",WINDOW_NORMAL);
    // imshow("test_img",test_img);
    // waitKey(10);
    // if(circles.size()==0){
    //     flag_circle=false;
    // }
    // else{
    //     flag_circle=true;
    //     Point res_circle=Point(0,0);
    //     int radius=0;
    //     //std::sort(circles.begin(),circles.end(),compare);
    //     vector<Vec3f>::iterator it=circles.begin();
    //     for(;it!=circles.end();it++){
    //         if((*it)[2]>radius){
    //             res_circle.x+=(*it)[0];
    //             res_circle.y+=(*it)[1];
    //             radius=(*it)[2];
    //         }
    //     }
      
    //     circle(input_img,res_circle,3,Scalar(255,255,255),-1);
    //     circle(input_img,res_circle,radius,Scalar(255,255,255));
    //     line(input_img,Point(240,320),res_circle,Scalar(255,255,255),5,LINE_8,0);
    //     height=res_circle.y-240;
    // }
}  

Point FindCircle::getPoint(){
    return res_circle;
}

bool FindCircle::getFlag(){
   return flag_circle;
}

void FindCircle::writeFileInit()
{
   FileStorage write_file("FindCircle.xml",FileStorage::WRITE);

   write_file<<"red_high"<<0;
   write_file<<"green_high"<<0;
   write_file<<"blue_high"<<0;
   write_file<<"red_low"<<126;
   write_file<<"green_low"<<79;
   write_file<<"blue_low"<<118;

   write_file.release();

}


void FindCircle::writeFile()
{
   FileStorage write_file("FindCircle.xml",FileStorage::WRITE);

   write_file<<"red_high"<<red_high;
   write_file<<"green_high"<<green_high;
   write_file<<"blue_high"<<blue_high;
   write_file<<"red_low"<<red_low;
   write_file<<"green_low"<<green_low;
   write_file<<"blue_low"<<blue_low;
   
   write_file.release();

}

void FindCircle::readFile()
{
    FileStorage read_file;
    read_file.open("FindCircle.xml",FileStorage::READ);
    
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

/*
input:rgb image;
output:gray_image which contain line
*/
void FindCircle::preProcessImg(Mat& input_img)
{
    points.clear();
    inRange(input_img,Scalar(red_low,green_low,blue_low),Scalar(red_high,green_high,blue_high),input_img);
    for(int i=0;i<input_img.rows;i++)
    {
        for(int j=0;j<input_img.cols;j++)
        {
          if(input_img.at<uchar>(i,j)==255){
            points.push_back(Point(j,i));
          }
        }
    }
    // int g_nStructElementSize = 3; //结构元素(内核矩阵)的尺寸
    // //获取自定义核
    // Mat element = getStructuringElement(MORPH_RECT,Size(2*g_nStructElementSize+1,2*g_nStructElementSize+1));
    circle(input_img,Point(320,240),10,Scalar(255,255,255),-1);
    namedWindow("circle_img",WINDOW_AUTOSIZE);
    imshow("circle_img",input_img);
    waitKey(10);

}

void FindCircle::getTheParameter(string _window_name)
{

    createTrackbar("redl change",_window_name,&red_low,180,0,0);
    createTrackbar("greenl change",_window_name,&green_low,255,0,0);
    createTrackbar("bluel change",_window_name,&blue_low,255,0,0);
    createTrackbar("redh change",_window_name,&red_high,180,0,0);
    createTrackbar("greenh change",_window_name,&green_high,255,0,0);
    createTrackbar("blueh change",_window_name,&blue_high,255,0,0);
    createTrackbar("thre change",_window_name,&thre,200,0,0);
   // createTrackbar("llen change",_window_name,&line_length,150,0,0);
    writeFile();

}
