#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
#include<ctime>
#include <stdlib.h>
#include<algorithm>

#define PI 3.1415926535 
using namespace std;
using namespace cv;

struct Cluster
{
    vector<double> centroid;
    vector<uint> samples;
}; 
void Lines(Mat &input_img);
void getCenter();
void standardLine();
void preProcess(Mat &src);
Mat thinImage(const cv::Mat & src, const int maxIterations);
void getTheParameter(string _window_name);
vector<Point> Vec2f2point(cv::Vec2f input_data);
void LargestConnecttedComponent(Mat srcImage, Mat &dstImage);
double cal_distance(vector<double> a, vector<double> b);
vector<Cluster> k_means(vector<vector<double> > trainX, uint k, uint maxepoches);
bool funcFindH();

int high_H=134;
int high_S=206;
int high_V=153;
int low_H=54;
int low_S=44;
int low_V=13;
int len=70;

static double slope;
static double intercept;
static double Angle;
vector<vector<double> > lines;
vector<Cluster> clusters_out;
// vector< vector<Point> > res_Pline;

Mat src;
Mat change_src;
Mat edge_src;
Mat gray_src;
Mat res;

bool findH=false;
Point center;
int main(){
    VideoCapture cap(0);
    
    if(!cap.isOpened()){
        cout<<"camera not find"<<endl;
        exit(1);
    } 
    namedWindow("detect_color", 2);
    namedWindow("src", 2);
    namedWindow("gray",2);
    while(1){
        cap>>src;
        resize(src,src,Size(640,480));
        lines.clear();

        preProcess(src);
        Lines(gray_src);
        getTheParameter("detect_color");
        
        findH=funcFindH();
        cout<<findH<<endl;
        imshow("src",src);
        imshow("detect_color",change_src); //原图
        imshow("gray",gray_src);
        waitKey(20);
    }
    return 0;
}

// void getCenter()
// {

// }

void getTheParameter(string _window_name)
{
   
    createTrackbar("redh change",_window_name,&high_H,180,0,0);
    createTrackbar("greenh change",_window_name,&high_S,255,0,0);
    createTrackbar("blueh change",_window_name,&high_V,255,0,0);
    createTrackbar("redhl change",_window_name,&low_H,180,0,0);
    createTrackbar("greenl change",_window_name,&low_S,255,0,0);
    createTrackbar("bluel change",_window_name,&low_V,255,0,0);
    createTrackbar("len change",_window_name,&len,180,0,0);

}

void calSlopeIncept(Point begin,Point end)
{
  
	if (abs(begin.x - end.x) <= 5)
	{
		
		slope = 1000;
        Angle = PI/2;
        intercept=begin.x;
	}
	else
	{
		slope = ((begin.y - end.y)*1.0 / (begin.x - end.x));
        Angle=atan(slope);
        
        //Angle=(180/PI)*Angle;
        intercept = begin.y - slope * begin.x;
	}

	
    vector<double> v;
    v.push_back(Angle);
    v.push_back(intercept);
    lines.push_back(v);
}

// void mergeLines()
// {

// }
bool comp(Cluster a, Cluster b)
{
  if(a.centroid[0]>b.centroid[0]){
      return true;
  } 
  else {
      if(a.centroid[1]>b.centroid[1]){
          return true;
      }
  }
  return false;
}
bool funcFindH()
{
  if(clusters_out.size()<3) return false;
  for(int i=0;i<3;i++){
      clusters_out[i].centroid[0]*=(180/PI);
  }

  //对平行的判断，应该排除重合
  bool parallel_1=((abs(clusters_out[2].centroid[0]-clusters_out[1].centroid[0]))<=8);
  bool parallel_2=(((abs(clusters_out[1].centroid[0]-clusters_out[0].centroid[0]))<=8));
  
  bool vertical=false;
  if(parallel_1){
      if(clusters_out[0].centroid[0]<0){
          if(abs((clusters_out[1].centroid[0]-clusters_out[0].centroid[0])-90)<=5){
              vertical=true;
          }
         // cout<<abs((clusters_out[1].centroid[0]-clusters_out[0].centroid[0])-90)<<"  ";
      }
      else{
          if(abs((clusters_out[0].centroid[0]-clusters_out[1].centroid[0])-90)<=5){
              vertical=true;
          }
          // cout<<abs((clusters_out[0].centroid[0]-clusters_out[1].centroid[0])-90)<<"  ";
      }
  }
  if(parallel_2){
      if(clusters_out[2].centroid[0]<0){
          if(abs((clusters_out[1].centroid[0]-clusters_out[2].centroid[0])-90)<=5){
              vertical=true;
          }
          // cout<<abs((clusters_out[1].centroid[0]-clusters_out[2].centroid[0])-90)<<"  ";
      }
      else{
          if(abs((clusters_out[2].centroid[0]-clusters_out[1].centroid[0])-90)<=5){
              vertical=true;
          }
          // cout<<abs((clusters_out[2].centroid[0]-clusters_out[1].centroid[0])-90)<<"  ";
      }
  }
  //cout<<clusters_out[0].centroid[0]<<"  "<<clusters_out[1].centroid[0]<<"  "<<clusters_out[2].centroid[0]<<endl;
  if((parallel_1||parallel_2)&&vertical) return true;
  return false;
}
void Lines(Mat &input_img)
{
    vector<cv::Vec2f> src_lines;
    vector<cv::Vec2f> res_lines;
    HoughLines(input_img,src_lines,1,PI/180,len,0,0);
    vector<cv::Vec2f>::iterator it;
   
    for(it=src_lines.begin();it!=src_lines.end();it++){
        vector<Point> cur_line=Vec2f2point(*it);
        //res_lines.push_back(*it);
        //line(src,cur_line[0],cur_line[1],Scalar(0,255,0),5,LINE_8,0);
        calSlopeIncept(cur_line[0],cur_line[1]);
        //k_means()
    }
    if(lines.size()==0){

    }
    else clusters_out = k_means(lines, 3, 100);
    //lines=res_lines;
    //standardLine();
    vector<Cluster>::iterator iter;
    for(iter=clusters_out.begin();iter!=clusters_out.end();iter++)
    {
       double slope=tan((*iter).centroid[0]);
       Point p1,p2;
       p1.x=(-(*iter).centroid[1])/slope;
       p1.y=0;
       p2.x=(480-(*iter).centroid[1])/slope;
       p2.y=480;
    //    =((int)(-(*iter).centroid[1])/slope,0);
    //    Point p2=((int)(480-(*iter).centroid[1])/slope,480);
       
       line(src,p1,p2,Scalar(0,255,0),5,LINE_8,0);
    //    cout<<(*iter).centroid[0]<<endl;
    }
    //iter=clusters_out.begin()
    sort(clusters_out.begin(),clusters_out.end(),comp);
}

vector<Point> Vec2f2point(cv::Vec2f input_data)
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

void preProcess(Mat &src)
{
  cvtColor(src,change_src,COLOR_BGR2HSV);
  inRange(change_src,Scalar(low_H,low_S,low_V),Scalar(high_H,high_S,high_V),change_src);
  int g_nStructElementSize = 5;
  Mat element = getStructuringElement(MORPH_RECT,Size(2*g_nStructElementSize+1,2*g_nStructElementSize+1));
  
  erode(change_src,change_src, element);
  dilate(change_src,change_src,element);

  LargestConnecttedComponent(change_src,change_src);
  namedWindow("change",2);
  imshow("change",change_src);
  waitKey(20);
  threshold(change_src,change_src,128,1,THRESH_BINARY);
  gray_src=thinImage(change_src,-1);
  gray_src *= 255;
  change_src*=255;
}

void LargestConnecttedComponent(Mat srcImage, Mat &dstImage)
{
    Mat temp;
    Mat labels;
    srcImage.copyTo(temp);

    //1. 标记连通域
    int n_comps = connectedComponents(temp, labels, 4, CV_16U);
    vector<int> histogram_of_labels;
    for (int i = 0; i < n_comps; i++)//初始化labels的个数为0
    {
        histogram_of_labels.push_back(0);
    }

    int rows = labels.rows;
    int cols = labels.cols;
    for (int row = 0; row < rows; row++) //计算每个labels的个数
    {
        for (int col = 0; col < cols; col++)
        {
            histogram_of_labels.at(labels.at<unsigned short>(row, col)) += 1;
        }
    }
    histogram_of_labels.at(0) = 0; //将背景的labels个数设置为0

    //2. 计算最大的连通域labels索引
    int maximum = 0;
    int max_idx = 0;
    for (int i = 0; i < n_comps; i++)
    {
        if (histogram_of_labels.at(i) > maximum)
        {
            maximum = histogram_of_labels.at(i);
            max_idx = i;
        }
    }

    //3. 将最大连通域标记为1
    for (int row = 0; row < rows; row++) 
    {
        for (int col = 0; col < cols; col++)
        {
            if (labels.at<unsigned short>(row, col) == max_idx)
            {
                labels.at<unsigned short>(row, col) = 255;
            }
            else
            {
                labels.at<unsigned short>(row, col) = 0;
            }
        }
    }

    //4. 将图像更改为CV_8U格式
    labels.convertTo(dstImage, CV_8U);
}
/**
 * @brief 对输入图像进行细化
 * @param src为输入图像,用cvThreshold函数处理过的8位灰度图像格式，元素中只有0与1,1代表有元素，0代表为空白
 * @param maxIterations限制迭代次数，如果不进行限制，默认为-1，代表不限制迭代次数，直到获得最终结果
 * @return 为对src细化后的输出图像,格式与src格式相同，元素中只有0与1,1代表有元素，0代表为空白
 */
Mat thinImage(const cv::Mat & src, const int maxIterations)
{
    assert(src.type() == CV_8UC1);
    cv::Mat dst;
    int width  = src.cols;
    int height = src.rows;
    src.copyTo(dst);
    int count = 0;  //记录迭代次数
    while (true)
    {
        count++;
        if (maxIterations != -1 && count > maxIterations) //限制次数并且迭代次数到达
            break;
        std::vector<uchar *> mFlag; //用于标记需要删除的点
        //对点标记
        for (int i = 0; i < height ;++i)
        {
            uchar * p = dst.ptr<uchar>(i);
            for (int j = 0; j < width; ++j)
            {
                //如果满足四个条件，进行标记
                //  p9 p2 p3
                //  p8 p1 p4
                //  p7 p6 p5
                uchar p1 = p[j];
                if (p1 != 1) continue;
                uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);
                uchar p8 = (j == 0) ? 0 : *(p + j - 1);
                uchar p2 = (i == 0) ? 0 : *(p - dst.step + j);
                uchar p3 = (i == 0 || j == width - 1) ? 0 : *(p - dst.step + j + 1);
                uchar p9 = (i == 0 || j == 0) ? 0 : *(p - dst.step + j - 1);
                uchar p6 = (i == height - 1) ? 0 : *(p + dst.step + j);
                uchar p5 = (i == height - 1 || j == width - 1) ? 0 : *(p + dst.step + j + 1);
                uchar p7 = (i == height - 1 || j == 0) ? 0 : *(p + dst.step + j - 1);
                if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) >= 2 && (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) <= 6)
                {
                    int ap = 0;
                    if (p2 == 0 && p3 == 1) ++ap;
                    if (p3 == 0 && p4 == 1) ++ap;
                    if (p4 == 0 && p5 == 1) ++ap;
                    if (p5 == 0 && p6 == 1) ++ap;
                    if (p6 == 0 && p7 == 1) ++ap;
                    if (p7 == 0 && p8 == 1) ++ap;
                    if (p8 == 0 && p9 == 1) ++ap;
                    if (p9 == 0 && p2 == 1) ++ap;
 
                    if (ap == 1 && p2 * p4 * p6 == 0 && p4 * p6 * p8 == 0)
                    {
                        //标记
                        mFlag.push_back(p+j);
                    }
                }
            }
        }
 
        //将标记的点删除
        for (std::vector<uchar *>::iterator i = mFlag.begin(); i != mFlag.end(); ++i)
        {
            **i = 0;
        }
 
        //直到没有点满足，算法结束
        if (mFlag.empty())
        {
            break;
        }
        else
        {
            mFlag.clear();//将mFlag清空
        }
 
        //对点标记
        for (int i = 0; i < height; ++i)
        {
            uchar * p = dst.ptr<uchar>(i);
            for (int j = 0; j < width; ++j)
            {
                //如果满足四个条件，进行标记
                //  p9 p2 p3
                //  p8 p1 p4
                //  p7 p6 p5
                uchar p1 = p[j];
                if (p1 != 1) continue;
                uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);
                uchar p8 = (j == 0) ? 0 : *(p + j - 1);
                uchar p2 = (i == 0) ? 0 : *(p - dst.step + j);
                uchar p3 = (i == 0 || j == width - 1) ? 0 : *(p - dst.step + j + 1);
                uchar p9 = (i == 0 || j == 0) ? 0 : *(p - dst.step + j - 1);
                uchar p6 = (i == height - 1) ? 0 : *(p + dst.step + j);
                uchar p5 = (i == height - 1 || j == width - 1) ? 0 : *(p + dst.step + j + 1);
                uchar p7 = (i == height - 1 || j == 0) ? 0 : *(p + dst.step + j - 1);
 
                if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) >= 2 && (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) <= 6)
                {
                    int ap = 0;
                    if (p2 == 0 && p3 == 1) ++ap;
                    if (p3 == 0 && p4 == 1) ++ap;
                    if (p4 == 0 && p5 == 1) ++ap;
                    if (p5 == 0 && p6 == 1) ++ap;
                    if (p6 == 0 && p7 == 1) ++ap;
                    if (p7 == 0 && p8 == 1) ++ap;
                    if (p8 == 0 && p9 == 1) ++ap;
                    if (p9 == 0 && p2 == 1) ++ap;
 
                    if (ap == 1 && p2 * p4 * p8 == 0 && p2 * p6 * p8 == 0)
                    {
                        //标记
                        mFlag.push_back(p+j);
                    }
                }
            }
        }
 
        //将标记的点删除
        for (std::vector<uchar *>::iterator i = mFlag.begin(); i != mFlag.end(); ++i)
        {
            **i = 0;
        }
 
        //直到没有点满足，算法结束
        if (mFlag.empty())
        {
            break;
        }
        else
        {
            mFlag.clear();//将mFlag清空
        }
    }
    return dst;
}

using namespace std;
typedef unsigned int uint;


double cal_distance(vector<double> a, vector<double> b)
{
    uint da = a.size();
    uint db = b.size();
    if (da != db) cerr << "Dimensions of two vectors must be same!!\n";
    double val = 0.0;
    for (uint i = 0; i < da; i++)
    {
        val += pow((a[i] - b[i]), 2);
    }
    return pow(val, 0.5);
}
vector<Cluster> k_means(vector<vector<double> > trainX, uint k, uint maxepoches)
{
    const uint row_num = trainX.size();
    const uint col_num = trainX[0].size();

    /*初始化聚类中心*/
    vector<Cluster> clusters(k);
    uint seed = (uint)time(NULL); 
    for (uint i = 0; i < k; i++)
    {
        srand(seed);
        int c = rand() % row_num;
        clusters[i].centroid = trainX[c];
        seed = rand();
    }

    /*多次迭代直至收敛，本次试验迭代100次*/
    for (uint it = 0; it < maxepoches; it++)
    {
        /*每一次重新计算样本点所属类别之前，清空原来样本点信息*/
        for (uint i = 0; i < k; i++)
        {
            clusters[i].samples.clear();
        }
        /*求出每个样本点距应该属于哪一个聚类*/
        for (uint j = 0; j < row_num; j++)
        {
            /*都初始化属于第0个聚类*/    
            uint c = 0;
            double min_distance = cal_distance(trainX[j],clusters[c].centroid);
            for (uint i = 1; i < k; i++)
            {
                double distance = cal_distance(trainX[j], clusters[i].centroid);
                if (distance < min_distance)
                {
                    min_distance = distance;
                    c = i;
                }
            }
            clusters[c].samples.push_back(j);
        }

        /*更新聚类中心*/
        for (uint i = 0; i < k; i++)
        {
            vector<double> val(col_num, 0.0); 
            for (uint j = 0; j < clusters[i].samples.size(); j++)
            {
                uint sample = clusters[i].samples[j];
                for (uint d = 0; d < col_num; d++)
                {
                    val[d] += trainX[sample][d];
                    if (j == clusters[i].samples.size() - 1)
                        clusters[i].centroid[d] = val[d] / clusters[i].samples.size();
                }
            }
        }
    }
    return clusters;
}

// #include <opencv2/opencv.hpp>
// #include <opencv2/core/core.hpp>
// #include <iostream>
// #include <vector>
 
 

 
 
// int main(int argc, char*argv[])
// {
//     //获取图像
//     if (argc != 2)
//     {
//         std::cout << "参数个数错误！" << std::endl;
//         return -1;
//     }
//     cv::Mat src = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
//     if (src.empty())
//     {
//         std::cout << "读取文件失败！" << std::endl;
//         return -1;
//     }
 
//     //将原图像转换为二值图像
//     cv::threshold(src, src, 128, 1, cv::THRESH_BINARY_INV);
//     //图像细化
//     cv::Mat dst = thinImage(src);
//     //显示图像
//     dst = dst * 255;
//     src = src * 255;
//     cv::namedWindow("src1", CV_WINDOW_AUTOSIZE);
//     cv::namedWindow("dst1", CV_WINDOW_AUTOSIZE);
//     cv::imshow("src1", src);
//     cv::imshow("dst1", dst);
//     cv::waitKey(0);
// }




// //将 DEPTH_8U型二值图像进行细化  经典的Zhang并行快速细化算法
// void thin(const Mat &src, Mat &dst, const int iterations)
// {
//     const int height =src.rows -1;
//     const int width  =src.cols -1;
 
//     //拷贝一个数组给另一个数组
//     if(src.data != dst.data)
//     {
//         src.copyTo(dst);
//     }
    
 
//     int n = 0,i = 0,j = 0;
//     Mat tmpImg;
//     uchar *pU, *pC, *pD;
//     BOOL isFinished =FALSE;
 
//      for(n=0; n<iterations; n++)
//      {
//          dst.copyTo(tmpImg); 
//          isFinished =FALSE;   //一次 先行后列扫描 开始
//          //扫描过程一 开始
//          for(i=1; i<height;  i++) 
//         {
//             pU = tmpImg.ptr<uchar>(i-1);
//             pC = tmpImg.ptr<uchar>(i);
//             pD = tmpImg.ptr<uchar>(i+1);
//            for(int j=1; j<width; j++)
//            {
//             if(pC[j] > 0)
//             {
//                  int ap=0;
//                  int p2 = (pU[j] >0);
//                  int p3 = (pU[j+1] >0);
//                  if (p2==0 && p3==1)
//                  {
//                   ap++;
//                  }
//                  int p4 = (pC[j+1] >0);
//                  if(p3==0 && p4==1)
//                  {
//                   ap++;
//                  }
//                  int p5 = (pD[j+1] >0);
//                  if(p4==0 && p5==1)
//                  {
//                   ap++;
//                  }
//                  int p6 = (pD[j] >0);
//                  if(p5==0 && p6==1)
//                  {
//                   ap++;
//                  }
//                  int p7 = (pD[j-1] >0);
//                  if(p6==0 && p7==1)
//                  {
//                   ap++;
//                  }
//                  int p8 = (pC[j-1] >0);
//                  if(p7==0 && p8==1)
//                  {
//                   ap++;
//                  }
//                  int p9 = (pU[j-1] >0);
//                  if(p8==0 && p9==1)
//                  {
//                   ap++;
//                  }
//                  if(p9==0 && p2==1)
//                  {
//                   ap++;
//                  }
//                  if((p2+p3+p4+p5+p6+p7+p8+p9)>1 && (p2+p3+p4+p5+p6+p7+p8+p9)<7)
//                  {
//                       if(ap==1)
//                       {
//                            if((p2*p4*p6==0)&&(p4*p6*p8==0))
//                            {                           
//                                 dst.ptr<uchar>(i)[j]=0;
//                                 isFinished =TRUE;                            
//                            }
                      
//                         //   if((p2*p4*p8==0)&&(p2*p6*p8==0))
//                        //    {                           
//                        //         dst.ptr<uchar>(i)[j]=0;
//                        //         isFinished =TRUE;                            
//                        //    }
                       
//                      }
//                 }                    
//             }
 
//            } //扫描过程一 结束
 
     
//          dst.copyTo(tmpImg); 
//          //扫描过程二 开始
//          for(i=1; i<height;  i++)  //一次 先行后列扫描 开始
//         {
//             pU = tmpImg.ptr<uchar>(i-1);
//             pC = tmpImg.ptr<uchar>(i);
//             pD = tmpImg.ptr<uchar>(i+1);
//            for(int j=1; j<width; j++)
//            {
//             if(pC[j] > 0)
//             {
//                  int ap=0;
//                  int p2 = (pU[j] >0);
//                  int p3 = (pU[j+1] >0);
//                  if (p2==0 && p3==1)
//                  {
//                   ap++;
//                  }
//                  int p4 = (pC[j+1] >0);
//                  if(p3==0 && p4==1)
//                  {
//                   ap++;
//                  }
//                  int p5 = (pD[j+1] >0);
//                  if(p4==0 && p5==1)
//                  {
//                   ap++;
//                  }
//                  int p6 = (pD[j] >0);
//                  if(p5==0 && p6==1)
//                  {
//                   ap++;
//                  }
//                  int p7 = (pD[j-1] >0);
//                  if(p6==0 && p7==1)
//                  {
//                   ap++;
//                  }
//                  int p8 = (pC[j-1] >0);
//                  if(p7==0 && p8==1)
//                  {
//                   ap++;
//                  }
//                  int p9 = (pU[j-1] >0);
//                  if(p8==0 && p9==1)
//                  {
//                   ap++;
//                  }
//                  if(p9==0 && p2==1)
//                  {
//                   ap++;
//                  }
//                  if((p2+p3+p4+p5+p6+p7+p8+p9)>1 && (p2+p3+p4+p5+p6+p7+p8+p9)<7)
//                  {
//                       if(ap==1)
//                       {
//                         //   if((p2*p4*p6==0)&&(p4*p6*p8==0))
//                         //   {                           
//                        //         dst.ptr<uchar>(i)[j]=0;
//                        //         isFinished =TRUE;                            
//                        //    }
                      
//                            if((p2*p4*p8==0)&&(p2*p6*p8==0))
//                            {                           
//                                 dst.ptr<uchar>(i)[j]=0;
//                                 isFinished =TRUE;                            
//                            }
                       
//                      }
//                 }                    
//             }
 
//            }
 
//           } //一次 先行后列扫描完成          
//         //如果在扫描过程中没有删除点，则提前退出
//          if(isFinished ==FALSE)
//          {
//             break; 
//          }
//         }
 
//     }
// }

// #include<iostream>
// #include<cmath>
// #include<vector>
// #include<ctime>
// #include <stdlib.h>

// using namespace std;
// typedef unsigned int uint;

// struct Cluster
// {
//     vector<double> centroid;
//     vector<uint> samples;
// };
// double cal_distance(vector<double> a, vector<double> b)
// {
//     uint da = a.size();
//     uint db = b.size();
//     if (da != db) cerr << "Dimensions of two vectors must be same!!\n";
//     double val = 0.0;
//     for (uint i = 0; i < da; i++)
//     {
//         val += pow((a[i] - b[i]), 2);
//     }
//     return pow(val, 0.5);
// }
// vector<Cluster> k_means(vector<vector<double> > trainX, uint k, uint maxepoches)
// {
//     const uint row_num = trainX.size();
//     const uint col_num = trainX[0].size();

//     /*初始化聚类中心*/
//     vector<Cluster> clusters(k);
//     uint seed = (uint)time(NULL); 
//     for (uint i = 0; i < k; i++)
//     {
//         srand(seed);
//         int c = rand() % row_num;
//         clusters[i].centroid = trainX[c];
//         seed = rand();
//     }

//     /*多次迭代直至收敛，本次试验迭代100次*/
//     for (uint it = 0; it < maxepoches; it++)
//     {
//         /*每一次重新计算样本点所属类别之前，清空原来样本点信息*/
//         for (uint i = 0; i < k; i++)
//         {
//             clusters[i].samples.clear();
//         }
//         /*求出每个样本点距应该属于哪一个聚类*/
//         for (uint j = 0; j < row_num; j++)
//         {
//             /*都初始化属于第0个聚类*/    
//             uint c = 0;
//             double min_distance = cal_distance(trainX[j],clusters[c].centroid);
//             for (uint i = 1; i < k; i++)
//             {
//                 double distance = cal_distance(trainX[j], clusters[i].centroid);
//                 if (distance < min_distance)
//                 {
//                     min_distance = distance;
//                     c = i;
//                 }
//             }
//             clusters[c].samples.push_back(j);
//         }

//         /*更新聚类中心*/
//         for (uint i = 0; i < k; i++)
//         {
//             vector<double> val(col_num, 0.0); 
//             for (uint j = 0; j < clusters[i].samples.size(); j++)
//             {
//                 uint sample = clusters[i].samples[j];
//                 for (uint d = 0; d < col_num; d++)
//                 {
//                     val[d] += trainX[sample][d];
//                     if (j == clusters[i].samples.size() - 1)
//                         clusters[i].centroid[d] = val[d] / clusters[i].samples.size();
//                 }
//             }
//         }
//     }
//     return clusters;
// }

// int main()
// {
//     vector<vector<double> > trainX;
//     //对9个数据{1 2 3 11 12 13 21 22 23}聚类
//     // double data = 1.0;
//     // for (uint i = 0; i < 9; i++)
//     // {
//     //     trainX[i][0] = data;
//     //     if ((i+1) % 3 == 0) data += 8;
//     //     else data++;
//     // }
//     // trainX={{-20,13},{-19,11},{2,45},{3,43},{-20,50},{-21,55}};
//     cout<<trainX.size()<<endl;
//     vector<double> v;
//     v.push_back(-20);
//     v.push_back(13);
//     trainX.push_back(v);
//     v.clear();
//     v.push_back(-19);
//     v.push_back(11);
//     trainX.push_back(v);
//     v.clear();
//     v.push_back(72);
//     v.push_back(45);
//     trainX.push_back(v);
//     v.clear();
//     v.push_back(73);
//     v.push_back(43);
//     trainX.push_back(v);
//     v.clear();
//     v.push_back(-20);
//     v.push_back(50);
//     trainX.push_back(v);
//     v.clear();
//     v.push_back(-21);
//     v.push_back(55);
//     trainX.push_back(v);
//     v.clear();
//     /*k-means聚类*/
//     vector<Cluster> clusters_out = k_means(trainX, 3, 100);

//     /*输出分类结果*/
//     for (uint i = 0; i < clusters_out.size(); i++)
//     {
//         cout << "Cluster " << i << " :" << endl; 

//         /*子类中心*/
//         cout << "\t" << "Centroid: " << "\n\t\t[ ";
//         for (uint j = 0; j < clusters_out[i].centroid.size(); j++)
//         {
//             cout << clusters_out[i].centroid[j] << " ";
//         }
//         cout << "]" << endl;

//         /*子类样本点*/
//         cout << "\t" << "Samples:\n";
//         for (uint k = 0; k < clusters_out[i].samples.size(); k++)
//         {
//             uint c = clusters_out[i].samples[k];
//             cout << "\t\t[ ";
//             for (uint m = 0; m < trainX[0].size(); m++)
//             {
//                 cout << trainX[c][m] << " ";
//             }
//             cout << "]\n";
//         }
//     }
//     return 0;
// }