#include <iostream>
#include <vector>
#include <cv.h>
#include "bspline.h"
#include <highgui.h>
using namespace std;
using namespace cv;
void f1(cv::Mat &);
int main (int argc, char * argv[]) 
{
  // const int resolution = 50;
  // int t = 3;
  // int n = 10;
  // CvPoint2D64f *pts = new CvPoint2D64f[n+1];
  // pts[0].x=270;  pts[0].y=290;
  // pts[1].x=270;  pts[1].y=390;
  // pts[2].x=370;  pts[2].y=390;
  // pts[3].x=470;  pts[3].y=390;
  // pts[4].x=470;  pts[4].y=290;
  // pts[5].x=470;  pts[5].y=190;
  // pts[6].x=370;  pts[6].y=190;
  // pts[7].x=270;  pts[7].y=190;
  // pts[8].x=270;  pts[8].y=290;
  // pts[9].x=270;  pts[9].y=390;
  // pts[10].x=370;  pts[10].y=390;

  // BSpline bs = BSpline(n, t , resolution, pts);

  // IplImage *img1 = cvLoadImage(argv[1],1);
  // IplImage *img2 = cvCreateImage(cvGetSize(img1), IPL_DEPTH_32F , 3);
  // // CvMemStorage* storage = cvCreateMemStorage(0);
  // // CvSeq* contour = cvCreateSeq( CV_SEQ_ELTYPE_POINT , sizeof(CvSeq), sizeof(CvPoint2D32f) , storage);
  // // for (int i = 0; i < resolution; ++i)
  // //   cvSeqPush(contour, &bs[i]);
  // CvPoint tmp1, tmp2;
  // for (int i = 0; i < resolution; ++i)
  // {
  //   cvLine(img2, bs[i], bs[i], CV_RGB(255, 0, 0), 2, 8 , 0);
  //   tmp1.x = bs[i].x + 6*bs.dt(i).x/cvSqrt(bs.dt(i).x*bs.dt(i).x + bs.dt(i).y*bs.dt(i).y);
  //   tmp1.y = bs[i].y + 6*bs.dt(i).y/cvSqrt(bs.dt(i).x*bs.dt(i).x + bs.dt(i).y*bs.dt(i).y);
  //   cout << " point: "  << i << " "<< tmp1.x << " "<< tmp1.y << std::endl;
  //   cvLine(img2, bs[i], tmp1, CV_RGB(0, 255, 0), 1, 8 , 0);

  //   tmp2.x = bs[i].x - 6*bs.dt(i).y/cvSqrt(bs.dt(i).x*bs.dt(i).x + bs.dt(i).y*bs.dt(i).y);
  //   tmp2.y = bs[i].y + 6*bs.dt(i).x/cvSqrt(bs.dt(i).x*bs.dt(i).x + bs.dt(i).y*bs.dt(i).y);
  //   cout << " point: "  << i << " "<< tmp2.x << " "<< tmp2.y << std::endl;
  //   cvLine(img2, bs[i], tmp2, CV_RGB(0, 0, 255), 1, 8 , 0);
  // }

  // for (int i = 0; i < resolution; ++i)
  // {
  //   const double *ptr = bs.basic_mat_.ptr<double>(i);
  //   for (int j = 0; j < n; ++j)
  //   {
  //     cout << ptr[j] << " ";
  //   }
  //   cout << endl;
  // }

  

  
  // cvNamedWindow("Circle", CV_WINDOW_AUTOSIZE);
  // cvShowImage("Circle", img2);
  // while(1)
  // {
  //   if(cvWaitKey(50) == 27) break;
  // }

  
  // // cvReleaseMemStorage(&storage);
  // cvReleaseImage(&img1);
  // cvReleaseImage(&img2);

  // Size s(3,3);
  // Mat m1 = Mat::eye(s, CV_64F);
  // Mat m2 = Mat::eye(s, CV_64F);
  // for (int i = 0; i < s.height; ++i){
  //   double *m1_ptr = m1.ptr<double>(i);
  //   double *m2_ptr = m2.ptr<double>(i);
  //   for (int j = 0; j < s.width; ++j)
  //   {
  //     m1_ptr[j] = (double)random();
  //     cout << m1_ptr[j] << " " ;
  //   }
  //   cout << endl;
  //   for (int j = 0 ; j < s.width; ++j)
  //   {
  //     m2_ptr[j] = (double)random();
  //     cout << m2_ptr[j] << " ";
  //   }
  //   cout << endl;
  // }
  // Mat m3(3, 3, CV_64F);
  // m3 = m1*m2;
  // for (int i = 0; i < s.height; ++i){
  //   double *m3_ptr = m3.ptr<double>(i);
  //   for (int j = 0; j < s.width; ++j){
  //     cout << m3_ptr[j] << " ";
  //   }
  //   cout << endl;
  // }
  // Mat m4 = Mat::zeros(3, 1, CV_8U), m5 = Mat::zeros(3, 1, CV_8U);
  // m4.at<uchar>(0,0) = 1;
  // m4.at<uchar>(1,0) = 2;
  // m4.at<uchar>(2,0) = 3;
  // m5.at<uchar>(0,0) = 4;
  // m5.at<uchar>(1,0) = 5;
  // m5.at<uchar>(2,0) = 6;
  // cout <<   norm(m4, m5) << endl;
  cv::Mat m1 = Mat::zeros(2,3, CV_32F);
  int iter = 0;  

  m1.release();
}

void f1(cv::Mat &m1){
  m1.at<int>(0,0) = 0;
  m1.at<int>(0,1) = ;
  m1.at<int>(0,2) = ;
  m1.at<int>(1,0) = ;
  m1.at<int>(1,1) = ;
  m1.at<int>(1,2) = ;
}
