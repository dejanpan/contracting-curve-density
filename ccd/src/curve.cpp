#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "bspline.h"
#include <iostream>
#include <string>
#include <cstdio>
#include <algorithm>
using namespace cv;
IplImage *img1;
std::vector<CvPoint2D64f> pts;

void on_mouse( int event, int x, int y, int flags, void* param )
{
  if( !img1 )
    return;

  if( img1->origin )
    y = img1->height - y;

  switch( event )
  {
    case CV_EVENT_LBUTTONDOWN:
      // std::cout << "Event = " << event << std::endl;
      break;
    case CV_EVENT_LBUTTONUP:
      // std::cout << "Event = " << event << std::endl;
      cvCircle(img1,cvPoint(x,y),2,cvScalar(0,0,255),2);
      pts.push_back(cvPoint2D64f(x,y));
      cvShowImage("Original",img1);
      break;
  }
}

int main (int argc, char * argv[]) 
{
  const int resolution = 50;
  int t = 3;
  IplImage dst_img;
  img1= cvLoadImage(argv[1], 1);
  cv::Mat img(img1);
  int tmp[3];
  char filename[30];
  CvPoint2D64f pts_tmp;
  cvNamedWindow("Original", 1);
  cvSetMouseCallback( "Original", on_mouse, 0 );
  cvShowImage("Original",img1);
  char key ;
  while (1)
  {
    key = cvWaitKey(10);
    if (key == 27) break;
  }

  if(pts.size() > 3)
  {
    pts.push_back(pts[0]);
    pts.push_back(pts[1]);
    pts.push_back(pts[2]);
  }
  for (size_t i = 0; i < pts.size(); ++i){
    std::cout<< pts[i].x << " " << pts[i].y << std::endl;
  }
  
  cv::Mat img_map;
  cv::Mat dx = Mat::zeros(6,1, CV_64F);
  cv::Mat dx_old = Mat::zeros(6,1, CV_64F);
  cv::Mat X = Mat::zeros(6,1, CV_64F);
  std::vector<CvPoint> curve;

  for (int i = 0; i < 6; ++i)
    X.at<double>(i,0) = X.at<double>(i,0) - dx.at<double>(i,0);
  for (size_t i = 0; i < pts.size(); ++i)
  {
    pts_tmp.x = X.at<double>(0,0) + (1+X.at<double>(2,0))*pts[i].x + X.at<double>(5,0)*pts[i].y;
    pts_tmp.y = X.at<double>(1,0) + (1+X.at<double>(3,0))*pts[i].y + X.at<double>(4,0)*pts[i].x;
    pts[i].x = round(pts_tmp.x);
    pts[i].y = round(pts_tmp.y);
  }
  std::cout << "before " << std::endl;
  BSpline bs(t , resolution, pts);
  CvPoint tmp1, tmp2;
  CvPoint2D64f tmp_dis1, tmp_dis2;
  int h = 40, dn = 4;
  double gamma_1 = 0.5, gamma_2 = 4, gamma_3 = 6, gamma_4 = 4;
  double sigma_t = max(double(h/cvSqrt(2*gamma_2)), gamma_4);
  double sigma = sigma_t/gamma_3;
  double kappa = 0.5;

  // set the size of dimension2 for Mat::vic
  // 8 represents x,y, dx(distance to curve), dy(distance to curve)
  int normal_points_number = floor(h/dn);
  double nx, ny;
  cv::Mat vic = Mat::zeros(resolution, 18*normal_points_number, CV_64F);
  std::vector< std::vector<CvPoint2D64f> > dis(resolution);
  std::vector<double> normalized_param(resolution);
  for(int i=0;i < resolution;i++){
    cvCircle( img1, bs[i], 2, CV_RGB(0,0, 255),2);
    nx = -bs.dt(i).y/cvSqrt(bs.dt(i).x*bs.dt(i).x + bs.dt(i).y*bs.dt(i).y);
    ny = bs.dt(i).x/cvSqrt(bs.dt(i).x*bs.dt(i).x + bs.dt(i).y*bs.dt(i).y);
    int k = 0;
    double alpha = 0.5;
    double normalized_sum = 0.0;
    for (int j = dn; j <= h; j+=dn, k++){
      tmp1.x = round(bs[i].x + j*nx);
      tmp1.y = round(bs[i].y + j*ny);
      tmp_dis1.x = (tmp1.x-bs[i].x)*nx + (tmp1.y-bs[i].y)*ny;
      tmp_dis1.y = (tmp1.x-bs[i].x)*ny - (tmp1.y-bs[i].y)*nx;
      vic.at<double>(i,9*k + 0) = tmp1.x;
      vic.at<double>(i,9*k + 1) = tmp1.y;
      vic.at<double>(i,9*k + 2) = tmp_dis1.x;
      vic.at<double>(i,9*k + 3) = tmp_dis1.y;
      vic.at<double>(i,9*k + 4) = 0.5*(erf((tmp_dis1.x)/(sqrt(2)*sigma)) + 1);
      double wp1 = (vic.at<double>(i,9*k + 4) - gamma_1)/(1-gamma_1);
      vic.at<double>(i,9*k + 5) = max(0.0, wp1*wp1*wp1*wp1*wp1*wp1);
      double wp2 = (1-vic.at<double>(i,9*k + 4) - gamma_1)/(1-gamma_1);
      vic.at<double>(i,9*k + 6) = max(0.0, wp2*wp2*wp2*wp2*wp2*wp2);
      vic.at<double>(i,9*k + 7) = max((exp(-vic.at<double>(i,9*k + 2)*vic.at<double>(i,9*k + 2)/(2*sigma_t*sigma_t)) - exp(-gamma_2)), 0.0);
      vic.at<double>(i, 9*k + 8) = 0.5*exp(-abs(tmp_dis1.y)/alpha)/alpha;
      normalized_sum += vic.at<double>(i, 9*k + 7);
      if(i == 0)
        std::cout << "tmp1 " << tmp1.x  << " " << tmp1.y << std::endl;
      cvCircle(img1, tmp1, 1, CV_RGB(0, 255, 255), 1, 8 , 0);
      tmp2.x = round(bs[i].x - j*nx);
      tmp2.y = round(bs[i].y - j*ny);
      if(i == 0)
        std::cout << "tmp2 " << tmp2.x  << " " << tmp2.y << std::endl;
      tmp_dis2.x = (tmp2.x-bs[i].x)*nx + (tmp2.y-bs[i].y)*ny;
      tmp_dis2.y = (tmp2.x-bs[i].x)*ny - (tmp2.y-bs[i].y)*nx;
      int negative_normal = k+normal_points_number;
      vic.at<double>(i,9*negative_normal + 0) = tmp2.x;
      vic.at<double>(i,9*negative_normal + 1) = tmp2.y;
      vic.at<double>(i,9*negative_normal + 2) = tmp_dis2.x;
      vic.at<double>(i,9*negative_normal + 3) = tmp_dis2.y;
      vic.at<double>(i,9*negative_normal + 4) = 0.5*(erf(tmp_dis2.x/(cvSqrt(2)*sigma)) + 1);
      wp1 = (vic.at<double>(i,9*negative_normal + 4) - gamma_1)/(1-gamma_1);
      vic.at<double>(i,9*negative_normal + 5) = max(0.0, wp1*wp1*wp1*wp1*wp1*wp1);
      wp2 = (1-vic.at<double>(i,9*negative_normal + 4) - gamma_1)/(1-gamma_1);
      vic.at<double>(i,9*negative_normal + 6) = max(0.0, wp2*wp2*wp2*wp2*wp2*wp2);
      vic.at<double>(i,9*negative_normal + 7) = max((exp(-vic.at<double>(i,9*negative_normal + 2)*vic.at<double>(i,9*negative_normal + 2)/(2*sigma_t*sigma_t)) - exp(-gamma_2)), 0.0);
      vic.at<double>(i, 9*k + 8) = 0.5*exp(-abs(tmp_dis2.y)/alpha)/alpha;
      normalized_sum += vic.at<double>(i, 9*negative_normal + 7);
      cvCircle(img1, tmp2, 1, CV_RGB(0, 255, 255), 1, 8 , 0);
    }
    normalized_param[i] = normalized_sum;
  }
  for (int  i = 0; i < 2*normal_points_number; ++i){
    std::cout << vic.at<double>(0,9*i) <<  " " << vic.at<double>(0,9*i+1) << " " << vic.at<double>(0,9*i+2) <<  " " << vic.at<double>(0,9*i+3) <<  " " << vic.at<double>(0,9*i+4) <<  " " << vic.at<double>(0,9*i+5)<<  std::endl;
  }

  cvShowImage("Original",img1);
  while (1)
  {
    key = cvWaitKey(10);
    if (key == 27) break;
  }
  
  // cv::Mat sigma_phi = Mat::zeros(6,6, CV_64F);

  cv::Mat mean_vic = Mat::zeros(resolution, 6, CV_64F);
  cv::Mat cov_vic = Mat::zeros(resolution, 18, CV_64F);
  for (int i = 0; i < resolution; ++i)
  {
    int k = 0;
    double w1 =0.0 , w2 = 0.0;
    vector<double> m1(3,0.0), m2(3,0.0);
    vector<double> m1_o2(9,0.0), m2_o2(9,0.0);
    
    for (int j = dn; j <= h; j+=dn, k++){
      double wp1 = 0.0, wp2 = 0.0;
      int negative_normal = k+normal_points_number;
      wp1 = vic.at<double>(i, 9*k+ 5)*vic.at<double>(i, 9*k+ 7)*vic.at<double>(i, 9*k+ 8);
      wp2 = vic.at<double>(i, 9*k+ 6)*vic.at<double>(i, 9*k+ 7)*vic.at<double>(i, 9*k+ 8);
      w1 += wp1;
      w2 += wp2;
      m1[0] += wp1*img.at<Vec3b>(vic.at<double>(i, 9*k + 0 ), vic.at<double>(i, 9*k + 1 ))[0];
      m1[1] += wp1*img.at<Vec3b>(vic.at<double>(i, 9*k + 0 ), vic.at<double>(i, 9*k + 1 ))[1];
      m1[2] += wp1*img.at<Vec3b>(vic.at<double>(i, 9*k + 0 ), vic.at<double>(i, 9*k + 1 ))[2];
      m2[0] += wp2*img.at<Vec3b>(vic.at<double>(i, 9*k + 0 ), vic.at<double>(i, 9*k + 1 ))[0];
      m2[1] += wp2*img.at<Vec3b>(vic.at<double>(i, 9*k + 0 ), vic.at<double>(i, 9*k + 1 ))[1];
      m2[2] += wp2*img.at<Vec3b>(vic.at<double>(i, 9*k + 0 ), vic.at<double>(i, 9*k + 1 ))[2];
      for (int m = 0; m < 3; ++m)
      {
        for (int n =0; i < 3; ++n)
        {
          m1_o2[m*3+n] += wp1*img.at<Vec3b>(vic.at<double>(i, 9*k + 0 ), vic.at<double>(i, 9*k + 1 ))[m]
                          *img.at<Vec3b>(vic.at<double>(i, 9*k + 0 ), vic.at<double>(i, 9*k + 1 ))[n];
          m2_o2[m*3+n] += wp2*img.at<Vec3b>(vic.at<double>(i, 9*negative_normal + 0 ), vic.at<double>(i, 9*negative_normal + 1 ))[m]
                          *img.at<Vec3b>(vic.at<double>(i, 9*negative_normal + 0 ), vic.at<double>(i, 9*negative_normal + 1 ))[n];
        }
      }
      wp1 = vic.at<double>(i, 9*negative_normal+ 5)*vic.at<double>(i, 9*negative_normal+ 7)*vic.at<double>(i, 9*negative_normal+ 8);
      wp2 = vic.at<double>(i, 9*negative_normal+ 6)*vic.at<double>(i, 9*negative_normal+ 7)*vic.at<double>(i, 9*negative_normal+ 8);
      w1 += wp1;
      w2 += wp2;
      m1[0] += wp1*img.at<Vec3b>(vic.at<double>(i, 9*k + 0 ), vic.at<double>(i, 9*k + 1 ))[0];
      m1[1] += wp1*img.at<Vec3b>(vic.at<double>(i, 9*k + 0 ), vic.at<double>(i, 9*k + 1 ))[1];
      m1[2] += wp1*img.at<Vec3b>(vic.at<double>(i, 9*k + 0 ), vic.at<double>(i, 9*k + 1 ))[2];
      m2[0] += wp2*img.at<Vec3b>(vic.at<double>(i, 9*k + 0 ), vic.at<double>(i, 9*k + 1 ))[0];
      m2[1] += wp2*img.at<Vec3b>(vic.at<double>(i, 9*k + 0 ), vic.at<double>(i, 9*k + 1 ))[1];
      m2[2] += wp2*img.at<Vec3b>(vic.at<double>(i, 9*k + 0 ), vic.at<double>(i, 9*k + 1 ))[2];
      for (int m = 0; m < 3; ++m)
      {
        for (int n =0; i < 3; ++n)
        {
          m1_o2[m*3+n] += wp1*img.at<Vec3b>(vic.at<double>(i, 9*k + 0 ), vic.at<double>(i, 9*k + 1 ))[m]
                          *img.at<Vec3b>(vic.at<double>(i, 9*k + 0 ), vic.at<double>(i, 9*k + 1 ))[n];
          m2_o2[m*3+n] += wp2*img.at<Vec3b>(vic.at<double>(i, 9*negative_normal + 0 ), vic.at<double>(i, 9*negative_normal + 1 ))[m]
                          *img.at<Vec3b>(vic.at<double>(i, 9*negative_normal + 0 ), vic.at<double>(i, 9*negative_normal + 1 ))[n];
        }
      }
      mean_vic.at<double>(i, 0) = m1[0]/w1;
      mean_vic.at<double>(i, 1) = m1[1]/w1;
      mean_vic.at<double>(i, 2) = m1[2]/w1;
      mean_vic.at<double>(i, 3) = m2[0]/w2;
      mean_vic.at<double>(i, 4) = m2[1]/w2;
      mean_vic.at<double>(i, 5) = m2[2]/w2;
      for (int m = 0; m < 3; ++m)
      {
        for (int n = 0 ; n < 3; ++n)
        {
          cov_vic.at<double>(i, m*3+n) = m1_o2[m*3+n]/w1 -m1[m]*m1[n]/(w1*w1);
          cov_vic.at<double>(i, 9+m*3+n) = m2_o2[m*3+n]/w2 -m2[m]*m2[n]/(w2*w2);
          if(m == n)
          {
            cov_vic.at<double>(i, m*3+n) += kappa;
            cov_vic.at<double>(i, 9+m*3+n) += kappa;
          }
        }
      }
    }
  }
  
  img_map = cv::Mat::zeros(cvGetSize(img1), CV_8U);
  // determine the location of pixels in the image ,inside , outside or on the curve
    
  img_map.release();
  bs.release();
  // bs1.release();
  // delete [] points;
  cvReleaseImage(&img1);
  X.release();
  dx.release();
  dx_old.release();
  return 0;
}
