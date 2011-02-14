#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "bspline.h"
#include <iostream>
#include <string>
#include <cstdio>
#include <algorithm>
using namespace cv;
struct CCDParams
{
  CCDParams(): gamma_1(0.5), gamma_2(4), gamma_3(4), gamma_4(3), h(40), delta_h(1), kappa(0.5), c(0.25), resolution(50), vic(Mat::zeros(resolution, 20*floor(h/delta_h), CV_64F)), Phi(Mat::zeros(6,1, CV_64F)), Sigma_Phi(Mat::zeros(6,6, CV_64F))
  {
 }
  ~CCDParams()
  {
    vic.release();
    Phi.release();
    Sigma_Phi.release();
  }
  double gamma_1;
  double gamma_2;
  double gamma_3;
  double gamma_4;
  int h;
  int delta_h;
  double kappa;
  double c;
  int resolution;
};

class CCD
{
public:
  CCD();
  ~CCD(){clear();}
private:
  void clear();
  void set_params();
  void init_cov(BSpline &bs, std::vector<CvPoint2D64f> &pts, int degree);
  void local_statistics(BSpline &bs);
  void refine_parameters(BSpline &bs);
  void run_ccd();
  CCDParams params_;
  cv::Mat img;
  cv::Mat vic;
  cv::Mat mean_vic;
  cv::Mat cov_vic;
  cv::Mat nv;
  cv::Mat Phi;
  cv::Mat Sigma_Phi;
  cv::Mat bs_old;
  cv::Mat nabla_E;
  cv::Mat hessian_E;
  cv::Mat delta_Phi;
};

