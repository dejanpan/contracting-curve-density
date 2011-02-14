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
  CCDParams(): gamma_1(0.5), gamma_2(4), gamma_3(4), gamma_4(3), h(40), delta_h(1), kappa(0.5), c(0.25), resolution(50)
  {
  }
  ~CCDParams()
  {
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
  CCD(cv::Mat &i):img(i),vic(cv::Mat(params_.resolution, 20*floor(params_.h/params_.delta_h), CV_64F)), mean_vic(cv::Mat(params_.resolution, 6, CV_64F)),cov_vic(cv::Mat(params_.resolution, 18, CV_64F)), nv(cv::Mat(params_.resolution, 2, CV_64F)), Phi(cv::Mat(6,1, CV_64F)),Sigma_Phi(cv::Mat(6,6, CV_64F)), delta_Phi(cv::Mat(6,1, CV_64F)), bs_old(cv::Mat(params_.resolution, 4, CV_64F)), nabla_E(cv::Mat(6,1, CV_64F)), hessian_E(cv::Mat(6,6, CV_64F))
  {};
  void init_pts(std::vector<CvPoint2D64f> &pts);
  void run_ccd();
  ~CCD(){clear();}
private:
  void clear();
  void set_params();
  void init_cov(BSpline &bs, int degree);
  void local_statistics(BSpline &bs);
  void refine_parameters(BSpline &bs);
  cv::Mat img;
  CCDParams params_;
  std::vector<CvPoint2D64f> pts;
  cv::Mat vic;
  cv::Mat mean_vic;
  cv::Mat cov_vic;
  cv::Mat nv;
  cv::Mat Phi;
  cv::Mat Sigma_Phi;
  cv::Mat delta_Phi;
  cv::Mat bs_old;
  cv::Mat nabla_E;
  cv::Mat hessian_E;
};
