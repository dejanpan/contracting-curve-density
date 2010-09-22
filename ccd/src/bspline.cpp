#include <vector>
#include <iostream>
#include <cv.h>
#include "bspline.h"

double BSpline::basic(int i,
                      int k,
                      double t)
{
  double value;
  if (k == 1)
    ((t >= knots[i]) && (t<knots[i+1]))? value = 1 : value = 0;
  else{
    if((knots[i+k-1] == knots[i]) && (knots[i+k] == knots[i+1])) value = 0;
    else if(knots[i+k-1] == knots[i]) value = (knots[i+k] -t )/(knots[i+k] -knots[i+1])*basic(i+1,k -1 , t);
    else if(knots[i+k] == knots[i+1]) value = (t - knots[i])/(knots[i+k-1] - knots[i]) * basic(i,k-1, t);
    else value = (t - knots[i])/(knots[i+k-1] - knots[i]) * basic(i,k-1, t) + (knots[i+k] -t )/(knots[i+k] -knots[i+1])*basic(i+1,k -1 , t);
  }
  return value;
}

double BSpline::basic(int i,
                      double t,
                      double *bp)
{
  double temp = 0;
  double t_floor = t-floor(t);
  //std::cout << " t_floor: " << t_floor<<std::endl;
  if(t -i >= 2 && t- i <= 3)
  {
    temp = 0.5*(1- t_floor)*(1-t_floor);
    *bp = t_floor - 1;
  }
  else if (t - i >= 1 && t-i< 2)
  {
    temp = -t_floor*t_floor + t_floor + 0.5;
    *bp = 1 - 2*t_floor;
  }
  else if (t-i >= 0 && t-i < 1)
  {
    temp = 0.5*t_floor*t_floor;
    *bp = t_floor;
  }
  else
  {
    temp = 0;
    *bp = 0;
  }
  return temp;
}


void BSpline::computeKnots()
{
    
  for (size_t j = 0; j < knots.size() ; ++j){
    knots[j] = j;
    // if (j < n_order)
    //     knots[j] = 0;
    // else if ((j >= n_order) && (j <= n_control_points))
    //     knots[j] = j-n_order + 1;
    // else if (j > n_control_points)
    //     knots[j] = n_control_points - n_order + 2;
    // std::cout << knots[j] << std::endl;
  }
}

void BSpline::computePoint(
    CvPoint2D64f *control,
    CvPoint2D64f *output,
    CvPoint2D64f *slope,
    double *mat_ptr,
    double t)
{
  double b = 0, bp = 0;
  // initialize the variables that will hold our outputted point
  output->x=0;
  output->y=0;
  slope->x = 0;
  slope->y = 0;
  for (size_t i = 0; i <= n_control_points_; i++)
  {
    b = basic(i, n_order_, t);
    b = basic(i, t, &bp);
    mat_ptr[i] = b;
    output->x += control[i].x * b;
    output->y += control[i].y * b;
    slope->x += (control[i]).x * bp;
    slope->y += (control[i]).y * bp;
  }
}  

BSpline::BSpline(int m,
                 int n,
                 int resolution,
                 CvPoint2D64f *control)
    :basic_mat_(cv::Mat(resolution, m+1, CV_64FC1)),
     n_control_points_(m), n_order_(n),
     knots(std::vector<int>(m+n+1, 0)),
     curve_(new CvPoint[resolution]),
     tangent_(new CvPoint[resolution])
{
  double increment, interval;
  CvPoint2D64f tmp_point, tmp_tangent;
  int i;
  computeKnots();
  increment = (double) (m - n + 1)/resolution;
  interval = 0;

  for (i = 0, interval = n-1; interval <= m ; ++i){
    double *mat_ptr = basic_mat_.ptr<double>(i);
    computePoint(control, &tmp_point, &tmp_tangent, mat_ptr, interval);
    curve_[i].x = round(tmp_point.x);
    curve_[i].y = round(tmp_point.y);
    tangent_[i].x = tmp_tangent.x;
    tangent_[i].y = tmp_tangent.y;
    interval += increment;
  }
  // curve_[resolution-1].x=control[m].x;
  // curve_[resolution-1].y=control[m].y;
}

CvPoint& BSpline::operator[](const size_t index)
{
  return curve_[index];
}

const CvPoint& BSpline::operator[](const size_t index) const
{
  return curve_[index];
}

CvPoint& BSpline::dt(const size_t index)
{
  return tangent_[index];
}
