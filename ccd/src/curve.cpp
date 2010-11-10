#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "bspline.h"
#include <iostream>
#include <string>
#include <cstdio>
#include <algorithm>
using namespace cv;
IplImage *img1;
inline double ccd_det(uchar *ptr, int offset)
{
  return ptr[offset+0]*(ptr[offset+4]*ptr[offset+8] - ptr[offset+5]*ptr[offset+7])
    *ptr[offset+1]*(ptr[offset+5]*ptr[offset+6] - ptr[offset+3]*ptr[offset+8])
    *ptr[offset+2]*(ptr[offset+3]*ptr[offset+7] - ptr[offset+4]*ptr[offset+6]);
}

std::vector<CvPoint2D64f> pts;


/** 
 * draw control points manually
 * 
 * @param event 
 * @param x 
 * @param y 
 * @param flags 
 * @param param 
 */
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
  img1= cvLoadImage(argv[1], 1);
  cv::Mat img(img1);
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

  // for closed curves, we have to append 3 more points
  // to the end, these 3 new points are the three one
  // located in the head of the array
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

  // update model parameters
  for (int i = 0; i < 6; ++i)
    X.at<double>(i,0) = X.at<double>(i,0) - dx.at<double>(i,0);

  // update the control points in terms of the change of
  // model parameters
  for (size_t i = 0; i < pts.size(); ++i)
  {
    pts_tmp.x = X.at<double>(0,0) + (1+X.at<double>(2,0))*pts[i].x + X.at<double>(5,0)*pts[i].y;
    pts_tmp.y = X.at<double>(1,0) + (1+X.at<double>(3,0))*pts[i].y + X.at<double>(4,0)*pts[i].x;
    pts[i].x = round(pts_tmp.x);
    pts[i].y = round(pts_tmp.y);
  }
  
  //  std::cout << "before " << std::endl;

  // create a new B-spline curve
  BSpline bs(t , resolution, pts);
  CvPoint tmp1, tmp2;
  CvPoint2D64f tmp_dis1, tmp_dis2;

  // h: search radius in the normal direction
  // dn: distance step in the normal direction
  int h = 40, dn = 4;
  
  // some useful parameters, give in hanek's paper 
  double gamma_1 = 0.5, gamma_2 = 4, gamma_3 = 6, gamma_4 = 4;
  double sigma_t = max(double(h/cvSqrt(2*gamma_2)), gamma_4);
  double sigma = sigma_t/gamma_3;
  
  // locate covariance formula:
  // Simage_v,s =  M_s^2(d^=) / omega_(d_v^=) - m_v,s * (m_v,s)^t + kappa*I
  // here, I is a identy matrix
  double kappa = 0.5;

  // set the size of dimension2 for Mat::vic
  // 8 represents x,y, dx(distance to curve), dy(distance to curve)

  // count the points in normal direction(only one side)
  int normal_points_number = floor(h/dn);
  
  double nx, ny;
  
  //vicinity matrix ,in cluding plenty amount of information
  // dimension-2: count(normal_points) * 9*2
  // col_1, col_2: coordinates of x and y
  // col_3, col_4: the distance between a normal points and the point on the curve d_v(x), d_v(y)
  // col_5: the probability P_v,1(x, m_phi, sigma_phi) = 0.5*erf(d_v(x)/(sqrt(2)*sigma_v))
  // TODO: how to calculate sigma_v
  // col_6: the probability of pixel p to belong to the desired side s.
  //        W_s(p) = max(0, [a-gamm1)/(1-gamma1)]^6)
  // col_7, col_8 : evaluates the proximity of pixel p to the curve
  //        W_p(d_p, simga_p) = c*max[0, exp(-d_p^2/2*sigma_p'^2) - exp(-gamma_2))]
  //        sigma_p' = gamma_3*sigma_p + gamma_4
  //        W_sp = W_s * W_p
  // col_9:  access the distance |d_v= - d_p=| between pixel p and pixel v along the curve
  //       W' = 0.5*exp(-|d_v= - d_p=|/alpha)/alpha
  // so last omega_ps = W_s * W' 
  cv::Mat vic = Mat::zeros(resolution, 18*normal_points_number, CV_64F);
  
  std::vector< std::vector<CvPoint2D64f> > dis(resolution);
  std::vector<double> normalized_param(resolution);
  for(int i=0;i < resolution;i++){
    cvCircle( img1, bs[i], 2, CV_RGB(0,0, 255),2);
    // normal vector (n_x, n_y)
    // tagent vector (ny, -n_x)
    nx = -bs.dt(i).y/cvSqrt(bs.dt(i).x*bs.dt(i).x + bs.dt(i).y*bs.dt(i).y);
    ny = bs.dt(i).x/cvSqrt(bs.dt(i).x*bs.dt(i).x + bs.dt(i).y*bs.dt(i).y);
    int k = 0;
    double alpha = 0.5;
    double normalized_sum = 0.0;
    for (int j = dn; j <= h; j+=dn, k++){
      // calculate in the direction (n_x, n_y)
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
      // calculate the normalization parameter c 
      normalized_sum += vic.at<double>(i, 9*k + 7);
      
      if(i == 0)
        std::cout << "tmp1 " << tmp1.x  << " " << tmp1.y << std::endl;
      
      cvCircle(img1, tmp1, 1, CV_RGB(0, 255, 255), 1, 8 , 0);
      tmp2.x = round(bs[i].x - j*nx);
      tmp2.y = round(bs[i].y - j*ny);
      if(i == 0)
        std::cout << "tmp2 " << tmp2.x  << " " << tmp2.y << std::endl;

      // start compute the size in the direction of -(n_x, n_y)
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
  for (int  i = 0; i < 2*normal_points_number; ++i)
  {
    std::cout << vic.at<double>(0,9*i) <<  " " << vic.at<double>(0,9*i+1) << " " << vic.at<double>(0,9*i+2) <<  " " << vic.at<double>(0,9*i+3) <<  " " << vic.at<double>(0,9*i+4) <<  " " << vic.at<double>(0,9*i+5)<<  std::endl;
  }

  cvShowImage("Original",img1);
  while (1)
  {
    key = cvWaitKey(10);
    if (key == 27) break;
  }
  
  // cv::Mat sigma_phi = Mat::zeros(6,6, CV_64F);

  // calculate the local statistics: mean and covariance
  // mean_vic = M_s(d_v=)/omega_s(d_v=)
  // cov_vic = M_s(d_v=)^2/omega_s(d_v=) - m_v,s * (m_v,s)' + kappa*I
  // a_vic[0] = c*Sigma_x(p_v,s(x, m_phi, sigma_phi)); a_vic[1] = 1-a_vic[0]
  // where:
  // omega_s(d_v=) = Simage(omega_p,s(d_v=))
  // M_s(d_v=) = Simga(omega_p,s(d_v=) * I_p), I_p is the pixel value in the point (x,y)
  // M_s(d_v=)^2 = Simga(omega_p,s(d_v=) * I_p*I_p'), I_p is the pixel value in the point (x,y)
  // here s = 1 or 2, where the 2rd dimesion is 3*3 and 9*3
  // we use last 3 or 9 elments to save the result
  cv::Mat mean_vic = Mat::zeros(resolution, 9, CV_64F);
  cv::Mat cov_vic = Mat::zeros(resolution, 27, CV_64F);
  cv::Mat a_vic = Mat::zeros(resolution, 2, CV_64F);
  cv::Mat mah_vic = Mat::zeros(resolution, 1, CV_64F);
  for (int i = 0; i < resolution; ++i)
  {
    int k = 0;
    double w1 =0.0 , w2 = 0.0;
    vector<double> m1(3,0.0), m2(3,0.0);
    vector<double> m1_o2(9,0.0), m2_o2(9,0.0);
    for (int j = dn; j <= h; j+=dn, k++)
    {
      double wp1 = 0.0, wp2 = 0.0;
      int negative_normal = k+normal_points_number;
      a_vic.at<double>(i,0) += vic.at<double>(i, 9*k + 4);
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
    }
    a_vic.at<double>(i, 0) /= normal_points_number;
    a_vic.at<double>(i, 1) = 1 - a_vic.at<double>(i, 0);
    mean_vic.at<double>(i, 0) = m1[0]/w1;
    mean_vic.at<double>(i, 1) = m1[1]/w1;
    mean_vic.at<double>(i, 2) = m1[2]/w1;
    mean_vic.at<double>(i, 3) = m2[0]/w2;
    mean_vic.at<double>(i, 4) = m2[1]/w2;
    mean_vic.at<double>(i, 5) = m2[2]/w2;
    mean_vic.at<double>(i, 6) = a_vic.at<double>(i, 0)*mean_vic.at<double>(i, 0)
                                + a_vic.at<double>(i,1)*mean_vic.at<double>(i, 3);
    mean_vic.at<double>(i, 7) = a_vic.at<double>(i, 0)*mean_vic.at<double>(i, 1)
                                + a_vic.at<double>(i,1)*mean_vic.at<double>(i, 4);
    mean_vic.at<double>(i, 8) = a_vic.at<double>(i, 0)*mean_vic.at<double>(i, 2)
                                + a_vic.at<double>(i,1)*mean_vic.at<double>(i, 5);
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
        cov_vic.at<double>(i, 18+m*3+n) = a_vic.at<double>(i, 0)*cov_vic.at<double>(i, m*3 + n)
                                          + a_vic.at<double>(i, 1)*cov_vic.at<double>(i, 9 + m*3 + n);
      }
    }
    for (int m = 0; m < 3; ++m)
    {
      for (int n = 0; n < 3; ++n)
      {
        mah_vic.at<double>(i, 0) += (img.at<Vec3b>(bs[i].x, bs[i].y)[m] - mean_vic.at<double>(m,6+m))
                        * cov_vic.at<double>(m,18+n*3 + m)
                        * (img.at<Vec3b>(bs[i].x, bs[i].y)[m] - mean_vic.at<double>(m,6+m));
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
