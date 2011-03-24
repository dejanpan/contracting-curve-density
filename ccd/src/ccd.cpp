#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <vector>
#include <iostream>
#include <algorithm>
#include "ccd/sift_init.h"
#include "ccd/bspline.h"
#include "ccd/ccd.h"

inline cv::Scalar random_color(CvRNG* rng)
{
  int color = cvRandInt(rng);
  return CV_RGB(color&255, (color>>8)&255, (color>>16)&255);
}


void on_mouse(int event, int x, int y, int flags, void* param )
{
  //  MaskParams* params = (MaskParams*)_params;
  
  CCD *my_ccd = (CCD *)param;
  cv::Mat image = my_ccd->canvas;
  if( image.empty())
    return;

  //caution: check
  // if( image1.at<double>() )
  //   y = image1->height - y;

  switch( event )
  {
    case CV_EVENT_LBUTTONDOWN:
      break;
    case CV_EVENT_LBUTTONUP:
      cv::circle(image,cv::Point(x,y),2,cv::Scalar(0,0,255),2);
      my_ccd->pts.push_back(cv::Point3d(x,y,1));
      cv::imshow("CCD", image);
      break;
  }
}

void CCD::read_params( const string& filename)
{
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  params_.gamma_1 = double(fs["gamma_1"]);      
  params_.gamma_2 = double(fs["gamma_2"]);      
  params_.gamma_3 = double(fs["gamma_3"]);      
  params_.gamma_4 = double(fs["gamma_4"]);      
  params_.kappa   = double(fs["kappa"]);        
  params_.c       = double(fs["c"]);            
  params_.h       = int(fs["h"]);            
  params_.delta_h = int(fs["delta_h"]);      
  params_.resolution = int(fs["resolution"]);   
  params_.degree  = int(fs["degree"]);
}

void CCD::init_cov(BSpline &bs, int degree)
{
  int n_dim = (int)pts.size() - 3;
  cv::Mat W = cv::Mat::zeros(2*n_dim,8, CV_64F);
  cv::Mat U = cv::Mat::zeros(2*n_dim, 2*n_dim, CV_64F);
  for (int i = 0; i < n_dim; ++i)
  {
    double *W_ptr = W.ptr<double>(i);
    W_ptr[0] = 1;
    W_ptr[1] = 0;
    W_ptr[2] = pts[i].x;
    W_ptr[3] = 0;
    W_ptr[4] = 0;
    W_ptr[5] = pts[i].y;
    W_ptr[6] = pts[i].z;
    W_ptr[7] = 0;
    W_ptr = W.ptr<double>(i+params_.resolution);
    W_ptr[0] = 0;
    W_ptr[1] = 1;
    W_ptr[2] = 0;
    W_ptr[3] = pts[i].y;
    W_ptr[4] = pts[i].x;
    W_ptr[5] = 0;
    W_ptr[6] = 0;
    W_ptr[7] = pts[i].z;
  }

  cv::Mat tmp_mat = cv::Mat::zeros(n_dim, n_dim, CV_64F);
  int interval = params_.resolution/(pts.size() - degree);

  for (int i = 0; i < n_dim; ++i)
  {
    double *basic_mat_ptr = bs.basic_mat_.ptr<double>(i*interval);
    for (int m = 0; m < n_dim; ++m)
    {
      double *tmp_mat_ptr = tmp_mat.ptr<double>(m);
      for (int n = 0; n < n_dim; ++n)
        tmp_mat_ptr[n] += basic_mat_ptr[m]*basic_mat_ptr[n];
    }
  }
  
  for (int i = 0; i < n_dim; ++i)
  {
    double *tmp_mat_ptr = tmp_mat.ptr<double>(i);
    double *U_ptr = U.ptr<double>(i);
    for (int j = 0; j < n_dim; ++j)
    {
      U_ptr[j] = tmp_mat_ptr[j]/n_dim;
      U_ptr = U.ptr<double>(i+n_dim);
      U_ptr[j+n_dim] = tmp_mat_ptr[j]/n_dim;
    }
  }

  cv::Mat tmp_cov;
  cv::gemm(W, U, 6/100,  cv::Mat(), 0, tmp_cov, cv::GEMM_1_T);
  cv::gemm(tmp_cov, W, 1,  cv::Mat(), 0, Sigma_Phi, 0);
  //Sigma_Phi = 6/(10*10)*W.t()*U*W;
  // cov = Nx/rou_0^2 * H
  // cov = 6/25*cov;
  // clean....
  // W.release();
  // U.release();
  // tmp_cov.release();
  // tmp_mat.release();
}


void CCD::clear()
{
  vic.release();
  mean_vic.release();
  cov_vic.release();
  nv.release();
  Phi.release();
  Sigma_Phi.release();
  delta_Phi.release();
  bs_old.release();
  nabla_E.release();
  hessian_E.release();
  image.release();
  canvas.release();
  if(!tpl.empty()) tpl.release();
}


void CCD::local_statistics(BSpline &bs)
{
  cv::Mat_<cv::Vec3b>& img = (cv::Mat_<cv::Vec3b>&)image;
  // std::cout << params_.gamma_1 << " " << params_.gamma_2 << " " << params_.gamma_3 << " " << params_.gamma_4 << std::endl;
  double sigma = params_.h/(1.3*params_.gamma_3);
  // sigma_hat = gamma_3 * sigma
  //  double sigma_hat = max(h/sqrt(2*gamma_2), gamma_4);
  double sigma_hat = params_.gamma_3*sigma + params_.gamma_4;

  // to save the normalized parameters of vic[i,8]
  // dimension: resolution x 2
  // the first column save the normalized coefficient outside the curve
  // the second column store the one inside the curve
  cv::Mat normalized_param = cv::Mat::zeros(params_.resolution, 2, CV_64F);
  
  vic = cv::Mat::zeros(params_.resolution, 20*floor(params_.h/params_.delta_h), CV_64F);

  // temporary points used to store those points in the
  // normal direction as well as negative normal direction
  cv::Point3d tmp1, tmp2;

  CvRNG rng;
  // store the distance from a point in normal(negative norml) direction
  // to the point on the curve
  cv::Point3d tmp_dis1, tmp_dis2;

  cv::Scalar color = random_color(&rng);
  for(int i=0; i < params_.resolution;i++)
  {
    cv::circle(canvas, cv::Point2d(bs[i].x, bs[i].y), 1,color, 1);
    
#ifdef DEBUG
    std::cout << bs[i].x  << " " << bs[i].y << std::endl;
    //ROS_DEBUG_STREAM(bs[i].x  << " " << bs[i].y);
#endif

    double *nv_ptr = nv.ptr<double>(i);
    // normal vector (n_x, n_y)
    // tagent vector (nv.at<double>(i,1), -n_x)
    nv_ptr[0] = -bs.dt(i).y/cvSqrt(bs.dt(i).x*bs.dt(i).x + bs.dt(i).y*bs.dt(i).y);
    nv_ptr[1] = bs.dt(i).x/cvSqrt(bs.dt(i).x*bs.dt(i).x + bs.dt(i).y*bs.dt(i).y);

    // dimension = 4
    // 0 - x value
    // 1 - y value
    // 2 - normal vector x
    // 3 - normal vector y
    bs_old = cv::Mat::zeros(params_.resolution, 4, CV_64F);
    double *bs_old_ptr = bs_old.ptr<double>(i);

    // save the old value of bspline
    bs_old_ptr[0] = bs[i].x;
    bs_old_ptr[1] = bs[i].y;

    // save the old normal vector of bspline
    bs_old_ptr[2] = nv_ptr[0];
    bs_old_ptr[3] = nv_ptr[1];
    

    // std::cout << nv_ptr[0] << " " << nv_ptr[1] << std::endl;
    int k = 0;
    double alpha = 0.5;
    double *vic_ptr = vic.ptr<double>(i);
    for (int j = params_.delta_h; j <= params_.h; j += params_.delta_h, k++)
    {
      ///////////////////////////////////////////////////////////////////////////////////////////
      // calculate in the direction +n: (n_x, n_y)
      /////////////////////////////////////////////////////////////////////////////////////////

      // x_{k,l}
      tmp1.x = round(bs[i].x + j*nv_ptr[0]);

      // y_{k,l}
      tmp1.y = round(bs[i].y + j*nv_ptr[1]);

      // distance between x_{k,l} and x_{k,0} in the normal direction
      // appoximately it is l*h, l = {1,2,3,.....}
      tmp_dis1.x = (tmp1.x-bs[i].x)*nv_ptr[0] + (tmp1.y-bs[i].y)*nv_ptr[1];

      // distance between y_{k,l} and y_{k,0} along the curve
      // it approximates 0
      tmp_dis1.y = (tmp1.x-bs[i].x)*nv_ptr[1] - (tmp1.y-bs[i].y)*nv_ptr[0];
      
      vic_ptr[10*k + 0] = tmp1.y;
      vic_ptr[10*k + 1] = tmp1.x;
      vic_ptr[10*k + 2] = tmp_dis1.x;
      vic_ptr[10*k + 3] = tmp_dis1.y;

      // fuzzy assignment a(d_{k,l}) = 1/2*(erf(d_{kl})/\sqrt(2)*sigma) + 1/2
      vic_ptr[10*k + 4] = 0.5*(erf((tmp_dis1.x)/(sqrt(2)*sigma)) + 1);

      // wp1 = (a_{d,l} - gamm_1) /(1-gamma_1)
      double wp1 = (vic_ptr[10*k + 4] - params_.gamma_1)/(1-params_.gamma_1);

      // wp1^4, why? if a_{d,l} \approx 0.5, do not count the point
      vic_ptr[10*k + 5] = wp1*wp1*wp1*wp1;

      // wp1 = (1-a_{d,l} - gamm_1) /(1-gamma_1)
      // double wp2 = (1-vic_ptr[10*k + 4] - gamma_1)/(1-gamma_1);
      double wp2 = (1-vic_ptr[10*k + 4] - 0.25);
      vic_ptr[10*k + 6] = -64*wp2*wp2*wp2*wp2 + 0.25;

      // W_p(d_p, simga_p) = c*max[0, exp(-d_p^2/2*sigma_p'^2) - exp(-gamma_2))]
      vic_ptr[10*k + 7] = std::max((exp(-0.5*tmp_dis1.x*tmp_dis1.x/(sigma_hat*sigma_hat)) - exp(-params_.gamma_2)), 0.0);

      // W' = 0.5*exp(-|d_v= - d_p=|/alpha)/alpha
      vic_ptr[ 10*k + 8] = 0.5*exp(-abs(tmp_dis1.y)/alpha)/alpha;
      
      // the derivative of col_5: 1/(sqrt(2*PI)*sigma)*exp{-d_{k,l}^2/(2*sigma*sigma)}
      vic_ptr[ 10*k + 9] = exp(-tmp_dis1.x*tmp_dis1.x/(2*sigma*sigma))/(sqrt(2*CV_PI)*sigma);

      // m1_debug[0] += img(tmp1.y, tmp1.x)[0];
      // m1_debug[1] += img(tmp1.y, tmp1.x)[1];
      // m1_debug[2] += img(tmp1.y, tmp1.x)[2];
      // calculate the normalization parameter c 
      normalized_param.at<double>(i, 0) += vic_ptr[ 10*k + 7];

        
#ifdef DEBUG
      if(i == 0)
        std::cout << "tmp1 " << tmp1.x  << " " << tmp1.y << std::endl;
#endif
      
      // cv::circle(img1, tmp1, 1, CV_RGB(0, 255, 255), 1, 8 , 0);

      // for (int m = 0; m < 3; ++m)
      // {
      //   vic_pixels.at<Vec3b>(i, k)[m] = img(tmp1.y, tmp1.x)[m];          
      // }

      ///////////////////////////////////////////////////////////////////////////////////////////
      // calculate in the direction -n: (-n_x, -n_y)
      /////////////////////////////////////////////////////////////////////////////////////////      
      tmp2.x = round(bs[i].x - j*nv_ptr[0]);
      tmp2.y = round(bs[i].y - j*nv_ptr[1]);

#ifdef DEBUG
      if(i == 0)
        std::cout << "tmp2 " << tmp2.x  << " " << tmp2.y << std::endl;
#endif

      // start compute the size in the direction of -(n_x, n_y)
      tmp_dis2.x = (tmp2.x-bs[i].x)*nv_ptr[0] + (tmp2.y-bs[i].y)*nv_ptr[1];
      tmp_dis2.y = (tmp2.x-bs[i].x)*nv_ptr[1] - (tmp2.y-bs[i].y)*nv_ptr[0];
      int negative_normal = k+ floor(params_.h/params_.delta_h);
      vic_ptr[10*negative_normal + 0] = tmp2.y;
      vic_ptr[10*negative_normal + 1] = tmp2.x;
      vic_ptr[10*negative_normal + 2] = tmp_dis2.x;
      vic_ptr[10*negative_normal + 3] = tmp_dis2.y;
      vic_ptr[10*negative_normal + 4] = 0.5*(erf(tmp_dis2.x/(cvSqrt(2)*sigma)) + 1);
      // vic_ptr[10*negative_normal + 4] = 0.5;
      wp1 = (vic_ptr[10*negative_normal + 4] - 0.25);
      vic_ptr[10*negative_normal + 5] = -64*wp1*wp1*wp1*wp1 + 0.25;
      wp2 = (1 - vic_ptr[10*negative_normal + 4] - params_.gamma_1)/(1-params_.gamma_1);
      vic_ptr[10*negative_normal + 6] = wp2*wp2*wp2*wp2;
      vic_ptr[10*negative_normal + 7] = std::max((exp(-0.5*tmp_dis2.x*tmp_dis2.x/(sigma_hat*sigma_hat)) - exp(-params_.gamma_2)), 0.0);
      vic_ptr[ 10*negative_normal + 8] = 0.5*exp(-abs(tmp_dis2.x)/alpha)/alpha;
      vic_ptr[ 10*negative_normal + 9] = exp(-tmp_dis2.x*tmp_dis2.x/(2*sigma*sigma))/(sqrt(2*CV_PI)*sigma);
      normalized_param.at<double>(i, 1) += vic_ptr[ 10*negative_normal + 7];
      // cv::circle(img1, tmp2, 1, CV_RGB(0, 255, 255), 1, 8 , 0);
      // for (int m = 0; m < 3; ++m)
      // {
      //   vic_pixels.at<Vec3b>(i, negative_normal)[m] = img(tmp2.y, tmp2.x)[m];          
      // }
    }
    // std::cout
    //     << m1_debug[0]/normal_points_number << " " 
    //     << m1_debug[1]/normal_points_number << " " 
    //     << m1_debug[2]/normal_points_number << " " 
    //     << m2_debug[0]/normal_points_number << " " 
    //     << m2_debug[1]/normal_points_number << " " 
    //     << m2_debug[2]/normal_points_number << " " 
    //     << std::endl;
  }


#ifdef DEBUG
  printf("%-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s\n",
         "x", "y", "dist_x", "dist_y", "a", "w1^4", "w2^4", "prox", "edf", "erf'"
         );
  for (int  i = 0; i < 20*floor(params_.h/params_.delta_h); ++i)
  {
    // std::cout << vic.at<double>(0,i) << "    ";
    printf("%-5f   ", vic.at<double>(0,i));
    if((i+1)%10 == 0)
      std::cout << std::endl;
  }
#endif

  for (int i = 0; i < params_.resolution; ++i)
  {
    int k = 0;
    // w1 = \sum wp_1, w2 = \sum wp_2
    double w1 =0.0 , w2 = 0.0;

    // store mean value near the curve
    std::vector<double> m1(3,0.0), m2(3,0.0);
    
    // store the second mean value near the curve
    std::vector<double> m1_o2(9,0.0), m2_o2(9,0.0);

    ////////////////////////////////////////////////////////////////////////
    // compute local statistics
    // ////////////////////////////////////////////////////////////////////
    // start search the points in the +n direction as well as -n direction
    double wp1 = 0.0, wp2 = 0.0;

    double *vic_ptr = vic.ptr<double>(i);
    double *mean_vic_ptr = mean_vic.ptr<double>(i);
    double *cov_vic_ptr = cov_vic.ptr<double>(i);
    for (int j = params_.delta_h; j <= params_.h; j += params_.delta_h, k++)
    {
      wp1 = 0.0, wp2 = 0.0;
      int negative_normal = k + floor(params_.h/params_.delta_h);
      
      // wp1 = w(a_{k,l})*w(d_{k,l})*w(d)
      wp1 = vic_ptr[ 10*k+ 5]*vic_ptr[ 10*k+ 7]/normalized_param.at<double>(i,0);

      // wp2 = w(a_{k,l})*w(d_{k,l})*w(d)
      wp2 = vic_ptr[ 10*k+ 6]*vic_ptr[ 10*k+ 7]/normalized_param.at<double>(i,1);;

      //w1 = \sum{wp1}
      w1 += wp1;

      //w2 = \sum{wp2}
      w2 += wp2;

      // compute the mean value in the vicinity of a point
      // m_{ks} = I{k}^{s} = \sum_{l} w_{kls}{I_{kl}} : s = 1 or 2

      m1[0] += wp1*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[0];
      m1[1] += wp1*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[1];
      m1[2] += wp1*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[2];
      m2[0] += wp2*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[0];
      m2[1] += wp2*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[1];
      m2[2] += wp2*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[2];

      // compute second order local statistics
      // m_{k,s} = \sum_{l} w_{kls} I_{kl}*I_{kl}^T

      for (int m = 0; m < 3; ++m)
      {
        for (int n =0; n < 3; ++n)
        {
          m1_o2[m*3+n] += wp1*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[m]
                          *img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[n];
          m2_o2[m*3+n] += wp2*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[m]
                          *img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[n];
        }
      }
        
      wp1 = vic_ptr[ 10*negative_normal+ 5]*vic_ptr[ 10*negative_normal+ 7]/normalized_param.at<double>(i,0);
      wp2 = vic_ptr[ 10*negative_normal+ 6]*vic_ptr[ 10*negative_normal+ 7]/normalized_param.at<double>(i,1);
      
      w1 += wp1;
      w2 += wp2;
      
      m1[0] += wp1*img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[0];
      m1[1] += wp1*img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[1];
      m1[2] += wp1*img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[2];
      m2[0] += wp2*img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[0];
      m2[1] += wp2*img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[1];
      m2[2] += wp2*img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[2];
      
      for (int m = 0; m < 3; ++m)
      {
        for (int n =0; n < 3; ++n)
        {
          m1_o2[m*3+n] += wp1*img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[m]
                          *img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[n];
          m2_o2[m*3+n] += wp2*img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[m]
                          *img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[n];
        }
      }
    }

    // std::cout << "w1: " << "              w2:" << std::endl;
    // std::cout << "w1 == " << w1 << "  w2== " << w2 << std::endl;
    mean_vic_ptr[ 0] = m1[0]/w1;
    mean_vic_ptr[ 1] = m1[1]/w1;
    mean_vic_ptr[ 2] = m1[2]/w1;
    mean_vic_ptr[ 3] = m2[0]/w2;
    mean_vic_ptr[ 4] = m2[1]/w2;
    mean_vic_ptr[ 5] = m2[2]/w2;
    
    for (int m = 0; m < 3; ++m)
    {
      for (int n = 0 ; n < 3; ++n)
      {
        cov_vic_ptr[ m*3+n] = m1_o2[m*3+n]/w1 -m1[m]*m1[n]/(w1*w1);
        cov_vic_ptr[ 9+m*3+n] = m2_o2[m*3+n]/w2 -m2[m]*m2[n]/(w2*w2);
        if(m == n)
        {
          cov_vic_ptr[ m*3+n] += params_.kappa;
          cov_vic_ptr[ 9+m*3+n] += params_.kappa;
        }
      }
    }
  }
  normalized_param.release();
}


void CCD::refine_parameters(BSpline &bs)
{
  cv::Mat_<cv::Vec3b>& img = (cv::Mat_<cv::Vec3b>&)image;
  cv::Mat tmp_cov(3,3,CV_64F);
  cv::Mat tmp_cov_inv(3,3,CV_64F);
  cv::Mat tmp_jacobian(6,3,CV_64F);
  cv::Mat tmp_pixel_diff(3, 1, CV_64F);
  
  for (int i = 0; i < params_.resolution; ++i)
  {

    // vic_pixels.at<Vec3b>(i,normal_points_number)[0] = img(bs[i].x, bs[i].y)[0];
    // vic_pixels.at<Vec3b>(i,normal_points_number)[1] = img(bs[i].x, bs[i].y)[1];
    // vic_pixels.at<Vec3b>(i,normal_points_number)[2] = img(bs[i].x, bs[i].y)[2];
    double *vic_ptr = vic.ptr<double>(i);
    double *nv_ptr = nv.ptr<double>(i);
    double *mean_vic_ptr = mean_vic.ptr<double>(i);
    double *cov_vic_ptr = cov_vic.ptr<double>(i);
    double normal_points_number = floor(params_.h/params_.delta_h);
    for (int j = 0; j < 2*normal_points_number; ++j)
    {
      tmp_cov = cv::Mat::zeros(3,3,CV_64F);
      tmp_cov_inv = cv::Mat::zeros(3,3,CV_64F);
      
      // \hat{}\Sigma_{kl}} = a_{kl}\Sigma_{k}^{1} + (1 - a_{kl})\Sigma_{k}^{2}
      for (int m = 0; m < 3; ++m)
      {
        double *tmp_cov_ptr = tmp_cov.ptr<double>(m);
        for (int n = 0; n < 3; ++n)
        {
          tmp_cov_ptr[n] = vic_ptr[10*j+4] * cov_vic_ptr[m*3+n]
                                     +(1-vic_ptr[10*j+4])* cov_vic_ptr[m*3+n+9];
        }
      }

      tmp_cov_inv = tmp_cov.inv(cv::DECOMP_SVD);
      // std::cout << "debug " << std::endl;

      // for (int m = 0; m < 18; ++m){
      //   std::cout << cov_vic_ptr[m] << " " ;
      // }

      // std::cout << std::endl;
      
      tmp_pixel_diff = cv::Mat::zeros(3, 1, CV_64F);

      // std::cout << " pixel_diff: " ;
      //compute the difference between I_{kl} and \hat{I_{kl}}
      for (int m = 0; m < 3; ++m)
      {
        // if(j < normal_points_number)
        //   vic_pixels.at<Vec3b>(i, j)[m] = img(vic_ptr[10*j+0], vic_ptr[10*j+1])[m];
        // else if(j >= normal_points_number)
        // {
        //   vic_pixels.at<Vec3b>(i, j+1)[m] = img(vic_ptr[10*j+0], vic_ptr[10*j+1])[m];
        //   if ( m ==0 )
        //     std::cout << "i = " << i << " bs.x " << bs[i].x << " bs.y " << bs[i].y << " vic.x "<<  vic_ptr[10*j+0]<< " vic.y " << vic_ptr[10*j+1] <<std::endl;
        // }

          
        tmp_pixel_diff.at<double>(m,0) = img(vic_ptr[10*j+0], vic_ptr[10*j+1])[m]- vic_ptr[10*j+4] * mean_vic_ptr[m]- (1-vic_ptr[10*j+4])* mean_vic_ptr[m+3];
      }
      // std::cout << std::endl;
      //compute jacobian matrix
        
      tmp_jacobian = cv::Mat::zeros(8,3,CV_64F);
 
      for (int n = 0; n < 3; ++n)
      {
        tmp_jacobian.at<double>(0,n) = vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3])*nv_ptr[0];
        tmp_jacobian.at<double>(1,n) = vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3])*nv_ptr[1];
        tmp_jacobian.at<double>(2,n) = vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3])*nv_ptr[0]*bs[i].x;
        tmp_jacobian.at<double>(3,n) = vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3])*nv_ptr[1]*bs[i].y;
        tmp_jacobian.at<double>(4,n) = vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3])*nv_ptr[1]*bs[i].x;
        tmp_jacobian.at<double>(5,n) = vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3])*nv_ptr[0]*bs[i].y;
        tmp_jacobian.at<double>(6,n) = vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3])*nv_ptr[0]*bs[i].z;
        tmp_jacobian.at<double>(7,n) = vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3])*nv_ptr[1]*bs[i].z;

        // std::cout << mean_vic_ptr[n] << " " << mean_vic_ptr[n+3] << std::endl;
      }


#ifdef DEBUG
      for (int m = 0; m < 8; ++m)
      {
        for (int n = 0; n < 3; ++n)
        {
          std::cout << tmp_jacobian.at<double>(m,n) << " ";
        }
        std::cout << std::endl;
      }
      std::cout << std::endl;
      
      for (int m = 0; m < 3; ++m)
      {
        for (int n = 0; n < 3; ++n)
        {
          std::cout << tmp_cov_inv.at<double>(m,n) << " ";
        }
        std::cout << std::endl;
      }
      std::cout << std::endl;
#endif

      //\nabla{E_2} = \sum J * \Sigma_{kl}^{-1} * (I_{kl} - \hat{I_{kl}})
      nabla_E += tmp_jacobian*tmp_cov_inv*tmp_pixel_diff;
      //Hessian{E_2} = \sum J * \Sigma_{kl}^{-1} * J
      hessian_E += tmp_jacobian*tmp_cov_inv*tmp_jacobian.t();
    }
  }

  cv::Mat Sigma_Phi_inv = Sigma_Phi.inv(cv::DECOMP_SVD);
  hessian_E += Sigma_Phi_inv;
  nabla_E += 2*Sigma_Phi_inv*Phi;

  cv::Mat hessian_E_inv = hessian_E.inv(cv::DECOMP_SVD);
  delta_Phi = hessian_E_inv*nabla_E;
  // delta_Phi.at<double>(0,0) = 0.0;
  // delta_Phi.at<double>(1,0) = 0.0;
  // delta_Phi.at<double>(2,0) = 0.0;
  // delta_Phi.at<double>(3,0) = 0.0;
  // delta_Phi.at<double>(4,0) = 0.0;
  // delta_Phi.at<double>(5,0) = 0.0;
  // #ifdef DEBUG
    std::cout << delta_Phi.at<double>(0,0) << " "
              << delta_Phi.at<double>(1,0) << " " 
              << delta_Phi.at<double>(2,0) << " " 
              << delta_Phi.at<double>(3,0) << " " 
              << delta_Phi.at<double>(4,0) << " " 
              << delta_Phi.at<double>(5,0) << " " 
              << delta_Phi.at<double>(6,0) << " " 
              << delta_Phi.at<double>(7,0) << " " 
              << std::endl;
  // #endif
  // cv::norm(delta_Phi);
  
  Phi -= delta_Phi;
  Sigma_Phi = params_.c*Sigma_Phi + 2*(1-params_.c)*hessian_E_inv;

  Sigma_Phi_inv.release();
  hessian_E_inv.release();
  tmp_cov.release();
  tmp_cov_inv.release();
  tmp_jacobian.release();
  tmp_pixel_diff.release();
}

void CCD::run_ccd()
{
  // store the control points trasformed in the shape space

  int iter = 0;
  double tol = 0.0;
  bool convergence = false;
  double norm = 0.0;

  do{ 
    // update model parameters
    // for (int i = 0; i < 6; ++i)
    //   Phi.at<double>(i,0) = Phi.at<double>(i,0) - delta_Phi.at<double>(i,0);

    // update the control points in terms of the change of
    // model parameters
    
    for (size_t i = 0; i < pts.size(); ++i)
    {
      // C = W*\Phi + C_0
      //           1  0  x_0  0  0  y_0
      //     C = [                       ][\phi_0 \phi_1 \phi_2 \phi_3 \phi_4 \phi_5 ]^T + C_0
      //           0  1  0   y_0 x_0  0
      //
      pts[i].x = Phi.at<double>(0,0) + (1+Phi.at<double>(2,0))*pts[i].x + Phi.at<double>(5,0)*pts[i].y + Phi.at<double>(6,0)*pts[i].z;
      pts[i].y = Phi.at<double>(1,0) + (1+Phi.at<double>(3,0))*pts[i].y + Phi.at<double>(4,0)*pts[i].x + Phi.at<double>(7,0)*pts[i].z;
      pts[i].z = 1;
    }

    
    nv = cv::Mat::zeros(params_.resolution, 2, CV_64F);
    mean_vic = cv::Mat::zeros(params_.resolution, 6, CV_64F);
    cov_vic = cv::Mat::zeros(params_.resolution, 18, CV_64F);
    nabla_E = cv::Mat::zeros(8,1, CV_64F);
    hessian_E = cv::Mat::zeros(8,8, CV_64F);
    // Phi.zeros(6,1,CV_64F);

    // create a new B-spline curve: degree =2


    BSpline bs(params_.degree , params_.resolution, pts);
    
    // std::cout << "BSPLINE POINTS:" << std::endl;
    for (int i = 0; i < params_.resolution; ++i)
    {
      //std::cout << bs[i].x << " " << bs[i].y << std::endl;
      cv::circle(canvas, cv::Point2d(bs[i].x, bs[i].y), 1 ,CV_RGB(0,255,0), 1); 
    }

    // converge condition
    // tol = \int (r - r_f)*n
    tol = 0.0;
    if(iter > 0)
    {
      for (int k = 0; k < params_.resolution; ++k)
      {
        tol += pow((bs[k].x - bs_old.at<double>(k, 0))*bs_old.at<double>(k, 2) +
                   (bs[k].y - bs_old.at<double>(k, 1))*bs_old.at<double>(k, 3), 2);
      }
    }
    
    //HACK!!!!!
    // bool invalid = false;
    // for (int i = 0; i < params_.resolution; i++)
    // {
    //   if (abs(bs[i].x) > 640 || abs(bs[i].y) > 480)
	//   {
	//     invalid = true;
	//     continue;
	//     // std::cerr << "bs sample : " << bs[i].x << " " << bs[i].y << std::endl;
	//   }
    // }
    // if (invalid)
    //   continue;
    //END HACK!!!!!

    local_statistics(bs);

    
    // img1.release();

    refine_parameters(bs);

    norm =  0.0;
    for (int i = 0; i < 6; ++i){
      if(i == 0 || i == 1)
        norm += delta_Phi.at<double>(i, 0)*delta_Phi.at<double>(i, 0)/10000;
      else
        norm += delta_Phi.at<double>(i, 0)*delta_Phi.at<double>(i, 0);
    }
    norm = cv::sqrt(norm);
    std::cerr << "iter: " << iter << "   tol: " << tol  << " norm: " << cv::norm(delta_Phi, cv::NORM_L2)  << " norm_tmp:" << norm<< std::endl;

// std::stringstream name;
// name << iter;
// cv::imwrite(name.str() + ".png", canvas);

    cv::imshow("CCD", canvas);    
    cv::waitKey(2);
    // cerr << std::endl;
    // std::cerr << params_.gamma_1 << " " ;
    // std::cerr << params_.gamma_2 << " " ;
    // std::cerr << params_.gamma_3 << " " ;
    // std::cerr << params_.gamma_4 << " " ;
    // std::cerr << params_.kappa << " " ;
    // std::cerr << params_.c << " " ;
    // std::cerr << params_.h << " " ;
    // std::cerr << params_.delta_h << " " ;
    // std::cerr << params_.resolution << " " ;
    // std::cerr << params_.degree << " " ;
    // std::cerr << std::endl;


    // if((tol - 0.0 < 0.001) && (norm < 0.01))
    // {
    //   convergence = true;
    //   char key;
      
    //   // while (1)
    //   // {
    //   //   key = cvWaitKey(10);
    //   //   if (key == 27)
    //   //   {
    //   //     std::cerr<< "key = " << (int)key << std::endl;
    //   //     break;          
    //   //   }
    //   // }

    //   break;
    // }
    if(iter >= 40)
    {
      convergence = true;
      init_cov(bs, params_.degree);
    }
    iter += 1;
    // bs.release();
  }while(!convergence);
}

void CCD::contour_sift()
{
  int row;
  IplImage sift_tpl = tpl;
  IplImage sift_tpl_img = canvas;
  IplImage *tpl_ptr = &sift_tpl;
  IplImage *tpl_img_ptr = &sift_tpl_img;
  // CvMat points_mat = sift_init(tpl_ptr, tpl_img_ptr, 30);
  CvMat points_mat = sift_init(tpl_ptr, tpl_img_ptr, 30);
  CvMat *points_mat_ptr = &points_mat;
  double *ptr = points_mat_ptr->data.db;
  int step = points_mat.step/sizeof(double);
  for (row = 0; row < points_mat_ptr->rows; ++row)
  {
    pts.push_back(cv::Point3d((ptr+step*row)[0]/(ptr+step*row)[2], (ptr+step*row)[1]/(ptr+step*row)[2], 1));
    // cv::circle(my_ccd.canvas, cvPoint((ptr+step*row)[0]/(ptr+step*row)[2], (ptr+step*row)[1]/(ptr+step*row)[2]), 2, CV_RGB(0,255,0), 2, 8, 0);
  }
  //clean..........
  // cvReleaseImage(&tpl_ptr);
  // cvReleaseImage(&tpl_img_ptr);
  // cvReleaseMat(&points_mat_ptr);
  // cvReleaseImageHeader(&tpl_ptr);
  // cvReleaseImageHeader(&tpl_img_ptr);
}


void CCD::contour_manually()
{
  char key;
  cv::namedWindow("CCD", 1);
  cv::setMouseCallback( "CCD", on_mouse,  (void*)this);
  cv::imshow("CCD", canvas);
  while (1)
  {
    key = cv::waitKey(10);
    if (key == 27) break;
  }
}
