#include "ccd_panin.h"
using namespace cv;
/** 
 * compute the determinate  of a 3x3 matrix
 * 
 * @param ptr 
 * @param offset 
 * 
 * @return 
 */
inline double ccd_det(uchar *ptr, int offset)
{
  return ptr[offset+0]*(ptr[offset+4]*ptr[offset+8] - ptr[offset+5]*ptr[offset+7])
      *ptr[offset+1]*(ptr[offset+5]*ptr[offset+6] - ptr[offset+3]*ptr[offset+8])
      *ptr[offset+2]*(ptr[offset+3]*ptr[offset+7] - ptr[offset+4]*ptr[offset+6]);
}
inline double double_rand(double d_min, double d_max)
{
  double d = (double)rand() / RAND_MAX;
  return d_min + d * (d_max - d_min);
}

// input image
// IplImage *img1;
cv::Mat img1;

// control points initialized mannually
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
  if( img1.empty())
    return;

  //caution: check
  // if( img1.at<double>() )
  //   y = img1->height - y;

  switch( event )
  {
    case CV_EVENT_LBUTTONDOWN:
      // std::cout << "Event = " << event << std::endl;
      break;
    case CV_EVENT_LBUTTONUP:
      // std::cout << "Event = " << event << std::endl;
      cv::circle(img1,cv::Point(x,y),2,cv::Scalar(0,0,255),2);
      pts.push_back(cvPoint2D64f(x,y));
      // cvShowImage("Original",img1);
      cv::imshow("Original",img1);
      break;
  }
}

void cov_init(cv::Mat &cov, BSpline &bs, int resolution, std::vector<CvPoint2D64f> &pts, int degree)
{
  int n_dim = (int)pts.size() - 3;
  cv::Mat W = Mat::zeros(2*n_dim,6, CV_64F);
  cv::Mat U = Mat::zeros(2*n_dim, 2*n_dim, CV_64F);
  for (int i = 0; i < n_dim; ++i)
  {
    W.at<double>(i,0) = 1;
    W.at<double>(i,1) = 0;
    W.at<double>(i,2) = pts[i].x;
    W.at<double>(i,3) = 0;
    W.at<double>(i,4) = 0;
    W.at<double>(i,5) = pts[i].y;
    W.at<double>(i+resolution,0) = 0;
    W.at<double>(i+resolution,1) = 1;
    W.at<double>(i+resolution,2) = 0;
    W.at<double>(i+resolution,3) = pts[i].y;
    W.at<double>(i+resolution,4) = pts[i].x;
    W.at<double>(i+resolution,5) = 0;
  }
  cv::Mat tmp_mat = Mat::zeros(n_dim, n_dim, CV_64F);
  int interval = resolution/(pts.size() - degree);
  for (int i = 0; i < n_dim; ++i)
  {
        for (int m = 0; m < n_dim; ++m)
        {
          for (int n = 0; n < n_dim; ++n)
          {
            tmp_mat.at<double>(m,n) += bs.basic_mat_.at<double>(i*interval,m)*bs.basic_mat_.at<double>(i*interval,n);
          }
        }
  }
  for (int i = 0; i < n_dim; ++i)
  {
    for (int j = 0; j < n_dim; ++j)
    {
      U.at<double>(i,j) = tmp_mat.at<double>(i,j)/n_dim;
      U.at<double>(i+n_dim, j+n_dim) = tmp_mat.at<double>(i,j)/n_dim;
    }
  }
  cov = 6/(100*100)*W.t()*U*W;
  // cov = Nx/rou_0^2 * H
  // cov = 6/25*cov;
  W.release();
  U.release();
}

int main (int argc, char * argv[]) 
{
  // if (argc < 2)
  // {
  //   printf("Usage %s image.png \n", argv[0]);
  //   exit(0);
  // }
  // 
  // the count of points on curve, equidistant distributed
  const int resolution = 50;
  
  // the degree of B-Spline curve
  int t = 3;
  
  // load image from a specified file
  //img1= cvLoadImage(argv[1], 1);
  // img1 = imread(argv[1], 1);
  // cv::Mat img = imread(argv[1], 1);  

  img1 = imread("../data/ball.png", 1);
  cv::Mat img = imread("../data/ball.png", 1);
  // convert the image into Mat fortmat
  //cv::Mat img(img1);

  // store the control points trasformed in the shape space
  CvPoint2D64f pts_tmp;

  ////////////////////////////////////////////////////////////////
  // mannaully initialize the control points
  ///////////////////////////////////////////////////////////////
  cv::namedWindow("Original", 1);
  cvSetMouseCallback( "Original", on_mouse, 0 );
  // cvShowImage("Original",img1);
  cv::imshow("Original", img1);
  char key ;
  while (1)
  {
    key = cvWaitKey(10);
    if (key == 27) break;
  }
  ////////////////////////////////////////////////////////////////

  
  // for closed curves, we have to append 3 more points
  // to the end, these 3 new points are the three one
  // located in the head of the array
  if(pts.size() > 3)
  {
    pts.push_back(pts[0]);
    pts.push_back(pts[1]);
    pts.push_back(pts[2]);
  }

  // for debug
  //#ifdef DEBUG
  for (size_t i = 0; i < pts.size(); ++i)
    
  {
    std::cout<< pts[i].x << " " << pts[i].y << std::endl;
  }
  //#endif

  
  // model parameters, it is a 6x1 matrix
  cv::Mat Phi(6,1, CV_64F);
  // Phi.at<double>(3,0) = 0.25;
  
  // \delta_Phi: the difference of model parameters
  // between two iteration steps
  cv::Mat delta_Phi;
  delta_Phi = Mat::zeros(6,1, CV_64F);

  // covariance matrix of model parameters
  // dimension: 6x6
  cv::Mat Sigma_Phi;
  Sigma_Phi = Mat::zeros(6,6, CV_64F);

  // srand(time(0));
  // for (int m = 0; m < 6; ++m)
  // {
  //   for (int n = 0; n < 6; ++n)
  //   {
  //     if(m == n)
  //       Sigma_Phi.at<double>(m,n) = double_rand(0.1, 3.0);
  //       // Sigma_Phi.at<double>(m,n) = 1.0;
  //     printf("%-5f ", Sigma_Phi.at<double>(m,n));
  //   }
  //   std::cout << std::endl;
  // }

  // mean value of vicinity regions of points on the curve
  // dimension: resolution x 6
  // the first 3 are r,g,b mean values outside the curve
  // the last 3 are r,g,b mean values inside the curve
  cv::Mat mean_vic(resolution, 6, CV_64F);

  // covariance matrix of vicinity regions of points on the curve
  // dimension: resolution x 18
  // the first 9 (3x3) are values outside the curve
  // the last 9 (3x3) are values in -n direction
  cv::Mat cov_vic(resolution, 18, CV_64F);

  //\nabla_E = \nabla_E_1 + \nabla_E_2
  //         = 2*(\Sigma_\Phi^*)^{-1}*\Phi
  //         - \sum_{k,l}J_{a_1} \hat{\Sigma_{k,l}^{-1}} (I_{k,l} - \hat{I}_{k,l}) 
  cv::Mat nabla_E(6,1, CV_64F);
  
  //\hessian_E = hessian_E_1 + hessian_E_2
  //           = (\Sigma_\Phi^*)^{-1} +
  //           \sum_{k,l} J_{a_1} \hat{\Sigma_{k,l}^{-1}} J_{a_1}
  cv::Mat hessian_E(6,6, CV_64F);
  
  // \hat{\Sigma_{k,l}}
  cv::Mat tmp_cov(3, 3, CV_64F);
  
  // \hat{\Sigma_{k,l}}^{-1}
  cv::Mat tmp_cov_inv(3,3, CV_64F);
  
  // J_{a_1} \hat{\Sigma_{k,l}}
  cv::Mat tmp_jacobian(6,3,CV_64F);
  cv::Mat tmp_pixel_diff(3,1,CV_64F);

  cv::Mat nv(resolution, 2, CV_64F);

  // temporary points used to store those points in the
  // normal direction as well as negative normal direction
  CvPoint tmp1, tmp2;

  // store the distance from a point in normal(negative norml) direction
  // to the point on the curve
  CvPoint2D64f tmp_dis1, tmp_dis2;

  // double *basic_ptr = bs.basic_mat_.ptr<double>(0);
  // for (size_t i = 0; i < pts.size(); ++i){
  //   std::cout << basic_ptr[i] << " ";
  // }
  // std::cout << std::endl;

  // h: search radius in the normal direction
  // delta_h: distance step in the normal direction
  int h = 40, delta_h = 2;

  // sigma_hat = gamma_3 * sigma
  //  double sigma_hat = max(h/sqrt(2*gamma_2), gamma_4);
  double sigma_hat = h/2.5;

  // some useful parameters, give in hanek's paper 
  double gamma_1 = 0.5, gamma_2 = 4, gamma_3 = 4;

  //double sigma_t = max(double(h/cvSqrt(2*gamma_2)), gamma_4);
  double sigma = sigma_hat/gamma_3;
  
  // locate covariance formula:
  // Sigma_v,s =  M_s^2(d^=) / omega_(d_v^=) - m_v,s * (m_v,s)^t + kappa*I
  // here, I is a identy matrix, to avoid signular
  double kappa = 0.5;


  // parameter used to update model covariance matrix
  // an exponential decay rule
  // \Sigma_{\Phi}^{new} = c * \Sigma_{\Phi} + (1 -c)(H_{\Phi}E)^{-1}
  double c = 0.25;

  // set the size of dimension2 for Mat::vic
  // 8 represents x,y, delta_Phi(distance to curve), dy(distance to curve)

  // count the points in normal direction(only one side)
  int normal_points_number = floor(h/delta_h);


  //vicinity matrix ,in cluding plenty amount of information
  // dimension-2: count(normal_points) * 10*2
  // col_1, col_2: coordinates of x and y
  // col_3, col_4: the distance between a normal points and the point on the curve d_v(x), d_v(y)
  // col_5: the probability P_v,1(x, m_phi, sigma_hat) = 0.5*erf(d_v(x)/(sqrt(2)*sigma_hat))
  // TODO: how to calculate sigma_v
  // col_6: the probability of pixel p to belong to the desired side s.
  //        W_s(p) = max(0, [a-gamm1)/(1-gamma1)]^4)
  // col_7, col_8 : evaluates the proximity of pixel p to the curve
  //        W_p(d_p, simga_p) = c*max[0, exp(-d_p^2/2*sigma_p'^2) - exp(-gamma_2))]
  //        sigma_p' = gamma_3*sigma_p + gamma_4
  //        W_sp = W_s * W_p
  // col_9:  access the distance |d_v= - d_p=| between pixel p and pixel v along the curve
  //       W' = 0.5*exp(-|d_v= - d_p=|/alpha)/alpha
  // col_10: the derivative of col_5: 1/(sqrt(2*PI)*sigma)*exp{-d_{k,l}^2/(2*sigma_hat*sigma_hat)}
  // so last omega_ps = W_s * W' 
  cv::Mat vic(resolution, 20*normal_points_number, CV_64F);

  // to save the normalized parameters of vic[i,8]
  // dimension: resolution x 2
  // the first column save the normalized coefficient outside the curve
  // the second column store the one inside the curve
  cv::Mat normalized_param(resolution, 2, CV_64F);


  // dimension = 4
  // 0 - x value
  // 1 - y value
  // 2 - normal vector x
  // 3 - normal vector y
  cv::Mat bs_old(resolution, 4, CV_64F);

  // tolerance
  double tol = 0;
  
  //iteration count
  double iter = 0;
  
  ////////////////////////////////////////////////////////////////////////////
  //main loop starts from here
  ////////////////////////////////////////////////////////////////////////////
  
  while(iter < 100)
  {
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
      pts_tmp.x = Phi.at<double>(0,0) + (1+Phi.at<double>(2,0))*pts[i].x + Phi.at<double>(5,0)*pts[i].y;
      pts_tmp.y = Phi.at<double>(1,0) + (1+Phi.at<double>(3,0))*pts[i].y + Phi.at<double>(4,0)*pts[i].x;
      pts[i].x = round(pts_tmp.x);
      pts[i].y = round(pts_tmp.y);
    }
  
    nv = Mat::zeros(resolution, 2, CV_64F);
    mean_vic = Mat::zeros(resolution, 6, CV_64F);
    cov_vic = Mat::zeros(resolution, 18, CV_64F);
    nabla_E = Mat::zeros(6,1, CV_64F);
    hessian_E = Mat::zeros(6,6, CV_64F);

    // Phi.zeros(6,1,CV_64F);


  
    // create a new B-spline curve: degree =2
    BSpline bs(t , resolution, pts);


    if(iter == 0)
    {
      cov_init(Sigma_Phi, bs, resolution, pts, t);
      std::cout << " sigma: " << endl;
      for (int m = 0 ; m < 6; ++m)
      {
        for (int n = 0; n < 6; ++n)
        {
          printf("%-5f ", Sigma_Phi.at<double>(m,n));
        }
      }

    }


    // converge condition
    // tol = \int (r - r_f)*n
    tol = 0;
    if(iter > 0)
    {
      for (int k = 0; k < resolution; ++k)
      {
        tol += (bs[k].x - bs_old.at<double>(k, 0))*bs_old.at<double>(k, 2) +
               (bs[k].y - bs_old.at<double>(k, 1))*bs_old.at<double>(k, 3);
      }
    }

  
    for(int i=0;i < resolution;i++)
    {
    
      cv::circle( img1, bs[i], 2, CV_RGB(0,0, 255),2);
    
#ifdef DEBUG
      std::cout << bs[i].x  << " " << bs[i].y << std::endl;
      //ROS_DEBUG_STREAM(bs[i].x  << " " << bs[i].y);
#endif
    
      // normal vector (n_x, n_y)
      // tagent vector (nv.at<double>(i,1), -n_x)
      nv.at<double>(i,0) = -bs.dt(i).y/cvSqrt(bs.dt(i).x*bs.dt(i).x + bs.dt(i).y*bs.dt(i).y);
      nv.at<double>(i,1) = bs.dt(i).x/cvSqrt(bs.dt(i).x*bs.dt(i).x + bs.dt(i).y*bs.dt(i).y);

      // save the old value of bspline
      bs_old.at<double>(i,0) = bs[i].x;
      bs_old.at<double>(i,1) = bs[i].y;

      // save the old normal vector of bspline
      bs_old.at<double>(i,2) = nv.at<double>(i,0);
      bs_old.at<double>(i,3) = nv.at<double>(i,1);
    

      // std::cout << nv.at<double>(i,0) << " " << nv.at<double>(i,1) << std::endl;
      int k = 0;
      double alpha = 0.5;
      for (int j = delta_h; j <= h; j+=delta_h, k++)
      {
        ///////////////////////////////////////////////////////////////////////////////////////////
        // calculate in the direction +n: (n_x, n_y)
        /////////////////////////////////////////////////////////////////////////////////////////

        // x_{k,l}
        tmp1.x = round(bs[i].x + j*nv.at<double>(i,0));

        // y_{k,l}
        tmp1.y = round(bs[i].y + j*nv.at<double>(i,1));

        // distance between x_{k,l} and x_{k,0} in the normal direction
        // appoximately it is l*h, l = {1,2,3,.....}
        tmp_dis1.x = (tmp1.x-bs[i].x)*nv.at<double>(i,0) + (tmp1.y-bs[i].y)*nv.at<double>(i,1);

        // distance between y_{k,l} and y_{k,0} along the curve
        // it approximates 0
        tmp_dis1.y = (tmp1.x-bs[i].x)*nv.at<double>(i,1) - (tmp1.y-bs[i].y)*nv.at<double>(i,0);
      
        vic.at<double>(i,10*k + 0) = tmp1.x;
        vic.at<double>(i,10*k + 1) = tmp1.y;
        vic.at<double>(i,10*k + 2) = tmp_dis1.x;
        vic.at<double>(i,10*k + 3) = tmp_dis1.y;

        // fuzzy assignment a(d_{k,l}) = 1/2*(erf(d_{kl})/\sqrt(2)*sigma) + 1/2
        vic.at<double>(i,10*k + 4) = 0.5*(erf((tmp_dis1.x)/(sqrt(2)*sigma)) + 1);

        // wp1 = (a_{d,l} - gamm_1) /(1-gamma_1)
        double wp1 = (vic.at<double>(i,10*k + 4) - gamma_1)/(1-gamma_1);

        // wp1^4, why? if a_{d,l} \approx 0.5, do not count the point
        vic.at<double>(i,10*k + 5) = wp1*wp1*wp1*wp1;

        // wp1 = (1-a_{d,l} - gamm_1) /(1-gamma_1)
        // double wp2 = (1-vic.at<double>(i,10*k + 4) - gamma_1)/(1-gamma_1);
        double wp2 = (1-vic.at<double>(i,10*k + 4) - 0.25);
        vic.at<double>(i,10*k + 6) = -64*wp2*wp2*wp2*wp2 + 0.25;

        // W_p(d_p, simga_p) = c*max[0, exp(-d_p^2/2*sigma_p'^2) - exp(-gamma_2))]
        vic.at<double>(i,10*k + 7) = max((exp(-0.5*tmp_dis1.x*tmp_dis1.x/(sigma_hat*sigma_hat)) - exp(-gamma_2)), 0.0);

        // W' = 0.5*exp(-|d_v= - d_p=|/alpha)/alpha
        vic.at<double>(i, 10*k + 8) = 0.5*exp(-abs(tmp_dis1.y)/alpha)/alpha;
      
        // the derivative of col_5: 1/(sqrt(2*PI)*sigma)*exp{-d_{k,l}^2/(2*sigma*sigma)}
        vic.at<double>(i, 10*k + 9) = exp(-tmp_dis1.x*tmp_dis1.x/(2*sigma*sigma))/(sqrt(2*CV_PI)*sigma);
      
        // calculate the normalization parameter c 
        normalized_param.at<double>(i, 0) += vic.at<double>(i, 10*k + 7);

#ifdef DEBUG
        if(i == 0)
          std::cout << "tmp1 " << tmp1.x  << " " << tmp1.y << std::endl;
#endif
      
        cv::circle(img1, tmp1, 1, CV_RGB(0, 255, 255), 1, 8 , 0);

        ///////////////////////////////////////////////////////////////////////////////////////////
        // calculate in the direction -n: (-n_x, -n_y)
        /////////////////////////////////////////////////////////////////////////////////////////      
        tmp2.x = round(bs[i].x - j*nv.at<double>(i,0));
        tmp2.y = round(bs[i].y - j*nv.at<double>(i,1));

#ifdef DEBUG
        if(i == 0)
          std::cout << "tmp2 " << tmp2.x  << " " << tmp2.y << std::endl;
#endif

        // start compute the size in the direction of -(n_x, n_y)
        tmp_dis2.x = (tmp2.x-bs[i].x)*nv.at<double>(i,0) + (tmp2.y-bs[i].y)*nv.at<double>(i,1);
        tmp_dis2.y = (tmp2.x-bs[i].x)*nv.at<double>(i,1) - (tmp2.y-bs[i].y)*nv.at<double>(i,0);
        int negative_normal = k+normal_points_number;
        vic.at<double>(i,10*negative_normal + 0) = tmp2.x;
        vic.at<double>(i,10*negative_normal + 1) = tmp2.y;
        vic.at<double>(i,10*negative_normal + 2) = tmp_dis2.x;
        vic.at<double>(i,10*negative_normal + 3) = tmp_dis2.y;
        vic.at<double>(i,10*negative_normal + 4) = 0.5*(erf(tmp_dis2.x/(cvSqrt(2)*sigma)) + 1);
        wp1 = (vic.at<double>(i,10*negative_normal + 4) - 0.25);
        vic.at<double>(i,10*negative_normal + 5) = -64*wp1*wp1*wp1*wp1 + 0.25;
        wp2 = (1 - vic.at<double>(i,10*negative_normal + 4) - gamma_1)/(1-gamma_1);
        vic.at<double>(i,10*negative_normal + 6) = wp2*wp2*wp2*wp2;
        vic.at<double>(i,10*negative_normal + 7) = max((exp(-0.5*vic.at<double>(i,10*negative_normal + 2)*vic.at<double>(i,10*negative_normal + 2)/(sigma_hat*sigma_hat)) - exp(-gamma_2)), 0.0);
        vic.at<double>(i, 10*negative_normal + 8) = 0.5*exp(-abs(tmp_dis2.x)/alpha)/alpha;
        vic.at<double>(i, 10*negative_normal + 9) = exp(-tmp_dis2.x*tmp_dis2.x/(2*sigma*sigma))/(sqrt(2*CV_PI)*sigma);
        //      vic.at<double>(i, 10*k + 10) = ;
        normalized_param.at<double>(i, 1) += vic.at<double>(i, 10*negative_normal + 7);
        cv::circle(img1, tmp2, 1, CV_RGB(0, 255, 255), 1, 8 , 0);
      }
    }
  

    //#ifdef DEBUG
    printf("%-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s\n",
           "x", "y", "dist_x", "dist_y", "a", "w1^4", "w2^4", "prox", "edf", "erf'"
           );
    for (int  i = 0; i < 20*normal_points_number; ++i)
    {
      // std::cout << vic.at<double>(0,i) << "    ";
      printf("%-5f   ", vic.at<double>(0,i));
      if((i+1)%10 == 0)
        std::cout << std::endl;
    }
    //#endif
  
    ///////////////////////////////////////////////////////////////////////////////////
    // cvShowImage("Original",img1);
    cv::imshow("Original", img1);
  
  
    while (1)
    {
      key = cvWaitKey(10);
      if (key == 27) break;
    }
    ///////////////////////////////////////////////////////////////////////////////
  
    // cv::Mat sigma_phi(6,6, CV_64F);

    // calculate the local statistics: mean and covariance
    // mean_vic = M_s(d_v=)/omega_s(d_v=)
    // cov_vic = M_s(d_v=)^2/omega_s(d_v=) - m_v,s * (m_v,s)' + kappa*I
    // a_vic[0] = c*Sigma_x(p_v,s(x, m_phi, sigma_phi)); a_vic[1] = 1-a_vic[0]
    // where:
    // omega_s(d_v=) = Simage(omega_p,s(d_v=))
    // M_s(d_v=) = Simga(omega_p,s(d_v=) * I_p), I_p is the pixel value in the point (x,y)
    // M_s(d_v=)^2 = Simga(omega_p,s(d_v=) * I_p*I_p'), I_p is the pixel value in the point (x,y)
    // here s = 1 or 2, where the 2rd dimesion is 3*3 and 10*3
    // we use last 3 or 10 elments to save the result

  
    for (int i = 0; i < resolution; ++i)
    {
      int k = 0;
      // w1 = \sum wp_1, w2 = \sum wp_2
      double w1 =0.0 , w2 = 0.0;

      // store mean value near the curve
      vector<double> m1(3,0.0), m2(3,0.0);
    
      // store the second mean value near the curve
      vector<double> m1_o2(9,0.0), m2_o2(9,0.0);

      ////////////////////////////////////////////////////////////////////////
      // compute local statistics
      // ////////////////////////////////////////////////////////////////////
      // start search the points in the +n direction as well as -n direction
      for (int j = delta_h; j <= h; j+=delta_h, k++)
      {
        double wp1 = 0.0, wp2 = 0.0;
      
        int negative_normal = k + normal_points_number;
      
        // wp1 = w(a_{k,l})*w(d_{k,l})*w(d)
        wp1 = vic.at<double>(i, 10*k+ 5)*vic.at<double>(i, 10*k+ 7)*vic.at<double>(i, 10*k+ 8) / normalized_param.at<double>(i,0);

        // wp2 = w(a_{k,l})*w(d_{k,l})*w(d)
        wp2 = vic.at<double>(i, 10*k+ 6)*vic.at<double>(i, 10*k+ 7)*vic.at<double>(i, 10*k+ 8) / normalized_param.at<double>(i,1);

        //w1 = \sum{wp1}
        w1 += wp1;

        //w2 = \sum{wp2}
        w2 += wp2;

        // compute the mean value in the vicinity of a point
        // m_{ks} = I{k}^{s} = \sum_{l} w_{kls}{I_{kl}} : s = 1 or 2
        m1[0] += wp1*img.at<Vec3b>(vic.at<double>(i, 10*k + 0 ), vic.at<double>(i, 10*k + 1 ))[0];
        m1[1] += wp1*img.at<Vec3b>(vic.at<double>(i, 10*k + 0 ), vic.at<double>(i, 10*k + 1 ))[1];
        m1[2] += wp1*img.at<Vec3b>(vic.at<double>(i, 10*k + 0 ), vic.at<double>(i, 10*k + 1 ))[2];
        m2[0] += wp2*img.at<Vec3b>(vic.at<double>(i, 10*k + 0 ), vic.at<double>(i, 10*k + 1 ))[0];
        m2[1] += wp2*img.at<Vec3b>(vic.at<double>(i, 10*k + 0 ), vic.at<double>(i, 10*k + 1 ))[1];
        m2[2] += wp2*img.at<Vec3b>(vic.at<double>(i, 10*k + 0 ), vic.at<double>(i, 10*k + 1 ))[2];

        // compute second order local statistics
        // m_{k,s} = \sum_{l} w_{kls} I_{kl}*I_{kl}^T

        for (int m = 0; m < 3; ++m)
        {
          for (int n =0; n < 3; ++n)
          {
            m1_o2[m*3+n] += wp1*img.at<Vec3b>(vic.at<double>(i, 10*k + 0 ), vic.at<double>(i, 10*k + 1 ))[m]
                            *img.at<Vec3b>(vic.at<double>(i, 10*k + 0 ), vic.at<double>(i, 10*k + 1 ))[n];
            m2_o2[m*3+n] += wp2*img.at<Vec3b>(vic.at<double>(i, 10*k + 0 ), vic.at<double>(i, 10*k + 1 ))[m]
                            *img.at<Vec3b>(vic.at<double>(i, 10*k + 0 ), vic.at<double>(i, 10*k + 1 ))[n];
          }
        }

        wp2 = vic.at<double>(i, 10*negative_normal+ 5)*vic.at<double>(i, 10*negative_normal+ 7)*vic.at<double>(i, 10*negative_normal+ 8);
        wp1 = vic.at<double>(i, 10*negative_normal+ 6)*vic.at<double>(i, 10*negative_normal+ 7)*vic.at<double>(i, 10*negative_normal+ 8);
      
        w1 += wp1;
        w2 += wp2;
      
        m1[0] += wp1*img.at<Vec3b>(vic.at<double>(i, 10*negative_normal + 0 ), vic.at<double>(i, 10*negative_normal + 1 ))[0];
        m1[1] += wp1*img.at<Vec3b>(vic.at<double>(i, 10*negative_normal + 0 ), vic.at<double>(i, 10*negative_normal + 1 ))[1];
        m1[2] += wp1*img.at<Vec3b>(vic.at<double>(i, 10*negative_normal + 0 ), vic.at<double>(i, 10*negative_normal + 1 ))[2];
        m2[0] += wp2*img.at<Vec3b>(vic.at<double>(i, 10*negative_normal + 0 ), vic.at<double>(i, 10*negative_normal + 1 ))[0];
        m2[1] += wp2*img.at<Vec3b>(vic.at<double>(i, 10*negative_normal + 0 ), vic.at<double>(i, 10*negative_normal + 1 ))[1];
        m2[2] += wp2*img.at<Vec3b>(vic.at<double>(i, 10*negative_normal + 0 ), vic.at<double>(i, 10*negative_normal + 1 ))[2];
      
        for (int m = 0; m < 3; ++m)
        {
          for (int n =0; n < 3; ++n)
          {
            m1_o2[m*3+n] += wp1*img.at<Vec3b>(vic.at<double>(i, 10*negative_normal + 0 ), vic.at<double>(i, 10*negative_normal + 1 ))[m]
                            *img.at<Vec3b>(vic.at<double>(i, 10*negative_normal + 0 ), vic.at<double>(i, 10*negative_normal + 1 ))[n];
            m2_o2[m*3+n] += wp2*img.at<Vec3b>(vic.at<double>(i, 10*negative_normal + 0 ), vic.at<double>(i, 10*negative_normal + 1 ))[m]
                            *img.at<Vec3b>(vic.at<double>(i, 10*negative_normal + 0 ), vic.at<double>(i, 10*negative_normal + 1 ))[n];
          }
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

    //debug
#ifdef DEBUG
    for (int i = 0; i < resolution; ++i)
    {
      std::cout << mean_vic.at<double>(i, 0) << " "
                << mean_vic.at<double>(i, 1) << " "
                << mean_vic.at<double>(i, 2) << " "
                << mean_vic.at<double>(i, 3) << " "
                << mean_vic.at<double>(i, 4) << " "
                << mean_vic.at<double>(i, 5) << " "
                << std::endl;
    }
#endif

    ////////////////////////////////////////////////////////////////////////////////
    //end to compute local statistics
    ///////////////////////////////////////////////////////////////////////////////

  
    for (int i = 0; i < resolution; ++i)
    {
      for (int j = 0; j < 2*normal_points_number; ++j)
      {
        tmp_cov = Mat::zeros(3,3,CV_64F);
        tmp_cov_inv = Mat::zeros(3,3,CV_64F);
      
        // \hat{}\Sigma_{kl}} = a_{kl}\Sigma_{k}^{1} + (1 - a_{kl})\Sigma_{k}^{2}
        for (int m = 0; m < 3; ++m)
        {
          for (int n = 0; n < 3; ++n)
          {
            tmp_cov.at<double>(m, n) = vic.at<double>(i,10*j+4) * cov_vic.at<double>(i,m*3+n)
                                       +(1-vic.at<double>(i,10*j+4))* cov_vic.at<double>(i,m*3+n+9);
          }
        }

        tmp_cov_inv = tmp_cov.inv(DECOMP_SVD);
        // std::cout << "debug " << std::endl;
        
      
        tmp_pixel_diff = Mat::zeros(3, 1, CV_64F);


        // std::cout << " pixel_diff: " ;
        //compute the difference between I_{kl} and \hat{I_{kl}}
        for (int m = 0; m < 3; ++m)
        {
          tmp_pixel_diff.at<double>(m,0) = img.at<Vec3b>(vic.at<double>(i,10*j+0), vic.at<double>(i,10*j+1))[m]- vic.at<double>(i,10*j+4) * mean_vic.at<double>(i,m)- (1-vic.at<double>(i,10*j+4))* mean_vic.at<double>(i,m+3);
        }
        // std::cout << std::endl;
        //compute jacobian matrix
        
        tmp_jacobian = Mat::zeros(6,3,CV_64F); 
        for (int n = 0; n < 3; ++n)
        {
          tmp_jacobian.at<double>(0,n) = vic.at<double>(i, 10*j + 9)*(mean_vic.at<double>(i, n) - mean_vic.at<double>(i,n+3))*nv.at<double>(i,0);
          tmp_jacobian.at<double>(1,n) = vic.at<double>(i, 10*j + 9)*(mean_vic.at<double>(i, n) - mean_vic.at<double>(i,n+3))*nv.at<double>(i,1);
          tmp_jacobian.at<double>(2,n) = vic.at<double>(i, 10*j + 9)*(mean_vic.at<double>(i, n) - mean_vic.at<double>(i,n+3))*nv.at<double>(i,0)*bs[i].x;
          tmp_jacobian.at<double>(3,n) = vic.at<double>(i, 10*j + 9)*(mean_vic.at<double>(i, n) - mean_vic.at<double>(i,n+3))*nv.at<double>(i,1)*bs[i].y;
          tmp_jacobian.at<double>(4,n) = vic.at<double>(i, 10*j + 9)*(mean_vic.at<double>(i, n) - mean_vic.at<double>(i,n+3))*nv.at<double>(i,1)*bs[i].x;
          tmp_jacobian.at<double>(5,n) = vic.at<double>(i, 10*j + 9)*(mean_vic.at<double>(i, n) - mean_vic.at<double>(i,n+3))*nv.at<double>(i,0)*bs[i].y;
          // std::cout << mean_vic.at<double>(i,n) << " " << mean_vic.at<double>(i,n+3) << std::endl;
        }


#ifdef DEBUG
        for (int m = 0; m < 6; ++m)
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

  
    hessian_E += Sigma_Phi.inv(DECOMP_SVD);
    nabla_E += 2*Sigma_Phi.inv(DECOMP_SVD)*Phi;
  
    delta_Phi = hessian_E.inv(DECOMP_SVD)*nabla_E;

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
              << std::endl;
    // #endif

    Phi -= delta_Phi;
    Sigma_Phi = c*Sigma_Phi + (1-c)*hessian_E.inv(DECOMP_SVD);


    std::cout << "tol : " << tol << std::endl;
    bs.release();
    iter += 1;
    cv::imwrite("res1.png", img);
  }
  //main loop ended here

  bs_old.release();
  Phi.release();
  delta_Phi.release();
  Sigma_Phi.release();
  nv.release();
  mean_vic.release();
  cov_vic.release();
  nabla_E.release();
  hessian_E.release();
  tmp_cov.release();
  tmp_cov_inv.release();
  tmp_jacobian.release();
  tmp_pixel_diff.release();
  // img1.release();
  return 0;
}
