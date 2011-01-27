#include "ccd_panin.h"
using namespace cv;
cv::Mat img1;

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
  // cvSetMouseCallback( "Original", on_mouse, 0 );
  // cvShowImage("Original",img1);
  cv::imshow("Original", img1);
  char key ;
  // while (1)
  // {
  //   key = cvWaitKey(10);
  //   if (key == 27) break;
  // }
  ////////////////////////////////////////////////////////////////
  std::vector<CvPoint2D64f> pts;  
  pts_tmp.x = 173, pts_tmp.y = 299;
  pts.push_back(pts_tmp);
  pts_tmp.x = 177, pts_tmp.y = 323;
  pts.push_back(pts_tmp);
  pts_tmp.x = 189, pts_tmp.y = 343;
  pts.push_back(pts_tmp);
  pts_tmp.x = 206, pts_tmp.y = 353;
  pts.push_back(pts_tmp);
  pts_tmp.x = 223, pts_tmp.y = 360;
  pts.push_back(pts_tmp);
  pts_tmp.x = 246, pts_tmp.y = 362;
  pts.push_back(pts_tmp);
  pts_tmp.x = 267, pts_tmp.y = 352;
  pts.push_back(pts_tmp);
  pts_tmp.x = 282, pts_tmp.y = 335;
  pts.push_back(pts_tmp);
  pts_tmp.x = 295, pts_tmp.y = 315;
  pts.push_back(pts_tmp);
  pts_tmp.x = 294, pts_tmp.y = 290;
  pts.push_back(pts_tmp);
  pts_tmp.x = 289, pts_tmp.y = 268;
  pts.push_back(pts_tmp);
  pts_tmp.x = 276, pts_tmp.y = 249;
  pts.push_back(pts_tmp);
  pts_tmp.x = 248, pts_tmp.y = 238;
  pts.push_back(pts_tmp);
  pts_tmp.x = 214, pts_tmp.y = 239;
  pts.push_back(pts_tmp);
  pts_tmp.x = 192, pts_tmp.y = 254;
  pts.push_back(pts_tmp);
  pts_tmp.x = 177, pts_tmp.y = 276;
  pts.push_back(pts_tmp);
  pts_tmp.x = 173, pts_tmp.y = 299;
  pts.push_back(pts_tmp);
  pts_tmp.x = 177, pts_tmp.y = 323;
  pts.push_back(pts_tmp);
  pts_tmp.x = 189, pts_tmp.y = 343;
  pts.push_back(pts_tmp);

  std::cout<<  "number of points: " << pts.size() << std::endl;
  // for closed curves, we have to append 3 more points
  // to the end, these 3 new points are the three one
  // located in the head of the array
  // if(pts.size() > 3)
  // {
  //   pts.push_back(pts[0]);
  //   pts.push_back(pts[1]);
  //   pts.push_back(pts[2]);
  // }

  // for debug
#ifdef DEBUG
  for (size_t i = 0; i < pts.size(); ++i)
  {
    std::cout<< pts[i].x << " " << pts[i].y << std::endl;
  }
#endif

  

  
  // model parameters, it is a 6x1 matrix
  cv::Mat Phi = Mat::zeros(6,1, CV_64F);
  // Phi.zeros(6, 1, CV_64F);
  // Phi.at<double>(3,0) = 0.25;
  // std::cout << " phi 0: " << Phi.at<double>(0,0) << " phi 1: " << Phi.at<double>(1,0) << std::endl;  
  // \delta_Phi: the difference of model parameters
  // between two iteration steps
  cv::Mat delta_Phi = Mat::zeros(6,1, CV_64F);
  delta_Phi.zeros(6,1, CV_64F);
  delta_Phi.at<double>(0,0) = -13.0;
  delta_Phi.at<double>(1,0) = -15.0;
  delta_Phi.at<double>(2,0) = 0.05;
  delta_Phi.at<double>(3,0) = 0.05;
  // delta_Phi.at<double>(4,0) = -0.01;
  // delta_Phi.at<double>(5,0) = -0.01;
  // delta_Phi.at<double>(2,0) = -0.26;
  // delta_Phi.at<double>(5,0) = 0.22;

  // covariance matrix of model parameters
  // dimension: 6x6
  
    // update model parameters
  for (int i = 0; i < 6; ++i)
    Phi.at<double>(i,0) = Phi.at<double>(i,0) - delta_Phi.at<double>(i,0);

  for (size_t i = 0; i < pts.size(); ++i)
  {
    pts_tmp.x = Phi.at<double>(0,0) + (1+Phi.at<double>(2,0))*pts[i].x + Phi.at<double>(5,0)*pts[i].y;
    pts_tmp.y = Phi.at<double>(1,0) + (1+Phi.at<double>(3,0))*pts[i].y + Phi.at<double>(4,0)*pts[i].x;

    pts[i].x = round(pts_tmp.x);
    pts[i].y = round(pts_tmp.y);
  }
  
    BSpline bs(t , resolution, pts);
  
    for(int i=0;i < resolution;i++)
    {
    
      cv::circle( img1, bs[i], 2, CV_RGB(0,0, 255),1);
    
#ifdef DEBUG
      std::cout << bs[i].x  << " " << bs[i].y << std::endl;
      //ROS_DEBUG_STREAM(bs[i].x  << " " << bs[i].y);
#endif
    }

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

  
  //main loop ended here

  Phi.release();
  delta_Phi.release();
  // img1.release();
  return 0;
}
