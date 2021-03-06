#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <iostream>
#include <vector>
#include <string>
#include "ccd/sift_init.h"
#include "ccd/bspline.h"
#include "ccd/ccd.h"
#include <fstream>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/range_image/range_image.h"

using namespace cv;
using namespace std;
using namespace pcl;

typedef PointWithViewpoint PointType;

std::vector<cv::Point3d> pts;
void on_mouse(int event, int x, int y, int flags, void *param )
{
  CCD *my_ccd = (CCD *)param;
  cv::Mat image = my_ccd->canvas;
  if( image.empty())
    return ;

  //caution: check
  // if( image1.at<double>() )
  //   y = image1->height - y;
  switch( event )
  {
    case CV_EVENT_LBUTTONDOWN:
      break;
    case CV_EVENT_LBUTTONUP:
      cv::circle(image,cv::Point(x,y),1,cv::Scalar(0,0,255),1);
      my_ccd->pts.push_back(cv::Point3d(x,y,1));
      cv::imshow("CCD", image);
      break;
  }
}

void contourDraw(CCD &my_ccd)
{
  my_ccd.pts.push_back(cv::Point3d(257.0,269.0,1.0));
  my_ccd.pts.push_back(cv::Point3d(259.0,296.0,1.0));
  my_ccd.pts.push_back(cv::Point3d(261.0,331.0,1.0));
  my_ccd.pts.push_back(cv::Point3d(264.0,368.0,1.0));
  my_ccd.pts.push_back(cv::Point3d(265.0,382.0,1.0));
  my_ccd.pts.push_back(cv::Point3d(276.0,389.0,1.0));
  my_ccd.pts.push_back(cv::Point3d(315.0,412.0,1.0));
  my_ccd.pts.push_back(cv::Point3d(379.0,447.0,1.0));
  my_ccd.pts.push_back(cv::Point3d(418.0,467.0,1.0));
  my_ccd.pts.push_back(cv::Point3d(427.0,459.0,1.0));
  my_ccd.pts.push_back(cv::Point3d(498.0,391.0,1.0));
  my_ccd.pts.push_back(cv::Point3d(564.0,328.0,1.0));
  my_ccd.pts.push_back(cv::Point3d(569.0,323.0,1.0));
  my_ccd.pts.push_back(cv::Point3d(570.0,315.0,1.0));
  my_ccd.pts.push_back(cv::Point3d(577.0,272.0,1.0));
  my_ccd.pts.push_back(cv::Point3d(586.0,225.0,1.0));
  my_ccd.pts.push_back(cv::Point3d(587.0,213.0,1.0));
  my_ccd.pts.push_back(cv::Point3d(577.0,211.0,1.0));
  my_ccd.pts.push_back(cv::Point3d(507.0,188.0,1.0));
  my_ccd.pts.push_back(cv::Point3d(445.0,168.0,1.0));
  my_ccd.pts.push_back(cv::Point3d(435.0,165.0,1.0));
  my_ccd.pts.push_back(cv::Point3d(428.0,168.0,1.0));
  my_ccd.pts.push_back(cv::Point3d(365.0,207.0,1.0));
  my_ccd.pts.push_back(cv::Point3d(283.0,252.0,1.0));
  my_ccd.pts.push_back(cv::Point3d(262.0,265.0,1.0));
}

void contourSift(CCD &my_ccd)
{
  int row;
  IplImage sift_tpl = my_ccd.tpl;
  IplImage sift_tpl_img = my_ccd.canvas;
  IplImage *tpl_ptr = &sift_tpl;
  IplImage *tpl_img_ptr = &sift_tpl_img;
  // CvMat points_mat = sift_init(tpl_ptr, tpl_img_ptr, 30);
  CvMat points_mat = sift_init(tpl_ptr, tpl_img_ptr, 30);
  CvMat *points_mat_ptr = &points_mat;
  double *ptr = points_mat_ptr->data.db;
  int step = points_mat.step/sizeof(double);
  for (row = 0; row < points_mat_ptr->rows; ++row)
  {
    my_ccd.pts.push_back(cv::Point3d((ptr+step*row)[0]/(ptr+step*row)[2], (ptr+step*row)[1]/(ptr+step*row)[2], 1));
  }
}

void contourPCD(CCD &my_ccd)
{
  pcl::PointCloud<PointXYZ>::Ptr cloud (new pcl::PointCloud<PointXYZ>);

  if (pcl::io::loadPCDFile<PointXYZ> ("data/contour.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    exit(-1);
  }
  std::cerr << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  float factor = 0.5*my_ccd.image.rows/40*0.6;
  double x_value =0 , y_value = 0, angle= 0.1;
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    // double x_tmp = cloud->points[i].x*factor + my_ccd.image.cols/2;
    // double y_tmp = cloud->points[i].y*factor + my_ccd.image.rows/2;
    double x_tmp = cloud->points[i].x*cos(angle) + cloud->points[i].y*sin(angle);
    double y_tmp = -cloud->points[i].x*sin(angle) + cloud->points[i].y*cos(angle);
    x_value = x_tmp*factor + my_ccd.image.cols/2;
    y_value = y_tmp*factor + my_ccd.image.rows/2;
    // std::cerr << "    " << cloud->points[i].x*factor + my_ccd.image.cols/2
    //           << " "    << cloud->points[i].y*factor + my_ccd.image.rows/2 << std::endl;
    my_ccd.pts.push_back(cv::Point3d(x_value, y_value, 1));    
  }
}

  
void contourManually(CCD &my_ccd)
{
  int key;
  cv::namedWindow("CCD", 1);
  cv::setMouseCallback( "CCD", on_mouse,  (void*)&my_ccd);
  cv::imshow("CCD", my_ccd.canvas);
  while (1)
  {
    key = cv::waitKey(10);
    if (key == 27) break;
  }
}

static int print_help()
{
  cout << "Usage:\n ./test -m init_method [-t template_image] -i input_image params.xml"<< endl;
  return 0;
}

int main (int argc, char * argv[]) 
{
  int key;
  std::vector<cv::Point2d> pts;


  string  image_path, template_path, params_file_path;
  int init_method;
  if(argc <= 1)
    return print_help();
  
  for( int i = 1; i < argc; i++ )
  {
    if( string(argv[i]) == "-m" )
    {
      if( sscanf(argv[++i], "%d", &init_method) != 1 || (init_method < 0  ||  init_method >3))
      {
        cout << "invalid initialization method" << endl;
        return print_help();
      }
    }
    else if( string(argv[i]) == "-i" )
    {
      if( string(argv[++i]).length() <= 0 )
      {
        cout << "invalid image file path" << endl;
        return print_help();
      }
      else
        image_path = argv[i];
    }
    else if( string(argv[i]) == "-t" )
    {
      if( string(argv[++i]).length() <= 0 )
      {
        cout << "invalid template image file path" << endl;
        return print_help();
      }
      else
        template_path = argv[i];
    }
    else if( string(argv[i]) == "--help" )
      return print_help();
    else if( argv[i][0] == '-' )
    {
      cout << "invalid option " << argv[i] << endl;
      return 0;
    }
    else
      params_file_path = argv[i];

  }
  if(params_file_path == "")
  {
    params_file_path = "ccd_params.xml";
  }
    
  // double *params = new double[10];

  
  CCD my_ccd;
  my_ccd.canvas = cv::imread(image_path, 1);
  my_ccd.image = cv::imread(image_path, 1);
  if(template_path != "")
    my_ccd.tpl = cv::imread(template_path, 1 );
  if(init_method == 0)
    contourManually(my_ccd);
  else if(init_method == 1)
    contourSift(my_ccd);
  else if(init_method == 2)
    contourPCD(my_ccd);
  else if(init_method == 3)
  {
    contourDraw(my_ccd);
  }
  else
  {
    print_help();
    exit(0);
  }
  // return 0;
  if((int)my_ccd.pts.size() > my_ccd.degree())
  {
    for (int i = 0; i < my_ccd.degree(); ++i)
      my_ccd.pts.push_back(my_ccd.pts[i]);
  }
  // my_ccd.init_pts(init_method);
  // std::cout << "hellooooooo" << std::endl;
  my_ccd.read_params(params_file_path);
  my_ccd.init_mat();
  
  my_ccd.run_ccd();
  cv::imshow("CCD", my_ccd.canvas);
  while (1)
  {
    key = cv::waitKey(10);
    if (key == 27) break;
  }
  // double interval = (pts.size() - params[9])/params[8];
  // std::cout << "resolution: " << params[8] << " pts_number - degree: " << (pts.size() - params[9]) << " increment: " <<  interval  << " interval " << params[8]/(pts.size() - params[9]) << std::endl;
  return 0;
}
