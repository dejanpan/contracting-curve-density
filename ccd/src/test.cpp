#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <iostream>
#include <vector>
#include <string>
#include "ccd/sift_init.h"
#include "ccd/bspline.h"
#include "ccd/ccd.h"
#include <fstream>

using namespace cv;
using namespace std;

int print_help()
{
  cout << "Usage:\n ./test -m init_method [-t template_image] -i input_image params.xml"<< endl;
  return 0;
}

int main (int argc, char * argv[]) 
{
  char key;
  std::vector<cv::Point2d> pts;


  string  image_path, template_path, params_file_path;
  int init_method;
  if(argc <= 1)
    return print_help();
  
  for( int i = 1; i < argc; i++ )
  {
    if( string(argv[i]) == "-m" )
    {
      if( sscanf(argv[++i], "%d", &init_method) != 1 || (init_method <= 0  ||  init_method >=3))
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
  my_ccd.init_pts(init_method);
  // std::cout << "hellooooooo" << std::endl;
  my_ccd.read_params(params_file_path);
  
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
