#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <iostream>
#include <vector>
#include <string>
#include "sift_init.h"
#include "ccd/bspline.h"
#include "ccd/ccd.h"
using namespace cv;
using namespace std;

int print_help()
{
  cout << "Usage:\n ./test -m init_method [-t template_image] -i input_image params.xml"<< endl;
  return 0;
}

static bool read_params( const string& filename, vector<double> &params)
{
    FileStorage fs(filename, FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != FileNode::SEQ )
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for(; it < it_end; it++ )
    {
      std::cout << (double)*it << std::endl;
      params.push_back((double)*it);
    }
    // l.push_back((double)*it);
    return true;
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
  vector<double> params(10,0.0);
  // read_params(params_file_path, params);
  params[0] = 0.5;
  params[1] = 4;
  params[2] = 4;
  params[3] = 3;
  params[4] = 0.5;
  params[5] = 0.25;
  params[6] = 40;
  params[7] = 1;
  params[8] = 100;
  params[9] = 4;   

  // for (int i = 0; i < 10; ++i){
  //   std::cout << (double)params[i] << " ";
  // }
  // cout<< std::endl;
  
  CCD my_ccd;
  my_ccd.set_params(params);
  
  my_ccd.canvas = cv::imread(image_path, 1);
  my_ccd.image = cv::imread(image_path, 1);
  if(template_path.size() > 0)
    my_ccd.tpl = cv::imread(template_path, 1 );

  my_ccd.init_pts(init_method);
  
  cv::imshow("CCD", my_ccd.canvas);

  while (1)
  {
    key = cv::waitKey(10);
    if (key == 27) break;
  }
  
  my_ccd.run_ccd();

  double interval = (pts.size() - params[9])/params[8];
  std::cout << "resolution: " << params[8] << " pts_number - degree: " << (pts.size() - params[9]) << " increment: " <<  interval  << " interval " << params[8]/(pts.size() - params[9]) << std::endl;
  return 0;
}
