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

bool read_params( const string& filename, vector<double> &params)
{
    // FileStorage fs(filename, FileStorage::READ);
    // if( !fs.isOpened() )
    //     return false;
    // FileNode n = fs["paramlist"];
    // if( n.type() != FileNode::SEQ )
    //     return false;
    // FileNodeIterator it = n.begin(), it_end = n.end();
    // for(; it < it_end; it++ )
    // {
    //   // std::cout << (double)*it << std::endl;
    //   params.push_back((double)*it);
    // }
    // l.push_back((double)*it);
  // ifstream in(filename.c_str());
  // string line, word;
  // int line_number = 0;
  // while(getline(in, line)){
  //   istringstream line_string(line);
  //   int index = 0;
  //   while(line_string >> word)
  //   {
  //     params[index++] = strtod(word.c_str(), NULL);
  //     // std::cout << strtod(word.c_str()) << std::endl;
  //   }
  //   line_number++;
  // }
  FILE *fp = fopen(filename.c_str(), "r");
  fscanf(fp, "%lf", &params[0]);
  fscanf(fp, "%lf", &params[1]);
  fscanf(fp, "%lf", &params[2]);
  fscanf(fp, "%lf", &params[3]);
  fscanf(fp, "%lf", &params[4]);
  fscanf(fp, "%lf", &params[5]);
  fscanf(fp, "%lf", &params[6]);
  fscanf(fp, "%lf", &params[7]);
  fscanf(fp, "%lf", &params[8]);
  fscanf(fp, "%lf", &params[9]);
  // fclose(fp);
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
  read_params(params_file_path, params);
  
  // sleep(2);
  // vector<double> params(10,0.0);
  // params[0] = 0.5;
  // params[1] = 4;
  // params[2] = 4;
  // params[3] = 3;
  // params[4] = 0.5;
  // params[5] = 0.25;
  // params[6] = 40;
  // params[7] = 1;
  // params[8] = 100;
  // params[9] = 4;  

  // std::cout << params.size() << std::endl;
  // cout<< std::endl;
  
  CCD my_ccd;
  
  
  my_ccd.canvas = cv::imread(image_path, 1);
  my_ccd.image = cv::imread(image_path, 1);
  if(template_path != "")
    my_ccd.tpl = cv::imread(template_path, 1 );
  my_ccd.init_pts(init_method);
  // std::cout << "hellooooooo" << std::endl;

    my_ccd.set_params(params);

  for (int i = 0; i < 10; ++i)
    cerr << (double)params[i] << " ";

  // sleep(20);
  
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
