struct CCDParams
{
 CCDParams(): gamma_1(0.5), gamma_2(4), gamma_3(4), gamma_4(3), kappa(0.5),c(0.25), h(40), delta_h(1),resolution(100), degree(4)
  {
  }
  CCDParams(double p1,
            double p2,
            double p3,
            double p4,
            double p5,
            double p6,
            int p7,
            int p8,
            int p9,
            int p10
            )
  {
    gamma_1 = p1;
    gamma_2 = p2;
    gamma_3 = p3;
    gamma_4 = p4;
    kappa = p5;
    c = p6;
    h = p7;
    delta_h = p8;
    resolution = p9;
    degree = p10;
  }

  ~CCDParams()
  {
  }
  double gamma_1;
  double gamma_2;
  double gamma_3;
  double gamma_4;
  double kappa;
  double c;
  int h;
  int delta_h;
  int resolution;
  int degree;
};

class CCD
{
public:
  cv::Mat image, canvas, tpl;
  std::vector<cv::Point2d> pts;
CCD():Phi(cv::Mat::zeros(6,1, CV_64F)),Sigma_Phi(cv::Mat::zeros(6,6, CV_64F)), delta_Phi(cv::Mat::zeros(6,1, CV_64F))
  {};
  void read_params( const string& filename);
  void run_ccd();
  double get_resolution(){return params_.resolution;}
  inline void init_pts(int init_method);
  void init_cov(BSpline &bs, int degree);
  ~CCD(){clear();}
private:
  void clear();
  void contour_sift();
  void contour_manually();
  void local_statistics(BSpline &bs);
  void refine_parameters(BSpline &bs);
  /* void on_mouse( int event, int x, int y, int flags, void* param ); */
  CCDParams params_;
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

inline void CCD::init_pts(int init_method)
{
  if(init_method == 1)    
    contour_manually();
  else if(init_method == 2)
    contour_sift();
  if((int)pts.size() > params_.degree)
  {
    for (int i = 0; i < params_.degree; ++i)
      pts.push_back(pts[i]);
  }
}
