class BSpline{
public:
  BSpline(){}
  BSpline(const BSpline &bs){}
  BSpline(int m, int n, int resolution, CvPoint2D64f *control_points);
  ~BSpline(){delete [] curve_; delete [] tangent_;}
  CvPoint& operator[](const size_t);
  const CvPoint& operator[](const size_t) const;
  CvPoint& dt(const size_t);
  cv::Mat basic_mat_;
private:
  void computeKnots();
  void computePoint(CvPoint2D64f *control,
                    CvPoint2D64f *p,
                    CvPoint2D64f *tangent,
                    double *mat_ptr,
                    double v);
  double basic(int k, int t, double v);
  double basic(int i, double t, double *bp);
  size_t n_control_points_;
  size_t n_order_;
  std::vector<int> knots;
  CvPoint *curve_;
  CvPoint *tangent_;
};
