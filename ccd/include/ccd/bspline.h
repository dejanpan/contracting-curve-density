class BSpline{
public:
  BSpline();
  BSpline(const BSpline &bs){}
  BSpline(int n, int resolution, std::vector<CvPoint2D64f> control_points);
  ~BSpline(){}
  void release(){
      basic_mat_.release();
      if (curve_ != NULL) delete [] curve_;
      if (curve_ != NULL) delete [] tangent_;    
  }

  /* void clear(){delete [] basic_mat_; delete [] curve_; delete [] tangent_;} */
  CvPoint& operator[](const size_t);
  const CvPoint& operator[](const size_t) const;
  CvPoint& dt(const size_t);
  cv::Mat basic_mat_;
private:
  void computeKnots();
  void computePoint(std::vector<CvPoint2D64f> control,
                    CvPoint2D64f *p,
                    CvPoint2D64f *tangent,
                    double *mat_ptr,
                    double v);
  double basic(int k, int t, double v);
  double basic(int i, double t, double *bp);
  std::vector<int> knots;
  CvPoint *curve_;
  CvPoint *tangent_;
};