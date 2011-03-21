class BSpline{
public:
  BSpline();
  BSpline(const BSpline &bs){}
  BSpline(int n, int resolution, std::vector<CvPoint2D64f> control_points);
  ~BSpline();
  /* 
   * void release(){
   *     /\* basic_mat_.release(); *\/
   *     /\* 
   *      * if (curve_ != NULL) delete [] curve_;
   *      * if (tangent_ != NULL) delete [] tangent_;    
   *      *\/
   * }
   */

  /* void clear(){delete [] basic_mat_; delete [] curve_; delete [] tangent_;} */
  inline CvPoint& operator[](const size_t index)
  {
    return curve_[index];
  }

  inline const CvPoint& operator[](const size_t index) const
  {
    return curve_[index];
  }

  inline CvPoint& dt(const size_t index)
  {
    return tangent_[index];
  }
  cv::Mat basic_mat_;
private:
  void computeKnots();
  void computePoint(std::vector<CvPoint2D64f> control,
                    CvPoint2D64f *p,
                    CvPoint2D64f *tangent,
                    double *mat_ptr,
                    double v,
                    int degree);
  double basic(int k, int t, double v);
  double basic(int i, int degree, double t, double *bp);
  std::vector<int> knots;
  CvPoint *curve_;
  CvPoint *tangent_;
};
