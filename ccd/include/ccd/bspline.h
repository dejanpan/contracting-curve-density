class BSpline{
public:
  BSpline();
  BSpline(const BSpline &bs){}
  BSpline(int n, int resolution, std::vector<cv::Point2d> control_points);
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
  inline cv::Point2d& operator[](const size_t index)
  {
    return curve_[index];
  }

  inline const cv::Point2d& operator[](const size_t index) const
  {
    return curve_[index];
  }

  inline cv::Point2d& dt(const size_t index)
  {
    return tangent_[index];
  }
  cv::Mat basic_mat_;
private:
  void computeKnots();
  void computePoint(std::vector<cv::Point2d> control,
                    cv::Point2d *p,
                    cv::Point2d *tangent,
                    double *mat_ptr,
                    double v,
                    int degree);
  double basic(int k, int t, double v);
  double basic(int i, int degree, double t, double *bp);
  std::vector<int> knots;
  cv::Point2d *curve_;
  cv::Point2d *tangent_;
};
