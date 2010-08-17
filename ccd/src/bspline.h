struct point {
  double x;
  double y;
};
class BSpline{
public:
    BSpline(){};
    BSpline(const BSpline &bs){};
    BSpline(int m, int n, int resolution, point *control_points, CvPoint *curve);
private:
    void computeKnots();
    void computePoint(point *control, point *p, double v);
    double blend(int k, int t, double v);
    size_t n_control_points;
    size_t n_degree;
    std::vector<int> knots;
};
