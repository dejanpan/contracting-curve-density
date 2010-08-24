#include <vector>
#include <cv.h>
#include "bspline.h"
double BSpline::blend(int k, int t, double v)
{
	double value;
	if (t == 0)
        ((v >= knots[k]) && (v<knots[k+1]))? value = 1 : value = 0;
    else{
        if((knots[k+t] == knots[k]) && (knots[k+t+1] == knots[k+1])) value = 0;
        else if(knots[k+t] == knots[k]) value = (knots[k+t+1] -v )/(knots[k+t+1] -knots[k+1])*blend(k+1,t -1 ,v );
        else if(knots[k+t+1] == knots[k+1]) value = (v - knots[k])/(knots[k+t] - knots[k]) * blend(k,t-1, v);
        else value = (v - knots[k])/(knots[k+t] - knots[k]) * blend(k,t-1, v) + (knots[k+t+1] -v )/(knots[k+t+1] -knots[k+1])*blend(k+1,t -1 ,v );
    }
	return value;
}

void BSpline::computeKnots()
{
    
	for (size_t j = 0; j < knots.size() ; ++j){
            if (j <= n_degree)
                knots[j] = 0;
            else if ((j > n_degree) && (j <= n_control_points))
                knots[j] = j-n_degree;
            else if (j > n_control_points)
                knots[j] = n_control_points - n_degree + 1;
            // std::cout << knots[j] << std::endl;
        }
}

void BSpline::computePoint(BSPoint *control, BSPoint *output, double v)
{
	double temp;
	// initialize the variables that will hold our outputted point
	output->x=0;
	output->y=0;
	for (size_t k = 0; k <= n_control_points; k++)
        {
            temp = blend(k, n_degree, v);
            output->x += (control[k]).x * temp;
            output->y += (control[k]).y * temp;
        }
}

BSpline::BSpline(int m, int n, int resolution, BSPoint *control)
    :n_control_points(m), n_degree(n), knots(std::vector<int>(m+n+2, 0)), curve(new CvPoint[resolution])
{
	double increment, interval;
	BSPoint tmp_point;
	int output_index;
    
	computeKnots();
	increment = (double) (m - n + 1)/(resolution - 1); // why ?
	interval = 0;    

    for (output_index = 0; output_index < resolution - 1; ++output_index){
        computePoint(control, &tmp_point, interval);
        curve[output_index].x = round(tmp_point.x);
        curve[output_index].y = round(tmp_point.y);
        interval += increment;
    }
	curve[resolution-1].x=control[m].x;
	curve[resolution-1].y=control[m].y;
}
CvPoint& BSpline::operator[](const size_t index){
    return curve[index];
}

const CvPoint& BSpline::operator[](const size_t index) const{
    return curve[index];
}
