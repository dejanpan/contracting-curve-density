#include <stdlib.h>
struct Point {
	double x;
	double y;
	double z;
};
void computeIntervals(int *u, int n, int t);
double blend(int k, int t, int *u, double v);
void computePoint(int *u, int n, int t, double v, point *control, point *output);
void bspline(int n, int t, point *control, point *output, int num_output)
{
	int *u;
	double increment,interval;
	point calcxyz;
	int output_index;
    
	u=new int[n+t+1];
	computeIntervals(u, n, t);
	increment=(double) (n-t+2)/(num_output-1);  // how much parameter goes up each time
	interval=0;
	for (output_index=0; output_index<num_output-1; output_index++)
	{
		computePoint(u, n, t, interval, control, &calcxyz);
		output[output_index].x = calcxyz.x;
		output[output_index].y = calcxyz.y;
		output[output_index].z = calcxyz.z;
		interval=interval+increment;  // increment our parameter
	}
	output[num_output-1].x=control[n].x;   // put in the last point
	output[num_output-1].y=control[n].y;
	output[num_output-1].z=control[n].z;
	delete u;
}

double blend(int k, int t, int *u, double v)  // calculate the blending value
{
	double value;
	// base case for the recursion
	if (t==1)
	{
		if ((u[k]<=v) && (v<u[k+1]))
			value=1;
		else
			value=0;
	}
	else
	{
		if ((u[k+t-1]==u[k]) && (u[k+t]==u[k+1]))  // check for divide by zero
			value = 0;
		else if (u[k+t-1]==u[k]) // if a term's denominator is zero,use just the other
			value = (u[k+t] - v) / (u[k+t] - u[k+1]) * blend(k+1, t-1, u, v);
		else if (u[k+t]==u[k+1])
			value = (v - u[k]) / (u[k+t-1] - u[k]) * blend(k, t-1, u, v);
		else
			value = (v - u[k]) / (u[k+t-1] - u[k]) * blend(k, t-1, u, v) + (u[k+t] - v) / (u[k+t] - u[k+1]) * blend(k+1, t-1, u, v);
	}
	return value;
}

void computeIntervals(int *u, int n, int t)   // figure out the knots
{
	int j;
	for (j=0; j<=n+t; j++)
	{
		if (j<t)
			u[j]=0;
		else if ((t<=j) && (j<=n))
			u[j]=j-t+1;
		else if (j>n)
			u[j]=n-t+2;  // if n-t=-2 then we're screwed, everything goes to 0
	}
}

void computePoint(int *u, int n, int t, double v, point *control,
		point *output)
{
	int k;
	double temp;

	// initialize the variables that will hold our outputted point
	output->x=0;
	output->y=0;
	output->z=0;
	for (k=0; k<=n; k++)
	{
		temp = blend(k,t,u,v);  // same blend is used for each dimension coordinate
		output->x = output->x + (control[k]).x * temp;
		output->y = output->y + (control[k]).y * temp;
		output->z = output->z + (control[k]).z * temp;
	}
}

int main()
{
	int n,t;
	n=9;          // number of control Points = n+1
	t=5;           // degree of polynomial = t-1
	Point *pts;          // allocate our control Point array
	pts=new Point[n+1];
	pts[0].x=212;  pts[0].y=5;  pts[0].z=0;
	pts[1].x=72;  pts[1].y=43;  pts[1].z=0;
	pts[2].x=105;  pts[2].y=158;  pts[2].z=0;
	pts[3].x=120;  pts[3].y=276;  pts[3].z=0;
	pts[4].x=214;  pts[4].y=308;  pts[4].z=0;
	pts[5].x=314;  pts[5].y=276;  pts[5].z=0;
	pts[6].x=339;  pts[6].y=174;  pts[6].z=0;
	pts[7].x=345;  pts[7].y=27;   pts[7].z=0;
	pts[8].x=212;  pts[8].y=5;  pts[8].z=0;

	int resolution = 100; 
	point *out_pts;
	out_pts = new point[resolution];
	bspline(n, t, pts, out_pts, resolution);
	delete out_pts;
	delete pts;
	return 0;
}
