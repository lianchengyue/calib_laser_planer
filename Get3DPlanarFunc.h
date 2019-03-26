#ifndef GET3DPLANARFUNC_H
#define GET3DPLANARFUNC_H

#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

//空间平面方程Ax+By+Cz+D=0
typedef struct PlanarFunctionCoefficients
{
    double a;
    double b;
    double c;
    double d;
} PlanarFunctionCoefficients;

int FitPlane(const cv::Mat &points, cv::Mat& plane);

#endif // GET3DPLANARFUNC_H
