#ifndef GET2DLINEARFUNC_H
#define GET2DLINEARFUNC_H

#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

//a*x + b*y + c = 0
typedef struct LinearFuncCoefficients
{
    double a;
    double b;
    double c;
} LinearFuncCoefficients;

int StegerLine(std::vector<cv::Point2d> &p2d);
int StegerLine2(char* filename,std::vector<cv::Point2d> &p2d);
bool lineFit(/*const */std::vector<cv::Point2d> &points, double &a, double &b, double &c);

int Get2DLaserliner(int picnum);
int Get2DCornerliner(int picnum);

#endif // GET2DLINEARFUNC_H
