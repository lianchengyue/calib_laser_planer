#ifndef GET2DLINEARFUNC_H
#define GET2DLINEARFUNC_H

#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"


int StegerLine(std::vector<cv::Point2d> &p2d);
bool lineFit(/*const */std::vector<cv::Point2d> &points, double &a, double &b, double &c);

#endif // GET2DLINEARFUNC_H
