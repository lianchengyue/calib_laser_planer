#ifndef LINECROSS_H
#define LINECROSS_H

#include <iostream>

#include <vector>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include  "Get2DLinearFunc.h"//for LinearFuncCoefficients

using namespace std;

int GetCrossPoint(vector<vector<LinearFuncCoefficients>> mycornerPointLineCoeffs,
                  LinearFuncCoefficients mylaserLineCoeffs,
                  vector<vector<cv::Point2d>>& cross_PtSet,
                  int PicNum,
                  int board_size_y
                  );

#endif // LINECROSS_H
