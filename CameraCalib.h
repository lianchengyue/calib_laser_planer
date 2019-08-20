#ifndef CAMERACALIB_H
#define CAMERACALIB_H

#ifdef DO_CALIB

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "utils.h"

int initMetadata(int pic_num, int board_size_x, int board_size_y);
int CameraCalib();
#endif

#endif // CAMERACALIB_H
