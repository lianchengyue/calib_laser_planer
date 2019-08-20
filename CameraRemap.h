#ifndef CAMERAREMAP_H
#define CAMERAREMAP_H

#ifndef DO_CALIB

#include "utils.h"

int initMetadata_In_CameraRemap(int pic_num, int board_size_x, int board_size_y);
int CameraRemap();
//图像去畸变
int DoRemap(cv::Mat &InputImg, cv::Mat &OutputImg);
//提取角点
int ExtractChessboard(cv::Mat &RemapedImg, std::vector<cv::Point2f> &image_points_buf);
//获取角点在世界坐标系下的3D坐标
int get3DPointSet(std::vector<cv::Point3f> &tempPointSet);
//提取R与T矩阵
int CalcRAndT(std::vector<cv::Point2f> _2dPtsInOneImg, std::vector<cv::Point3f> _3dPtsInOneImg);
#endif


#endif // CAMERAREMAP_H
