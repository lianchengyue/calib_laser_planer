#ifndef UTILS_H
#define UTILS_H

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#ifdef HOME
#define INPUT_FILE "calibdata_home.txt"
#define OUTPUT_FILE "calibdata_home_result.txt"
//横竖方向的角点数
#define BOARD_SIZE_X 9
#define BOARD_SIZE_Y 6
#define GRID_WIDTH 23  //Unit:mm
#define GRID_HEIGHT 23  //Unit:mm
#define PICTURE_COUNT 6

#else
#define INPUT_FILE "calibdata_usb_without_laser.txt"
#define OUTPUT_FILE "calibdata_usb_without_laser_result.txt"
//横竖方向的角点数
#define BOARD_SIZE_X 11
#define BOARD_SIZE_Y 8
#define GRID_WIDTH 10  //Unit:mm
#define GRID_HEIGHT 10  //Unit:mm
#define PICTURE_COUNT 5
#endif

#define IMG_WIDTH 1280
#define IMG_HEIGHT 960

typedef struct _MetadataFromPic
{
    //2D
    std::vector<std::vector<cv::Point2f>> mPic_2DChessboardPointSet;
    //3D
    std::vector<std::vector<cv::Point3f>> mPic_WorldChessboardPointSet;  //(0,0,0)~(0,m,n)//获取到的float型
    std::vector<std::vector<cv::Point3d>> mPic_CameraChessboardPointSet;  //(0,0,0)~(0,m,n)
    //3d Mat
    std::vector<std::vector<cv::Mat>> mPic_WorldChessboard4NumPointSet;  //(0,0,0,1)~(0,m,n,1) //Mat PointW
    std::vector<std::vector<cv::Mat>> mPic_CameraChessboard4NumPointSet;  //在相机坐标系的位置

    std::vector<cv::Mat> mPic_rvecsMatSet;  /* 所有图像的旋转向量集合 */
    std::vector<cv::Mat> mPic_rotationMatrixSet;  /* 所有图像的旋转矩阵集合 */
    std::vector<cv::Mat> mPic_tvecsMatSet; /* 所有图像的平移向量集合 */
    cv::Mat mPic_Intrinsics;// Mat(3,3); /* 摄像机内参数矩阵 */
    cv::Mat mPic_DistCoeffs;// Mat(1,5);
} MetadataFromPic;


#endif // UTILS_H
