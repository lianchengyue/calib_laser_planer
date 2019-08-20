#include <iostream>
#include <fstream>

#include "CameraRemap.h"

#ifndef DO_CALIB
MetadataFromPic metadata_of_pic;
cv::Size image_size;  /* 图像的尺寸 */

int initMetadata_In_CameraRemap(int pic_num, int board_size_x, int board_size_y)
{
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3)
                         <<
                         2232.550777775964, 0, 669.3747410537889,
                         0, 2231.856456030659, 473.3409386246501,
                         0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<double>(5, 1)
                         <<
                         -0.09435279920653783, 0.2562075510521074, 0.0001898762951137649, 0.0005216752373549194, -0.7544675722826982);

    //metadata_of_pic.mPic_Intrinsics = cv::Mat(3,3,CV_32FC1,cv::Scalar::all(0));
    //metadata_of_pic.mPic_DistCoeffs = cv::Mat(1,5,CV_32FC1,cv::Scalar::all(0));

    metadata_of_pic.mPic_Intrinsics = cameraMatrix;
    metadata_of_pic.mPic_DistCoeffs = distCoeffs;

    return 0;
}

int CameraRemap()
{
    //实际测量得到的标定板上每个棋盘格的大小
    cv::Size square_size = cv::Size(GRID_WIDTH, GRID_HEIGHT);
    //保存第i张图像中检测到的2D角点
    std::vector<cv::Point2f> _2dPtsInOneImg;
    //保存缓存第i张图像中检测到的角点的3D坐标
    std::vector<cv::Point3f> _3dPtsInOneImg;
    //std::vector<std::vector<cv::Point3f>> object_points;

    int ret = 0;
    //矫正之后输出的图像，也是图像处理用来输入的图像
    cv::Mat rectifyImage;

    //输入文件路径
    std::ifstream fin(INPUT_FILE);

    std::string filename[PICTURE_COUNT];
    //图像数量
    int image_count=0;

    //Init Metadata
    initMetadata_In_CameraRemap(PICTURE_COUNT, BOARD_SIZE_X, BOARD_SIZE_Y);

    //加载图像
    while(getline(fin, filename[image_count]))
    {
        image_count++;
        cv::Mat imageInput = cv::imread(filename[image_count-1]);

        //1:对加载的图像做去畸变处理
        DoRemap(imageInput, rectifyImage);

        //2:提取角点
        ret = ExtractChessboard(rectifyImage, _2dPtsInOneImg);
        if (-1 == ret)
        {
            continue;
        }

        //3:获取空间点
        //参数1: 获得的3D点
        get3DPointSet(_3dPtsInOneImg);

        //4:根据角点计算相机的R与T
        //参数1: 2D点;
        //参数2: 3D点
        CalcRAndT(_2dPtsInOneImg, _3dPtsInOneImg);

        //5:清空临时的vector
        std::vector<cv::Point2f>().swap(_2dPtsInOneImg);
        std::vector<cv::Point3f>().swap(_3dPtsInOneImg);

        //6:所有图像处理完成，退出循环
        if(image_count == PICTURE_COUNT)
        {
             break;
        }
    }

    return 0;
}


int DoRemap(cv::Mat &InputImg, cv::Mat &OutputImg)
{
    //图像的尺寸
    cv::Size image_size;
    image_size.width = InputImg.cols;
    image_size.height =InputImg.rows;

    cv::Mat map1, map2;
    initUndistortRectifyMap(metadata_of_pic.mPic_Intrinsics, metadata_of_pic.mPic_DistCoeffs, cv::Mat()/*R*/,
                            getOptimalNewCameraMatrix(metadata_of_pic.mPic_Intrinsics, metadata_of_pic.mPic_DistCoeffs, image_size, 1, image_size, 0),
                            image_size, CV_16SC2, map1, map2);

    cv::namedWindow("Image View",CV_WINDOW_NORMAL);

    remap(InputImg, OutputImg, map1, map2, cv::INTER_LINEAR);

#ifdef DEBUG_VIEW
    cv::imshow("After Remap", OutputImg);
    cv::waitKey(400);
#endif

    //可选择是否转换为RGB三通道
    //cvtColor(OutputImg, OutputImg, CV_GRAY2BGR);
    return 0;

}

int ExtractChessboard(cv::Mat &RemapedImg, std::vector<cv::Point2f> &image_points_buf)
{
    //棋盘格尺寸
    cv::Size board_size = cv::Size(BOARD_SIZE_X,BOARD_SIZE_Y);
    //缓存每幅图像上检测到的角点
    //std::vector<cv::Point2f> image_points_buf;
    //保存检测到的所有角点
    std::vector<std::vector<cv::Point2f>> image_points_seq;

    /* 提取角点 */
    if (0 == findChessboardCorners(RemapedImg, board_size, image_points_buf))
    {
        std::cout<<"can not find chessboard corners!\n";
        return -1;
    }
    else
    {
        cv::Mat view_gray;
        cv::cvtColor(RemapedImg,view_gray,CV_RGB2GRAY);
        /* 亚像素精确化 */
        //find4QuadCornerSubpix(view_gray,image_points_buf,Size(5,5)); //对粗提取的角点进行精确化
        //flq, 默认屏蔽，打开后误差降低
        cornerSubPix(view_gray, image_points_buf, cv::Size(5,5), cv::Size(-1,-1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

        image_points_seq.push_back(image_points_buf);  //保存亚像素角点
        //added by flq
        metadata_of_pic.mPic_2DChessboardPointSet.push_back(image_points_buf);
        //added end
        /* 在图像上显示角点位置 */
        drawChessboardCorners(view_gray,board_size,image_points_buf,false); //用于在图片中标记角点
#ifdef DEBUG_VIEW
        imshow("Show ChessBoard",view_gray);//显示图片
        cv::waitKey(400);//暂停0.5S
#endif
        std::cout<<"本张图像角点提取完成！\n";

        return 0;
    }
}

int get3DPointSet(std::vector<cv::Point3f> &tempPointSet)
{
    //实际测量得到的标定板上每个棋盘格的大小
    cv::Size square_size = cv::Size(GRID_WIDTH, GRID_HEIGHT);
    //保存标定板上角点的三维坐标
    std::vector<std::vector<cv::Point3f>> object_points;

    //每幅图像的旋转向量
    std::vector<cv::Mat> rvecsMat;
    //每幅图像的平移向量
    std::vector<cv::Mat> tvecsMat;

    //棋盘格尺寸
    cv::Size board_size = cv::Size(BOARD_SIZE_X,BOARD_SIZE_Y);


    //初始化标定板上角点的三维坐标
    //std::vector<cv::Point3f> tempPointSet;
    std::vector<cv::Mat> temp4NumPointSet;

    int i,j;
    for (i=0; i< board_size.height; i++)
    {
        for (j=0; j<board_size.width; j++)
        {
            cv::Point3f realPoint;
            /* 假设标定板放在世界坐标系中z=0的平面上 */
            realPoint.x = i*square_size.width;
            realPoint.y = j*square_size.height;
            realPoint.z = 0;
            tempPointSet.push_back(realPoint);

            //added for 4:C = RT x W
            cv::Mat real4NumPoint  = (cv::Mat_<double>(4, 1) << 0, 10, 0, 1);

            real4NumPoint.at<double>(0, 0) = i*square_size.width;
            real4NumPoint.at<double>(1, 0) = j*square_size.height;
            real4NumPoint.at<double>(2, 0) = 0;
            real4NumPoint.at<double>(3, 0) = 1;

            temp4NumPointSet.push_back(real4NumPoint);

            //added end
        }
    }

    object_points.push_back(tempPointSet);
    //副产品
    metadata_of_pic.mPic_WorldChessboardPointSet.push_back(tempPointSet);
    //需要得到的
    metadata_of_pic.mPic_WorldChessboard4NumPointSet.push_back(temp4NumPointSet);

    return 0;
}

int CalcRAndT(std::vector<cv::Point2f> _2dPtsInOneImg, std::vector<cv::Point3f> _3dPtsInOneImg)
{
    //已知项:
    //1:相机内参与畸变系数
    //2:3D坐标
    //3:MxN个角点的2D坐标
    //欲求项:
    //R与T

    //第i张图像的旋转矩阵
    cv::Mat rotation_matrix;// = cv::Mat(3,3,CV_32FC1, cv::Scalar::all(0));
    //第i张图像的旋转向量
    cv::Mat rvecsMat;
    //第i张图像的平移向量
    cv::Mat tvecsMat;

    //PNP
    solvePnP(_3dPtsInOneImg, _2dPtsInOneImg, metadata_of_pic.mPic_Intrinsics, cv::Mat(), rvecsMat, tvecsMat, false, cv::SOLVEPNP_EPNP);

    //Set Value
    cv::Rodrigues(rvecsMat,rotation_matrix);
    metadata_of_pic.mPic_rvecsMatSet.push_back(rvecsMat);
    metadata_of_pic.mPic_rotationMatrixSet.push_back(rotation_matrix);
    metadata_of_pic.mPic_tvecsMatSet.push_back(tvecsMat);
}

#endif

