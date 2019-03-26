//USB:1280x960
#include <iostream>
#include <fstream>

#include "CameraCalib.h"

using namespace cv;
using namespace std;

MetadataFromPic metadata_of_pic;

int initMetadata(int pic_num, int board_size_x, int board_size_y)
{
    metadata_of_pic.mPic_Intrinsics = cv::Mat(3,3,CV_32FC1,cv::Scalar::all(0));
    metadata_of_pic.mPic_DistCoeffs = cv::Mat(1,5,CV_32FC1,cv::Scalar::all(0));
#if 0
    vector<Point3f> tempPointSet;

    for(int i=0;i < pic_num; i++)
    {
        //旋转与平移矩阵

        //标定板角点的世界坐标
        for(int y=0; y < BOARD_SIZE_Y; y++)
        {
            for(int x=0; x < BOARD_SIZE_X; x++)
            {
                Point3f realPoint;

                /* 初始化为全0 */
                realPoint.x = 0;
                realPoint.y = 0;
                realPoint.z = 0;
                tempPointSet.push_back(realPoint);
            }
        }
        metadata_of_pic.mPic_WorldChessboardPointSet.push_back(tempPointSet);
    }
#endif

   return 0;
}

int CameraCalib()
{
    ifstream fin(INPUT_FILE); /* 标定所用图像文件的路径 */
    ofstream fout(OUTPUT_FILE);  /* 保存标定结果的文件 */
    //读取每一幅图像，从中提取出角点，然后对角点进行亚像素精确化
    cout<<"开始提取角点………………";
    int image_count=0;  /* 图像数量 */
    Size image_size;  /* 图像的尺寸 */
    //Size board_size = Size(4,6);    /* 标定板上每行、列的角点数 */
    Size board_size = Size(BOARD_SIZE_X,BOARD_SIZE_Y); //before(6,9);   /* 标定板上每行、列的角点数 */
    vector<Point2f> image_points_buf;  /* 缓存每幅图像上检测到的角点 */
    vector<vector<Point2f>> image_points_seq; /* 保存检测到的所有角点 */
    string filename[PICTURE_COUNT];
    ///int count= -1 ;//用于存储角点个数。

    //Init Metadata
    initMetadata(PICTURE_COUNT, BOARD_SIZE_X, BOARD_SIZE_Y);

    while (getline(fin,filename[image_count]))
    {
        image_count++;
        // 用于观察检验输出
        cout<<"image_count = "<<image_count<<endl;
        /* 输出检验*/
        ///cout<<"-->count = "<<count;
        Mat imageInput=imread(filename[image_count-1]);
        printf("\nfilename[%d]=%s\n", image_count-1, filename[image_count-1].c_str());
        if (image_count == 1)  //读入第一张图片时获取图像宽高信息
        {
            image_size.width = imageInput.cols;
            image_size.height =imageInput.rows;
            cout<<"image_size.width = "<<image_size.width<<endl;
            cout<<"image_size.height = "<<image_size.height<<endl;
        }

        /* 提取角点 */
        if (0 == findChessboardCorners(imageInput,board_size,image_points_buf))
        {
            cout<<"can not find chessboard corners!\n"; //找不到角点
            exit(1);
        }
        else
        {
            Mat view_gray;
            cvtColor(imageInput,view_gray,CV_RGB2GRAY);
            /* 亚像素精确化 */
            //find4QuadCornerSubpix(view_gray,image_points_buf,Size(5,5)); //对粗提取的角点进行精确化

            //flq, 默认屏蔽，打开后误差降低
            cornerSubPix(view_gray,image_points_buf,Size(5,5),Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));

            image_points_seq.push_back(image_points_buf);  //保存亚像素角点
            //added by flq
            metadata_of_pic.mPic_2DChessboardPointSet.push_back(image_points_buf);
            //added end
            /* 在图像上显示角点位置 */
            drawChessboardCorners(view_gray,board_size,image_points_buf,false); //用于在图片中标记角点
            imshow("Camera Calibration",view_gray);//显示图片
            waitKey(50);//暂停0.5S
        }

        if(image_count == PICTURE_COUNT)
        {
            break;
        }
    }

    //image_points_seq: 保存所有二维角点的vector
    //total:成功监测到角点的图片数
    int total = image_points_seq.size();
    cout<<"total = "<<total<<endl;
    int CornerNum=board_size.width*board_size.height;  //每张图片上总的角点数
    printf("FLQ, CornerNum%=%d*%d=%d\n", board_size.width, board_size.height, CornerNum);
    for (int ii=0 ; ii<total ;ii++)
    {
        if (0 == ii%CornerNum)// BOARD_SIZE_X*BOARD_SIZE_Y=每幅图片的角点个数。此判断语句是为了输出 图片号，便于控制台观看
        {
            int i = -1;
            i = ii/CornerNum;
            int j=i+1;
            cout<<"--> 第 "<<j <<" 个图片的数据 --> : "<<endl;
        }
        if (0 == ii%3)	// 此判断语句，格式化输出，便于控制台查看
        {
            cout<<endl;
        }
        else
        {
            //cout.width(GRID_WIDTH);
        }

        //输出所有的(0，0)角点,total=19个
        cout<<" -->"<<image_points_seq[ii][0].x;
        cout<<" -->"<<image_points_seq[ii][0].y <<endl;
    }
    cout<<"角点提取完成！\n";

    //以下是摄像机标定
    cout<<"开始标定………………" << endl;
    /*棋盘三维信息*/
    Size square_size = Size(GRID_WIDTH,GRID_HEIGHT);  /* 实际测量得到的标定板上每个棋盘格的大小 */ //10
    vector<vector<Point3f>> object_points; /* 保存标定板上角点的三维坐标 */
    /*内外参数*/
    Mat cameraMatrix=Mat(3,3,CV_32FC1,Scalar::all(0)); /* 摄像机内参数矩阵 */
    vector<int> point_counts;  // 每幅图像中角点的数量
    Mat distCoeffs=Mat(1,5,CV_32FC1,Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
    vector<Mat> rvecsMat;  /* 每幅图像的旋转向量 */
    vector<Mat> tvecsMat; /* 每幅图像的平移向量 */
    /* 初始化标定板上角点的三维坐标 */
    int i,j,t;
    for (t=0; t<image_count; t++)
    {
        vector<Point3f> tempPointSet;
        vector<Mat> temp4NumPointSet;
        for (i=0; i<board_size.height; i++)
        {
            for (j=0; j<board_size.width; j++)
            {
                Point3f realPoint;
                /* 假设标定板放在世界坐标系中z=0的平面上 */
                realPoint.x = i*square_size.width;
                realPoint.y = j*square_size.height;
                realPoint.z = 0;
                tempPointSet.push_back(realPoint);

                //added for 4:C = RT x W
                Mat real4NumPoint  = (Mat_<double>(4, 1) << 0, 10, 0, 1);

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
    }

//    std::cout << "object_points[2]:" << object_points[2] << endl;
//    std::cout << "TTTmetadata_of_pic.mPic_WorldChessboardPointSet[2]:" << metadata_of_pic.mPic_WorldChessboardPointSet[0] << endl;
//    std::cout << "QQQmetadata_of_pic.mPic_WorldChessboard4NumPointSet[2]\n:" << metadata_of_pic.mPic_WorldChessboard4NumPointSet[1][2] << endl;

    /* 初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板 */
    for (i=0;i<image_count;i++)
    {
        point_counts.push_back(board_size.width*board_size.height);
    }
    /* 开始标定 */
    //获取cameraMatrix,distCoeffs,rvecsMat,tvecsMat
    calibrateCamera(object_points,image_points_seq,image_size,cameraMatrix,distCoeffs,rvecsMat,tvecsMat,0); //0 or flags|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5

    metadata_of_pic.mPic_Intrinsics = cameraMatrix;
    metadata_of_pic.mPic_DistCoeffs = distCoeffs;
    cout << " metadata_of_pic.mPic_Intrinsics:" << endl <<  metadata_of_pic.mPic_Intrinsics << endl;
    cout << " metadata_of_pic.mPic_DistCoeffs:" << endl <<  metadata_of_pic.mPic_DistCoeffs << endl;

    cout<<"标定完成！\n";
    //对标定结果进行评价
    cout<<"开始评价标定结果………………\n";
    double total_err = 0.0; /* 所有图像的平均误差的总和 */
    double err = 0.0; /* 每幅图像的平均误差 */
    vector<Point2f> image_points2; /* 保存重新计算得到的投影点 */
    cout<<"\t每幅图像的标定误差：\n";
    fout<<"每幅图像的标定误差：\n";
    for (i=0;i<image_count;i++)
    {
        vector<Point3f> tempPointSet=object_points[i];

        /* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
        projectPoints(tempPointSet,rvecsMat[i],tvecsMat[i],cameraMatrix,distCoeffs,image_points2);

        /* 计算新的投影点和旧的投影点之间的误差*/
        vector<Point2f> tempImagePoint = image_points_seq[i];
        Mat tempImagePointMat = Mat(1,tempImagePoint.size(),CV_32FC2);
        Mat image_points2Mat = Mat(1,image_points2.size(), CV_32FC2);
        for (int j = 0 ; j < tempImagePoint.size(); j++)
        {
            image_points2Mat.at<Vec2f>(0,j) = Vec2f(image_points2[j].x, image_points2[j].y);
            tempImagePointMat.at<Vec2f>(0,j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
        }
        err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
        total_err += err/=  point_counts[i];
        std::cout<<"第"<<i+1<<"幅图像的平均误差："<<err<<"像素"<<endl;
        fout<<"第"<<i+1<<"幅图像的平均误差："<<err<<"像素"<<endl;
    }
    std::cout<<"总体平均误差："<<total_err/image_count<<"像素"<<endl;
    fout<<"总体平均误差："<<total_err/image_count<<"像素"<<endl<<endl;
    std::cout<<"评价完成！"<<endl;

    //保存定标结果
    std::cout<<"开始保存定标结果………………"<<endl;
    Mat rotation_matrix[image_count] = Mat(3,3,CV_32FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */
    fout<<"相机内参数矩阵："<<endl;
    fout<<cameraMatrix<<endl<<endl;
    fout<<"畸变系数：\n";
    fout<<distCoeffs<<endl<<endl<<endl;

    for (int i=0; i<image_count; i++)
    {
        fout<<"第"<<i+1<<"幅图像的旋转向量："<<endl;
        fout<<rvecsMat[i]<<endl;
        /* 将旋转向量转换为相对应的旋转矩阵 */
        Rodrigues(rvecsMat[i],rotation_matrix[i]);
        metadata_of_pic.mPic_rvecsMatSet.push_back(rvecsMat[i]);
        metadata_of_pic.mPic_rotationMatrixSet.push_back(rotation_matrix[i]);
        metadata_of_pic.mPic_tvecsMatSet.push_back(tvecsMat[i]);
        fout<<"第"<<i+1<<"幅图像的旋转矩阵："<<endl;
        fout<<rotation_matrix[i]<<endl;
        fout<<"第"<<i+1<<"幅图像的平移向量："<<endl;
        fout<<tvecsMat[i]<<endl<<endl;
    }
    std::cout<<"完成保存"<<endl;
    fout<<endl;

    //printf value, flq
    for(int i=0;i < PICTURE_COUNT; i++)
    {
        cout << " metadata_of_pic.mPic_rvecsMatSet:" << endl <<  metadata_of_pic.mPic_rvecsMatSet[i] << endl;
        cout << " metadata_of_pic.mPic_rotationMatrixSet:" << endl <<  metadata_of_pic.mPic_rotationMatrixSet[i] << endl;
        cout << " metadata_of_pic.mPic_tvecsMatSet:" << endl <<  metadata_of_pic.mPic_tvecsMatSet[i] << endl;

    }
    /************************************************************************
    显示定标结果
    *************************************************************************/
    Mat R = Mat::eye(3,3,CV_32F);

    {
        Mat view, rview, map1, map2;
        initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat()/*R*/,
                                getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, image_size, 1, image_size, 0),
                                image_size, CV_16SC2, map1, map2);

        for(int i = 0 ; i != image_count ; i++)
        {
            view = imread(filename[i], 1);
            if(!view.data)
            {
                continue;
            }

            //undistort( view, rview, cameraMatrix, distCoeffs, cameraMatrix ); //旧的去畸变函数
            //remap,解决“极线对齐”的问题
            remap(view, rview, map1, map2, INTER_LINEAR);
            imshow("Image View", rview);
#if 0
            int c = 0xff & waitKey();

            if( (c & 255) == 27 || c == 'q' || c == 'Q' )
            {
                break;
            }
#else
            waitKey(50);
#endif
        }
    }


    return 0;
}
