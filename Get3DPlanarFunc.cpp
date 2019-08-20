#include "Get3DPlanarFunc.h"

using namespace std;
using namespace cv;
/*
 *
 *
 *  最小二乘拟合平面，平面方程：Ax+By+Cz=D
 *  A = plane.at<double>(0,0)
 *  B = plane.at<double>(1,0)
 *  C = plane.at<double>(2,0)
 *  D = plane.at<double>(3,0)
 *
 * */
//Param1:输入的空间坐标点
//param2:输出的平面方程

int FitBoardPlane(const cv::Mat &Inputpoints, cv::Mat& plane)
{
    int rows = Inputpoints.rows;  //height  88
    int cols = Inputpoints.cols;  //width 3

    cout << endl <<  "Inputpoints:" << endl << Inputpoints <<endl;

    ///1:计算x,y,z的平均值
    //centroid:x,y,z的平均值
    cv::Mat centroid = cv::Mat::zeros(1,cols,CV_64FC1);
    for(int i=0; i<cols; i++)  //3
    {
        //求和
        for(int j=0; j<rows; j++) //88
        {
            centroid.at<double>(0,i) += Inputpoints.at<double>(j,i);
        }
        centroid.at<double>(0,i)/=rows;
        cout << "  centroid.at<double>(0," << i << ") :"<< centroid.at<double>(0,i) <<endl;
    }

    ///2:求与均值的差值
    cv::Mat points2 = cv::Mat::ones(rows,cols,CV_64FC1);
    for(int i=0;i<rows;i++) //88
    {
        for(int j=0;j<cols;j++) //3
        {
            points2.at<double>(i,j) = Inputpoints.at<double>(i,j) - centroid.at<double>(0,j);
            //cout << "  points2.at<double>("<< i<< "," << j << ") :"<< points2.at<double>(i,j);
        }
        //cout << endl;
    }

    ///3:Evaluate SVD of covariance matrix.
    cv::Mat A,W,U,V;
    cv::gemm(points2, Inputpoints, 1, NULL, 0, A, CV_GEMM_A_T);
    std::cout << "A:" << std::endl << A << std::endl;
    cv::SVD::compute(A,W,U,V);
    std::cout << "V:" << std::endl << V << std::endl;

    ///4:计算平面方程
    plane = cv::Mat::zeros(cols+1,1,CV_64FC1);
    for (int c = 0; c<cols; c++)
    {
        plane.at<double>(c,0) = V.at<double>(cols-1,c);
        plane.at<double>(cols,0) += plane.at<double>(c,0)*centroid.at<double>(0,c);
        std::cout << "plane.at<double>(c,0):" << plane.at<double>(c,0) << std::endl;
    }
    std::cout << "plane.at<double>(cols,0):" << plane.at<double>(cols,0) << std::endl;
#ifdef DEBUG
    std::cout << "Board Plane:" << std::endl << plane << std::endl;
#endif
    return 0;
}

int FitLaserPlane(const cv::Mat &Inputpoints, cv::Mat& plane, int PtSet_num)
{
    int rows = Inputpoints.rows;  //height  88
    int cols = Inputpoints.cols;  //width 3
    double AA, BB , CC, DD = 0;

    cout << endl <<  "Inputpoints:" << endl << Inputpoints <<endl;

    ///1:计算x,y,z的平均值
    //centroid:x,y,z的平均值
    cv::Mat centroid = cv::Mat::zeros(1,cols,CV_64FC1);
    for(int i=0; i<cols; i++)  //3
    {
        //求和
        for(int j=0; j<rows; j++) //88
        {
            centroid.at<double>(0,i) += Inputpoints.at<double>(j,i);
        }
        centroid.at<double>(0,i)/=rows;
        cout << "  centroid.at<double>(0," << i << ") :"<< centroid.at<double>(0,i) <<endl;
    }

    ///2:求与均值的差值
    cv::Mat points2 = cv::Mat::ones(rows,cols,CV_64FC1);
    for(int i=0;i<rows;i++) //88
    {
        for(int j=0;j<cols;j++) //3
        {
            points2.at<double>(i,j) = Inputpoints.at<double>(i,j) - centroid.at<double>(0,j);
            cout << "  points2.at<double>("<< i<< "," << j << ") :"<< points2.at<double>(i,j);
        }
        cout << endl;
    }

    ///3:Evaluate SVD of covariance matrix.
    cv::Mat A,W,U,V;
    cv::gemm(points2, Inputpoints, 1, NULL, 0, A, CV_GEMM_A_T);
    std::cout << "A:" << std::endl << A << std::endl;
    cv::SVD::compute(A,W,U,V);
    std::cout << "V:" << std::endl << V << std::endl;

    ///4:计算平面方程
    plane = cv::Mat::zeros(cols+1,1,CV_64FC1);
    for (int c = 0; c<cols; c++)
    {
        plane.at<double>(c,0) = V.at<double>(cols-1,c);
        plane.at<double>(cols,0) += plane.at<double>(c,0)*centroid.at<double>(0,c);
        std::cout << "plane.at<double>(c,0):" << plane.at<double>(c,0) << std::endl;
    }
    //参数赋值
    AA = plane.at<double>(0,0);
    BB = plane.at<double>(1,0);
    CC = plane.at<double>(2,0);
    DD = plane.at<double>(3,0);
    std::cout << "plane.at<double>(cols,0):" << plane.at<double>(cols,0) << std::endl;
#ifdef DEBUG
    std::cout << "plane:" << std::endl << plane << std::endl;
#endif


#ifdef DEBUG
    std::cout << endl << "ERROR:" << std::endl;


    //求误差
    double ErrValue;
    double err = 0;
    for(int j=0; j<rows; j++) //36
    {
        //求差
        ErrValue = AA*Inputpoints.at<double>(j,0) + BB*Inputpoints.at<double>(j,1) + CC*Inputpoints.at<double>(j,2) - DD;
        //求均方差  fabs:求绝对值
        ErrValue = pow(AA*Inputpoints.at<double>(j,0) + BB*Inputpoints.at<double>(j,1) + CC*Inputpoints.at<double>(j,2), 2) - pow(DD, 2);
        ErrValue = sqrt(fabs(ErrValue));//平方根
        err += ErrValue;
        std::cout << "ErrValue:" << std::endl << ErrValue << std::endl;
    }
    std::cout << "total err:" << std::endl << err << std::endl;

#endif
    return 0;
}
