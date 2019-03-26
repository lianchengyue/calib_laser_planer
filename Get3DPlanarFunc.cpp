#include "Get3DPlanarFunc.h"

using namespace std;
using namespace cv;
/*
 *
 *
 *  最小二乘拟合平面，平面方程：Ax+By+Cz=D
 *  A = plane.at<float>(0,0)
 *  B = plane.at<float>(1,0)
 *  C = plane.at<float>(2,0)
 *  D = plane.at<float>(3,0)
 *
 * */
//Param1:输入的空间坐标点
//param2:输出的平面方程

int FitPlane(const cv::Mat &Inputpoints, cv::Mat& plane)
{
    int rows = Inputpoints.rows;  //height  88
    int cols = Inputpoints.cols;  //width 3

    printf("InputPoints Adress:0x%x\n", &Inputpoints);
    cout << "Inputpoints:" <<Inputpoints <<endl;

    /*
    for (int i = 0; i < 9; ++i)
    {
        printf("XXXInputpoints=(%lf,%lf,%lf)\n",Inputpoints.at<double>(i, 0),Inputpoints.at<double>(i, 1),Inputpoints.at<double>(i, 2));
    }
    */

    cv::Mat centroid = cv::Mat::zeros(1,cols,CV_64FC1/*CV_32FC1*/);  //centroid:x,y,z的平均值
    for(int i=0; i<cols; i++)  //3
    {
        for(int j=0; j<rows; j++) //88
        {
            printf("Inputpoints.at<double>(j,i)=%lf\n",Inputpoints.at<double>(j,i));
            //cout << Inputpoints.at<double>(j,i) <<endl;
            centroid.at<double>(0,i) += Inputpoints.at<double>(j,i);
        }
        centroid.at<double>(0,i)/=rows;
        cout << "  centroid.at<double>(0," << i << ") :"<< centroid.at<double>(0,i) <<endl;
    }



    cv::Mat points2 = cv::Mat::ones(rows,cols,CV_64FC1/*CV_32FC1*/);
    for(int i=0;i<rows;i++) //88
    {
        for(int j=0;j<cols;j++) //3
        {
            points2.at<double>(i,j) = Inputpoints.at<double>(i,j) - centroid.at<double>(0,j);
            cout << "  points2.at<double>("<< i<< "," << j << ") :"<< points2.at<double>(i,j);
        }
        cout << endl;
    }

    // Evaluate SVD of covariance matrix.
    cv::Mat A,W,U,V;
    cv::gemm(points2, Inputpoints, 1, NULL, 0, A, CV_GEMM_A_T);
    std::cout << "A:" << std::endl << A << std::endl;
    cv::SVD::compute(A,W,U,V);
    std::cout << "V:" << std::endl << V << std::endl;


    plane = cv::Mat::zeros(cols+1,1,CV_64FC1/*CV_32FC1*/);
    for (int c = 0; c<cols; c++)
    {
        plane.at<double>(c,0) = V.at<double>(cols-1,c);
        plane.at<double>(cols,0) += plane.at<double>(c,0)*centroid.at<double>(0,c);
        std::cout << "plane.at<double>(c,0):" << plane.at<double>(c,0) << std::endl;
    }
    std::cout << "plane.at<double>(cols,0):" << plane.at<double>(cols,0) << std::endl;

    std::cout << "plane:" << std::endl << plane << std::endl;
    return 0;
}
