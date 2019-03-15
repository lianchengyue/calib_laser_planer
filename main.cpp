#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "Get2DLinearFunc.h"
#include "Get3DLinearFunc.h"

using namespace std;
using namespace cv;


//https://blog.csdn.net/zwzsdy/article/details/69935602
void Doolittle(int n,double *A,double *b)//n为阶数 A为系数矩阵 b为常数矩阵
{
    double *L = new double[n*n];//开辟L矩阵空间
    double *U = new double[n*n];//开辟U矩阵空间
    double *x = new double[n];//开辟y矩阵空间
    double *y = new double[n];//开辟y矩阵空间

    //初始化U与L矩阵
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            *(U + i*n + j) = 0;//暂时全部赋值为0

            if (i==j)
            {
                *(L + i*n + j) = 1;//对角线赋值为1
            }
            else
            {

                *(L + i*n + j) = 0;//其他暂时赋值为0
            }
        }
    }


    //计算U和L矩阵的值
    for (int k = 0; k < n; k++)
    {
        for (int j = k; j < n; j++)
        {
            *(U + k*n + j) = *(A + k*n + j);//第一行
            for (int r = 0; r < k; r++)//接下来由L的前一列算u的下一行
            {
                *(U + k*n + j) = *(U + k*n + j) - (*(L + k*n + r)*(*(U + r*n + j)));
            }
        }
        for (int i = k+1; i < n; i++)//计算L的列
        {
            *(L + i*n + k) = *(A + i*n + k);
            for (int r = 0; r < k; r++)
            {
                *(L + i*n + k) = *(L + i*n + k) - (*(L + i*n + r)*(*(U + r*n + k)));
            }
            *(L + i*n + k) = *(L + i*n + k) / (*(U + k*n + k));
        }
    } for (int i = 0; i < n; i++)//由Ly=b算y
    {
        *(y + i) = *(b + i);
        for (int j = 0; j < i; j++)
        {
            *(y + i) = *(y + i) - *(L + i*n + j)*(*(y + j));
        }
    }
    for (int i = n-1; i >= 0; i--)//由Ux=y算x
    {
        *(x + i) = *(y + i);
        for (int j = i+1; j < n; j++)
        {
            *(y + i) = *(y + i) - *(U + i*n + j)*(*(x + j));
        }
        *(x + i) = *(y + i) / (*(U + i*n + i));
    }
    cout << "解：\n";//得出解
    for (int i = 0; i < n; i++)
    {
        cout <<"x"<<i+1<<"："<< *(x + i) << endl;
    }
    delete[]L;//释放空间
    delete[]U;
    delete[]y;
}


//基于最小二乘的理论，要得到观测值的误差的平方和的最小值
int LeastSquare_linear(vector<Point3d> PrimiDataSet)
{
    //https://blog.csdn.net/dangkie/article/details/52262546
    //https://blog.csdn.net/xiaoxiaoliluo917/article/details/83177216
    ///截距
    double a;
    ///斜率
    double b;
    ///相关常数
    double r;
    ///平均值
    double xa;
    double ya;

    double X=0.00,XX=0.00,Y=0.00,YY=0.00,XY=0.00;
    double ix = 0.00,iy = 0.00;
    int N = PrimiDataSet.size();

    for(int i=0;i<PrimiDataSet.size();i++)
    //for(int i=0;i<3;i++)
    {
        ix = PrimiDataSet[i].x;
        iy = PrimiDataSet[i].y;
        X+=ix;
        XX+=ix*ix;
        Y+=iy;
        YY+=iy*iy;
        XY+=ix*iy;
    }

    a = (XX*Y-X*XY)/(N*XX-X*X);
    b = (N*XY-X*Y)/(N*XX-X*X);

    xa = X/N;
    ya = Y/N;

    return 0;
}

int main(int argc, char *argv[])
{
    //cameraMatrix
    Mat Intrinsics = Mat(3,3,CV_64FC1,Scalar::all(0)); /* 摄像机内参数矩阵 */  //cameraMatrix

    Mat DistCoeffs = Mat(1,5,CV_64FC1,Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
    Mat CameraMatrix = Mat(3,4,CV_64FC1,Scalar::all(0));
    //vector<Mat> rvecsMat;  /* 每幅图像的旋转向量 */
    Mat rvecsMat = Mat(1,3,CV_64FC1,Scalar::all(0));  /* 每幅图像的旋转向量 */
    Mat rotation_matrix = Mat(3,3,CV_64FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */
    //vector<Mat> tvecsMat; /* 每幅图像的平移向量 */
    Mat tvecsMat = Mat(1,3,CV_64FC1,Scalar::all(0));  /* 每幅图像的平移向量 */

    Mat RT = Mat(4, 4 ,CV_64FC1,Scalar::all(0));  //RT Matrix

    ///1:init内参
    Intrinsics.at<double>(0, 2) = 644.6466866645241;  //u0
    Intrinsics.at<double>(1, 2) = 482.6045579244723;  //v0
    Intrinsics.at<double>(0, 0) = 1399.023439589231;  //fx
    Intrinsics.at<double>(1, 1) = 1401.050332122462;  //fy
    Intrinsics.at<double>(2, 2) = 1;

    DistCoeffs.at<double>(0, 0) = -0.04457400625712249;
    DistCoeffs.at<double>(0, 1) = 0.1470300318266282;
    DistCoeffs.at<double>(0, 2) = 0.001714109155278642;
    DistCoeffs.at<double>(0, 3) = 0.002511861372613767;
    DistCoeffs.at<double>(0, 4) = -1.095847450306373;

    ///2:init外参
    rvecsMat.at<double>(0, 0) = 2.02517075054481;
    rvecsMat.at<double>(0, 1) = 1.747835480970283;
    rvecsMat.at<double>(0, 2) = -0.3226978455624659;

    tvecsMat.at<double>(0, 0) = -53.80020601312009;
    tvecsMat.at<double>(0, 1) = 29.25313600086886;
    tvecsMat.at<double>(0, 2) = 417.0198668690805;

    std::cout << "旋转向量" << rvecsMat << endl;
    Rodrigues(rvecsMat,rotation_matrix);
    std::cout << "旋转矩阵" << rotation_matrix << endl;

    ///3:外参的旋转与平移矩阵  [R T 0 1]
    RT.at<double>(0, 0) = rotation_matrix.at<double>(0, 0);
    RT.at<double>(0, 1) = rotation_matrix.at<double>(0, 1);
    RT.at<double>(0, 2) = rotation_matrix.at<double>(0, 2);
    RT.at<double>(0, 3) = tvecsMat.at<double>(0, 0);

    RT.at<double>(1, 0) = rotation_matrix.at<double>(1, 0);
    RT.at<double>(1, 1) = rotation_matrix.at<double>(1, 1);
    RT.at<double>(1, 2) = rotation_matrix.at<double>(1, 2);
    RT.at<double>(1, 3) = tvecsMat.at<double>(0, 1);

    RT.at<double>(2, 0) = rotation_matrix.at<double>(2, 0);
    RT.at<double>(2, 1) = rotation_matrix.at<double>(2, 1);
    RT.at<double>(2, 2) = rotation_matrix.at<double>(2, 2);
    RT.at<double>(2, 3) = tvecsMat.at<double>(0, 2);

    RT.at<double>(3, 0) = 0;
    RT.at<double>(3, 1) = 0;
    RT.at<double>(3, 2) = 0;
    RT.at<double>(3, 3) = 1;
    std::cout << "camera RT矩阵:" << endl << RT << endl;

    ///4:C = RT x W,获取棋盘格上的点在相机坐标系下的p3d
    Mat PointW = (Mat_<double>(4, 1) << 0, 10, 0, 1);
    std::cout << "PointW" << endl << PointW << endl;
    Mat PointC;

    PointC = RT * PointW;
    std::cout << "PointC:" << RT * PointW << endl;

    //vector<Point3f> p3d
    vector<Point3d>  points3d(1);
    points3d[0].x = PointC.at<double>(0, 0);
    points3d[0].y = PointC.at<double>(1, 0);
    points3d[0].z = PointC.at<double>(2, 0);
    std::cout << "points3d:" << points3d << endl;

    ///5:计算得到ln, 即多点拟合空间中的直线
    //直线方程是1：A1x+B1y+C1z+D1=0,A2x+B2y+C2z+D2=0,联立
    //2:空间直线的标准式(x-x0)/a＝(y-y0)/b＝(z-z0)/c, 其中(a,b,c)为方向向量
    //3:空间直线的两点式： (x-x1)/(x-x2)＝(y-y1)/(y-y2)＝(z-z1)/(z-z2), take this
    vector<Point3d>  p3d(4);
    p3d[0].x = 1;
    p3d[0].y = 1;
    p3d[0].z = 2;
    p3d[1].x = 2;
    p3d[1].y = 2;
    p3d[1].z = 3;
    p3d[2].x = 3;
    p3d[2].y = 3;
    p3d[2].z = 4;
    p3d[3].x = 4;
    p3d[3].y = 4;
    p3d[3].z = 5;
    std::cout << "points3d:" << points3d << endl;
    //LeastSquare_linear(p3d);

    Get3DLinearValue(p3d);

    ///6,7,8:计算激光平面与ln延长线的交点
    ///6:计算激光线在像素坐标系的方程Ax+By+c=0
    double a;
    double b;
    double c;
    vector<Point2d>  p2d;
    //得到所有的激光点p2d
    StegerLine(p2d);
    //根据得到的激光点拟合直线
    lineFit(p2d, a, b, c);
    printf("a x + b y + c = 0\n%lfx + %lfy + %lf = 0\n", a, b, c);

    ///7:计算空间直线在像素坐标系的投影

    ///8:得到激光平面与ln的各个坐标Pi(xp,yp,zp)
    //原理:https://blog.csdn.net/lyl771857509/article/details/79633412
    #if 0
    Mat PointWtoC = Mat(4, 1 ,CV_64FC1,Scalar::all(0));  //RT Matrix
    PointWtoC.at<double>(0, 0) = PointC.at<double>(0, 0); //Xc
    PointWtoC.at<double>(1, 0) = PointC.at<double>(1, 0); //Yc
    PointWtoC.at<double>(2, 0) = PointC.at<double>(2, 0); //Zc
    PointWtoC.at<double>(3, 0) = 1;                       //1
    #endif

    //给3x4的相机内参矩阵赋值
    CameraMatrix.at<double>(0, 0) = Intrinsics.at<double>(0, 0);
    CameraMatrix.at<double>(0, 1) = Intrinsics.at<double>(0, 0);
    CameraMatrix.at<double>(0, 2) = Intrinsics.at<double>(0, 0);
    CameraMatrix.at<double>(0, 3) = 0;
    CameraMatrix.at<double>(1, 0) = Intrinsics.at<double>(1, 0);
    CameraMatrix.at<double>(1, 1) = Intrinsics.at<double>(1, 0);
    CameraMatrix.at<double>(1, 2) = Intrinsics.at<double>(1, 0);
    CameraMatrix.at<double>(1, 3) = 0;
    CameraMatrix.at<double>(2, 0) = Intrinsics.at<double>(2, 0);
    CameraMatrix.at<double>(2, 1) = Intrinsics.at<double>(2, 0);
    CameraMatrix.at<double>(2, 2) = Intrinsics.at<double>(2, 0);
    CameraMatrix.at<double>(2, 3) = 0;

    Point2d  testp2d;
    testp2d.x = 740;   //u
    testp2d.y = 580;   //v
    double aa = 1;
    double bb = 2;
    double cc = 3;
    double dd = -1400;//-1400;

    //Zc*[u,v,1] = CameraMatrix(3X4) * [Xc,Yc,Zc,1]
    //|
    //v
    //fx*Xc + u0*Zc + 0*1= u*Zc
    //fy*Yc + v0*Zc + 0*1= v*Zc
    //Zc = Zc
    //A*Xc + B*Yc + C*Zc + D = 0
    //|
    //v
    //fx*Xc + (u0-u)*Zc = 0
    //fy*Yc + (v0-v)*Zc = 0
    //A*Xc + B*Yc + C*Zc = -D
    //|
    //v
    //[fx, 0, (u0-u)]   [Xc]   [0 ]
    //[0, fy, (v0-v)] * [Yc] = [0 ]
    //[A,  B,    C  ]   [Zc]   [-D]

    Mat _2Dto3DMat = Mat(3,3,CV_64FC1,Scalar::all(0));

    _2Dto3DMat.at<double>(0, 0) = Intrinsics.at<double>(0, 0);
    _2Dto3DMat.at<double>(0, 1) = 0;
    _2Dto3DMat.at<double>(0, 2) = Intrinsics.at<double>(0, 2) - testp2d.x;// -u
    _2Dto3DMat.at<double>(1, 0) = 0;
    _2Dto3DMat.at<double>(1, 1) = Intrinsics.at<double>(1, 1);
    _2Dto3DMat.at<double>(1, 2) = Intrinsics.at<double>(1, 2) - testp2d.y;// -v
    _2Dto3DMat.at<double>(2, 0) = aa;
    _2Dto3DMat.at<double>(2, 1) = bb;
    _2Dto3DMat.at<double>(2, 2) = cc;

    std::cout << "_2Dto3DMat" << endl << _2Dto3DMat << endl;

    Mat _00D = (Mat_<double>(3, 1) << 0, 0, -dd);

    Mat XcYcZc;
    //CV_LU - 最佳主元选取的高斯消除法
    //CV_SVD - 奇异值分解法 (SVD)
    //CV_SVD_SYM - 对正定对称矩阵的 SVD 方法
    cv::solve(_2Dto3DMat, _00D, XcYcZc, CV_SVD);  //DECOMP_LU
    //SVD::solveZ(_2Dto3DMat, _00D, XcYcZc);

    std::cout << "XcYcZc" << XcYcZc<< endl;

    double InA[9] = {Intrinsics.at<double>(0, 0), 0, Intrinsics.at<double>(0, 2) - testp2d.x,
                    0, Intrinsics.at<double>(1, 1), Intrinsics.at<double>(1, 2) - testp2d.y,
                    aa, bb, cc};
    double InB[3] = {0,0,-dd};
    Doolittle(3, InA, InB);

    ///9:通过计算得到的多个3D激光点，拟合得到空间中激光平面与标定板平面的3D直线
    ///10:得到多组图像标定板与激光平面的N个交点，拟合激光平面.  ("N个交点"为ln与激光平面的交点)


    ///世界坐标系W下，标定板的平面坐标
    //0x + 0y + z = 0
    return 0;
}
