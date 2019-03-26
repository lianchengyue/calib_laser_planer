#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "Get2DLinearFunc.h"
#include "Get3DLinearFunc.h"
#include "CameraCalib.h"
#include "LineCross.h"
#include "Get3DPlanarFunc.h"

using namespace std;
using namespace cv;

extern MetadataFromPic metadata_of_pic;
extern LinearFunctionCoefficients laserLineCoeffs;  //一条激光线在图像上坐标的直线方程
extern vector<vector<LinearFunctionCoefficients>> cornerPointLineCoeffs;   //ln在平面上的投影

vector<vector<cv::Point2d>> cross_PtSet;

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
    int PicNum = PICTURE_COUNT;
    int i = 0;
    //cameraMatrix
    Mat Intrinsics = Mat(3,3,CV_64FC1,Scalar::all(0)); /* 摄像机内参数矩阵 */  //cameraMatrix

    Mat DistCoeffs = Mat(1,5,CV_64FC1,Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
    Mat CameraMatrix = Mat(3,4,CV_64FC1,Scalar::all(0));
    //vector<Mat> rvecsMat;  /* 每幅图像的旋转向量 */
    Mat rvecsMat[PicNum] = Mat(1,3,CV_64FC1,Scalar::all(0));  /* 每幅图像的旋转向量 */
    Mat rotation_matrix[PicNum] = Mat(3,3,CV_64FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */
    //vector<Mat> tvecsMat; /* 每幅图像的平移向量 */
    Mat tvecsMat[PicNum] = Mat(1,3,CV_64FC1,Scalar::all(0));  /* 每幅图像的平移向量 */

    Mat RT[PicNum] = Mat(4, 4 ,CV_64FC1,Scalar::all(0));  //RT Matrix

    //Mat RTX[PicNum] = Mat(4, 4 ,CV_64FC1,Scalar::all(1));  //RT Matrix
    //std::cout << "RTX1矩阵:" << endl << RTX[0] << endl;
    //std::cout << "RTX2矩阵:" << endl << RTX[1] << endl;

    //0:标定
    CameraCalib();

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
    for(i=0; i<PicNum; i++)
    {
        rvecsMat[i].at<double>(0, 0) = 2.02517075054481;
        rvecsMat[i].at<double>(0, 1) = 1.747835480970283;
        rvecsMat[i].at<double>(0, 2) = -0.3226978455624659;

        tvecsMat[i].at<double>(0, 0) = -53.80020601312009;
        tvecsMat[i].at<double>(0, 1) = 29.25313600086886;
        tvecsMat[i].at<double>(0, 2) = 417.0198668690805;

        std::cout << "旋转向量" << rvecsMat << endl;
        Rodrigues(rvecsMat[i], rotation_matrix[i]);
        std::cout << "旋转矩阵" << rotation_matrix << endl;
    }

    ///3:外参的旋转与平移矩阵  [R T 0 1]
    for(i=0; i<PicNum; i++)
    {
        RT[i].at<double>(0, 0) = rotation_matrix[i].at<double>(0, 0);
        RT[i].at<double>(0, 1) = rotation_matrix[i].at<double>(0, 1);
        RT[i].at<double>(0, 2) = rotation_matrix[i].at<double>(0, 2);
        RT[i].at<double>(0, 3) = tvecsMat[i].at<double>(0, 0);

        RT[i].at<double>(1, 0) = rotation_matrix[i].at<double>(1, 0);
        RT[i].at<double>(1, 1) = rotation_matrix[i].at<double>(1, 1);
        RT[i].at<double>(1, 2) = rotation_matrix[i].at<double>(1, 2);
        RT[i].at<double>(1, 3) = tvecsMat[i].at<double>(0, 1);

        RT[i].at<double>(2, 0) = rotation_matrix[i].at<double>(2, 0);
        RT[i].at<double>(2, 1) = rotation_matrix[i].at<double>(2, 1);
        RT[i].at<double>(2, 2) = rotation_matrix[i].at<double>(2, 2);
        RT[i].at<double>(2, 3) = tvecsMat[i].at<double>(0, 2);

        RT[i].at<double>(3, 0) = 0;
        RT[i].at<double>(3, 1) = 0;
        RT[i].at<double>(3, 2) = 0;
        RT[i].at<double>(3, 3) = 1;
        std::cout << "camera RT矩阵:" << endl << RT << endl;
    }

    ///4:C = RT x W,获取棋盘格上的点在相机坐标系下的p3d
    //从图像中取3D点,世界3D->相机3D
#ifdef TEST
    Mat PointW = (Mat_<double>(4, 1) << 0, 10, 0, 1);
    std::cout << "PointW" << endl << PointW << endl;
    Mat PointC;

    PointC = RT[0] * PointW;
    std::cout << "PointC:" << RT[0] * PointW << endl;

    //vector<Point3f> p3d
    vector<Point3d>  points3d(1);
    points3d[0].x = PointC.at<double>(0, 0);
    points3d[0].y = PointC.at<double>(1, 0);
    points3d[0].z = PointC.at<double>(2, 0);
    std::cout << "points3d:" << points3d << endl;
#else
    //std::cout << "metadata_of_pic.mPic_WorldChessboardPointSet[0]:" << metadata_of_pic.mPic_WorldChessboardPointSet[0][87] << endl;
    //metadata_of_pic.mPic_WorldChessboardPointSet[2].at<double>(0, 0)

    for(int n=0; n<PicNum; n++)
    {
        vector<Mat> tempCamera4NumPointSet;
        vector<Point3f> tempPointSet;
        //将PointC转换为points3d,作为输出
        Mat PointC;
        Point3d points3d;

        for(int i=0; i<BOARD_SIZE_Y; i++) //height
        {
            for(int j=0; j<BOARD_SIZE_X; j++) //width
            {
                //metadata_of_pic.mPic_WorldChessboard4NumPointSet[n][i*BOARD_SIZE_Y + j];
                //PointC = RT[0] * PointW;
                PointC = RT[n] * metadata_of_pic.mPic_WorldChessboard4NumPointSet[n][i*BOARD_SIZE_X + j];
                tempCamera4NumPointSet.push_back(PointC);

//                std::cout << "metadata_of_pic.mPic_CameraChessboard4NumPointSet["<<n<<"][" <<i<< "*BOARD_SIZE_X + "<<j<<"]:"  << metadata_of_pic.mPic_CameraChessboard4NumPointSet[n][i*BOARD_SIZE_X + j] << endl;

                //added by flq;
                //获取角点在相机坐标系的坐标
                points3d.x = PointC.at<double>(0, 0);
                points3d.y = PointC.at<double>(1, 0);
                points3d.z = PointC.at<double>(2, 0);
                tempPointSet.push_back(points3d);
                //added end

                //add for FitPlane
                //xxx
                //added end
            }
        }
        //副产品
        metadata_of_pic.mPic_CameraChessboard4NumPointSet.push_back(tempCamera4NumPointSet);
        //需要得到的
        metadata_of_pic.mPic_CameraChessboardPointSet.push_back(tempPointSet);
    }
    std::cout << "KKKmetadata_of_pic.mPic_CameraChessboard4NumPointSet[0]\n:" << metadata_of_pic.mPic_CameraChessboard4NumPointSet[0][2] << endl;

#endif
    ///5:计算得到ln, 即多点拟合空间中的直线
    //直线方程是1：A1x+B1y+C1z+D1=0,A2x+B2y+C2z+D2=0,联立
    //2:空间直线的标准式(x-x0)/a＝(y-y0)/b＝(z-z0)/c, 其中(a,b,c)为方向向量
    //3:空间直线的两点式： (x-x1)/(x-x2)＝(y-y1)/(y-y2)＝(z-z1)/(z-z2), take this
#ifdef TEST
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
    //LeastSquare_linear(p3d);

    Get3DLinearValue(p3d);
#else
    vector<Point3d>  p3d(BOARD_SIZE_X/* - 1*/);  //解决malloc(): memory corruption

    for(int n=0; n<PicNum; n++)
    {
        for(int i=0; i<BOARD_SIZE_Y; i++) //height
        {
            for(int j=0; j<BOARD_SIZE_X; j++) //width
            {
                p3d[j] = metadata_of_pic.mPic_CameraChessboardPointSet[n][i*BOARD_SIZE_X + j];
                cout << "p3d[" << j << "]" << p3d[j] <<endl;
            }
            cout << endl;
            //获取逐条角点连线的空间方程
            Get3DLinearValue(p3d);
        }

    }

/*
    vector<Point3d>  p3d(4);
    p3d[0].x = metadata_of_pic.mPic_CameraChessboardPointSet[0][0].x;
    p3d[0].y = metadata_of_pic.mPic_CameraChessboardPointSet[0][0].y;
    p3d[0].z = metadata_of_pic.mPic_CameraChessboardPointSet[0][0].z;
    p3d[1].x = metadata_of_pic.mPic_CameraChessboardPointSet[0][1].x;
    p3d[1].y = metadata_of_pic.mPic_CameraChessboardPointSet[0][1].y;
    p3d[1].z = metadata_of_pic.mPic_CameraChessboardPointSet[0][1].z;
    p3d[2].x = metadata_of_pic.mPic_CameraChessboardPointSet[0][2].x;
    p3d[2].y = metadata_of_pic.mPic_CameraChessboardPointSet[0][2].y;
    p3d[2].z = metadata_of_pic.mPic_CameraChessboardPointSet[0][2].z;
    p3d[3].x = metadata_of_pic.mPic_CameraChessboardPointSet[0][3].x;
    p3d[3].y = metadata_of_pic.mPic_CameraChessboardPointSet[0][3].y;
    p3d[3].z = metadata_of_pic.mPic_CameraChessboardPointSet[0][3].z;

    Get3DLinearValue(p3d);
*/
#endif
    ///5.1:拟合标定板平面
#if 0
    vector<Point3d> BoardP3d(BOARD_SIZE_X * BOARD_SIZE_Y);
    cv::Mat InputPoints = cv::Mat::ones(BOARD_SIZE_X*BOARD_SIZE_Y,3,CV_32FC1/*CV_64FC1*/);//cv::Mat::ones(rows,cols,CV_32FC1);

    for(int n=0; n<PicNum; n++)
    {
        for(int i=0; i<BOARD_SIZE_Y; i++) //height
        {
            for(int j=0; j<BOARD_SIZE_X; j++) //width
            {
                InputPoints.at<double>(i*BOARD_SIZE_X + j, 0) = metadata_of_pic.mPic_CameraChessboardPointSet[n][i*BOARD_SIZE_X + j].x;
                InputPoints.at<double>(i*BOARD_SIZE_X + j, 1) = metadata_of_pic.mPic_CameraChessboardPointSet[n][i*BOARD_SIZE_X + j].y;
                InputPoints.at<double>(i*BOARD_SIZE_X + j, 2) = metadata_of_pic.mPic_CameraChessboardPointSet[n][i*BOARD_SIZE_X + j].z;
                printf("AAInputPoints=(%f,%f,%f)\n",InputPoints.at<double>(i, 0),InputPoints.at<double>(i, 1),InputPoints.at<double>(i, 2));

            }
        }
        //拟合标定板的平面
        Mat plane;
        FitPlane(InputPoints, plane);
    }
#else
    //测试用
    /*
        double x_point[] = { 1, 2, 1, 4, 2, 6, 7, 3, 9 };
        double y_point[] = { 1, 1, 3, 4, 5, 2, 7, 8, 2 };
        double z_point[] = { 91, 102, 103, 104, 105, 106, 107, 108, 109 };
    */
    ///*
        double x_point[] = { 0, 0, 0, 1, 1, 1, 2, 2, 2 };
        double y_point[] = { 0, 1, 2, 0, 1, 2, 0, 1, 2 };
        double z_point[] = { 10, 10, 10, 10, 10, 10, 10, 10, 10 };
    //*/
    vector<double> X_vector(x_point, x_point + 9);
    vector<double> Y_vector(y_point, y_point + 9);
    vector<double> Z_vector(z_point, z_point + 9);

    cv::Mat InputPoints = cv::Mat::ones(X_vector.size(), 3, CV_64FC1/*CV_32FC1*/);//定义用来存储需要拟合点的矩阵
    printf("InputPoints Adress:0x%x\n", &InputPoints);
    for (int i = 0; i < X_vector.size(); ++i)
    {
        InputPoints.at<double>(i, 0) = X_vector[i];//矩阵的值进行初始化   X的坐标值
        InputPoints.at<double>(i, 1) = Y_vector[i];//  Y的坐标值
        InputPoints.at<double>(i, 2) = Z_vector[i]; //  Z的坐标值
        printf("InputPoints=(%f,%f,%f)\n",InputPoints.at<double>(i, 0),InputPoints.at<double>(i, 1),InputPoints.at<double>(i, 2));
    }
    Mat plane;
    FitPlane(InputPoints, plane);
#endif
    ///6,7,8:计算激光平面与ln延长线的交点
    ///6:计算激光线在像素坐标系的方程Ax+By+c=0
#ifdef TEST
    double a;
    double b;
    double c;
    vector<Point2d>  p2d;
    //得到所有的激光点p2d
    StegerLine(p2d);
    //根据得到的激光点拟合直线
    lineFit(p2d, a, b, c);
    printf("a x + b y + c = 0\n%lfx + %lfy + %lf = 0\n", a, b, c);
    printf("a=%lf, b=%lf, c=%lf = 0\n", a, b, c);
#else
    Get2DLaserliner(PicNum);
#endif

    ///7:计算每行角点连线在像素坐标系的方程Ax+By+c=0
    printf("\n\nGet2DCornerliner \n");
    Get2DCornerliner(PicNum);

    ///delete,8:计算空间直线在像素坐标系的投影
    ///8:计算,每行角点连线+激光线,在像素坐标的交点
    //F0(x) = a0*x + b0*y + c0 = 0, F1(x) = a1*x + b1*y + c1 = 0
    //|
    //v
    //a0/b0*x + y +c0/b0=0  &&  a1/b1*x+y+c1/b1=0 ,then抵消y
    //|
    //v
    //(a0/b0-a1*b1) * x + (c0/b0  - c1/b1) = 0
    //|
    //v
    //(a0*b1 - a1*b0)x = c1*b0 - c0*b1
    //|
    //v
    //x = (b0*c1 - b1*c0)/D
    //同理可得:
    //y = (a1*c0 - a0*c1)/D
    //D = a0*b1 - a1*b0， (D为0时，表示两直线平行)

    //vector<vector<cv::Point2d>> cross_PtSet;  //to global
    GetCrossPoint(cornerPointLineCoeffs, laserLineCoeffs, cross_PtSet, PicNum, BOARD_SIZE_Y);


    ///9:得到激光平面与ln的各个坐标Pi(xp,yp,zp)
#ifdef TEST
    //原理:https://blog.csdn.net/lyl771857509/article/details/79633412
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

    //2D点+已知平面，得到相机坐标系下的3D坐标
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
    cv::solve(_2Dto3DMat, _00D, XcYcZc, CV_LU);  //DECOMP_LU
    //SVD::solveZ(_2Dto3DMat, _00D, XcYcZc);

    std::cout << "XcYcZc" << XcYcZc<< endl;

    double InA[9] = {Intrinsics.at<double>(0, 0), 0, Intrinsics.at<double>(0, 2) - testp2d.x,
                    0, Intrinsics.at<double>(1, 1), Intrinsics.at<double>(1, 2) - testp2d.y,
                    aa, bb, cc};
    double InB[3] = {0,0,-dd};
    Doolittle(3, InA, InB);
#else
    //原理:https://blog.csdn.net/lyl771857509/article/details/79633412
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

    //2D点+已知平面，得到相机坐标系下的3D坐标    2D点:cross_PtSet[n][i];
    Point2d  testp2d;
    testp2d.x = 740;   //u
    testp2d.y = 580;   //v
    double aa = 1;
    double bb = 2;
    double cc = 3;
    double dd = -1400;//-1400;

    Mat _2Dto3DMat = Mat(3,3,CV_64FC1,Scalar::all(0));

    for(int n=0; n<PicNum; n++)
    {
        for(int i=0; i<BOARD_SIZE_Y; i++)
        {
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
            _2Dto3DMat.at<double>(0, 0) = Intrinsics.at<double>(0, 0);
            _2Dto3DMat.at<double>(0, 1) = 0;
            _2Dto3DMat.at<double>(0, 2) = Intrinsics.at<double>(0, 2) - cross_PtSet[n][i].x;// -u
            _2Dto3DMat.at<double>(1, 0) = 0;
            _2Dto3DMat.at<double>(1, 1) = Intrinsics.at<double>(1, 1);
            _2Dto3DMat.at<double>(1, 2) = Intrinsics.at<double>(1, 2) - cross_PtSet[n][i].y;// -v
            _2Dto3DMat.at<double>(2, 0) = aa;
            _2Dto3DMat.at<double>(2, 1) = bb;
            _2Dto3DMat.at<double>(2, 2) = cc;
#if 0//def DEBUG
            std::cout << "_2Dto3DMat" << endl << _2Dto3DMat << endl;
#endif

            Mat _00D = (Mat_<double>(3, 1) << 0, 0, -dd);

            Mat XcYcZc;
            //CV_LU - 最佳主元选取的高斯消除法
            //CV_SVD - 奇异值分解法 (SVD)
            //CV_SVD_SYM - 对正定对称矩阵的 SVD 方法
            cv::solve(_2Dto3DMat, _00D, XcYcZc, CV_LU);  //DECOMP_LU
            //SVD::solveZ(_2Dto3DMat, _00D, XcYcZc);
#ifdef DEBUG
            std::cout << "第" << n << "张图的第" << i <<"个激光与角点连线的交点:XcYcZc" << XcYcZc<< endl;
#endif
        }

    }
#endif
    ///9:通过计算得到的多个3D激光点，拟合得到空间中激光平面与标定板平面的3D直线
    ///10:得到多组图像标定板与激光平面的N个交点，拟合激光平面.  ("N个交点"为ln与激光平面的交点)


    ///世界坐标系W下，标定板的平面坐标
    //0x + 0y + z = 0
    return 0;
}
