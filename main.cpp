#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "Get2DLinearFunc.h"
#include "Get3DLinearFunc.h"
#ifdef DO_CALIB
#include "CameraCalib.h"
#else
#include "CameraRemap.h"
#endif
#include "LineCross.h"
#include "Get3DPlanarFunc.h"

using namespace std;
using namespace cv;

extern MetadataFromPic metadata_of_pic;
extern vector<LinearFunctionCoefficients> laserLineCoeffs;  //一条激光线在图像上坐标的直线方程
extern vector<vector<LinearFunctionCoefficients>> cornerPointLineCoeffs;   //ln在平面上的投影

vector<vector<cv::Point2d>> cross_PtSet;

//后期移动到.h文件中
vector<PlanarFunctionCoefficients> BoardPlaneCoeffs;//标定板平面坐标系数
vector<Point2d> LaserPlane;//标定板平面坐标系数
cv::Mat LaserPlanarPointMat = cv::Mat::ones(BOARD_SIZE_Y*PICTURE_COUNT, 3, CV_64FC1);

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
    }

    for (int i = 0; i < n; i++)//由Ly=b算y
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

    Mat CameraMatrix = Mat(3,4,CV_64FC1,Scalar::all(0));

    //每幅图像的平移向量
    Mat tvecsMat[PicNum] = Mat(1,3,CV_64FC1,Scalar::all(0));
    //每幅图像的旋转向量
    Mat rvecsMat[PicNum] = Mat(1,3,CV_64FC1,Scalar::all(0));
    Mat rotation_matrix[PicNum] = Mat(3,3,CV_64FC1, Scalar::all(0));

    Mat RT[PicNum] = Mat(4, 4 ,CV_64FC1,Scalar::all(0));  //RT Matrix

    //0:标定
#ifdef DO_CALIB
    CameraCalib();
#else
    CameraRemap();
#endif

    ///1,2,3:外参的旋转与平移矩阵  [R T 0 1]
    for(i=0; i<PicNum; i++)
    {
        RT[i].at<double>(0, 0) = metadata_of_pic.mPic_rotationMatrixSet[i].at<double>(0, 0);
        RT[i].at<double>(0, 1) = metadata_of_pic.mPic_rotationMatrixSet[i].at<double>(0, 1);
        RT[i].at<double>(0, 2) = metadata_of_pic.mPic_rotationMatrixSet[i].at<double>(0, 2);
        RT[i].at<double>(0, 3) = metadata_of_pic.mPic_tvecsMatSet[i].at<double>(0, 0);

        RT[i].at<double>(1, 0) = metadata_of_pic.mPic_rotationMatrixSet[i].at<double>(1, 0);
        RT[i].at<double>(1, 1) = metadata_of_pic.mPic_rotationMatrixSet[i].at<double>(1, 1);
        RT[i].at<double>(1, 2) = metadata_of_pic.mPic_rotationMatrixSet[i].at<double>(1, 2);
        RT[i].at<double>(1, 3) = metadata_of_pic.mPic_tvecsMatSet[i].at<double>(0, 1);

        RT[i].at<double>(2, 0) = metadata_of_pic.mPic_rotationMatrixSet[i].at<double>(2, 0);
        RT[i].at<double>(2, 1) = metadata_of_pic.mPic_rotationMatrixSet[i].at<double>(2, 1);
        RT[i].at<double>(2, 2) = metadata_of_pic.mPic_rotationMatrixSet[i].at<double>(2, 2);
        RT[i].at<double>(2, 3) = metadata_of_pic.mPic_tvecsMatSet[i].at<double>(0, 2);

        RT[i].at<double>(3, 0) = 0;
        RT[i].at<double>(3, 1) = 0;
        RT[i].at<double>(3, 2) = 0;
        RT[i].at<double>(3, 3) = 1;
        std::cout << "camera RT[" << i << "]矩阵:" << endl << RT[i] << endl << endl;
    }

    ///4:C = RT x W,获取棋盘格上的点在相机坐标系下的p3d
    printf("\n\n4:C = RT x W,获取棋盘格上的点在相机坐标系下的p3d\n");
    //从图像中取3D点,世界3D->相机3D
    for(int n=0; n<PicNum; n++)
    {
        vector<Mat> tempCamera4NumPointSet;
        vector<Point3d> tempPointSet;
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

                if (0 == i && 0 == j)
                {
                    printf("\n第%d张照片的第一个角点的相机坐标系空间坐标:(%f,%f,%f)\n", n, points3d.x, points3d.y, points3d.z);
                }
                //added end
            }
        }
        //副产品
        metadata_of_pic.mPic_CameraChessboard4NumPointSet.push_back(tempCamera4NumPointSet);
        //需要得到的
        metadata_of_pic.mPic_CameraChessboardPointSet.push_back(tempPointSet);
    }
    std::cout << "KKKmetadata_of_pic.mPic_CameraChessboard4NumPointSet[0]\n:" << metadata_of_pic.mPic_CameraChessboard4NumPointSet[0][2] << endl;

    ///5:计算得到ln, 即多点拟合空间中的直线.      得到空间直线，暂时未用
    //直线方程是1：A1x+B1y+C1z+D1=0,A2x+B2y+C2z+D2=0,联立
    //2:空间直线的标准式(x-x0)/a＝(y-y0)/b＝(z-z0)/c, 其中(a,b,c)为方向向量
    //3:空间直线的两点式： (x-x1)/(x-x2)＝(y-y1)/(y-y2)＝(z-z1)/(z-z2), take this

    printf("\n\n5:计算得到ln, 即多点拟合空间中的直线.      得到空间直线，暂时未用\n");
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
        //continue;  //only for add breakpoint
    }

    ///5.1:拟合标定板平面
    printf("\n\n5.1:拟合标定板平面\n");
    vector<Point3d> BoardP3d(BOARD_SIZE_X * BOARD_SIZE_Y);
    cv::Mat InputPoints = cv::Mat::ones(BOARD_SIZE_X*BOARD_SIZE_Y, 3, CV_64FC1);//cv::Mat::ones(rows,cols,CV_32FC1);

    for(int n=0; n<PicNum; n++)
    {
        for(int i=0; i<BOARD_SIZE_Y; i++) //height
        {
            for(int j=0; j<BOARD_SIZE_X; j++) //width
            {
                //图片n中相机坐标系所有的3DFitPlane点坐标
                InputPoints.at<double>(i*BOARD_SIZE_X + j, 0) = metadata_of_pic.mPic_CameraChessboardPointSet[n][i*BOARD_SIZE_X + j].x;
                InputPoints.at<double>(i*BOARD_SIZE_X + j, 1) = metadata_of_pic.mPic_CameraChessboardPointSet[n][i*BOARD_SIZE_X + j].y;
                InputPoints.at<double>(i*BOARD_SIZE_X + j, 2) = metadata_of_pic.mPic_CameraChessboardPointSet[n][i*BOARD_SIZE_X + j].z;
                //printf("mPic_CameraChessboardPointSet=(%f,%f,%f)\n",metadata_of_pic.mPic_CameraChessboardPointSet[n][i*BOARD_SIZE_X + j].x, metadata_of_pic.mPic_CameraChessboardPointSet[n][i*BOARD_SIZE_X + j].y, metadata_of_pic.mPic_CameraChessboardPointSet[n][i*BOARD_SIZE_X + j].z);
                //error        //printf("AAInputPoints=(%f,%f,%f)\n",InputPoints.at<float>(i*BOARD_SIZE_X + j, 0), (i*BOARD_SIZE_X + j, 1), (i*BOARD_SIZE_X + j, 2));
                //ok
                ///printf("AAInputPoints=(%f,%f,%f)\n", InputPoints.at<double>(i*BOARD_SIZE_X + j, 0), InputPoints.at<double>(i*BOARD_SIZE_X + j, 1), InputPoints.at<double>(i*BOARD_SIZE_X + j, 2));
            }
        }
        //拟合标定板的平面
        Mat BoardPlane;
        FitBoardPlane(InputPoints, BoardPlane);
        //赋值
        PlanarFunctionCoefficients tmpPlanarCoeff;
        tmpPlanarCoeff.a = BoardPlane.at<double>(0,0);
        tmpPlanarCoeff.b = BoardPlane.at<double>(1,0);
        tmpPlanarCoeff.c = BoardPlane.at<double>(2,0);
        tmpPlanarCoeff.d = -BoardPlane.at<double>(3,0);
        BoardPlaneCoeffs.push_back(tmpPlanarCoeff);
    }

    ///6,7,8:计算激光平面与ln延长线的交点
    ///6:计算激光线在像素坐标系的方程Ax+By+c=0
    printf("\n\n6:计算激光线在像素坐标系的方程Ax+By+c=0\n");
    Get2DLaserliner(PicNum);

    ///7:计算每行角点连线在像素坐标系的方程Ax+By+c=0
    printf("\n\n7:计算每行角点连线在像素坐标系的方程Ax+By+c=0\n");
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

    printf("\n\n8:计算,每行角点连线+激光线,在像素坐标的交点\n");
    //vector<vector<cv::Point2d>> cross_PtSet;  //to global
    GetCrossPoint(cornerPointLineCoeffs, laserLineCoeffs, cross_PtSet, PicNum, BOARD_SIZE_Y);


    ///9:得到激光平面与ln的各个坐标Pi(xp,yp,zp)
    //原理:https://blog.csdn.net/lyl771857509/article/details/79633412
    printf("\n\n9:得到激光平面与ln的各个坐标Pi(xp,yp,zp)\n");
    //给3x4的相机内参矩阵赋值
    CameraMatrix.at<double>(0, 0) = metadata_of_pic.mPic_Intrinsics.at<double>(0, 0);
    CameraMatrix.at<double>(0, 1) = metadata_of_pic.mPic_Intrinsics.at<double>(0, 0);
    CameraMatrix.at<double>(0, 2) = metadata_of_pic.mPic_Intrinsics.at<double>(0, 0);
    CameraMatrix.at<double>(0, 3) = 0;
    CameraMatrix.at<double>(1, 0) = metadata_of_pic.mPic_Intrinsics.at<double>(1, 0);
    CameraMatrix.at<double>(1, 1) = metadata_of_pic.mPic_Intrinsics.at<double>(1, 0);
    CameraMatrix.at<double>(1, 2) = metadata_of_pic.mPic_Intrinsics.at<double>(1, 0);
    CameraMatrix.at<double>(1, 3) = 0;
    CameraMatrix.at<double>(2, 0) = metadata_of_pic.mPic_Intrinsics.at<double>(2, 0);
    CameraMatrix.at<double>(2, 1) = metadata_of_pic.mPic_Intrinsics.at<double>(2, 0);
    CameraMatrix.at<double>(2, 2) = metadata_of_pic.mPic_Intrinsics.at<double>(2, 0);
    CameraMatrix.at<double>(2, 3) = 0;

    //2D点+已知平面，得到相机坐标系下的3D坐标    2D点:cross_PtSet[n][i];
    double aa;
    double bb;
    double cc;
    double dd;

    Mat _2Dto3DMat = Mat(3,3,CV_64FC1,Scalar::all(0));

    int PtSet_valid_num = 0;

    for(int n=0; n<PicNum; n++)
    {
        aa = BoardPlaneCoeffs[n].a;
        bb = BoardPlaneCoeffs[n].b;
        cc = BoardPlaneCoeffs[n].c;
        dd = BoardPlaneCoeffs[n].d;

        for(int i=0; i<BOARD_SIZE_Y; i++)   //BOARD_SIZE_Y==8
        {
            //抛弃图像之外的点
            if(0 == cross_PtSet[n][i].x && 0 == cross_PtSet[n][i].x)
            {
                LaserPlanarPointMat.at<double>(n*BOARD_SIZE_Y+i, 0) = 0;
                LaserPlanarPointMat.at<double>(n*BOARD_SIZE_Y+i, 1) = 0;
                LaserPlanarPointMat.at<double>(n*BOARD_SIZE_Y+i, 2) = 0;
                continue;
            }

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
            _2Dto3DMat.at<double>(0, 0) = metadata_of_pic.mPic_Intrinsics.at<double>(0, 0);
            _2Dto3DMat.at<double>(0, 1) = 0;
            _2Dto3DMat.at<double>(0, 2) = metadata_of_pic.mPic_Intrinsics.at<double>(0, 2) - cross_PtSet[n][i].x;// -u
            _2Dto3DMat.at<double>(1, 0) = 0;
            _2Dto3DMat.at<double>(1, 1) = metadata_of_pic.mPic_Intrinsics.at<double>(1, 1);
            _2Dto3DMat.at<double>(1, 2) = metadata_of_pic.mPic_Intrinsics.at<double>(1, 2) - cross_PtSet[n][i].y;// -v
            _2Dto3DMat.at<double>(2, 0) = aa;
            _2Dto3DMat.at<double>(2, 1) = bb;
            _2Dto3DMat.at<double>(2, 2) = cc;
#ifdef DEBUG
            std::cout << "_2Dto3DMat" << endl << _2Dto3DMat << endl;
#endif

            Mat _00D = (Mat_<double>(3, 1) << 0, 0, -dd);

            Mat XcYcZc;
            //CV_LU - 最佳主元选取的高斯消除法
            //CV_SVD - 奇异值分解法 (SVD)
            //CV_SVD_SYM - 对正定对称矩阵的 SVD 方法
            cv::solve(_2Dto3DMat, _00D, XcYcZc, CV_LU);  //DECOMP_LU
            //cv::SVD::solveZ(_2Dto3DMat, _00D, XcYcZc);
#ifdef DEBUG
            std::cout << "第" << n << "张图的第" << i <<"个激光与角点连线的交点:XcYcZc" << XcYcZc<< endl;
#endif

            ///cv::Mat LaserPlanarPointMat = cv::Mat::ones(BOARD_SIZE_Y*PicNum, 3, CV_64FC1);  //to global
            LaserPlanarPointMat.at<double>(n*BOARD_SIZE_Y+i, 0) = XcYcZc.at<double>(0, 0);
            LaserPlanarPointMat.at<double>(n*BOARD_SIZE_Y+i, 1) = XcYcZc.at<double>(1, 0);
            LaserPlanarPointMat.at<double>(n*BOARD_SIZE_Y+i, 2) = XcYcZc.at<double>(2, 0);

            PtSet_valid_num++;
        }

    }

    ///拟合激光平面
    Mat myLaserPlane;

    cv::Mat LaserPlanarPointMat_valid = cv::Mat::ones(PtSet_valid_num, 3, CV_64FC1);
    int num = 0;
    for(int n=0; n<PicNum; n++)
    {
        for(int i=0; i<BOARD_SIZE_Y; i++)
        {
            if(0 == LaserPlanarPointMat.at<double>(n*BOARD_SIZE_Y+i, 0)
                    && 0 == LaserPlanarPointMat.at<double>(n*BOARD_SIZE_Y+i, 0)
                    && 0 == LaserPlanarPointMat.at<double>(n*BOARD_SIZE_Y+i, 0))

            {
                continue;
            }
            else
            {
                LaserPlanarPointMat_valid.at<double>(num, 0) = LaserPlanarPointMat.at<double>(n*BOARD_SIZE_Y+i, 0);
                LaserPlanarPointMat_valid.at<double>(num, 1) = LaserPlanarPointMat.at<double>(n*BOARD_SIZE_Y+i, 1);
                LaserPlanarPointMat_valid.at<double>(num, 2) = LaserPlanarPointMat.at<double>(n*BOARD_SIZE_Y+i, 2);
                num++;
            }
        }
    }
    FitLaserPlane(LaserPlanarPointMat_valid, myLaserPlane, PtSet_valid_num);

    ///9:通过计算得到的多个3D激光点，拟合得到空间中激光平面与标定板平面的3D直线
    ///10:得到多组图像标定板与激光平面的N个交点，拟合激光平面.  ("N个交点"为ln与激光平面的交点)


    ///世界坐标系W下，标定板的平面坐标
    //0x + 0y + z = 0
    return 0;
}
