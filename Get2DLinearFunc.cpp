#include "Get2DLinearFunc.h"

#include "CameraCalib.h"
using namespace std;
using namespace cv;

extern MetadataFromPic metadata_of_pic;

vector<LinearFunctionCoefficients> laserLineCoeffs;  //一条激光线在图像上坐标的直线方程
vector<vector<LinearFunctionCoefficients>> cornerPointLineCoeffs;   //ln在平面上的投影

///从图像中计算得到亚像素级的二维点
///ScanThread::FindBrightestPointInRow,找到每一行中最亮的点
///p2d: 从图像中获取到的二维点坐标
int StegerLine(std::vector<cv::Point2d> &p2d)
{
    Mat img0 = imread("2.bmp", 1); //0_laser.jpg  //2.bmp
    Mat img;
    cvtColor(img0, img0, CV_BGR2GRAY);
    img = img0.clone();

    //高斯滤波
    //注意:在求Hessian矩阵之前需要对图像进行高斯滤波
    img.convertTo(img, CV_32FC1);
    GaussianBlur(img, img, Size(0, 0), 6, 6);

    //一阶偏导数
    Mat m1, m2;
    m1 = (Mat_<float>(1, 2) << 1, -1);  //x偏导
    m2 = (Mat_<float>(2, 1) << 1, -1);  //y偏导

    Mat dx, dy;
    filter2D(img, dx, CV_32FC1, m1);
    filter2D(img, dy, CV_32FC1, m2);

    //二阶偏导数
    Mat m3, m4, m5; m3 = (Mat_<float>(1, 3) << 1, -2, 1);  //二阶x偏导
    m4 = (Mat_<float>(3, 1) << 1, -2, 1);  //二阶y偏导
    m5 = (Mat_<float>(2, 2) << 1, -1, -1, 1);  //二阶xy偏导

    Mat dxx, dyy, dxy;
    filter2D(img, dxx, CV_32FC1, m3);
    filter2D(img, dyy, CV_32FC1, m4);
    filter2D(img, dxy, CV_32FC1, m5);

    //hessian矩阵
    double maxD = -1;
    int imgcol = img.cols;
    int imgrow = img.rows;
    vector<double> Pt;

    printf("imgcol=%d, imgrow=%d\n", imgcol, imgrow);
    for (int i=0;i<imgcol;i++)//1280
    {
        for (int j=0;j<imgrow;j++) //960
        {
            //200:亮度门限
            if (img0.at<uchar>(j,i)>200)
            {
                Mat hessian(2, 2, CV_32FC1);
                hessian.at<float>(0, 0) = dxx.at<float>(j, i);
                hessian.at<float>(0, 1) = dxy.at<float>(j, i);
                hessian.at<float>(1, 0) = dxy.at<float>(j, i);
                hessian.at<float>(1, 1) = dyy.at<float>(j, i);
                //std::cout << "hessian:" << endl << hessian << endl;

                Mat eValue;
                Mat eVectors;

                //src – input matrix that must have CV_32FC1 or CV_64FC1 type, square size and be symmetrical (src T == src).
                //eigenvalues – output vector of eigenvalues of the same type as src; the eigenvalues are stored in the descending order.
                //eigenvectors – output matrix of eigenvectors; it has the same size and type as src; the eigenvectors are stored as subsequent matrix rows, in the same order as the corresponding eigenvalues.
                //特征值（eigenvalue),特征向量（eigenvector）,特征值分解（eigenvalue decomposition）
                //https://blog.csdn.net/zhengwei223/article/details/78913898
                cv::eigen(hessian, eValue, eVectors);

                double nx, ny;
                double fmaxD = 0;
                //fabs功能：求浮点数x的绝对值
                if (fabs(eValue.at<float>(0,0))>= fabs(eValue.at<float>(1,0))) //求特征值最大时对应的特征向量
                {
                    nx = eVectors.at<float>(0, 0);
                    ny = eVectors.at<float>(0, 1);
                    fmaxD = eValue.at<float>(0, 0);
                }
                else
                {
                    nx = eVectors.at<float>(1, 0);
                    ny = eVectors.at<float>(1, 1);
                    fmaxD = eValue.at<float>(1, 0);
                }

                double t = -(nx*dx.at<float>(j, i) + ny*dy.at<float>(j, i)) / (nx*nx*dxx.at<float>(j,i)+2*nx*ny*dxy.at<float>(j,i)+ny*ny*dyy.at<float>(j,i));

                if (fabs(t*nx)<=0.5 && fabs(t*ny)<=0.5)
                {

                    Pt.push_back(i);
                    Pt.push_back(j);
                }

            }
        }
    }

    printf("图像中获取激光点的个数:%d\n\n", Pt.size()/2);
    for (int k = 0;k<Pt.size()/2;k++)
    {
        Point rpt;
        rpt.x = Pt[2 * k + 0];
        rpt.y = Pt[2 * k + 1];
        circle(img0, rpt, 1.0/*1*/, Scalar(127, 127, 127));//255
#ifdef DEBUG
        printf("(%d,%d)\n",rpt.x, rpt.y);
#endif
        //set value
        Point2d point2d;
        point2d.x = rpt.x;
        point2d.y = rpt.y;
        p2d.push_back(point2d);
    }

    //imshow("result", img0);
    //waitKey(0);

    return 0;
}

int StegerLine2(char* filename,std::vector<cv::Point2d> &p2d)
{
    Mat img0 = imread(filename, 1); //0_laser.jpg
    Mat img;
    cvtColor(img0, img0, CV_BGR2GRAY);
    img = img0.clone();

    //高斯滤波
    //注意:在求Hessian矩阵之前需要对图像进行高斯滤波
    img.convertTo(img, CV_32FC1);
    GaussianBlur(img, img, Size(0, 0), 6, 6);

    //一阶偏导数
    Mat m1, m2;
    m1 = (Mat_<float>(1, 2) << 1, -1);  //x偏导
    m2 = (Mat_<float>(2, 1) << 1, -1);  //y偏导

    Mat dx, dy;
    filter2D(img, dx, CV_32FC1, m1);
    filter2D(img, dy, CV_32FC1, m2);

    //二阶偏导数
    Mat m3, m4, m5; m3 = (Mat_<float>(1, 3) << 1, -2, 1);  //二阶x偏导
    m4 = (Mat_<float>(3, 1) << 1, -2, 1);  //二阶y偏导
    m5 = (Mat_<float>(2, 2) << 1, -1, -1, 1);  //二阶xy偏导

    Mat dxx, dyy, dxy;
    filter2D(img, dxx, CV_32FC1, m3);
    filter2D(img, dyy, CV_32FC1, m4);
    filter2D(img, dxy, CV_32FC1, m5);

    //hessian矩阵
    double maxD = -1;
    int imgcol = img.cols;
    int imgrow = img.rows;
    vector<double> Pt;

    printf("imgcol=%d, imgrow=%d\n", imgcol, imgrow);
    for (int i=0;i<imgcol;i++)//1280
    {
        for (int j=0;j<imgrow;j++) //960
        {
            //200:亮度门限
            if (img0.at<uchar>(j,i)>200)
            {
                Mat hessian(2, 2, CV_32FC1);
                hessian.at<float>(0, 0) = dxx.at<float>(j, i);
                hessian.at<float>(0, 1) = dxy.at<float>(j, i);
                hessian.at<float>(1, 0) = dxy.at<float>(j, i);
                hessian.at<float>(1, 1) = dyy.at<float>(j, i);
                //std::cout << "hessian:" << endl << hessian << endl;

                Mat eValue;
                Mat eVectors;

                //src – input matrix that must have CV_32FC1 or CV_64FC1 type, square size and be symmetrical (src T == src).
                //eigenvalues – output vector of eigenvalues of the same type as src; the eigenvalues are stored in the descending order.
                //eigenvectors – output matrix of eigenvectors; it has the same size and type as src; the eigenvectors are stored as subsequent matrix rows, in the same order as the corresponding eigenvalues.
                //特征值（eigenvalue),特征向量（eigenvector）,特征值分解（eigenvalue decomposition）
                //https://blog.csdn.net/zhengwei223/article/details/78913898
                cv::eigen(hessian, eValue, eVectors);

                double nx, ny;
                double fmaxD = 0;
                //fabs功能：求浮点数x的绝对值
                if (fabs(eValue.at<float>(0,0))>= fabs(eValue.at<float>(1,0))) //求特征值最大时对应的特征向量
                {
                    nx = eVectors.at<float>(0, 0);
                    ny = eVectors.at<float>(0, 1);
                    fmaxD = eValue.at<float>(0, 0);
                }
                else
                {
                    nx = eVectors.at<float>(1, 0);
                    ny = eVectors.at<float>(1, 1);
                    fmaxD = eValue.at<float>(1, 0);
                }

                double t = -(nx*dx.at<float>(j, i) + ny*dy.at<float>(j, i)) / (nx*nx*dxx.at<float>(j,i)+2*nx*ny*dxy.at<float>(j,i)+ny*ny*dyy.at<float>(j,i));

                if (fabs(t*nx)<=0.5 && fabs(t*ny)<=0.5)
                {

                    Pt.push_back(i);
                    Pt.push_back(j);
                }

            }
        }
    }

    printf("图像中获取激光点的个数:%d\n\n", Pt.size()/2);
    for (int k = 0;k<Pt.size()/2;k++)
    {
        Point rpt;
        rpt.x = Pt[2 * k + 0];
        rpt.y = Pt[2 * k + 1];
        circle(img0, rpt, 1.0/*1*/, Scalar(127, 127, 127));//255
#ifdef DEBUG
        printf("(%d,%d)\n",rpt.x, rpt.y);
#endif
        //set value
        Point2d point2d;
        point2d.x = rpt.x;
        point2d.y = rpt.y;
        p2d.push_back(point2d);
    }

    //imshow("result", img0);
    //waitKey(0);

    return 0;
}

/**
  * 最小二乘法直线拟合（不是常见的一元线性回归算法）
  * 将离散点拟合为  a x + b y + c = 0 型直线
  * 假设每个点的 X Y 坐标的误差都是符合 0 均值的正态分布的。
  * 与一元线性回归算法的区别：一元线性回归算法假定 X 是无误差的，只有 Y 有误差。
  */
//直线拟合算法
bool lineFit(/*const */std::vector<cv::Point2d> &points, double &a, double &b, double &c)
{
     int size = points.size();

     if(size < 2)
     {
         a = 0;
         b = 0;
         c = 0;
         return false;
     }

     double x_mean = 0;
     double y_mean = 0;

     for(int i = 0; i < size; i++)
     {
         x_mean += points[i].x;
         y_mean += points[i].y;
     }

     x_mean /= size; y_mean /= size; //至此，计算出了 x y 的均值

     double Dxx = 0, Dxy = 0, Dyy = 0;

     for(int i = 0; i < size; i++)
     {
         Dxx += (points[i].x - x_mean) * (points[i].x - x_mean);
         Dxy += (points[i].x - x_mean) * (points[i].y - y_mean);
         Dyy += (points[i].y - y_mean) * (points[i].y - y_mean);
     }

     double lambda = ( (Dxx + Dyy) - sqrt( (Dxx - Dyy) * (Dxx - Dyy) + 4 * Dxy * Dxy) ) / 2.0;
     double den = sqrt( Dxy * Dxy + (lambda - Dxx) * (lambda - Dxx));
     a = Dxy / den;
     b = (lambda - Dxx) / den;
     c = - a * x_mean - b * y_mean;
     //printf("x_mean:%lf, y_mean:%lf\n", x_mean, y_mean);
     //printf("a x + b y + c = 0\n%lfx + %lfy + %lf = 0\n", a, b, c);

     return true;

}

//获取所有图像的激光直线方程ax+by+c=0
int Get2DLaserliner(int picnum)
{
    //vector<LinearFunctionCoefficients> laserLineCoeffs;  ////to global
    LinearFunctionCoefficients tmplaserLineCoeffs;

    vector<Point2d>  p2d;

    //input file
    char* filename = new char[256];
    memset(filename, 0, 256);
    for(int i=0; i<picnum; i++)
    {
        sprintf(filename, "%d_laser.jpg", i);

        ///1:图像中的激光线方程
        //得到所有的激光点p2d
        StegerLine2(filename, p2d);
        ///2:拟合,激光条投影在标定板上的直线
        //根据得到的激光点拟合直线

        //为laserLineCoeffs.a, laserLineCoeffs.b, laserLineCoeffs.c赋值
        lineFit(p2d, tmplaserLineCoeffs.a, tmplaserLineCoeffs.b, tmplaserLineCoeffs.c);
        //给激光线的2D方程赋值
        laserLineCoeffs.push_back(tmplaserLineCoeffs);

        printf("\n\n第%d张图像中的激光直线方程ax+by+c=0如下：\n", i);
        printf("a x + b y + c = 0\n%lfx + %lfy + %lf = 0\n", laserLineCoeffs[i].a, laserLineCoeffs[i].b, laserLineCoeffs[i].c);
        printf("a=%lf, b=%lf, c=%lf = 0\n", laserLineCoeffs[i].a, laserLineCoeffs[i].b, laserLineCoeffs[i].c);

#ifdef GUI
        Point2d PtStart , PtEnd;
        Mat frame=imread(filename);


        PtStart.x = 0;
        PtStart.y = -laserLineCoeffs[i].c/laserLineCoeffs[i].b;  // -c/b

        PtEnd.x = frame.cols - 1;
        PtEnd.y = (-laserLineCoeffs[i].c -laserLineCoeffs[i].a*(frame.cols - 1)) / laserLineCoeffs[i].b;   // (-c -a*1279)/b

        line(frame, PtStart, PtEnd, Scalar(255, 0, 0), 1);
        imshow("LaserLine",frame);
        cv::waitKey(41);
#endif

    }

    delete(filename);

    return 0;
}

////获取角点连线的直线方程
int Get2DCornerliner(int picnum)
{
    //vector<vector<LinearFunctionCoefficients>> cornerPointLineCoeffs;  //to global
    vector<LinearFunctionCoefficients> tempVectorCoeffs;
    LinearFunctionCoefficients tempCoeffs;

    vector<Point2d>  CornerLine_p2d(BOARD_SIZE_X);

    //input file
    char* filename = new char[256];
    memset(filename, 0, 256);
    for(int n=0; n<picnum; n++)
    {
        sprintf(filename, "%d_laser.jpg", n);

#ifdef GUI
        Point2d PtStart , PtEnd;
        Mat frame=imread(filename);
#endif
        ///1:将mPic_2DChessboardPointSet中的2D点逐线解析出来
        for(int i=0; i<BOARD_SIZE_Y; i++) //height
        {
            for(int j=0; j<BOARD_SIZE_X; j++) //width
            {
                CornerLine_p2d[j] = metadata_of_pic.mPic_2DChessboardPointSet[n][i*BOARD_SIZE_X + j];
                cout << "CornerLine_p2d[" << j << "]" << CornerLine_p2d[j] <<endl;
            }
            cout << endl;
            ///2:拟合,每一行角点的直线方程
            lineFit(CornerLine_p2d, tempCoeffs.a, tempCoeffs.b, tempCoeffs.c);
            printf("a x + b y + c = 0\n%lfx + %lfy + %lf = 0\n", n, i,tempCoeffs.a, tempCoeffs.b, tempCoeffs.c);
            printf("a=%lf, b=%lf, c=%lf = 0\n", tempCoeffs.a, tempCoeffs.b, tempCoeffs.c);

            tempVectorCoeffs.push_back(tempCoeffs);

#ifdef GUI
            PtStart.x = 0;
            PtStart.y = -tempCoeffs.c/tempCoeffs.b;  // -c/b

            PtEnd.x = frame.cols - 1;
            PtEnd.y = (-tempCoeffs.c -tempCoeffs.a*(frame.cols - 1)) / tempCoeffs.b;   // (-c -a*1279)/b

            line(frame, PtStart, PtEnd, Scalar(0, 0, 255), 1);
#endif
        }
        cornerPointLineCoeffs.push_back(tempVectorCoeffs);

        vector<LinearFunctionCoefficients>().swap(tempVectorCoeffs);//added by flq,  清空临时的vector

#ifdef GUI
        imshow("LaserLine",frame);
        cv::waitKey(41);
#endif
    }

    delete(filename);

    return 0;
}
