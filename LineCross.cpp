#include "LineCross.h"

extern cv::Size image_size;  /* 图像的尺寸 */

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
//(a0*b1 - a1*b0)x = b0*C1 - b1*c0
//|
//v
//x = (b0*c1 - b1*c0)/D
//同理可得:
//y = (a1*c0 - a0*c1)/D
//D = a0*b1 - a1*b0， (D为0时，表示两直线平行)
///Input: cornerPointLineCoeffs, laserLineCoeffs;  Output:cross_PtSet
int GetCrossPoint(vector<vector<LinearFunctionCoefficients>> mycornerPointLineCoeffs, vector<LinearFunctionCoefficients> mylaserLineCoeffs, vector<vector<cv::Point2d>>& cross_PtSet, int PicNum, int board_size_y)
{
    double a0,b0,c0 = 0;
    double a1,b1,c1 = 0;
    vector<cv::Point2d> tmp_cross_Pt;

    for(int n=0; n<PicNum; n++)
    {
#ifdef DEBUG
        cout << "第" << n << "张图片中的激光线的方程:" << "(" << mylaserLineCoeffs[n].a <<","<< mylaserLineCoeffs[n].b << "," << mylaserLineCoeffs[n].c<<   ")" << endl;
#endif
        //激光线方程系数
        a0 = mylaserLineCoeffs[n].a;
        b0 = mylaserLineCoeffs[n].b;
        c0 = mylaserLineCoeffs[n].c;

        cv::Point2d tmp_pt;
        for(int i=0; i<board_size_y; i++) //height
        {
            //第i条角点连线的直线方程系数
            a1 = mycornerPointLineCoeffs[n][i].a;
            b1 = mycornerPointLineCoeffs[n][i].b;
            c1 = mycornerPointLineCoeffs[n][i].c;

#ifdef DEBUG
            //cout << "第" << n << "张图片中的第"<< i <<"行直线的方程:" << "(" << a1 <<","<< b1 << "," << c1<<   ")" << endl;
#endif

            //x,y为所求得的交点坐标
            double D = a0*b1 - a1*b0;
            double x = (b0*c1 - b1*c0) / D;
            double y = (a1*c0 - a0*c1) / D;

            if(x>0 && x<image_size.width && y>0 && y<image_size.height) //if(1)
            {
                tmp_pt.x = x;
                tmp_pt.y = y;
                tmp_cross_Pt.push_back(tmp_pt);
#ifdef DEBUG
                cout << "第" << n << "张图片中的激光与角点连线的第" << i << "个交点"<< tmp_pt << endl;
#endif
            }
            else
            {
                tmp_pt.x = 0;
                tmp_pt.y = 0;
                tmp_cross_Pt.push_back(tmp_pt);
                continue;
            }
        }

        cout << endl;
        cross_PtSet.push_back(tmp_cross_Pt);
        vector<cv::Point2d>().swap(tmp_cross_Pt);//added by flq,  清空临时的vector
    }

    return 0;
}
