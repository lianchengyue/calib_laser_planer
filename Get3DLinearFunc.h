#ifndef GET3DLINEARFUNC_H
#define GET3DLINEARFUNC_H

#define MAX 10
//#define PtSum 38 //点的个数
#define MTX_AMT 100

#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"


void Inverse(double *matrix1[],double *matrix2[],int n,double d);
double Determinant(double* matrix[],int n);
double AlCo(double* matrix[],int jie,int row,int column);
double Cofactor(double* matrix[],int jie,int row,int column);
void Empty(double *matrix[],int row,int column);
int Get3DLinearValue(std::vector<cv::Point3d> PrimiDataSet);


#endif // GET3DLINEARFUNC_H
