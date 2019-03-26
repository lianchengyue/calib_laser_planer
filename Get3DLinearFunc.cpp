#include "Get3DLinearFunc.h"

void Empty(double *matrix[],int row,int column)//初始化
{
    for (int i = 0;i < row;i++)
    {
        for (int j = 0;j < column;j++)
        {
            *(matrix[i] + j) = 0.0;
        }
    }
}


void Inverse(double *matrix1[],double *matrix2[],int n,double d) //
{
    int i,j;
    for(i=0;i<n;i++)
    {
         matrix2[i]=(double *)malloc(n*sizeof(double)); //给IMatrix一维指针分配空间
    }
    for(i=0;i<n;i++)
    {
         for(j=0;j<n;j++)
         {
             *(matrix2[j]+i)=(AlCo(matrix1,n,i,j)/d); //给IMatrix二维数据分配空间
         }
    }
}


double Determinant(double* matrix[],int n) // 计算行列式
{
    double result=0,temp;
    int i;
    if(n == 1)
    {
        result=(*matrix[0]);
    }
    else
    {
        for(i=0;i<n;i++)
         {
            temp=AlCo(matrix,n,n-1,i);
            result+=(*(matrix[n-1]+i))*temp;
        }
    }
    return result;
}


double AlCo(double* matrix[],int jie,int row,int column)  //
{
    double result;
    if((row+column)%2 == 0) //如果不是偶数倍
    {
        result = Cofactor(matrix,jie,row,column);
    }
    else
    {
        result=(-1)*Cofactor(matrix,jie,row,column);
    }
    return result;
}


double Cofactor(double* matrix[],int jie,int row,int column)
{
    double result;
    int i,j;
    double* smallmatr[MAX-1];
    for(i=0;i<jie-1;i++)
    {
         smallmatr[i]= new double[jie - 1];//n*n矩阵
    }
    for(i=0;i<row;i++)
    {
         for(j=0;j<column;j++)
         {
             *(smallmatr[i]+j)=*(matrix[i]+j);  //给smallmatr赋值
         }
    }
    for(i=row;i<jie-1;i++)
    {
         for(j=0;j<column;j++)
         {
             *(smallmatr[i]+j)=*(matrix[i+1]+j);
         }
    }
    for(i=0;i<row;i++)
    {
         for(j=column;j<jie-1;j++)
         {
             *(smallmatr[i]+j)=*(matrix[i]+j+1);
         }
    }
    for(i=row;i<jie-1;i++)
    {
         for(j=column;j<jie-1;j++)
         {
             *(smallmatr[i]+j)=*(matrix[i+1]+j+1);
         }
    }

    result = Determinant(smallmatr,jie-1);
    for(i=0;i<jie-1;i++)
    {
         delete[] smallmatr[i];
    }
    return result;
}

int Get3DLinearValue(std::vector<cv::Point3d> PrimiDataSet)
{
#if 1
    double array[MTX_AMT][3],*Matrix[2],*IMatrix[2];//允许有MTX_AMT个点
    long PtSum = PrimiDataSet.size();


    //引入噪声，only for 测试
#if 0//def DEBUG
    for (int i = 0; i < PtSum;i++)
    {
        array[i][0] = PrimiDataSet[i].x + rand()/(RAND_MAX+1.0)/10;  //引入噪声 rand()/RAND_MAX
        array[i][1] = PrimiDataSet[i].y + rand()/(RAND_MAX+1.0)/10;// rand()/(RAND_MAX+1.0)/10;  //引入噪声
        array[i][2] = PrimiDataSet[i].z + rand()/(RAND_MAX+1.0)/10;
        //printf("rand()%2=%lf,array[i][0]=%lf\n", rand()%2, array[i][0]);
        printf("Point%d:(%lf,%lf,%lf)\n",i, array[i][0], array[i][1], array[i][2]);
    }
#else
    for (int i = 0; i < PtSum;i++)
    {
        array[i][0] = PrimiDataSet[i].x;  //引入噪声 rand()/RAND_MAX
        array[i][1] = PrimiDataSet[i].y; // rand()/(RAND_MAX+1.0)/10;  //引入噪声
        array[i][2] = PrimiDataSet[i].z;
        //printf("rand()%2=%lf,array[i][0]=%lf\n", rand()%2, array[i][0]);
        printf("Point%d:(%lf,%lf,%lf)\n",i, array[i][0], array[i][1], array[i][2]);
    }
#endif

    double X0,Y0,m,n;//这是待拟合的四个参数，已知分别对应：1、2、3、2现在我们来拟合
    for (int i = 0;i < 2;i++)
    {
        Matrix[i] =  new double[2];
        IMatrix[i] = new double[2];
    }
    Empty(Matrix,2,2);//将矩阵的前两行两列置位0


    for (int j = 0;j < PtSum;j++)//
    {
        *(Matrix[0] + 0) += array[j][2]*array[j][2];//Matrix[0][0]=array[0][2]*array[0][2]=z*z
        *(Matrix[0] + 1) += array[j][2];//Matrix[0][1]=array[0][2]=z
    }
    *(Matrix[1] + 0) = *(Matrix[0] + 1);//Matrix[1][0]=z;
    *(Matrix[1] + 1) = 1.0*PtSum;//Matrix[1][1]=n;


//下面两步是求Matrix的逆矩阵并存到IMatrix中
//A^(-1)=(1/|A|)×A* ，其中A^(-1)表示矩阵A的逆矩阵，其中|A|为矩阵A的行列式，A*为矩阵A的伴随矩阵。
    double d = Determinant(Matrix,2);
    Inverse(Matrix,IMatrix,2,d);//反转存到IMatrix中

    Empty(Matrix,2,2);
    for (int i = 0;i < PtSum;i++)
    {
        *(Matrix[0] + 0) += array[i][0]*array[i][2];//xi*zi的和
        *(Matrix[0] + 1) += array[i][0];//xi
        *(Matrix[1] + 0) += array[i][1]*array[i][2];//yi*zi的和
        *(Matrix[1] + 1) += array[i][1];//yi
    }

    m = *(Matrix[0] + 0)**(IMatrix[0] + 0) + *(Matrix[0] + 1)**(IMatrix[1] + 0);
    X0= *(Matrix[0] + 0)**(IMatrix[0] + 1) + *(Matrix[0] + 1)**(IMatrix[1] + 1);
    n = *(Matrix[1] + 0)**(IMatrix[0] + 0) + *(Matrix[1] + 1)**(IMatrix[1] + 0);
    Y0= *(Matrix[1] + 0)**(IMatrix[0] + 1) + *(Matrix[1] + 1)**(IMatrix[1] + 1);
    printf("%5.2f\t%5.2f\t%5.2f\t%5.2f\t\n",X0,Y0,m,n);
    //空间直线的坐标
    printf("ln的空间坐标为:\n  x=%5.2lfz + %5.2f\n  y=%5.2lfz + %5.2f\n",m,X0,n,Y0);
    printf("其向量式为为:\n  (x - %5.2lf)/%5.2lf = (y - %5.2lf)/%5.2lf = z/1\n",X0,m,Y0,n);


    for (int i = 0;i < 2;i++)
    {
        delete[] Matrix[i];
        delete[] IMatrix[i];
    }
    //getchar();
    return 0;
#else
    //测试数据
    #define PtSum 38
    double array[MTX_AMT][3],*Matrix[2],*IMatrix[2];//允许有MTX_AMT个点
    //X0:1.00	 Y0:2.00	m:3.00	  n:2.00
    //设定待拟合的直线(x -1)/3 = (y - 2)/4 = z/1
    //x = 3z + 1
    //y = 4z + 2
    for (int i = 0; i < PtSum;i++)
    {
        array[i][2] = (double)i;
        array[i][0] = 3*array[i][2] + 1 + rand()/(RAND_MAX+1.0)/10;  //引入噪声 rand()/RAND_MAX
        array[i][1] = 4*array[i][2] + 2 + rand()/(RAND_MAX+1.0)/10;  //引入噪声
//        printf("rand()%2=%lf,array[i][0]=%lf\n", rand()%2, array[i][0]);
    }

    double X0,Y0,m,n;//这是待拟合的四个参数，已知分别对应：1、2、3、2现在我们来拟合
    for (int i = 0;i < 2;i++)
    {
        Matrix[i] =  new double[2];
        IMatrix[i] = new double[2];
    }
    Empty(Matrix,2,2);//将矩阵的前两行两列置位0


    for (int j = 0;j < PtSum;j++)//
    {
        *(Matrix[0] + 0) += array[j][2]*array[j][2];//Matrix[0][0]=array[0][2]*array[0][2]=z*z
        *(Matrix[0] + 1) += array[j][2];//Matrix[0][1]=array[0][2]=z
    }
    *(Matrix[1] + 0) = *(Matrix[0] + 1);//Matrix[1][0]=z;
    *(Matrix[1] + 1) = 1.0*PtSum;//Matrix[1][1]=n;


//下面两步是求Matrix的逆矩阵并存到IMatrix中
//A^(-1)=(1/|A|)×A* ，其中A^(-1)表示矩阵A的逆矩阵，其中|A|为矩阵A的行列式，A*为矩阵A的伴随矩阵。
    double d = Determinant(Matrix,2);
    Inverse(Matrix,IMatrix,2,d);//反转存到IMatrix中


    Empty(Matrix,2,2);
    for (int i = 0;i < PtSum;i++)
    {
        *(Matrix[0] + 0) += array[i][0]*array[i][2];//xi*zi的和
        *(Matrix[0] + 1) += array[i][0];//xi
        *(Matrix[1] + 0) += array[i][1]*array[i][2];//yi*zi的和
        *(Matrix[1] + 1) += array[i][1];//yi
    }
    m = *(Matrix[0] + 0)**(IMatrix[0] + 0) + *(Matrix[0] + 1)**(IMatrix[1] + 0);
    X0= *(Matrix[0] + 0)**(IMatrix[0] + 1) + *(Matrix[0] + 1)**(IMatrix[1] + 1);
    n = *(Matrix[1] + 0)**(IMatrix[0] + 0) + *(Matrix[1] + 1)**(IMatrix[1] + 0);
    Y0= *(Matrix[1] + 0)**(IMatrix[0] + 1) + *(Matrix[1] + 1)**(IMatrix[1] + 1);
    printf("%5.2f\t%5.2f\t%5.2f\t%5.2f\t\n",X0,Y0,m,n);
    //空间直线的坐标
    printf("ln的空间坐标为:\n  x=%5.2lfz + %5.2f\n  y=%5.2lfz + %5.2f\n",m,X0,n,Y0);
    printf("其向量式为为:\n  (x - %5.2lf)/%5.2lf = (y - %5.2lf)/%5.2lf = z/1\n",X0,m,Y0,n);
    for (int i = 0;i < 2;i++)
    {
        delete[] Matrix[i];
        delete[] IMatrix[i];
    }
    //getchar();
    return 0;
#endif
}
