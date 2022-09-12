/*求解 A * x = B 这个方程*/

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>

using namespace std;
using namespace Eigen;

#define MATRIX_SIZE 100

int main( int argc,char** argv )
{
    MatrixXd A_pre = MatrixXd::Random( MATRIX_SIZE, MATRIX_SIZE );
    MatrixXd A = A_pre.transpose()*A_pre ;                  //使得A为正定对称矩阵，才能使得cholesky分解成功
    VectorXd B = VectorXd::Random( MATRIX_SIZE );
    VectorXd x = A.colPivHouseholderQr().solve(B);          //调用QR分解求解
    VectorXd y = A.llt().solve(B);                          //调用cholesky分解求解

    cout <<"A*x=B方程的解为 \n"<< x.transpose() << endl;
    cout <<"A*y=B方程的解为 \n"<< y.transpose() << endl;
}
