#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>


using namespace std;
using namespace Eigen;

#define MATRIX_SIZE 100

int main( int argc,char** argv )
{
    Eigen::Quaterniond qwr = {0.1,0.35,0.2,0.3};
    Eigen::Quaterniond qrb = {0.1,0.35,0.2,0.3};
    Eigen::Quaterniond qbl = {0.1,0.35,0.2,0.3};
    Eigen::Quaterniond qbc = {0.1,0.35,0.2,0.3};

    Eigen::Vector3d twr = {0.1,0.2,0.3};
    Eigen::Vector3d trb = {0.05,0,0.5};
    Eigen::Vector3d tbl = {0.4,0,0.5};
    Eigen::Vector3d tbc = {0.5,0.1,0.5};

    qwr.normalize();
    Eigen::Matrix3d Rwr = qwr.toRotationMatrix();

    qrb.normalize();
    Eigen::Matrix3d Rrb = qwr.toRotationMatrix();

    qbl.normalize();
    Eigen::Matrix3d Rbl = qwr.toRotationMatrix();

    qbc.normalize();
    Eigen::Matrix3d Rbc = qwr.toRotationMatrix();

    Eigen::Vector3d pc = {0.3,0.2,1.2};

    Eigen::Vector3d pl = Rbl.inverse() * ((Rbc*pl+tbc)-tbl);
    cout << pl.transpose() <<endl;


    Eigen::Vector3d pw = Rwr*(Rrb * (Rbc*pl+tbc)+trb)+twr;
    cout << pw.transpose() <<endl;

}

