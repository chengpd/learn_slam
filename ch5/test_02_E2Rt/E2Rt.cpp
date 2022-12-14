//
// Created by 高翔 on 2017/12/19.
// 本程序演示如何从Essential矩阵计算R,t
//

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

#include <sophus/so3.h>

#include <iostream>

using namespace std;

int main(int argc, char **argv) {

    // 给定Essential矩阵
    Matrix3d E;
    E << -0.0203618550523477, -0.4007110038118445, -0.03324074249824097,
            0.3939270778216369, -0.03506401846698079, 0.5857110303721015,
            -0.006788487241438284, -0.5815434272915686, -0.01438258684486258;

    // 待计算的R,t
    Matrix3d R;
    Vector3d t;

    // SVD and fix sigular values
    // START YOUR CODE HERE
    Eigen::JacobiSVD<Matrix3d> svd(E,ComputeFullU|ComputeFullV); //svd分解
    Matrix3d U=svd.matrixU();
    Matrix3d V=svd.matrixV();
    Vector3d sigma_value = svd.singularValues();


    //处理后的sigma值
    Vector3d sigma_value2={(sigma_value[0]+sigma_value[1])/2,(sigma_value[0]+sigma_value[1])/2,0};
    Matrix3d SIGMA=sigma_value2.asDiagonal();
    // END YOUR CODE HERE

    // set t1, t2, R1, R2 
    // START YOUR CODE HERE
    Matrix3d t_wedge1;
    Matrix3d t_wedge2;

    Matrix3d R1;
    Matrix3d R2;

    Matrix3d RZ1=AngleAxisd(M_PI/2,Vector3d(0,0,1)).toRotationMatrix();
    Matrix3d RZ2=AngleAxisd(-M_PI/2,Vector3d(0,0,1)).toRotationMatrix();
    t_wedge1=U*RZ1*SIGMA*U.transpose();
    t_wedge2=U*RZ2*SIGMA*U.transpose();
    R1=U*RZ1.transpose()*V.transpose();
    R2=U*RZ2.transpose()*V.transpose();
    // END YOUR CODE HERE

    cout << "R1 = \n" << R1 << endl;
    cout<<endl;
    cout << "R2 = \n" << R2 << endl;
    cout<<endl;

    cout << "t1^T = " << Sophus::SO3::vee(t_wedge1).transpose() << endl;
    cout << "t2^T = " << Sophus::SO3::vee(t_wedge2).transpose() << endl;

    // check t^R=E up to scale
    Matrix3d tR = t_wedge1 * R1;
    cout<<endl;
    cout << "t^R = "<<endl;
    cout << tR << endl;

    return 0;
}