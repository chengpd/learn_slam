#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel_impl.h>
#include <iostream>

#include "common.h"
#include "sophus/se3.h"

using namespace Sophus;
using namespace Eigen;
using namespace std;

/// 姿态和内参的结构
struct PoseAndIntrinsics {
    PoseAndIntrinsics() {}

    /// set from given data address
    explicit PoseAndIntrinsics(double *data_addr) {
        rotation = SO3::exp(Vector3d(data_addr[0], data_addr[1], data_addr[2]));
        translation = Vector3d(data_addr[3], data_addr[4], data_addr[5]);
        focal = data_addr[6];
        k1 = data_addr[7];
        k2 = data_addr[8];
    }

    /// 将估计值放入内存
    void set_to(double *data_addr) {
        auto r = rotation.log();
        for (int i = 0; i < 3; ++i) data_addr[i] = r[i];
        for (int i = 0; i < 3; ++i) data_addr[i + 3] = translation[i];
        data_addr[6] = focal;
        data_addr[7] = k1;
        data_addr[8] = k2;
    }

    SO3 rotation;
    Vector3d translation = Vector3d::Zero();
    double focal = 0;
    double k1 = 0, k2 = 0;
};



/// 位姿加相机内参的顶点，9维，前三维为so3，接下去为t, f, k1, k2
class VertexPoseAndIntrinsics : public g2o::BaseVertex<9, PoseAndIntrinsics> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexPoseAndIntrinsics() {}

    virtual void setToOriginImpl() override {
        _estimate = PoseAndIntrinsics();
    }

	//更新估计值
    virtual void oplusImpl(const double *update) override {
        _estimate.rotation = SO3::exp(Vector3d(update[0], update[1], update[2])) * _estimate.rotation;
        _estimate.translation += Vector3d(update[3], update[4], update[5]);
        _estimate.focal += update[6];
        _estimate.k1 += update[7];
        _estimate.k2 += update[8];
    }

    /// 根据估计值投影一个点,
    Vector2d project(const Vector3d &point) {
		//把一个世界的3D点变换到当前相机点
        Vector3d pc = _estimate.rotation * point + _estimate.translation;
        pc = -pc / pc[2];  //投影到前方的距离1的相机平面
        double r2 = pc.squaredNorm();  //r2
		//去畸变 1 + k1*r2 + k2*r2*r2  
        double distortion = 1.0 + r2 * (_estimate.k1 + _estimate.k2 * r2);
		//得到投影的像素坐标
        return Vector2d(_estimate.focal * distortion * pc[0],
                        _estimate.focal * distortion * pc[1]);
    }

    virtual bool read(istream &in) {}

    virtual bool write(ostream &out) const {}
};

//路标3D点的顶点
class VertexPoint : public g2o::BaseVertex<3, Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexPoint() {}

    virtual void setToOriginImpl() override {
        _estimate = Vector3d(0, 0, 0);
    }

	//更新估计值
    virtual void oplusImpl(const double *update) override {
        _estimate += Vector3d(update[0], update[1], update[2]);
    }

    virtual bool read(istream &in) {}

    virtual bool write(ostream &out) const {}
};


//误差模型  观测维度2，类型为2d，   连接2个顶点类型
class EdgeProjection :
    public g2o::BaseBinaryEdge<2, Vector2d, VertexPoseAndIntrinsics, VertexPoint> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	//计算模型误差 ,投影-观测
    virtual void computeError() override {
        auto v0 = (VertexPoseAndIntrinsics *) _vertices[0];  //位姿
        auto v1 = (VertexPoint *) _vertices[1]; //路标
        auto proj = v0->project(v1->estimate());//观测路标投影一个像素点
        _error = proj - _measurement;   //误差
    }

    // use numeric derivatives
    virtual bool read(istream &in) {}

    virtual bool write(ostream &out) const {}

};

void SolveBA(BALProblem &bal_problem);

int main(int argc, char **argv) {

    if (argc != 2) {
        cout << "usage: bundle_adjustment_g2o bal_data.txt" << endl;
        return 1;
    }

    BALProblem bal_problem(argv[1]);  //读取BAL数据集
    bal_problem.Normalize();  //对相机参数和路标点3D数据进行处理
    bal_problem.Perturb(0.1, 0.5, 0.5); //给路标3D点添加噪声
    bal_problem.WriteToPLYFile("initial.ply"); //生成噪声ply文件
    SolveBA(bal_problem);  //BA优化
    bal_problem.WriteToPLYFile("final.ply"); //生成优化后的ply文件

    return 0;
}




void SolveBA(BALProblem &bal_problem) {
    const int point_block_size = bal_problem.point_block_size();
    const int camera_block_size = bal_problem.camera_block_size();
    double *points = bal_problem.mutable_points(); //3D点
    double *cameras = bal_problem.mutable_cameras();//相机

    // pose dimension 9, landmark is 3
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<9, 3>> BlockSolverType;
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType; //线性求解器
    // use LM
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;  //图模型
    optimizer.setAlgorithm(solver);  //设置求解器
    optimizer.setVerbose(true);   //打开调试输出

    /// build g2o problem
    const double *observations = bal_problem.observations();  //获取观测数据
    // vertex
    vector<VertexPoseAndIntrinsics *> vertex_pose_intrinsics;
    vector<VertexPoint *> vertex_points;
    for (int i = 0; i < bal_problem.num_cameras(); ++i) {  //16个相机位姿
        VertexPoseAndIntrinsics *v = new VertexPoseAndIntrinsics();   
        double *camera = cameras + camera_block_size * i;
        v->setId(i);   //顶点设置ID，
        v->setEstimate(PoseAndIntrinsics(camera));  //往图里增加顶点位姿，相机的位姿数据9维
        optimizer.addVertex(v);
        vertex_pose_intrinsics.push_back(v);
    }
    for (int i = 0; i < bal_problem.num_points(); ++i) {  //22106个路标点
        VertexPoint *v = new VertexPoint();    
        double *point = points + point_block_size * i;
        v->setId(i + bal_problem.num_cameras());  //设置ID,不能和上面重复，直接往后排
        v->setEstimate(Vector3d(point[0], point[1], point[2]));    //路标点  3维
        // g2o在BA中需要手动设置待Marg的顶点
        v->setMarginalized(true);  //路标要被边缘化计算，所以设置边缘化属性为true
        optimizer.addVertex(v);
        vertex_points.push_back(v);
    }

    // edge
    for (int i = 0; i < bal_problem.num_observations(); ++i) {   //增加边，总共83718个观测数据
        EdgeProjection *edge = new EdgeProjection;
        edge->setVertex(0, vertex_pose_intrinsics[bal_problem.camera_index()[i]]);  //设置链接的顶点，取出标号，对应数据
        edge->setVertex(1, vertex_points[bal_problem.point_index()[i]]);   //设置链接的顶点  
        edge->setMeasurement(Vector2d(observations[2 * i + 0], observations[2 * i + 1])); //观测数据
        edge->setInformation(Matrix2d::Identity());  //信息矩阵：协方差矩阵之逆
        edge->setRobustKernel(new g2o::RobustKernelHuber());
        optimizer.addEdge(edge);
    }

    optimizer.initializeOptimization();
    optimizer.optimize(40);   //迭代40次

    // set to bal problem
    for (int i = 0; i < bal_problem.num_cameras(); ++i) {
        double *camera = cameras + camera_block_size * i;
        auto vertex = vertex_pose_intrinsics[i];
        auto estimate = vertex->estimate();  //获取位姿的最优值9维  
        estimate.set_to(camera);   
    }
    for (int i = 0; i < bal_problem.num_points(); ++i) {
        double *point = points + point_block_size * i;
        auto vertex = vertex_points[i];     //获取3D路标的最优值3维
        for (int k = 0; k < 3; ++k) point[k] = vertex->estimate()[k];  //路标覆盖保存
    }
}


