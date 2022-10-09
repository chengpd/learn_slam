#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "common.h"
#include "rotation.h"
#include "random.h"

typedef Eigen::Map<Eigen::VectorXd> VectorRef;
typedef Eigen::Map<const Eigen::VectorXd> ConstVectorRef;

//这个函数从fptr文件中读出一个format类型的值，赋值给参数value，从开头开始，找到一个合适的就停止。
//这个函数主要是给BALProblem()构造函数读取txt数据文件用的，比较简陋
template<typename T>
void FscanfOrDie(FILE *fptr, const char *format, T *value) {
    int num_scanned = fscanf(fptr, format, value);
    if (num_scanned != 1)
        std::cerr << "Invalid UW data file. ";
}

//给一个三维向量加入噪声，很简单xyz依次加入随机值就好了。定义这个的目的是为了后面的Perturb()函数在增加噪声时，
// 是分开对路标点，相机的旋转，相机的平移分别加入噪声的，并且这三个量都是三维的，所以定义一个三维向量添加噪声的函数
void PerturbPoint3(const double sigma, double *point) {
    for (int i = 0; i < 3; ++i)
        point[i] += RandNormal() * sigma;
}

double Median(std::vector<double> *data) {
    int n = data->size();
    std::vector<double>::iterator mid_point = data->begin() + n / 2;
    std::nth_element(data->begin(), mid_point, data->end());
    return *mid_point;
}

BALProblem::BALProblem(const std::string &filename, bool use_quaternions) {
    FILE *fptr = fopen(filename.c_str(), "r");

    if (fptr == NULL) {
        std::cerr << "Error: unable to open file " << filename;
        return;
    };

    // This wil die horribly on invalid files. Them's the breaks.
    FscanfOrDie(fptr, "%d", &num_cameras_);  //读取总的相机数
    FscanfOrDie(fptr, "%d", &num_points_);   //读取总的路标数
    FscanfOrDie(fptr, "%d", &num_observations_);//读取总的观测数据个数

    std::cout << "Header: " << num_cameras_
              << " " << num_points_
              << " " << num_observations_;

    point_index_ = new int[num_observations_];  //取出3D路标点的标号
    camera_index_ = new int[num_observations_]; //相机的标号
    observations_ = new double[2 * num_observations_]; //观测的像素点

    num_parameters_ = 9 * num_cameras_ + 3 * num_points_;//每个相机9个参数，每个路标3个参数
    parameters_ = new double[num_parameters_];  //参数的总大小

    for (int i = 0; i < num_observations_; ++i) {  //拷贝数据
        FscanfOrDie(fptr, "%d", camera_index_ + i);  //第几个相机
        FscanfOrDie(fptr, "%d", point_index_ + i);   //第几个路标
        for (int j = 0; j < 2; ++j) {
            FscanfOrDie(fptr, "%lf", observations_ + 2 * i + j);//观测到的像素坐标
        }
    }

    //每个相机是一组9个参数，-R:3维(罗德里格斯向量)  t:3维  f,k1,k2。后面是3D路标的数据3维
    for (int i = 0; i < num_parameters_; ++i) {
        FscanfOrDie(fptr, "%lf", parameters_ + i);
    }

    fclose(fptr);

    use_quaternions_ = use_quaternions;
    if (use_quaternions) {
        // Switch the angle-axis rotations to quaternions.
        num_parameters_ = 10 * num_cameras_ + 3 * num_points_;
        double *quaternion_parameters = new double[num_parameters_];//指针指向一个新的四元数数组
        double *original_cursor = parameters_;   //指针指向原始数据参数数组
        double *quaternion_cursor = quaternion_parameters;//指针指向指向四元数数组的指针
        for (int i = 0; i < num_cameras_; ++i) {
            AngleAxisToQuaternion(original_cursor, quaternion_cursor); //R转换为四元数
            quaternion_cursor += 4;
            original_cursor += 3;
            for (int j = 4; j < 10; ++j) {
                *quaternion_cursor++ = *original_cursor++;
            }
        }
        // Copy the rest of the points.
        for (int i = 0; i < 3 * num_points_; ++i) {
            *quaternion_cursor++ = *original_cursor++;
        }
        // Swap in the quaternion parameters.
        delete[]parameters_;
        parameters_ = quaternion_parameters;
    }
}

//构造函数读入数据txt，将数据存入类成员中。猜测这里是反向过程，由类成员中存储的数据，生成一个待优化数据.txt。
void BALProblem::WriteToFile(const std::string &filename) const {
    FILE *fptr = fopen(filename.c_str(), "w");

    if (fptr == NULL) {
        std::cerr << "Error: unable to open file " << filename;
        return;
    }

    fprintf(fptr, "%d %d %d %d\n", num_cameras_, num_cameras_, num_points_, num_observations_);

    for (int i = 0; i < num_observations_; ++i) {
        fprintf(fptr, "%d %d", camera_index_[i], point_index_[i]);
        for (int j = 0; j < 2; ++j) {
            fprintf(fptr, " %g", observations_[2 * i + j]);
        }
        fprintf(fptr, "\n");
    }

    for (int i = 0; i < num_cameras(); ++i) {
        double angleaxis[9];
        if (use_quaternions_) {
            //OutPut in angle-axis format.
            QuaternionToAngleAxis(parameters_ + 10 * i, angleaxis);
            memcpy(angleaxis + 3, parameters_ + 10 * i + 4, 6 * sizeof(double));
        } else {
            memcpy(angleaxis, parameters_ + 9 * i, 9 * sizeof(double));
        }
        for (int j = 0; j < 9; ++j) {
            fprintf(fptr, "%.16g\n", angleaxis[j]);
        }
    }

    const double *points = parameters_ + camera_block_size() * num_cameras_;
    for (int i = 0; i < num_points(); ++i) {
        const double *point = points + i * point_block_size();
        for (int j = 0; j < point_block_size(); ++j) {
            fprintf(fptr, "%.16g\n", point[j]);
        }
    }

    fclose(fptr);
}

//将相机的世界坐标位移和3D路标点写入文件
// Write the problem to a PLY file for inspection in Meshlab or CloudCompare
void BALProblem::WriteToPLYFile(const std::string &filename) const {
    std::ofstream of(filename.c_str());

    of << "ply"
       << '\n' << "format ascii 1.0"
       << '\n' << "element vertex " << num_cameras_ + num_points_
       << '\n' << "property float x"
       << '\n' << "property float y"
       << '\n' << "property float z"
       << '\n' << "property uchar red"
       << '\n' << "property uchar green"
       << '\n' << "property uchar blue"
       << '\n' << "end_header" << std::endl;

    // Export extrinsic data (i.e. camera centers) as green points.
    double angle_axis[3];
    double center[3];
    for (int i = 0; i < num_cameras(); ++i) {
        const double *camera = cameras() + camera_block_size() * i;
        CameraToAngelAxisAndCenter(camera, angle_axis, center);
        of << center[0] << ' ' << center[1] << ' ' << center[2]
           << "0 255 0" << '\n';
    }

    // Export the structure (i.e. 3D Points) as white points.
    const double *points = parameters_ + camera_block_size() * num_cameras_;
    for (int i = 0; i < num_points(); ++i) {
        const double *point = points + i * point_block_size();
        for (int j = 0; j < point_block_size(); ++j) {
            of << point[j] << ' ';
        }
        of << "255 255 255\n";
    }
    of.close();
}

/**
 *
 * 由camera数据中的旋转向量和平移向量解析出相机世界坐标系下的姿态(依旧是旋转向量)和位置(世界坐标系下的xyz)，也是用于生成点云用的
 * @param camera 要解析的相机参数，前三维旋转，接着三维平移，这里指用到这6维
 * @param angle_axis 解析出的相机姿态承接数组，也是旋转向量形式
 * @param center 解析出来的相机原点在世界坐标系下的坐标承接数组，XYZ
 */
void BALProblem::CameraToAngelAxisAndCenter(const double *camera,
                                            double *angle_axis,
                                            double *center) const {
    VectorRef angle_axis_ref(angle_axis, 3);
    if (use_quaternions_) {
        QuaternionToAngleAxis(camera, angle_axis);
    } else {
        angle_axis_ref = ConstVectorRef(camera, 3); //读取R
    }

    Eigen::VectorXd inverse_rotation = -angle_axis_ref;  //-R，BAL文件定义，取负号


    /**
     * 这里解释一下center的计算逻辑：
     * center是指相机原点在世界坐标系下的坐标，那么定义一下：
     * PW_center, 世界坐标系下相机原点的坐标
     * PC_center, 相机坐标系下的相机原点坐标
     * 它俩的关系是什么呢？
     * PW_center*R+t = PC_center
	 * 反向过程就是
	 * PC_center * T^(-1) = PW_center
	 * 那么相机坐标系的原点，在世界坐标系下的坐标就可以求出来了
	 * [R^(T)  -R^(T)*t ] * [相机原点也就是000]
	 * [0      1        ]   [ 1 ]
     * 结果就是   -R^(T) * t
     *由旋转向量形式表示的旋转，反向过程(也就是求逆)就是旋转向量取负即可。
	 * 所以结果就是cos(theta) * t + ( 1 - cos(theta) ) (n 点乘 t) n  + sin(theta) ( n 叉乘 t )
	 */

    AngleAxisRotatePoint(inverse_rotation.data(),  //R
                         camera + camera_block_size() - 6, //平移t的数据
                         center);   //结果

    //最后加上负号。记住，map类型构造的是引用，能直接对原构造数组进行操作的。
    //说一下这句，这句还是，用center数组的前3维，去构造一个无名的map类型矩阵，并且后面直接跟上*-1操作。
    //VectorRef是Map的一个define。
    //记住Map构造出来是引用，能对原始值直接操作。
    VectorRef(center, 3) *= -1.0;
}

/**
 * 由世界坐标系下的相机姿态和原点位置，生成一个camera数据
 * @param angle_axis 世界坐标到相机坐标变化的旋转向量数据
 * @param center 相机中心在世界坐标系下的位置坐标
 * @param camera 承接数据的camera数组，由于这里只是生成旋转和平移，所以是camera的前6维
 */
void BALProblem::AngleAxisAndCenterToCamera(const double *angle_axis,
                                            const double *center,
                                            double *camera) const {
    ConstVectorRef angle_axis_ref(angle_axis, 3);
    if (use_quaternions_) {
        AngleAxisToQuaternion(angle_axis, camera);
    } else {
        VectorRef(camera, 3) = angle_axis_ref;
    }

    //这里相机姿态R没有取反，原始数据是-R，代表是相机坐标系对世界坐标系的变换

    /* 和上面类似
     * 结果就是   -R^(T) * t
     *
     * 所以结果就是cos(theta) * t + ( 1 - cos(theta) ) (n 点乘 t) n  + sin(theta) ( n 叉乘 t )
     */

    //该函数直接修改了储存相机平移数据的数据
    AngleAxisRotatePoint(angle_axis, center, camera + camera_block_size() - 6);

    //最后再取个反
    VectorRef(camera + camera_block_size() - 6, 3) *= -1.0;
}


void BALProblem::Normalize() {
    // Compute the marginal median of the geometry
    std::vector<double> tmp(num_points_);
    Eigen::Vector3d median;
    double *points = mutable_points();//获取路标3D点的位置
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < num_points_; ++j) {
            tmp[j] = points[3 * j + i];
        }
        median(i) = Median(&tmp);  //返回中位数，如果是偶数，取平均值
    }

    for (int i = 0; i < num_points_; ++i) {
        VectorRef point(points + 3 * i, 3);
        tmp[i] = (point - median).lpNorm<1>(); //每个点 - 中位数 的LP范数
    }

    const double median_absolute_deviation = Median(&tmp); //再取中位数

    // Scale so that the median absolute deviation of the resulting
    // reconstruction is 100

    const double scale = 100.0 / median_absolute_deviation;

    // X = scale * (X - median)
    for (int i = 0; i < num_points_; ++i) {
        VectorRef point(points + 3 * i, 3);  //
        point = scale * (point - median);   //对每个3D点进行处理，MAP是引用，会改变原数据
    }

    double *cameras = mutable_cameras(); //相机参数
    double angle_axis[3];
    double center[3];
    for (int i = 0; i < num_cameras_; ++i) {
        double *camera = cameras + camera_block_size() * i;
        //angle_axis赋值了R，center为结果
        CameraToAngelAxisAndCenter(camera, angle_axis, center);  //求解世界坐标系下的相机中心坐标
        // center = scale * (center - median)
        VectorRef(center, 3) = scale * (VectorRef(center, 3) - median);  //因为世界路标3D点做了处理，所以这个也要处理

        //最终，修改了*camera指向储存的数据的平移数据
        AngleAxisAndCenterToCamera(angle_axis, center, camera);  //因为世界坐标进行处理了，所以将处理后的数据转到相机坐标去
    }
}

//添加噪声
void BALProblem::Perturb(const double rotation_sigma,
                         const double translation_sigma,
                         const double point_sigma) {
    assert(point_sigma >= 0.0);
    assert(rotation_sigma >= 0.0);
    assert(translation_sigma >= 0.0);

    double *points = mutable_points();
    if (point_sigma > 0) {
        for (int i = 0; i < num_points_; ++i) {
            PerturbPoint3(point_sigma, points + 3 * i);
        }
    }

    //这里相机是被分成两块，旋转和平移，
    //旋转是考虑到四元数形式，增加了一步用CameraToAngelAxisAndCenter()从camera中取出三维的angle_axis,
    //然后添加噪声，添加完后再用AngleAxisAndCenterToCamera()重构camera参数
    //平移部分就直接用PerturbPoint3()添加了

    for (int i = 0; i < num_cameras_; ++i) {
        double *camera = mutable_cameras() + camera_block_size() * i;

        double angle_axis[3];
        double center[3];
        // Perturb in the rotation of the camera in the angle-axis
        // representation
        CameraToAngelAxisAndCenter(camera, angle_axis, center);
        if (rotation_sigma > 0.0) {
            PerturbPoint3(rotation_sigma, angle_axis);
        }
        AngleAxisAndCenterToCamera(angle_axis, center, camera);

        if (translation_sigma > 0.0)
            PerturbPoint3(translation_sigma, camera + camera_block_size() - 6);
    }
}