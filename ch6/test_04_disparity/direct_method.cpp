#include <opencv2/opencv.hpp>
#include <sophus/se3.h>
#include <Eigen/Core>
#include <vector>
#include <string>
#include <boost/format.hpp>
#include <pangolin/pangolin.h>

using namespace std;

typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;

// Camera intrinsics
// 内参
double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
// 基线
double baseline = 0.573;
// paths
string left_file = "../left.png";
string disparity_file = "../disparity.png";
string right_flie = ("../right.png");    // other files

vector<double> disp;
vector<double> error;

// useful typedefs
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 2, 6> Matrix26d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;


void cal_disparity(cv::Mat left_img,cv::Mat right_img);

// TODO implement this function
/**
 * pose estimation using direct method
 * @param img1
 * @param img2
 * @param px_ref
 * @param depth_ref
 * @param T21
 */
void DirectPoseEstimationSingleLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3 &T21
);

// bilinear interpolation
inline float GetPixelValue(const cv::Mat &img, float x, float y) {
    uchar *data = &img.data[int(y) * img.step + int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
            (1 - xx) * (1 - yy) * data[0] +
            xx * (1 - yy) * data[1] +
            (1 - xx) * yy * data[img.step] +
            xx * yy * data[img.step + 1]
    );
}

int main(int argc, char **argv) {

    cv::Mat left_img = cv::imread(left_file, 0);
    cv::Mat right_img = cv::imread(right_flie,0);
    cv::Mat disparity_img = cv::imread(disparity_file, 0);

    // let's randomly pick pixels in the first image and generate some 3d points in the first image's frame
    cv::RNG rng;
    int nPoints = 1000;
    int boarder = 40;
    VecVector2d pixels_ref;
    vector<double> depth_ref;



    // generate pixels in ref and load depth data
    for (int i = 0; i < nPoints; i++) {
        int x = rng.uniform(boarder, left_img.cols - boarder);  // don't pick pixels close to boarder
        int y = rng.uniform(boarder, left_img.rows - boarder);  // don't pick pixels close to boarder
        cal_disparity(left_img,right_img);
        float disparity = disparity_img.at<uchar>(y, x);
        double depth = fx * baseline / disparity; // you know this is disparity to depth
        depth_ref.push_back(depth);
        pixels_ref.push_back(Eigen::Vector2d(x, y));
    }

    // estimates 01~05.png's pose using this information
    Sophus::SE3 T_cur_ref;

        cv::Mat img = cv::imread(right_flie, 0);
        DirectPoseEstimationSingleLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref);    // first you need to test single layer
        // DirectPoseEstimationMultiLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref);

}

void cal_disparity(cv::Mat left_img,cv::Mat right_img){

    for (int x = -half_patch_size; x < half_patch_size; x++)
        for (int y = -half_patch_size; y < half_patch_size; y++) {





        }




}


void DirectPoseEstimationSingleLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3 &T21
) {

    // parameters
    int half_patch_size = 4;
    int iterations = 100;

    double cost = 0, lastCost = 0;
    int nGood = 0;  // good projections
    VecVector2d goodProjection;
    ///用于画图对应
    VecVector2d goodRef;

    for (int iter = 0; iter < iterations; iter++) {
        nGood = 0;
        goodProjection.clear();
        goodRef.clear();

        // Define Hessian and bias
        Matrix6d H = Matrix6d::Zero();  // 6x6 Hessian
        Vector6d b = Vector6d::Zero();  // 6x1 bias

        for (size_t i = 0; i < px_ref.size(); i++) {

            // compute the projection in the second image
            // TODO START YOUR CODE HERE
            double tempXw = (px_ref[i][0] - cx)/fx*depth_ref[i];
            double tempYw = (px_ref[i][1] - cy)/fy*depth_ref[i];
            Eigen::Matrix<double, 3, 1> pw(tempXw,tempYw,depth_ref[i]);
            Eigen::Matrix<double, 3, 1> pw2 = T21*pw;
            if (pw2[2] < 0)   /// depth invalid
                continue;

            double u = fx*pw2[0]/pw2[2]+cx;
            double v = fy*pw2[1]/pw2[2]+cy;
            if (u < half_patch_size || u > img2.cols - half_patch_size || v < half_patch_size || v > img2.rows - half_patch_size)
                continue;

            nGood++;
            goodProjection.push_back(Eigen::Vector2d(u, v));
            goodRef.push_back(Eigen::Vector2d(px_ref[i][0], px_ref[i][1]));

            double X = pw2[0], Y = pw2[1], Z = pw2[2], Z_inv = 1.0 / Z, Z2_inv = Z_inv * Z_inv;

            // and compute error and jacobian
            for (int x = -half_patch_size; x < half_patch_size; x++)
                for (int y = -half_patch_size; y < half_patch_size; y++) {

                    double error = GetPixelValue(img1, px_ref[i][0] + x, px_ref[i][1] + y) -
                                   GetPixelValue(img2, u + x, v + y);

                    Matrix26d J_pixel_xi;   // pixel to \xi in Lie algebra
                    Eigen::Vector2d J_img_pixel;    // image gradients

                    J_pixel_xi(0, 0) = fx * Z_inv;
                    J_pixel_xi(0, 1) = 0;
                    J_pixel_xi(0, 2) = -fx * X * Z2_inv;
                    J_pixel_xi(0, 3) = -fx * X * Y * Z2_inv;
                    J_pixel_xi(0, 4) = fx + fx * X * X * Z2_inv;
                    J_pixel_xi(0, 5) = -fx * Y * Z_inv;

                    J_pixel_xi(1, 0) = 0;
                    J_pixel_xi(1, 1) = fy * Z_inv;
                    J_pixel_xi(1, 2) = -fy * Y * Z2_inv;
                    J_pixel_xi(1, 3) = -fy - fy * Y * Y * Z2_inv;
                    J_pixel_xi(1, 4) = fy * X * Y * Z2_inv;
                    J_pixel_xi(1, 5) = fy * X * Z_inv;

                    J_img_pixel = Eigen::Vector2d(
                            0.5 * (GetPixelValue(img2, u + 1 + x, v + y) - GetPixelValue(img2, u - 1 + x, v + y)),
                            0.5 * (GetPixelValue(img2, u + x, v + 1 + y) - GetPixelValue(img2, u + x, v - 1 + y))
                    );

                    // total jacobian
                    Vector6d J = -1.0 * (J_img_pixel.transpose() * J_pixel_xi).transpose();

                    H += J * J.transpose();
                    b += -error * J;
                    cost += error * error;
                }
            // END YOUR CODE HERE
        }

        // solve update and put it into estimation
        // TODO START YOUR CODE HERE
        Vector6d update = H.ldlt().solve(b);;
        T21 = Sophus::SE3::exp(update) * T21;
        // END YOUR CODE HERE

        cost /= nGood;

        if (isnan(update[0])) {
            // sometimes occurred when we have a black or white patch and H is irreversible
            cout << "update is nan" << endl;
            break;
        }
        if (iter > 0 && cost > lastCost) {
            cout << "cost increased: " << cost << ", " << lastCost << endl;
            break;
        }
        lastCost = cost;
        cout << "cost = " << cost << ", good = " << nGood << endl;
    }
    cout << "good projection: " << nGood << endl;
    cout << "T21 = \n" << T21.matrix() << endl;

    // in order to help you debug, we plot the projected pixels here
    cv::Mat img1_show, img2_show;
    cv::cvtColor(img1, img1_show, CV_GRAY2BGR);
    cv::cvtColor(img2, img2_show, CV_GRAY2BGR);
    for (auto &px: px_ref) {
        cv::rectangle(img1_show, cv::Point2f(px[0] - 2, px[1] - 2), cv::Point2f(px[0] + 2, px[1] + 2),
                      cv::Scalar(0, 250, 0));
    }
//    for (auto &px: goodProjection) {
//        cv::rectangle(img2_show, cv::Point2f(px[0] - 2, px[1] - 2), cv::Point2f(px[0] + 2, px[1] + 2),
//                      cv::Scalar(0, 250, 0));
//    }
    for (size_t i = 0; i < goodProjection.size(); ++i) {
        auto p_ref = goodRef[i];
        auto p_cur = goodProjection[i];

        if (p_cur[0] > 0 && p_cur[1] > 0) {
            cv::circle(img2_show, cv::Point2f(p_cur[0], p_cur[1]), 1, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_show, cv::Point2f(p_ref[0], p_ref[1]), cv::Point2f(p_cur[0], p_cur[1]),
                     cv::Scalar(0, 255, 0));
        }
    }

    cv::imwrite("reference.bmp", img1_show);
    cv::imwrite("current.bmp", img2_show);

    cv::imshow("reference", img1_show);
    cv::imshow("current", img2_show);
    cv::waitKey();
}
