//
// Created by 高翔 on 2017/12/19.
// 本程序演示ORB是如何提取、计算和匹配的
//
#include <opencv2/opencv.hpp>
#include "iostream"
#include <string>
#include "vector"


using namespace std;

// global variables
string first_file  = "../1.png";
string second_file = "../2.png";

const double pi = 3.1415927;

void computeAngle(const cv::Mat &image, vector<cv::KeyPoint> &keypoints);



int main(int argc, char **argv) {

    /// load image
    cv::Mat first_image  = cv::imread(first_file);   // load grayscale image
    cv::Mat second_image = cv::imread(second_file);  // load grayscale image

    /// plot the image
    cv::imshow("first image", first_image);
    cv::imshow("second image", second_image);
    //cv::waitKey(0);

    /// detect FAST keypoints using threshold=40
    vector<cv::KeyPoint> keypoints;
    cv::FAST(first_image, keypoints, 40);
    cout << "keypoints: " << keypoints.size() << endl;

    /// compute angle for each keypoint
    computeAngle(first_image, keypoints);

    /// test multi thread version
//    computeAngleMT(first_image, keypoints);
//
//    // compute ORB descriptors
//    vector<DescType> descriptors;
//    computeORBDesc(first_image, keypoints, descriptors);
//
//
//    vector<DescType> descriptors_mt;
//    computeORBDescMT(first_image, keypoints, descriptors_mt);
//
//    // plot the keypoints
//    cv::Mat image_show;
//    cv::drawKeypoints(first_image, keypoints, image_show, cv::Scalar::all(-1),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
//    cv::imshow("features", image_show);
//    cv::imwrite("feat1.png", image_show);
//    cv::waitKey(0);
//
//    // we can also match descriptors between images
//    // same for the second
//    vector<cv::KeyPoint> keypoints2;
//    cv::FAST(second_image, keypoints2, 40);
//    cout << "keypoints2: " << keypoints2.size() << endl;
//
//    // compute angle for each keypoint
//    computeAngle(second_image, keypoints2);
//
//    // compute ORB descriptors
//    vector<DescType> descriptors2;
//    computeORBDesc(second_image, keypoints2, descriptors2);
//
//    // find matches
//    vector<cv::DMatch> matches;
//    evaluate_and_call([&]() { bfMatch(descriptors, descriptors2, matches); },
//                      "bf match", 1);
//    cout << "matches: " << matches.size() << endl;
//
//    // plot the matches
//    cv::drawMatches(first_image, keypoints, second_image, keypoints2, matches,
//                    image_show);
//    cv::imshow("matches", image_show);
//    cv::imwrite("matches.png", image_show);
//    cv::waitKey(0);
//
//    cout << "done." << endl;
    return 0;
}

void computeAngle(const cv::Mat &image, vector<cv::KeyPoint> &keypoints) {
    int half_patch_size = 8;
    for (auto &kp : keypoints) {
        // START YOUR CODE HERE (~7 lines)
        int u = kp.pt.x, v = kp.pt.y;

        //判断是否出界，并选择是否跳出
        if(u>=half_patch_size&&v>=half_patch_size&&u<=image.cols-half_patch_size &&v <=image.rows-half_patch_size)
        {
            int m01=0, m10=0;
            for (int i = u-half_patch_size+1; i < u+half_patch_size-1; ++i) {
                for (int j = v-half_patch_size+1; j < v+half_patch_size-1; ++j) {
                    m10 += i * image.at<uchar>(j,i);
                    m01 += j * image.at<uchar>(j,i);
                }
            }
            //atan会返回弧度制的旋转角,但 OpenCV 中使用角度制,需要进行弧度转换
            kp.angle = (float)atan(m01/m10)*180/pi;
        }
        // END YOUR CODE HERE
    }
    return;
}