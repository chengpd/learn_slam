#include <opencv2/opencv.hpp>
#include <string>
#include "chrono"
#include "mutex"

using namespace std;

// global variables
string first_file = "../1.png";
string second_file = "../2.png";

const double pi = 3.1415927;

typedef vector<bool> DescType;

void computeAngle(const cv::Mat &image, vector<cv::KeyPoint> &keypoints);

void computeORBDesc(const cv::Mat &image, vector<cv::KeyPoint> &keypoints,vector<DescType> &desc);

int main(int argc, char **argv) {

    // load image
    cv::Mat first_image = cv::imread(first_file);   // load grayscale image
    cv::Mat second_image = cv::imread(second_file); // load grayscale image

    // plot the image
    cv::imshow("first image", first_image);
    cv::imshow("second image", second_image);
    //cv::waitKey(0);

    // detect FAST keypoints using threshold=40
    vector<cv::KeyPoint> keypoints;
    cv::FAST(first_image, keypoints, 40);
    cout << "keypoints: " << keypoints.size() << endl;

    computeAngle(first_image, keypoints);

    vector<DescType> descriptors;
    computeORBDesc(first_image, keypoints, descriptors);

    cv::Mat image_show;
    cv::drawKeypoints(first_image, keypoints, image_show, cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow("features", image_show);
    cv::imwrite("feat1.png", image_show);
    cv::waitKey(0);


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

// compute the descriptor
void computeORBDesc(const cv::Mat &image, vector<cv::KeyPoint> &keypoints,vector<DescType> &desc) {
    for (auto &kp : keypoints) {
        DescType d(256, false);
        for (int i = 0; i < 256; i++) {
            // START YOUR CODE HERE (~7 lines)

            // END YOUR CODE HERE
        }
        desc.push_back(d);
    }

    int bad = 0;
    for (auto &d : desc) {
        if (d.empty())
            bad++;
    }
    cout << "bad/total: " << bad << "/" << desc.size() << endl;
    return;
}










