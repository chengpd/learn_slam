#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>
#include "unistd.h"

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;

// path to trajectory file
string estimated = "../estimated.txt";
string groundtruth = "../groundtruth.txt";
// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>,
                    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> vector);

int main(int argc, char **argv) {

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> tra_estimated;

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> tra_groundtruth;
    /// implement pose reading code
    // start your code here (5~10 lines)
    ifstream fin(estimated);
    while (!fin.eof()) {
        double time, tx, ty, tz, qx, qy, qz, qw;
        fin >> time;
        fin >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Sophus::SE3 p1(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
        tra_estimated.push_back(p1);
    }
    fin.close();

    ifstream gro(groundtruth);
    while (!gro.eof()) {
        double gro_time, gro_tx, gro_ty, gro_tz, gro_qx, gro_qy, gro_qz, gro_qw;
        gro >> gro_time;
        gro >> gro_tx >> gro_ty >> gro_tz >> gro_qx >> gro_qy >> gro_qz >> gro_qw;
        Sophus::SE3 p1(Eigen::Quaterniond(gro_qw, gro_qx, gro_qy, gro_qz), Eigen::Vector3d(gro_tx, gro_ty, gro_tz));
        tra_groundtruth.push_back(p1);
    }
    gro.close();
    /// end your code here



    double rmse = 0;
    for (size_t i = 0; i < tra_estimated.size(); i++) {
        Sophus::SE3 p1 = tra_estimated[i],p2 = tra_groundtruth[i];
        double error = (p2.inverse()*p1).log().norm();
        rmse += error * error;
    }
        rmse = rmse/double (tra_estimated.size());
        rmse = sqrt(rmse);
        cout<<"RMSE = "<<rmse<<endl;

    DrawTrajectory(tra_estimated, tra_groundtruth);
    return 0;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> est,vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> gro) {
    if (gro.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < est.size() - 1; i++) {
            glColor3f(1 - (float) i / est.size(), 0.0f, (float) i / est.size());
            glBegin(GL_LINES);
            auto p1 = est[i], p2 = est[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }


        for (size_t i = 0; i < est.size() - 1; i++) {
            glColor3f(1 - (float) i / gro.size(), 0.0f, (float) i / gro.size());
            glBegin(GL_LINES);
            auto p1 = gro[i], p2 = gro[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }





        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}