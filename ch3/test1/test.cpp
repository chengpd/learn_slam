#include <string>
#include <iostream>
#include <fstream>
#include <sophus/se3.h>

using namespace std;

string trajectory_file = "./trajectory.txt";

int main(int argc, char **argv) {

    //vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses;

    ifstream fin(trajectory_file);
    while (!fin.eof()) {
        double time, tx, ty, tz, qx, qy, qz, qw;
        fin >> time;
        fin >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        cout<<time <<"    "<<tx   <<ty   <<tz   <<qw   <<qx   <<qy   <<qz   << endl;
    }
    fin.close();

    return 0;


}
