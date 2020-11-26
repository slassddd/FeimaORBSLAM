/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/



#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<cmath>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;
using namespace cv;
void dcm2angle(double dcm[9], int type, double* roll, double* pitch, double* yaw);
void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

static double duPerFrame = 0.0;
static int frameRate = 0;

int main(int argc, char **argv)
{  
    cout << "exe               " << string(argv[0]) << endl;
    cout << "vocabulary        " << string(argv[1]) << endl;
    cout << "setting file      " << string(argv[2]) << endl;
    cout << "avi               " << string(argv[3]) << endl;
    int min0, sec0;
    min0 = 0, sec0 = 0;    
    if( argc==6 ){
        min0 = atoi(string(argv[4]).c_str());
        sec0 = atoi(string(argv[5]).c_str());
    }
    cout << "start min         " << min0 << endl;
    cout << "start sec         " << sec0 << endl;    
    if(argc < 4)
    {
        cerr << endl << "Usage: ./mono_euroc path_to_vocabulary path_to_settings path_to_avi" << endl;
        return 1;
    }

    cout << endl << "-------" << endl;
    cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, true);

    cv::Mat Tcw;
    cv::VideoCapture cap; // read avi file or camera
    cv::Mat frame; // 
    // 记录系统时间
    auto start = chrono::system_clock::now();
    auto beforeTrackStep = chrono::system_clock::now();
    auto afterTrackStep = chrono::system_clock::now();
    auto timePerFrame = chrono::duration_cast<chrono::milliseconds>(afterTrackStep-beforeTrackStep);
 
    // cap.open("/home/sl/Desktop/software/FeimaDataSet/data_tag/v_determine_axis.avi"); 
    cap.open(string(argv[3]));
    if (!cap.isOpened()){
        cout << "avi file or camera is not opened successfully" << endl;
        return -1;
    }

    int framestamps = 0;
    int framerate = cap.get(CAP_PROP_FPS);
    int time_sec = 0;
    int clock_minute = 0;
    int clock_sec = 0;
    cout << "frame rate: " << framerate << " (FPS)" << endl;
    int time_sec_start = min0*60 + sec0;
    while (true) {
        framestamps ++;
        time_sec = framestamps/framerate;
        clock_minute = time_sec/60;
        clock_sec = time_sec%60;
        cap >> frame;
        if (frame.empty()){
            cout << "第 " << framestamps << " 帧 empty. " << endl;
            continue;
        }
        if (time_sec < time_sec_start){
            if (framestamps%(10*framerate) == 0)
            {
                cout.precision(5);
                cout << "skip image: " << clock_minute << " min" << clock_sec << " sec\t\t";
                cout << "( work after  " << min0 << " min" << sec0 << " sec )" << endl;
                cout.precision(17);
            }
            continue;
        }

        auto now = chrono::system_clock::now();
        double timestamp = std::chrono::duration_cast<std::chrono::duration<double> >(now - start).count();
        cout << "timestamp: " << timestamp/1000.0 << endl;
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        Tcw = SLAM.TrackMonocular(frame, timestamp/1000);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        if(duPerFrame>1)
        {
            cout << "frame rate: " << frameRate << "\t\t\t(" << duPerFrame << "sec )" << endl;
            duPerFrame = 0.0;
            frameRate = 0;
        }
        else
        {
            frameRate++;
            duPerFrame += ttrack;
        }
        
        if (Tcw.data == NULL){
        }
        else{            
            float* data = (float*)Tcw.data;

            double Rcw[9],tcw[3];
            Rcw[0] = data[0];
            Rcw[1] = data[1];
            Rcw[2] = data[2];
            
            Rcw[3] = data[4];
            Rcw[4] = data[5];
            Rcw[5] = data[6];
            
            Rcw[6] = data[8];
            Rcw[7] = data[9];
            Rcw[8] = data[10];

            tcw[0] = data[3];
            tcw[1] = data[7];
            tcw[2] = data[11];
            double roll;
            double pitch;
            double yaw;
            dcm2angle(Rcw, 1, &roll, &pitch, &yaw);
            if (framestamps%10 == 0){
                cout.precision(5);
                cout << "tcw: " << tcw[0] << "  " << tcw[1] << "  "  << tcw[2] << "\t\t"  << "roll: " << roll*57.3 << "   pitch: " << pitch*57.3 << "   yaw: " << yaw*57.3 << endl;
                cout.precision(17);
            }
        }
    }


    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");

    return 0;
}

// DCM转换euler角 Z-Y-X顺序
void dcm2angle(double dcm[9], int type, double* roll, double* pitch, double* yaw) {
    // 旋转顺序zyx
    double r11 = dcm[2 - 1]; // r11 = m( 1,2 )
    double r12 = dcm[1 - 1]; // r12 = m( 1,1 )
    double r21 = -dcm[3 - 1]; // r21 =-m( 1,3 )
    double r31 = dcm[6 - 1]; // r31 = m( 2,3 )
    double r32 = dcm[9 - 1]; // r32 = m( 3,3 )
    
    double yaw_tmp = atan2(r11, r12);

    if (r21 < -1) {
        r21 = -1;
    }
    else if (r21 > 1) {
        r21 = 1;
    }
    
    double pitch_tmp = asin(r21);
    double roll_tmp = atan2(r31, r32);

    *yaw = yaw_tmp;
    *pitch = pitch_tmp;
    *roll = roll_tmp;
}
