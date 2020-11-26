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
    cout << "images            " << string(argv[3]) << endl;
    cout << "timestamp         " << string(argv[4]) << endl;
    if(argc < 5)
    {
        cerr << endl << "Usage: ./mono_euroc path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) (trajectory_file_name)" << endl;
        return 1;
    }

    const int num_seq = (argc-3)/2;
    bool bFileName= (((argc-3) % 2) == 1);
    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc-1]);
        cout << "file name: " << file_name << endl;
    }

    // Load all sequences:
    int seq;
    vector< vector<string> > vstrImageFilenames;
    vector< vector<double> > vTimestampsCam;
    vector<int> nImages;

    vstrImageFilenames.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    nImages.resize(num_seq);

    int tot_images = 0;
    for (seq = 0; seq<num_seq; seq++)
    {
        cout << string(argv[(2*seq)+3]) << endl;
        cout << "Loading images for sequence " << seq << "...";
        LoadImages(string(argv[(2*seq)+3]) + "/mav0/cam0/data", string(argv[(2*seq)+4]), vstrImageFilenames[seq], vTimestampsCam[seq]);
        cout << "LOADED!" <<endl;

        nImages[seq] = vstrImageFilenames[seq].size();
        tot_images += nImages[seq];
    }

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout << endl << "-------" << endl;
    cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, true);

    cv::Mat Tcw;
    for (seq = 0; seq<num_seq; seq++)
    {

        // Main loop
        cv::Mat im;
        int proccIm = 0;
        for(int ni=0; ni<nImages[seq]; ni++, proccIm++)
        {

            // Read image from file
            im = cv::imread(vstrImageFilenames[seq][ni],CV_LOAD_IMAGE_UNCHANGED);
            double tframe = vTimestampsCam[seq][ni];

            if(im.empty())
            {
                cerr << endl << "Failed to load image at: "
                     <<  vstrImageFilenames[seq][ni] << endl;
                return 1;
            }

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    #endif

            // Pass the image to the SLAM system
            // cout << "tframe = " << tframe << endl;
            Tcw = SLAM.TrackMonocular(im,tframe); // TODO change to monocular_inertial            
            // cout << "Tcw :\n" << Tcw << endl;
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
    #endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

            if(duPerFrame>1)
            {
                cout << "image :" << vstrImageFilenames[seq][ni] << "\t\t";
                cout << "frame rate: " << frameRate << "\t\t(" << duPerFrame << "sec )" << endl;
                duPerFrame = 0.0;
                frameRate = 0;
            }
            else
            {
                frameRate++;
                duPerFrame += ttrack;
            }
            
            if (Tcw.data == NULL){
//                 cout << "null" << endl;
            }
            else{            
//                 cout << "Tcw:" << endl;
                float* data = (float*)Tcw.data; 
//                 for (int i = 0; i < Tcw.rows * Tcw.cols; i++)  
//                 {                  
//                     if (i == 4 || i == 8 || i == 12)
//                         cout << endl;
//                     cout << "  " << data[i];
//                 }  
//                 cout << endl;

                double dcm[9];
                dcm[0] = data[0];
                dcm[1] = data[1];
                dcm[2] = data[2];
                
                dcm[3] = data[4];
                dcm[4] = data[5];
                dcm[5] = data[6];
                
                dcm[6] = data[8];
                dcm[7] = data[9];
                dcm[8] = data[10];
                double roll;
                double pitch;
                double yaw;
                dcm2angle(dcm, 1, &roll, &pitch, &yaw);
                // cout << "roll: " << roll << "   pitch: " << pitch*57.3 << "   yaw: " << yaw*57.3 << endl;
            }
            
            vTimesTrack[ni]=ttrack;

            // Wait to load the next frame
            double T=0;
            if(ni<nImages[seq]-1)
                T = vTimestampsCam[seq][ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestampsCam[seq][ni-1];

            if(ttrack<T)
                usleep((T-ttrack)*1e6); // 1e6
        }

        if(seq < num_seq - 1)
        {
            cout << "Changing the dataset" << endl;

            SLAM.ChangeDataset();
        }

    }
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    if (bFileName)
    {
        const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
        const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
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
