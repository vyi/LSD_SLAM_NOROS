/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam>
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "live_slam_wrapper.h"

#include <boost/thread.hpp>
#include "util/settings.h"
#include "util/global_funcs.h"
#include "slam_system.h"

#include <sstream>
#include <fstream>
#include <dirent.h>
#include <algorithm>

//#include "IOWrapper/ROS/ROSOutput3DWrapper.h"
//#include "IOWrapper/ROS/rosReconfigure.h"

#include "util/undistorter.h"
//#include <ros/package.h>

#include "opencv2/opencv.hpp"


// Gets current projection matrix (= PerspectiveMatrix * CameraPoseMatrix)
template<typename Tracker>
Eigen::Matrix<double, 3, 4> get_projection(Tracker& tracker) {
    std::vector<double> cam_params = tracker.GetCameraParams();
    Eigen::Matrix3d intrinsics = Eigen::Matrix3d::Identity();
    intrinsics(0, 0) = cam_params[0];
    intrinsics(1, 1) = cam_params[1];
    intrinsics(0, 2) = cam_params[2];
    intrinsics(1, 2) = cam_params[3];

    std::vector<double> rot = tracker.GetCurrentPose();
    Eigen::Matrix<double, 3, 3> mrot;
    mrot << rot[0], rot[1], rot[2],
         rot[3], rot[4], rot[5],
         rot[6], rot[7], rot[8];
    Eigen::Matrix<double, 3, 4> pose;
    pose.setZero();
    pose.block<3, 3>(0, 0) = mrot;

    Eigen::Matrix<double, 3, 4> projection = intrinsics * pose;
    return projection;
}

// Draws desirable target in world coordinate to current color image
template<typename Tracker>
void draw_target(cv::Mat& rgb_img, Tracker& tracker) {
    const Eigen::Vector4d point_x(0.1, 0, 1, 1);
    const Eigen::Vector4d point_y(0, 0.1, 1, 1);
    const Eigen::Vector4d point_z(0, 0, 1.1, 1);
    const Eigen::Vector4d point_target(0, 0, 1.0, 1);
    Eigen::Matrix<double, 3, 4> proj = get_projection(tracker);
    Eigen::Vector3d point_cam = proj * point_target;
    Eigen::Vector3d pointx_cam = proj * point_x;
    Eigen::Vector3d pointy_cam = proj * point_y;
    Eigen::Vector3d pointz_cam = proj * point_z;
    cv::line(rgb_img, cv::Point(point_cam[0], point_cam[1]),
             cv::Point(pointx_cam[0], pointx_cam[1]), cv::Scalar(255, 0, 0), 3);
    cv::line(rgb_img, cv::Point(point_cam[0], point_cam[1]),
             cv::Point(pointy_cam[0], pointy_cam[1]), cv::Scalar(0, 255, 0), 3);
    cv::line(rgb_img, cv::Point(point_cam[0], point_cam[1]),
             cv::Point(pointz_cam[0], pointz_cam[1]), cv::Scalar(0, 0, 255), 3);
}


std::string &ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(),
                                    std::not1(std::ptr_fun<int, int>(std::isspace))));
    return s;
}
std::string &rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(),
                         std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
    return s;
}
std::string &trim(std::string &s) {
    return ltrim(rtrim(s));
}

std::string append_slash_to_dirname(std::string dirname) {
    if(dirname[dirname.length()-1] == '/') {
        return dirname;
    }
    return dirname + "/";
}

int getdir (std::string dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL)
    {
        return -1;
    }

    while ((dirp = readdir(dp)) != NULL) {
        std::string name = std::string(dirp->d_name);

        if(name != "." && name != "..")
            files.push_back(name);
    }
    closedir(dp);


    std::sort(files.begin(), files.end());
    dir = append_slash_to_dirname(dir);
    for(unsigned int i=0; i<files.size(); i++)
    {
        if(files[i].at(0) != '/')
            files[i] = dir + files[i];
    }

    return files.size();
}

int getFile (std::string source, std::vector<std::string> &files)
{
    std::ifstream f(source.c_str());

    if(f.good() && f.is_open())
    {
        while(!f.eof())
        {
            std::string l;
            std::getline(f,l);

            l = trim(l);

            if(l == "" || l[0] == '#')
                continue;

            files.push_back(l);
        }

        f.close();

        size_t sp = source.find_last_of('/');
        std::string prefix;
        if(sp == std::string::npos)
            prefix = "";
        else
            prefix = source.substr(0,sp);

        for(unsigned int i=0; i<files.size(); i++)
        {
            if(files[i].at(0) != '/')
                files[i] = prefix + "/" + files[i];
        }

        return (int)files.size();
    }
    else
    {
        f.close();
        return -1;
    }

}


using namespace lsd_slam;
int main(int argc, char* argv[])
{

    if(argc < 2) {
        std::cout << "Usage: $./bin/main_on_images data/sequence_${sequence_number}/"
                  << std::endl;
        exit(-1);
    }

    const std::string dataset_root = append_slash_to_dirname(std::string(argv[1]));
    const std::string calib_file = dataset_root + "camera.txt";
    const std::string source = dataset_root + "images/";

    std::cout << "path to images : " << source << std::endl;
    Undistorter* undistorter = 0;

    undistorter = Undistorter::getUndistorterForFile(calib_file);

    if(undistorter == 0)
    {
        printf("need camera calibration file! (set using _calib:=FILE)\n");
        exit(0);
    }

    std::cout << "Done : undistorter" << std::endl;
    
    int w = undistorter->getOutputWidth();
    int h = undistorter->getOutputHeight();
    float fx = undistorter->getK().at<double>(0, 0);
    float fy = undistorter->getK().at<double>(1, 1);
    float cx = undistorter->getK().at<double>(2, 0);
    float cy = undistorter->getK().at<double>(2, 1);
    Sophus::Matrix3f K;
    K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;


    // make output wrapper. just set to zero if no output is required.
    Output3DWrapper* outputWrapper = nullptr; // new ROSOutput3DWrapper(w,h);

    // make slam system
    std::cout << "Now going to create system!" << std::endl;
    SlamSystem* system = new SlamSystem(w, h, K, doSlam);
    std::cout << "Done : Created system!" << std::endl;


    std::cout << "Now going to setup visualizer system!" << std::endl;
    system->setVisualization(outputWrapper);
    std::cout << "Done : Visualizer setup!" << std::endl;
    

    std::vector<std::string> files;


    std::cout << "Now going to open image files" << std::endl;
    if(getdir(source, files) >= 0) {
        printf("found %d image files in folder %s!\n", (int)files.size(),
               source.c_str());
    } else {
        std::cerr << "Could not read the image directory" << std::endl;
        exit(-1);
    }
    std::cout << "Done : opening image files" << std::endl;


    cv::Mat image = cv::Mat(h,w,CV_8U);
    int runningIDX=0;
    float fakeTimeStamp = 0;

    //ros::Rate r(hz);

    for(unsigned int i=0; i<files.size(); i++)
    {
        std::cout << "reading " << files[i] << std::endl;

        cv::Mat imageDist = cv::imread(files[i], CV_LOAD_IMAGE_GRAYSCALE);

        assert(imageDist.type() == CV_8U);

        undistorter->undistort(imageDist, image);
        assert(image.type() == CV_8U);

        if(runningIDX == 0)
            system->randomInit(image.data, fakeTimeStamp, runningIDX);
        else
            system->trackFrame(image.data, runningIDX, false,fakeTimeStamp);
        runningIDX++;
        fakeTimeStamp+=0.03;

        //if(hz != 0)
        //    r.sleep();

        if(fullResetRequested)
        {

            printf("FULL RESET!\n");
            delete system;

            system = new SlamSystem(w, h, K, doSlam);
            system->setVisualization(outputWrapper);

            fullResetRequested = false;
            runningIDX = 0;
        }

        //ros::spinOnce();
    }


    system->finalize();



    delete system;
    delete undistorter;
    delete outputWrapper;
    return 0;
}
