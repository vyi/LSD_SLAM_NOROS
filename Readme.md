# LSD-SLAM: Large-Scale Direct Monocular SLAM

LSD-SLAM is a novel approach to real-time monocular SLAM. It is fully direct (i.e. does not use keypoints / features) and creates large-scale, 
semi-dense maps in real-time on a laptop. For more information see
[http://vision.in.tum.de/lsdslam](http://vision.in.tum.de/lsdslam)
where you can also find the corresponding publications and Youtube videos, as well as some 
example-input datasets, and the generated output as rosbag or .ply point cloud.


The source code is tested on Ubuntu 16.04 using cmake (3.5.1) and it depends on the following libraries: 
 * OpenCV 2.4
 * g20 (commit 9b41a4e on Apr 6)  for debugging purpose I've built g2o using the -DCMAKE\_BUILD\_TYPE=Debug
 * Eigen 3.2
 * openFABMap (Included as third party)
 * Sophus (Included as third party)
 

When built there will be two executables in the ./bin directory
 * sample\_app          : For use with the webcam 
 * main\_on\_images     : For use with dataset images (download from TUM page mentioned above)

There is a camera calibration application in the **camera_calibration** directory. Just run the script build.sh and it will create an executable by name **calibration** in the same directory.


