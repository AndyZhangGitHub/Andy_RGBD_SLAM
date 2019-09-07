#include<iostream>
#include<string>
using namespace std;

#include<opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include"slambase.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud; 


CAMERA_INTRINSIC_PARAMETERS camera;



int main()
{
    cv::Mat rgb,depth;
   
    // 相机内参
    camera.scale = 1000;
    camera.cx = 325.5;
    camera.cy = 253.5;
    camera.fx = 518.0;
    camera.fy = 519.0;
    
    FRAME frame;
    frame.rgb = cv::imread("../data/rgb1.png");
    frame.depth = cv::imread("../data/depth1.png",-1);
    
    FRAME frame2;
    frame2.rgb = cv::imread("../data/rgb2.png");
    frame2.depth = cv::imread("../data/depth2.png",-1);


    cv::imshow( "src", frame.rgb );
    cv::waitKey(0);
    computeKeyPointsAndDesp(frame);
    cv::Mat imgShow;
    cv::drawKeypoints( frame.rgb, frame.kp, imgShow, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    cv::imshow( "keypoints", imgShow );
    cv::waitKey(0);

    computeKeyPointsAndDesp(frame2);
    
    RESULT_OF_PNP pnp_result = estimateMotion(frame,frame2,camera);

    
    return 0;

}