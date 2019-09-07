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

    computeKeyPointsAndDesp(frame);
    computeKeyPointsAndDesp(frame2);

    RESULT_OF_PNP pnp_result = estimateMotion(frame,frame2,camera);

    cout<<"rvec:"<<pnp_result.rvec<<"  tvec: "<<pnp_result.tvec<<endl;

    
    // 将平移向量和旋转矩阵转换成变换矩阵
    Eigen::Isometry3d T = cvMat2Eigen(pnp_result.rvec,pnp_result.tvec);

   

    PointCloud::Ptr cloud1 = image2PointCloud( frame.rgb, frame.depth, camera );
    PointCloud::Ptr cloud2 = image2PointCloud( frame2.rgb, frame2.depth, camera );

    cloud1 = joinPointCloud(cloud1,frame2,T,camera);
    
    pcl::visualization::CloudViewer viewer( "viewer" );
    viewer.showCloud( cloud1 );
    while( !viewer.wasStopped() )
    {
        
    }
    return 0;

}