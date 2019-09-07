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
    rgb = cv::imread("../data/rgb.png");
    depth = cv::imread("../data/depth.png",-1);//不要忘记加 -1 这个参数

    // 相机内参
    camera.scale = 1000;
    camera.cx = 325.5;
    camera.cy = 253.5;
    camera.fx = 518.0;
    camera.fy = 519.0;

    PointCloud::Ptr cloud(new PointCloud);

    cloud = image2PointCloud(rgb,depth,camera);
    
    pcl::io::savePCDFile( "../pointcloud.pcd", *cloud );
    // 清除数据并退出
    cloud->points.clear();
    cout<<"Point cloud saved."<<endl;
    return 0;

}