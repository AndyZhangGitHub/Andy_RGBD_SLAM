#include<iostream>
#include<string>
using namespace std;

#include<opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud; 


// 相机内参
const double camera_factor = 1000;
const double camera_cx = 325.5;
const double camera_cy = 253.5;
const double camera_fx = 518.0;
const double camera_fy = 519.0;

int main()
{
    cv::Mat rgb,depth;
    rgb = cv::imread("../data/rgb.png");
    depth = cv::imread("../data/depth.png",-1);//不要忘记加 -1 这个参数

    PointCloud::Ptr cloud(new PointCloud);

    for(int i = 0;i < depth.rows;++i)
       for(int j = 0;j<depth.cols;++j)
       {
             ushort d = depth.ptr<ushort>(i)[j];
             if(d == 0) continue;
             else
             {
                 PointT p;

                 p.z = (double)d / camera_factor;
                 p.x = (j - camera_cx)*p.z / camera_fx; // 注意这里是 j
                 p.y = (i - camera_cy)*p.z / camera_fy;
                  
                 p.b = rgb.ptr<uchar>(i)[j*3];
                 p.g = rgb.ptr<uchar>(i)[j*3+1];
                 p.r = rgb.ptr<uchar>(i)[j*3+2];

                 cloud->points.push_back(p);

             }
             
       }

       // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cout<<"point cloud size = "<<cloud->points.size()<<endl;
    cloud->is_dense = false;
    pcl::io::savePCDFile( "../pointcloud.pcd", *cloud );
    // 清除数据并退出
    cloud->points.clear();
    cout<<"Point cloud saved."<<endl;
    return 0;

}