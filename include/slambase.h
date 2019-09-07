# pragma once

// 各种头文件 
// C++标准库
#include <fstream>
#include <vector>
using namespace std;

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<opencv2/opencv.hpp>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

// 类型定义
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 相机内参结构
struct CAMERA_INTRINSIC_PARAMETERS 
{ 
    double cx, cy, fx, fy, scale;
};

// 帧结构
struct FRAME
{
    cv::Mat rgb, depth; //该帧对应的彩色图与深度图
    cv::Mat desp;       //特征描述子
    vector<cv::KeyPoint> kp; //关键点
};

// PnP 结果
struct RESULT_OF_PNP
{
    cv::Mat rvec, tvec;
    int inliers;
};

FRAME LoadImages(int index);
double normofTransform( cv::Mat rvec, 
                        cv::Mat tvec );

// computeKeyPointsAndDesp 同时提取关键点与特征描述子
void computeKeyPointsAndDesp( FRAME& frame);

// estimateMotion 计算两个帧之间的运动
// 输入：帧1和帧2, 相机内参
RESULT_OF_PNP estimateMotion( FRAME& frame1,
                              FRAME& frame2, 
                              CAMERA_INTRINSIC_PARAMETERS& camera );

// 函数接口
// image2PonitCloud 将rgb图转换为点云
PointCloud::Ptr image2PointCloud( cv::Mat& rgb, 
                                  cv::Mat& depth, 
                                  CAMERA_INTRINSIC_PARAMETERS& camera );

// point2dTo3d 将单个点从图像坐标转换为空间坐标
// input: 3维点Point3f (u,v,d)
cv::Point3f point2d3d( cv::Point3f& point, 
                       CAMERA_INTRINSIC_PARAMETERS& camera );

//拼接点云
PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, 
                                FRAME& newFrame, 
                                Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera );

// cvMat2Eigen
Eigen::Isometry3d cvMat2Eigen( cv::Mat& rvec, 
                               cv::Mat& tvec );
