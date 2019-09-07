#include"slambase.h"






PointCloud::Ptr image2PointCloud( cv::Mat& rgb, 
                                  cv::Mat& depth, 
                                  CAMERA_INTRINSIC_PARAMETERS& camera )
{
     PointCloud::Ptr result(new PointCloud);

     for(int i = 0;i < depth.rows;++i)
       for(int j = 0;j<depth.cols;++j)
       {
             ushort d = depth.ptr<ushort>(i)[j];
             if(d == 0) continue;
             else
             {
                 PointT p;

                 p.z = (double)d / camera.scale;
                 p.x = (j - camera.cx)*p.z / camera.fx; // 注意这里是 j
                 p.y = (i - camera.cy)*p.z / camera.fy;
                  
                 p.b = rgb.ptr<uchar>(i)[j*3];
                 p.g = rgb.ptr<uchar>(i)[j*3+1];
                 p.r = rgb.ptr<uchar>(i)[j*3+2];

                 result->points.push_back(p);

             }
             
       }

     result->height = 1;
     result->width = result->points.size();
     result->is_dense = false;

     return result;

}

cv::Point3f point2dTo3d( cv::Point3f& point, 
                         CAMERA_INTRINSIC_PARAMETERS& camera )
{
     cv::Point3f result;
     result.z = (double)(point.z/camera.scale);
     result.x = (point.x - camera.cx)*point.z /camera.fx;
     result.y = (point.y - camera.cy)*point.z /camera.fy;

     return result;

}