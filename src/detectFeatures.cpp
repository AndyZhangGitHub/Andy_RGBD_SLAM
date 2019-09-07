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
    
   cv::FileStorage fs("param.yaml",0);
    // 相机内参
    camera.scale = 1000;
    camera.cx = 325.5;
    camera.cy = 253.5;
    camera.fx = 518.0;
    camera.fy = 519.0;
    
    int startIndex = fs["start_index"];
    int endIndex   = fs["end_index"];

    int curIndex = startIndex;
    FRAME lastFrame = LoadImages(curIndex);
    PointCloud::Ptr cloud = image2PointCloud(lastFrame.rgb,lastFrame.depth,camera);

    pcl::visualization::CloudViewer viewer("viewer");
    cv::namedWindow("image",cv::WINDOW_AUTOSIZE);

    int min_inliers = fs["min_inliers"];
    double max_norm = (double)fs["max_norm"];
    
    for ( curIndex=startIndex+1; curIndex<endIndex; curIndex++ )
    {
        
          FRAME curFrame = LoadImages(curIndex);
          cout<<"curIndex: "<<curIndex<<endl;
          
          cv::imshow("image",curFrame.rgb);
          cv::waitKey(30);
          RESULT_OF_PNP pnp_result = estimateMotion(lastFrame,curFrame,camera);
          if ( pnp_result.inliers < min_inliers ) //inliers不够，放弃该帧
              continue;
          // 计算运动范围是否太大
          double norm = normofTransform(pnp_result.rvec, pnp_result.tvec);
          cout<<"norm = "<<norm<<endl;
          if ( norm >= max_norm )
              continue;
          Eigen::Isometry3d T = cvMat2Eigen( pnp_result.rvec, pnp_result.tvec );
          cout<<"T="<<T.matrix()<<endl;
          
          //cloud = joinPointCloud( cloud, currFrame, T.inverse(), camera );
          cloud = joinPointCloud( cloud, curFrame, T, camera );
          
         
          viewer.showCloud( cloud );
  
          lastFrame = curFrame;
          cout<<"__________________________________________"<<endl;
    }

    pcl::io::savePCDFile( "../data/result.pcd", *cloud );
    fs.release();
    return 0;

}