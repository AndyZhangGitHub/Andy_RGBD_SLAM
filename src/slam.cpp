#include<iostream>
#include<string>
using namespace std;

#include<opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
 #include <pcl/filters/passthrough.h>

#include"slambase.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud; 


CAMERA_INTRINSIC_PARAMETERS camera;



int main()
{
    
   cv::FileStorage fs("param.yaml",0);

   vector<PointCloud> PointCloudDB;
   vector<FRAME>      KeyFrame;
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
    KeyFrame.push_back(lastFrame);
    PointCloud::Ptr cloud = image2PointCloud(lastFrame.rgb,lastFrame.depth,camera);
    PointCloudDB.push_back(*cloud);

    pcl::visualization::CloudViewer viewer("viewer");
    
    //cv::namedWindow("image",cv::WINDOW_AUTOSIZE);

    int min_inliers = fs["min_inliers"];
    double max_norm = (double)fs["max_norm"];

    // g2o 相关部分设定
    typedef g2o::BlockSolver_6_3 SlamBlockSolver; 
    typedef g2o::LinearSolverCSparse< SlamBlockSolver::PoseMatrixType > SlamLinearSolver; 

    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);

    g2o::SparseOptimizer globalOptimizer;
    globalOptimizer.setAlgorithm(solver);
    globalOptimizer.setVerbose(true);
    
    // 添加第一个定点
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId(lastFrame.frameId);
    v->setEstimate(Eigen::Isometry3d::Identity());
    v->setFixed(true);
    globalOptimizer.addVertex(v);


    int lastIndex = curIndex;

    
    for ( curIndex=startIndex+1; curIndex<endIndex; curIndex++ )
    {
        
          FRAME curFrame = LoadImages(curIndex);
          cout<<"curIndex: "<<curIndex<<endl;

          
          int result = checkKeyframes(KeyFrame.back(),curFrame,globalOptimizer,camera);

          if(result == 4)
          {
              checkNearbyLoops( KeyFrame, curFrame, globalOptimizer ,camera);
              checkRandomLoops( KeyFrame, curFrame, globalOptimizer ,camera);
              
              KeyFrame.push_back(curFrame);

              RESULT_OF_PNP result = estimateMotion( lastFrame, curFrame, camera );
              Eigen::Isometry3d T = cvMat2Eigen( result.rvec, result.tvec );

              //PointCloud::Ptr newcloud = image2PointCloud(curFrame.rgb,curFrame.depth,camera);

             cloud = joinPointCloud(cloud,curFrame,T,camera);
              
             viewer.showCloud( cloud );

             lastFrame = curFrame;
             lastIndex = curIndex;

          }
        
          cout<<"__________________________________________"<<endl;
    }

    pcl::io::savePCDFile( "../data/result_before.pcd", *cloud );

    cout<<"图中的顶点数：" <<globalOptimizer.vertices().size()<<endl;
    cout<<"关键帧数"<<PointCloudDB.size()<<endl;
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize( 100 ); //可以指定优化步数
    cout<<"Optimization done."<<endl;

    
    FRAME first_frame = LoadImages(1);
    //PointCloud::Ptr global_map_temp = image2PointCloud(first_frame.rgb,first_frame.depth,camera);
    
    PointCloud::Ptr gloabl_map(new PointCloud());
     pcl::VoxelGrid<PointT> voxel; // 网格滤波器，调整地图分辨率
     voxel.setLeafSize( 0.01, 0.01, 0.01 );
     PointCloud::Ptr temp(new PointCloud());
     pcl::PassThrough<PointT> pass;
     pass.setFilterFieldName("z");
     pass.setFilterLimits( 0.0, 14.0 ); //4m以上就不要了
     

    for(int i = 0; i < KeyFrame.size();++i)
    {
       g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(globalOptimizer.vertex( KeyFrame[i].frameId));
       Eigen::Isometry3d pose = vertex->estimate(); 
       
       PointCloud::Ptr newCloud = image2PointCloud(KeyFrame[i].rgb,KeyFrame[i].depth,camera);
       
       voxel.setInputCloud( newCloud );
       voxel.filter( *temp );
       pass.setInputCloud( temp );
       pass.filter( *newCloud );

       pcl::transformPointCloud(*newCloud,*temp, pose.matrix());
      
       
      *gloabl_map += *temp; 
       
       temp->clear();
       newCloud->clear();
     
    }

    
    voxel.setInputCloud(gloabl_map);
    voxel.filter(*temp);
    
    cout<<"----------------------优化后的全局地图---------------"<<endl;
   
    viewer.showCloud( temp );
    pcl::io::savePCDFile( "../data/result_after.pcd", *cloud );
    cout<<"地图已经保存"<<endl;
    while(1)
    {

    }

    globalOptimizer.clear();
    fs.release();
    return 0;

}