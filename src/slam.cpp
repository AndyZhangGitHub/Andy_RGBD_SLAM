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

    // g2o 相关部分设定
    typedef g2o::BlockSolver_6_3 SlamBlockSolver; 
    typedef g2o::LinearSolverCSparse< SlamBlockSolver::PoseMatrixType > SlamLinearSolver; 

    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);

    g2o::SparseOptimizer globalOptimizer;
    globalOptimizer.setAlgorithm(solver);
    globalOptimizer.setVerbose(false);
    
    // 添加第一个定点
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId(1);
    v->setEstimate(Eigen::Isometry3d::Identity());
    v->setFixed(true);
    globalOptimizer.addVertex(v);


    int lastIndex = curIndex;

    
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
          
          //构建定点并添加到图中
          g2o::VertexSE3* v = new g2o::VertexSE3();
          v->setId(curIndex);
          v->setEstimate(Eigen::Isometry3d::Identity());
          globalOptimizer.addVertex(v);

          //添加边
          g2o::EdgeSE3* edge = new g2o::EdgeSE3();
          edge->vertices()[1] = globalOptimizer.vertex(lastIndex);
          edge->vertices()[2] = globalOptimizer.vertex(curIndex);

          Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
          information(0,0) = information(1,1) = information(2,2) = 100;
          information(3,3) = information(4,4) = information(5,5) = 100;
          edge->setInformation(information);
          edge->setMeasurement(T);

          globalOptimizer.addEdge(edge);

          viewer.showCloud( cloud );
  
          lastFrame = curFrame;
          lastIndex = curIndex;
          cout<<"__________________________________________"<<endl;
    }

    cout<<"图中的定点数：" <<globalOptimizer.vertices().size()<<endl;
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize( 100 ); //可以指定优化步数
    cout<<"Optimization done."<<endl;
    globalOptimizer.clear();

     





    fs.release();
    return 0;

}