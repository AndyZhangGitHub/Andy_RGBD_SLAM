#include"slambase.h"



// computeKeyPointsAndDesp 同时提取关键点与特征描述子
void computeKeyPointsAndDesp( FRAME& frame)
{
      cv::Ptr<cv::ORB> orb = cv::ORB::create();
      orb->detect(frame.rgb,frame.kp);
      orb->compute(frame.rgb,frame.kp,frame.desp);
}

// estimateMotion 计算两个帧之间的运动
// 输入：帧1和帧2, 相机内参
RESULT_OF_PNP estimateMotion( FRAME& frame1,
                              FRAME& frame2,
                              CAMERA_INTRINSIC_PARAMETERS& camera )
{
      
      vector<cv::DMatch> matches;
      cv::BFMatcher mathcer;
      mathcer.match(frame1.desp,frame2.desp,matches);

      RESULT_OF_PNP result;
      cv::FileStorage fs("param.yaml",0);
      vector<cv::DMatch> goodmatches;
      double minDist = 9999;
      double goodmatches_threshold = (double)fs["good_match_threshold"];

      for(int i = 0;i < matches.size();++i)
         if(matches[i].distance < minDist) 
             minDist = matches[i].distance;
     
  
     for(int i = 0; i< matches.size();++i)
         if(matches[i].distance < goodmatches_threshold * minDist )
              goodmatches.push_back(matches[i]);
    
    // cout<< "优质匹配点数为： "<< goodmatches.size()<<endl;

    //  cv::Mat imgMatches;
    //  cv::drawMatches( frame1.rgb, frame1.kp, frame2.rgb, frame2.kp, goodmatches, imgMatches );
    //  cv::imshow( "matches", imgMatches );
    //  cv::waitKey( 20 );

     if (goodmatches.size() <= 5) 
     {
        result.inliers = -1;
        fs.release();
        return result;
     }

     vector<cv::Point3f> point_camera;
     vector<cv::Point2f> point_image;

     for(int i = 0; i < goodmatches.size();++i)
     {
         cv::Point2f p = frame1.kp[goodmatches[i].queryIdx].pt;//获取第一帧图像中特征点的像素坐标
         ushort d = frame1.depth.ptr<ushort>(int(p.y))[int(p.x)];
        

         if(d == 0) continue;
         else
         {
             cv::Point3f pt(p.x,p.y,d);
             cv::Point3f pd = point2d3d(pt,camera);
             //cout<<"pt:------"<<pt<<endl;
             //cout<<"pd:-------------"<<pd<<endl;
             point_camera.push_back(pd);
             point_image.push_back(cv::Point2f(frame2.kp[goodmatches[i].trainIdx].pt));
         }

     }
     
     if(point_camera.size() <= 4 || point_image.size() <= 4)
     {
         result.inliers = -1;
         fs.release();
         return result;
     }

    double camera_matrix_data[3][3] = {
        {camera.fx, 0, camera.cx},
        {0, camera.fy, camera.cy},
        {0, 0, 1}
    };

     // 构建相机矩阵
     cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
     //cv::Mat K;
     //fs["cam_intrinsic_matrix"] >> K;
     cv:: Mat rvec,tvec,inliers;

     //cout<<"求解pnp"<<endl;

     cv::solvePnPRansac( point_camera, point_image, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1, 0.99, inliers );

     result.rvec = rvec;
     result.tvec = tvec;
     result.inliers = inliers.rows;
     
     fs.release();
     return result;
}


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

cv::Point3f point2d3d( cv::Point3f& point, 
                         CAMERA_INTRINSIC_PARAMETERS& camera )
{
     cv::Point3f result;
     result.z =  point.z/camera.scale;
     result.x = (point.x - camera.cx)*result.z /camera.fx; //注意
     result.y = (point.y - camera.cy)*result.z /camera.fy;

     return result;

}


FRAME LoadImages(int index)
{

    cv::FileStorage fs("param.yaml",0);
    string rgb_path ;
    string depth_path ;
    fs["rbg_image_path"] >> rgb_path;
    fs["depth_image_path"] >> depth_path;
    

    stringstream ss;
    ss<< rgb_path<<index<<".png";
    string rgb_filename;
    ss>> rgb_filename;
    
     
    ss.clear();
    ss<<depth_path<<index<<".png";
    string depth_filename;
    ss>>depth_filename;
   
    FRAME frame;
    frame.rgb   = cv::imread( rgb_filename);
    frame.depth = cv::imread( depth_filename,-1);
    //frame.frameID = index;

    
    computeKeyPointsAndDesp(frame);
    fs.release();
    
    return frame;
   
}

PointCloud::Ptr joinPointCloud( PointCloud::Ptr original,
                                FRAME& newFrame, 
                                Eigen::Isometry3d T, 
                                CAMERA_INTRINSIC_PARAMETERS& camera ) 
{

    PointCloud::Ptr newCloud = image2PointCloud( newFrame.rgb, newFrame.depth, camera );

    // 合并点云
    PointCloud::Ptr output (new PointCloud());
    pcl::transformPointCloud( *original, *output, T.matrix() );
    *newCloud += *output;
    
    static pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize( 0.01, 0.01, 0.01 );
    voxel.setInputCloud( newCloud );
    PointCloud::Ptr tmp( new PointCloud() );
    voxel.filter( *tmp );
    
    return tmp;
}

// cvMat2Eigen
Eigen::Isometry3d cvMat2Eigen( cv::Mat& rvec, 
                               cv::Mat& tvec )
{
    cv::Mat R;
    cv::Rodrigues( rvec, R );
    Eigen::Matrix3d r;
    for ( int i=0; i<3; i++ )
        for ( int j=0; j<3; j++ ) 
            r(i,j) = R.at<double>(i,j);
  
    // 将平移向量和旋转矩阵转换成变换矩阵
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    Eigen::AngleAxisd angle(r);
    T = angle;
    T(0,3) = tvec.at<double>(0,0); 
    T(1,3) = tvec.at<double>(1,0); 
    T(2,3) = tvec.at<double>(2,0);
    return T;
}
double normofTransform( cv::Mat rvec, 
                        cv::Mat tvec )
{
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}