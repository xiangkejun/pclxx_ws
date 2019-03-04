#ifndef PCL_xx__
#define	PCL_xx__

/************************************************
关于如何使用PCL在ROS 中，实现简单的数据转化
时间：2017.3.31

****************************************************/

#include <iostream>
#include <string.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>


// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>   // 计算重心

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>

//#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>

//#include <pcl/filters/crop_hull.h>
#include <pcl/filters/conditional_removal.h>    //条件滤波
#include <pcl/filters/project_inliers.h>      //模型投影
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>   //模型分割
#include <pcl/filters/extract_indices.h>       // 索引提取
#include <pcl/sample_consensus/model_types.h>    //模型分类
#include <pcl/segmentation/extract_clusters.h>    //
#include <pcl/features/moment_of_inertia_estimation.h>  //画框

#include <pcl/kdtree/kdtree.h>

pcl::visualization::PCLVisualizer viewer ("3D Viewer");
//pcl::visualization::CloudViewer viewer("Cloud Viewer");

typedef pcl::PointXYZ PointT;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_condition (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_planar (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_statiscal_removal(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<PointT>::Ptr cloud_rest_line (new pcl::PointCloud<PointT> ());
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_linenei (new pcl::PointCloud<pcl::PointXYZ>);

pcl::ModelCoefficients::Ptr coefficients_seg (new pcl::ModelCoefficients);

pcl::ExtractIndices<PointT> extract;

Eigen::Vector4f  centroid_cluster;

void condition_duo_filter(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
//       11111111111111111111111111111111111111111111111111
// ************ 条件滤波 ***********************************************/////
///// 两个条件 1）  -3<z<3;    2)   x*x + y*y -64 <=0
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
        pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, -1.0)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
        pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 3)));   //1.5

    // 做一个半圆    x>0.5
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
    pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, 0.5)));  
   
      //(p'Ap + 2v'p + c [OP] 0)
        // x*x + y*y - 100 <=0   半径 10m    得到一个圆柱
     Eigen::Matrix3f A;A(0, 0) = 1;A(0, 1) = 0;A(0, 2) = 0;A(1, 0) = 0;A(1, 1) = 1;A(1, 2) = 0;A(2, 0) = 0;A(2, 1) = 0;A(2, 2) = 0; 
     Eigen::Vector3f v;v(0)=0;v(1)=0;v(2)=0; 
     float c = -1600;//radius^2 
    range_cond->addComparison(pcl::TfQuadraticXYZComparison < pcl::PointXYZ >::Ptr (new pcl::TfQuadraticXYZComparison < pcl::PointXYZ >(pcl::ComparisonOps::LT , A, v, c))); 



    //  //(p'Ap + 2v'p + c [OP] 0)
    //     // k*x+ y <0     做一个任意角度的半圆柱
    //  Eigen::Matrix3f A1;A1(0, 0) = 0;A1(0, 1) = 0;A1(0, 2) = 0;A1(1, 0) = 0;A1(1, 1) = 0;A1(1, 2) = 0;A1(2, 0) = 0;A1(2, 1) = 0;A1(2, 2) = 0; 
    //  Eigen::Vector3f v1;v1(0)=0;v1(1)=-0.5;v1(2)=0;   //sqrt(3)= 1.732    
    //  float c1 = 2;   //  y+2<0
    // range_cond->addComparison(pcl::TfQuadraticXYZComparison < pcl::PointXYZ >::Ptr (new pcl::TfQuadraticXYZComparison < pcl::PointXYZ >(pcl::ComparisonOps::LT , A1, v1, c1))); 



    // bobjectuild the filter
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (cloud);
   // condrem.setKeepOrganized(true);
    // apply filter
    condrem.filter (*cloud_condition);
   // viewer.showCloud(cloud_condition);

}

void plane_touying_filter(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    // 222222222222222222222222222222222222222222222222222222222222
  //******************** ax+by+cz+d = 0 ***平面投影滤波(z= -0.5 即 x-y 平面)
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = coefficients->values[1] = 0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0.5;  

  // Create the filtering object
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_planar);
}

void statiscal_removal(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
 // 统计法移除离群点
   // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (20);   // 查询点邻近点数
  sor.setStddevMulThresh (2);  //判断是否为离群点的阈值
  sor.filter (*cloud_statiscal_removal);

}

void segmentation_line(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  /// 333333333333333333333333333333333333333333333////
//****************** 直线分割  得直线方程系数***********************//
  
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_LINE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (2);   // 2m
  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients_seg);

 // 得到直线系数
 std::cout <<"inliers"<< inliers->indices[0]  << std::endl;
  std::cerr << "Model coefficients: " << coefficients_seg->values[0] << " " 
                                      << coefficients_seg->values[1] << " "
                                      << coefficients_seg->values[2] << " " 
                                      << coefficients_seg->values[3] << " "
                                      << coefficients_seg->values[4] << " "
                                      << coefficients_seg->values[5] << " "<<std::endl;

  extract.setInputCloud (cloud_planar);
  extract.setIndices (inliers);
  extract.setNegative (true);  //true提取剩余的  false: 线上
  extract.filter (*cloud_rest_line);  //保存直线分割后的点云
}

void lines_filter(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    // 44444444444444444444444444444444
//**************直线滤波 ***************
// 思想： 同过2种情况使用直通滤波得到 里面的点
  pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond_line (new pcl::ConditionAnd<pcl::PointXYZ> ());
      //(p'Ap + 2v'p + c [OP] 0)
        // ax+ by + d >   0   //
     Eigen::Matrix3f A2;A2(0, 0) = 0;A2(0, 1) = 0;A2(0, 2) = 0;A2(1, 0) = 0;A2(1, 1) = 0;A2(1, 2) = 0;A2(2, 0) = 0;A2(2, 1) = 0;A2(2, 2) = 0; 
     Eigen::Vector3f v2;v2(0)=coefficients_seg->values[4]/(2*coefficients_seg->values[3]);  v2(1)=-0.5;    v2(2)=0; 
     float c2 = coefficients_seg->values[1] - (coefficients_seg->values[0]*coefficients_seg->values[4])/coefficients_seg->values[3];
      //  a   x的系数    当 a<0时，该等式会发生变号
       float a = coefficients_seg->values[4] / coefficients_seg->values[3];
       if( a < 0 )
         range_cond_line->addComparison(pcl::TfQuadraticXYZComparison < pcl::PointXYZ >::Ptr (new pcl::TfQuadraticXYZComparison < pcl::PointXYZ >(pcl::ComparisonOps::GT , A2, v2, c2))); 
       if(a > 0 || a == 0)
       range_cond_line->addComparison(pcl::TfQuadraticXYZComparison < pcl::PointXYZ >::Ptr (new pcl::TfQuadraticXYZComparison < pcl::PointXYZ >(pcl::ComparisonOps::LT , A2, v2, c2))); 

    // bobjectuild the filter
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem_line;
    condrem_line.setCondition (range_cond_line);
    
    condrem_line.setInputCloud (cloud);
   // condrem.setKeepOrganized(true);
    // apply filter
    condrem_line.filter (*cloud_linenei);
}

void draw_kuang_cloud( pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int j)
{
  //  对输入点云进行画框
  pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud (cloud);
  feature_extractor.compute ();

  std::vector <float> moment_of_inertia;
  std::vector <float> eccentricity;
  pcl::PointXYZ min_point_AABB;
  pcl::PointXYZ max_point_AABB;
  pcl::PointXYZ min_point_OBB;
  pcl::PointXYZ max_point_OBB;
  pcl::PointXYZ position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;
  float major_value, middle_value, minor_value;
  Eigen::Vector3f major_vector, middle_vector, minor_vector;
  Eigen::Vector3f mass_center;

  feature_extractor.getMomentOfInertia (moment_of_inertia);
  feature_extractor.getEccentricity (eccentricity);
  feature_extractor.getAABB (min_point_AABB, max_point_AABB);
//  feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
  feature_extractor.getEigenValues (major_value, middle_value, minor_value);
  feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
  feature_extractor.getMassCenter (mass_center);   //质心  中心

   std::cout<<"min_point_AABB"<<"x= "<<min_point_AABB.x<<"y= "<<min_point_AABB.y<<"z= "<<min_point_AABB.z<<std::endl;
   std::cout<<"max_point_AABB"<<"x= "<<max_point_AABB.x<<"y= "<<max_point_AABB.y<<"z= "<<max_point_AABB.z<<std::endl;

    std::cout << "mass center:(" << mass_center[0] <<","
        <<mass_center[1]<<","<<mass_center[2]<<")"<< std::endl;

//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
   
 // viewer.removeAllShapes();
// viewer.removeAllPointClouds();
  // viewer.removePointCloud("sample cloud",0);
  // viewer.removeShape("AABB",0);
  // viewer.removeShape("OBB",0);
  // viewer.removeShape("major eigen vector",0);
  // viewer.removeShape("middle eigen vector",0);
  // viewer.removeShape("minor eigen vector",0);
 // viewer.updatePointCloud(cloud, "sample cloud");

  viewer.setBackgroundColor (1, 1, 1);   //白
 // viewer.addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 200, 0, 0);  //点云颜色
  viewer.addPointCloud<pcl::PointXYZ> (cloud, single_color, std::to_string(j)+"sample cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, std::to_string(j)+"sample cloud");
  viewer.addCoordinateSystem (3.0);
 // viewer.initCameraParameters ();


  viewer.addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 0.0, 0.0, 200.0, std::to_string(j)+"AABB");

// Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
// Eigen::Quaternionf quat (rotational_matrix_OBB);
 // viewer.addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, std::to_string(j)+"OBB");

  pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
  pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
  pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
  pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
  viewer.addLine (center, x_axis, 1.0f, 0.0f, 0.0f, std::to_string(j)+"major eigen vector");
  viewer.addLine (center, y_axis, 0.0f, 1.0f, 0.0f, std::to_string(j)+"middle eigen vector");
  viewer.addLine (center, z_axis, 0.0f, 0.0f, 1.0f, std::to_string(j)+"minor eigen vector");
}


void cluster_extraction_oujilide(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
      // 55555555555555555555555555555555555555555555555555
  // *************************欧几里得聚类提取
  //  得到一堆一堆的点集
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (1.0); // 0.5m
  ec.setMinClusterSize (4);
  ec.setMaxClusterSize (10000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  int j=0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {   
    j++;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
           cloud_cluster->points.push_back (cloud->points[*pit]); // 得到 点集


    std::cout << "points_cloud have " <<  cloud_cluster->points.size()
            << " data points from  cloud_cluster"
            << std::endl;
    // for(int k=0;k<cloud_cluster->points.size();k++)
    // {
    //   std::cout<<"cluster "<<k<<"z="<<cloud_cluster->points[k].z<<std::endl;
    // }

    // pcl::compute3DCentroid(*cloud_cluster, centroid_cluster);  //计算重心
    // std::cout << "XYZ centroid:(" << centroid_cluster[0] <<","
    //     <<centroid_cluster[1]<<","<<centroid_cluster[2]<<")"<< std::endl;

       draw_kuang_cloud(cloud_cluster,j);
        //   viewer.showCloud(cloud_cluster);
  }
  j=0;
}




// void  viewer_origin_point(pcl::visualization::PCLVisualizer& viewer)
// {
//     // 显示聚类的重心
//     viewer.setBackgroundColor (0.0, 0.0, 0.0);
//     pcl::PointXYZ o;
//     // o.x = centroid_cluster[0];
//     // o.y = centroid_cluster[1];
//     // o.z = centroid_cluster[2];

//     o.x = 0;
//     o.y = 0;
//     o.z = 0;

//     viewer.removeShape("sphere",0);  //注销ID
//     viewer.addSphere (o, 0.3,"sphere", 0);     // r=1m
//   //  std::cout << "i only run once" << std::endl;   
// }

void  viewer_origin_point()
{
    // 显示聚类的重心
   // viewer.setBackgroundColor (0.0, 0.0, 0.0);
    pcl::PointXYZ o;
    // o.x = centroid_cluster[0];
    // o.y = centroid_cluster[1];
    // o.z = centroid_cluster[2];

    o.x = 0;
    o.y = 0;
    o.z = 0;

    viewer.removeShape("sphere",0);  //注销ID
    viewer.addSphere (o, 0.3,"sphere", 0);     // r=1m
  //  std::cout << "i only run once" << std::endl;   
}


void simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  viewer.setBackgroundColor (0, 0, 0);    //white
  viewer.addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
  viewer.addCoordinateSystem (10.0);
  viewer.initCameraParameters ();
}


void  customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer  viewer ("3D Viewer");
  viewer.setBackgroundColor (1, 1, 1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 200, 0, 0);  //点云颜色
  viewer.addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer.addCoordinateSystem (3.0);  //坐标
  viewer.initCameraParameters ();

 //viewer.removeAllPointClouds();

}

void rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  viewer.setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer.addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer.addCoordinateSystem (1.0);
  viewer.initCameraParameters ();
}

#endif