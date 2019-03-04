#ifndef PCL_xx1__
#define	PCL_xx1__

/************************************************
关于如何使用PCL在ROS 中，实现简单的数据转化
时间：2017.3.31

****************************************************/

#include <iostream>
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

/************************************************
关于如何使用PCL在ROS 中，实现简单的数据转化
时间：2017.3.31

****************************************************/

#include <iostream>
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
#include <pcl/kdtree/kdtree.h>



typedef pcl::PointXYZ PointT;

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_condition (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_planar (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_statiscal_removal(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<PointT>::Ptr cloud_rest_line (new pcl::PointCloud<PointT> ());
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_linenei (new pcl::PointCloud<pcl::PointXYZ>);

pcl::ModelCoefficients::Ptr coefficients_seg (new pcl::ModelCoefficients);

pcl::ExtractIndices<PointT> extract;

Eigen::Vector4f  centroid_cluster;

void condition_duo_filter(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud)
{
//       11111111111111111111111111111111111111111111111111
// ************ 条件滤波 ***********************************************/////
///// 两个条件 1）  -3<z<3;    2)   x*x + y*y -64 <=0
    pcl::ConditionAnd<pcl::PointXYZI>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZI> ());
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new
        pcl::FieldComparison<pcl::PointXYZI> ("z", pcl::ComparisonOps::GT, -0.5)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new
        pcl::FieldComparison<pcl::PointXYZI> ("z", pcl::ComparisonOps::LT, 2.0)));

    // 做一个半圆    x>0.5
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new
    pcl::FieldComparison<pcl::PointXYZI> ("x", pcl::ComparisonOps::GT, -0.5)));  
   
      //(p'Ap + 2v'p + c [OP] 0)
        // x*x + y*y - 100 <=0   半径 10m    得到一个圆柱
     Eigen::Matrix3f A;A(0, 0) = 1;A(0, 1) = 0;A(0, 2) = 0;A(1, 0) = 0;A(1, 1) = 1;A(1, 2) = 0;A(2, 0) = 0;A(2, 1) = 0;A(2, 2) = 0; 
     Eigen::Vector3f v;v(0)=0;v(1)=0;v(2)=0; 
     float c = -900;//radius^2 
    range_cond->addComparison(pcl::TfQuadraticXYZComparison < pcl::PointXYZI >::Ptr (new pcl::TfQuadraticXYZComparison < pcl::PointXYZI >(pcl::ComparisonOps::LT , A, v, c))); 



    //  //(p'Ap + 2v'p + c [OP] 0)
    //     // k*x+ y <0     做一个任意角度的半圆柱
    //  Eigen::Matrix3f A1;A1(0, 0) = 0;A1(0, 1) = 0;A1(0, 2) = 0;A1(1, 0) = 0;A1(1, 1) = 0;A1(1, 2) = 0;A1(2, 0) = 0;A1(2, 1) = 0;A1(2, 2) = 0; 
    //  Eigen::Vector3f v1;v1(0)=0;v1(1)=-0.5;v1(2)=0;   //sqrt(3)= 1.732    
    //  float c1 = 2;   //  y+2<0
    // range_cond->addComparison(pcl::TfQuadraticXYZComparison < pcl::PointXYZ >::Ptr (new pcl::TfQuadraticXYZComparison < pcl::PointXYZ >(pcl::ComparisonOps::LT , A1, v1, c1))); 



    // bobjectuild the filter
    pcl::ConditionalRemoval<pcl::PointXYZI> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (cloud);
   // condrem.setKeepOrganized(true);
    // apply filter
    condrem.filter (*cloud_condition);
   // viewer.showCloud(cloud_condition);

}




#endif