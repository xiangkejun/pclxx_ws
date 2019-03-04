// xx
//  PCLVisualizer 实时显示

#include <pcl_ros/pcl_xx.h>
//#include <pcl_ros/pcl_xx1.h>
#include <ros/ros.h>
//pcl::visualization::CloudViewer viewer("Cloud Viewer");


void   cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
 // 创建一个输出的数据格式
  sensor_msgs::PointCloud2 output;  //ROS中点云的数据格式
  sensor_msgs::PointCloud2 output_deal;  //处理后的ros点云格式

  //对数据进行处理
  output = *input;
 pcl::fromROSMsg(output,*cloud);


  std::cout << "points_cloud have " << cloud->width * cloud->height
            << " data points from velodyne_points "
            << std::endl;

//   for(size_t i=0; i < cloud->points.size(); i++)
//   {
//         std::cout << " x=  " << cloud->points[i].x
//               << " y =  "    << cloud->points[i].y
//               << "  z = "    << cloud->points[i].z << std::endl;
//   }

      condition_duo_filter(cloud);   //初步处理

    //  plane_touying_filter(cloud_condition);  //平面投影


    //    segmentation_line(cloud_planar);   //直线分割

     //  lines_filter(cloud_rest_line);    //滤掉直线外的点

    cluster_extraction_oujilide(cloud_condition);   //聚类提取
 //     draw_kuang_cloud(cloud_condition);
//处理后的点云 cloud  cloud_condition   cloud_planar    cloud_rest_line 
//    cloud_linenei  


  // viewer.spinOnce (100);    //  PCLVisualizer显示用
   //   viewer.showCloud(cloud); 
    //      //This will only get called once
    //  viewer.runOnVisualizationThreadOnce (viewer_origin_point);
}


int main (int argc, char** argv)
{ 
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/velodyne_points", 10, cloud_cb);
  // ros::Rate loop_rate(100);
   viewer.initCameraParameters ();
  // Spin
    ros::spin ();

  // ros::AsyncSpinner spinner(1);  //给节点开辟多进程
  // spinner.start();
  // // if we close application, still wait for ros to shutdown
  // ros::waitForShutdown();

}
