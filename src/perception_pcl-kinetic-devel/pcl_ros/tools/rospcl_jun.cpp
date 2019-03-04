#include <pcl_ros/pcl_xx.h>
#include <ros/ros.h>
//pcl::visualization::CloudViewer viewer("Cloud Viewer");

ros::Publisher pub;

void   cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
 // 创建一个输出的数据格式
  sensor_msgs::PointCloud2 output;  //ROS中点云的数据格式
  sensor_msgs::PointCloud2 output_deal;  //处理后的ros点云格式

  output_deal.header.frame_id = "velodyne";   //方便在rviz中显示

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

      plane_touying_filter(cloud_condition);  //平面投影


    //    segmentation_line(cloud_planar);   //直线分割

    //   lines_filter(cloud_rest_line);    //滤掉直线外的点

  //   cluster_extraction_oujilide(cloud_planar);   //聚类提取
//处理后的点云 cloud  cloud_condition   cloud_planar    cloud_rest_line 
//    cloud_linenei  

    //  viewer.showCloud(cloud); 
    //      //This will only get called once
    //  viewer.runOnVisualizationThreadOnce (viewer_origin_point);

  pcl::toROSMsg(*cloud_condition, output_deal);
  pub.publish (output_deal);
}



int main (int argc, char** argv)
{ 

  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, cloud_cb);
   ros::Rate loop_rate(100);



  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output_deal", 1);
  
  // Spin
  ros::spin ();

}
