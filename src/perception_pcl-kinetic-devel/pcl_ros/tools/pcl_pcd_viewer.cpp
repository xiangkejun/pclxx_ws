#include <pcl_ros/pcl_xx.h>
#include <pcl/console/parse.h>

//pcl::visualization::PCLVisualizer viewer;
//pcl::visualization::CloudViewer viewer("Cloud Viewer");

int main (int argc, char** argv)
{ 
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // ------------------------------------------------------------------
  // -----Read pcd file or create example point cloud if not given-----
  // ------------------------------------------------------------------
  std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");
  if (!pcd_filename_indices.empty ())
  {
    std::string filename = argv[pcd_filename_indices[0]];
    if (pcl::io::loadPCDFile (filename, *cloud) == -1)
    {
      std::cout << "Was not able to open file \""<<filename<<"\".\n";
    //  printUsage (argv[0]);
      return 0;
    }
 
  }


  // if (pcl::io::loadPCDFile<pcl::PointXYZ> ("b10.pcd", *cloud) == -1) //* load the file
  // {
  //   PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
  //   return (-1);
  // }  
  // std::cout << "points_cloud have " << cloud->width * cloud->height
  //           << " data points from velodyne_points "
  //           << std::endl;


       condition_duo_filter(cloud);   //初步处理

     //  plane_touying_filter(cloud_condition);  //平面投影

      statiscal_removal(cloud_condition);   //统计法移除离群点

  // for(size_t i=0; i < cloud_planar->points.size(); i++)
  // {
  //       std::cout << " x=  " << cloud_planar->points[i].x
  //             << " y =  "    << cloud_planar->points[i].y
  //             << "  z = "    << cloud_planar->points[i].z << std::endl;
  // }


      //    segmentation_line(cloud_statiscal_removal);   //直线分割

      //   lines_filter(cloud_rest_line);    //滤掉直线外的点

      viewer_origin_point();

      cluster_extraction_oujilide(cloud_statiscal_removal);   //聚类提取 添加矩形框
     //       draw_kuang_cloud(cloud_condition);
//处理后的点云 cloud  cloud_condition   cloud_planar   cloud_statiscal_removal  cloud_rest_line 
//    cloud_linenei  
    
   //  viewer.showCloud(cloud_condition); 

         //This will only get called once
       //  viewer.runOnVisualizationThreadOnce (viewer_origin_point);

//  simpleVis(cloud_condition);
// customColourVis(cloud_statiscal_removal);   //cloud_condition  单独显示用
 // customColourVis(cloud);   //cloud_condition  单独显示用

  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

ros::spin ();


 
}
