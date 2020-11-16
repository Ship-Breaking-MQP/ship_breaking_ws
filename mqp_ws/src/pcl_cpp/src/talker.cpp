#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <string>

ros::Publisher pub;
ros::Publisher pointcloud;



inline void PointCloudXYZRGBAtoXYZRGB(pcl::PointCloud<pcl::PointXYZRGBA>& in, pcl::PointCloud<pcl::PointXYZRGB>& out)
{
  out.header = in.header;
  out.width   = in.width;
  out.height  = in.height;
  out.points.resize(in.points.size());
  for (size_t i = 0; i < in.points.size (); i++)
  {
    out.points[i].x = in.points[i].x;
    out.points[i].y = in.points[i].y;
    out.points[i].z = in.points[i].z;
    out.points[i].r = in.points[i].r;
    out.points[i].g = in.points[i].g;
    out.points[i].b = in.points[i].b;
  }
}



void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& msg)
{

// load rod type point cloud and convert to pcl::PointCloud<pcl::PointXYZRGB>
  pcl::PCLPointCloud2 pcl_pc2;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGBA> temp_cloud;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, temp_cloud);

  PointCloudXYZRGBAtoXYZRGB(temp_cloud, *cloud);

  // for(int i = 0;i < temp_cloud.points.size();i++){
  //  ROS_INFO("Point %d %d\n",i,temp_cloud.points[i].r);
  // }

// create object to store filtered pontcloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>); 

// build the condition
  int rMax = 255;
  int rMin = 0;
  int gMax = 0;
  int gMin = 0;
  int bMax = 0;
  int bMin = 0;
  pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, rMax)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, rMin)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, gMax)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, gMin)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, bMax)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, bMin)));

  // for(int i = 0;i < temp_cloud.points.size();i++){
  //  ROS_INFO("Point %d %d\n",i,temp_cloud.points[i].r);
  // }
  // build the filter
  pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem (color_cond);
  condrem.setInputCloud (cloud);
  condrem.setKeepOrganized(true);
  // apply filter
  condrem.filter (*cloud_filtered);


  sensor_msgs::PointCloud2 cloud2;
//  ROS_INFO("%s",cloud2.header.frame_id.c_str());
  pcl::toROSMsg(*cloud_filtered, cloud2);
//  ROS_INFO("%s",cloud2.header.frame_id.c_str());
  pub.publish (cloud2);


  
  


  // // Container for original & filtered data
  // pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  // pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  // pcl::PCLPointCloud2 cloud_filtered;
  // // Convert to PCL data type
  // pcl_conversions::toPCL(*cloud_msg, *cloud);
  // for(int i = 0;i < cloud->data.size();i++){
  // ROS_INFO("Point %d %d\n",i,cloud->data[i]);
  // }


  // // Perform the actual filtering
  // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  // sor.setInputCloud (cloudPtr);
  // sor.setLeafSize (0.1, 0.1, 0.1);
  // sor.filter (cloud_filtered);

  // // Convert to ROS data type
  // sensor_msgs::PointCloud2 output;
  // pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data
  

  /*


 pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
 pcl::fromROSMsg(*cloud_msg, *rgb_cloud);
 pcl::PCLPointCloud2 cloud_filtered_ros;

 pcl::ConditionalRemoval<pcl::PointXYZRGB> color_filter;

 pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr
     red_condition(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::GT, 200));
 pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
 color_cond->addComparison (red_condition);

 // Build the filter
 color_filter.setInputCloud(rgb_cloud);
 color_filter.setCondition (color_cond);
 color_filter.filter(*cloud_filtered);

  sensor_msgs::PointCloud2 pc2;  
  pcl::PCLPointCloud2::Ptr pcl_pc_2(new pcl::PCLPointCloud2());
  pcl::toPCLPointCloud2 (*cloud_filtered, *pcl_pc_2);
  pcl_conversions::fromPCL( *pcl_pc_2, pc2 );
  pub.publish(pc2);
 

*/
  /*
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);
PCLPointCloud2
  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
 

  // Publish the data
  pub.publish (output);
  */
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "frank_camera_processing");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("camera/depth/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);



  // Spin
  ros::spin ();
}



