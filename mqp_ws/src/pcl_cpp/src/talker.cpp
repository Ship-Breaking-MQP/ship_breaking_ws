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
#include <pcl/surface/on_nurbs/fitting_curve_pdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>


ros::Publisher pub;
ros::Publisher pointcloud;



inline void PointCloudXYZRGBAtoXYZRGB(pcl::PointCloud<pcl::PointXYZRGBA>& in, pcl::PointCloud<pcl::PointXYZRGB>& out)
{
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
  int rMin = 130;
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



  // convert to NURBS data structure
  pcl::on_nurbs::NurbsDataCurve data;
  PointCloud2Vector2d (cloud_filtered, data.interior);


   // #################### CURVE PARAMETERS #########################
  unsigned order (3);
  unsigned n_control_points (20);

  pcl::on_nurbs::FittingCurve::Parameter curve_params;
  curve_params.smoothness = 0.000001;

  // #################### CURVE FITTING #########################
  ON_NurbsCurve curve = pcl::on_nurbs::FittingCurve::initNurbsCurvePCA (order, data.interior, n_control_points);

  pcl::on_nurbs::FittingCurve fit (&data, curve);
  fit.assemble (curve_params);
  fit.solve ();

  ROS_INFO("data  %s",fit.m_data.c_str());
  ROS_INFO("nurbs %s",fit.m_nurbs.c_str());



  sensor_msgs::PointCloud2 cloud2;
  ROS_INFO("%s",cloud2.header.frame_id);
  pcl::toROSMsg(*cloud_filtered, cloud2);
  ROS_INFO("%s",cloud2.header.frame_id);
  pub.publish (cloud2);


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



