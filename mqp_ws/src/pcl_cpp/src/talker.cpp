#include <ros/ros.h>
#include <string>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <iostream>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl-1.8/pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/surface/on_nurbs/fitting_curve_pdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>
#include <pcl/2d/morphology.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

ros::Publisher pub;
ros::Publisher pcl_pub;

void publishPclPath(nav_msgs::Path path) {
    ROS_INFO("Publish Path");
    pcl_pub.publish(path);
}

void createPath(nav_msgs::Path &path, geometry_msgs::Pose poses[], int size) {
    for (unsigned i = 0; i<size; i++) {
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.pose = poses[i];
        poseStamped.header.frame_id = "/camera_color_optical_frame";
        path.poses.push_back(poseStamped);
    }
    path.header.frame_id="/camera_color_optical_frame";
}

void createPoses(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& curve_filtered, geometry_msgs::Pose *poses) {
    int counter = 0;
    for (const auto &p: curve_filtered->points) {
        if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z)) {
            geometry_msgs::Pose new_pose;
            new_pose.position.x = p.x;
            new_pose.position.y = p.y;
            new_pose.position.z = p.z;
            new_pose.orientation.w = 1;
            new_pose.orientation.x = 1;
            new_pose.orientation.y = 1;
            new_pose.orientation.z = 1;
            poses[counter] = new_pose;
            counter++;
        }
    }
}

void PointCloudXYZRGBAtoXYZRGB(pcl::PointCloud<pcl::PointXYZRGBA> &in, pcl::PointCloud<pcl::PointXYZRGB> &out) {
    out.header = in.header;
    out.width = in.width;
    out.height = in.height;
    out.points.resize(in.points.size());
    for (size_t i = 0; i < in.points.size(); i++) {
        out.points[i].x = in.points[i].x;
        out.points[i].y = in.points[i].y;
        out.points[i].z = in.points[i].z;
        out.points[i].r = in.points[i].r;
        out.points[i].g = in.points[i].g;
        out.points[i].b = in.points[i].b;
    }
}

void PointCloud2Vector2d(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data) {
    for (const auto &p : *cloud) {
        if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z)) {
            data.emplace_back(p.x, p.y, p.z);
        }
    }
}

void FilterCloudFromCurve(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &world, pcl::PointCloud<pcl::PointXYZRGB> &out, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &curve) {
    float radius = 0.01;
    for (const auto &p : *curve) {
        for (const auto &c : *world) {
            if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z)) {
                if ((p.x <= c.x+radius && p.x >= c.x-radius) &&
                    (p.y <= c.y+radius && p.y >= c.y-radius) &&
                    (p.z <= c.z+radius && p.z >= c.z-radius)) {

                    out.push_back(p);
                    break;
                }
            }
        }
    }
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg) {

// load rod type point cloud and convert to pcl::PointCloud<pcl::PointXYZRGB>
    ROS_INFO("Press enter to continue...");
    getchar();
    pcl::PCLPointCloud2 pcl_pc2;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGBA> temp_cloud;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, temp_cloud);

    PointCloudXYZRGBAtoXYZRGB(temp_cloud, *cloud);

// create object to store filtered pontcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);


    ROS_INFO("Build Condition");
// build the condition
//    int rMax = 60;
//    int rMin = 0;
//    int gMax = 30;
//    int gMin = 0;
//    int bMax = 30;
//    int bMin = 0;
    int rMax = 40;
    int rMin = 0;
    int gMax = 30;
    int gMin = 0;
    int bMax = 40;
    int bMin = 0;
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>());
    color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(
            new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::LT, rMax)));
    color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(
            new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::GT, rMin)));
    color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(
            new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::LT, gMax)));
    color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(
            new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::GT, gMin)));
    color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(
            new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::LT, bMax)));
    color_cond->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(
            new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::GT, bMin)));

    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem(color_cond);
    condrem.setInputCloud(cloud);
    condrem.setKeepOrganized(true);
    // apply filter
    ROS_INFO("Apply Filter");
    condrem.filter(*cloud_filtered);

    sensor_msgs::PointCloud2 cloud2;

    // convert to NURBS data structure
    pcl::on_nurbs::NurbsDataCurve data;
    PointCloud2Vector2d(cloud_filtered, data.interior);


    ROS_INFO("Downsample Curve");
    pcl::on_nurbs::NurbsTools::downsample_random(data.interior, 500);

    // #################### CURVE PARAMETERS #########################
    unsigned order(3);
    unsigned refinement(4);
    unsigned n_control_points(20);

    pcl::on_nurbs::FittingCurve::Parameter curve_params;
    curve_params.smoothness = 5;
    // #################### CURVE FITTING #########################
    ON_NurbsCurve curve = pcl::on_nurbs::FittingCurve::initNurbsCurvePCA(order, data.interior, n_control_points);


    pcl::on_nurbs::FittingCurve fit(&data, curve);
    ROS_INFO("Fit Refinement");
    for (unsigned i = 0; i < refinement; i++) {
        fit.refine();
        fit.assemble(curve_params);
        fit.solve();
    }

    unsigned resolution(1);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr curve_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::on_nurbs::Triangulation::convertCurve2PointCloud(fit.m_nurbs, curve_filtered, resolution);
    ROS_INFO("Filter Curve");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr world_curve_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    FilterCloudFromCurve(cloud_filtered, *world_curve_filtered, curve_filtered);
//    pcl::VoxelGrid<pcl::PointXYZRGB> avg;
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZRGB>);
//    avg.setInputCloud(world_curve_filtered);
//    avg.setLeafSize(0.01f, 0.01f, 0.01f);
//    avg.filter(*cloudFiltered);
    pcl::toROSMsg(*world_curve_filtered, cloud2);
    cloud2.header = msg->header;
    ROS_INFO("%s", cloud2.header.frame_id.c_str());

    ROS_INFO("Publish Cloud");
    pub.publish(cloud2);
    nav_msgs::Path path;
    int size = world_curve_filtered->points.size();
    ROS_INFO("Create Poses");
    geometry_msgs::Pose poses[size];
    createPoses(world_curve_filtered, poses);
    ROS_INFO("Create Path");
    createPath(path, poses, size);
    publishPclPath(path);
}

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "frank_camera_processing");
    ros::NodeHandle nh;
    ROS_INFO("PCL Version: %s", PCL_VERSION_PRETTY);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
    pcl_pub = nh.advertise<nav_msgs::Path>("pcl", 1);
    ros::Rate loop_rate(10);
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, cloud_cb);
    // Spin
    ros::spin();
}



