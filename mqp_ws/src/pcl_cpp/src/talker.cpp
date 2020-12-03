#include <ros/ros.h>
#include <string>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl-1.8/pcl/filters/voxel_grid.h>
#include <pcl-1.8/pcl/filters/conditional_removal.h>
#include <pcl-1.8/pcl/surface/on_nurbs/fitting_curve_pdm.h>
#include <pcl-1.8/pcl/surface/on_nurbs/triangulation.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.8/pcl/2d/morphology.h>
#include <pcl-1.8/pcl/point_types_conversion.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

ros::Publisher pub;
ros::Publisher pcl_pub;
//pcl::visualization::PCLVisualizer viewer("Curve Fitting 3D");

void publishPclPath(nav_msgs::Path path) {
    pcl_pub.publish(path);
}

void createPath(nav_msgs::Path &path, geometry_msgs::Pose poses[], int size) {
    for (unsigned i = 0; i<size; i++) {
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.pose = poses[i];
        poseStamped.header.frame_id = "/world";
        path.poses.push_back(poseStamped);
    }
    path.header.frame_id="/world";
}

void createPoses(pcl::PointCloud<pcl::PointXYZRGB>::Ptr curve_filtered, geometry_msgs::Pose *poses) {
    int counter = 0;
    for (const auto &p: curve_filtered->points) {
        if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z)) {
            geometry_msgs::Pose new_pose;
            new_pose.position.x = p.x*(-1);
            new_pose.position.y = p.y;
            new_pose.position.z = p.z-1;
            new_pose.orientation.w = 0.5;
            new_pose.orientation.x = 0.5;
            new_pose.orientation.y = 0.5;
            new_pose.orientation.z = 0.5;
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

//void VisualizeCurve(ON_NurbsCurve &curve, double r, double g, double b, bool show_cps) {
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::on_nurbs::Triangulation::convertCurve2PointCloud(curve, cloud, 8);
//
//    for (std::size_t i = 0; i < cloud->size() - 1; i++) {
//        pcl::PointXYZRGB &p1 = cloud->at(i);
//        pcl::PointXYZRGB &p2 = cloud->at(i + 1);
//        std::ostringstream os;
//        os << "line_" << r << "_" << g << "_" << b << "_" << i;
//        viewer.addLine<pcl::PointXYZRGB>(p1, p2, r, g, b, os.str());
//    }
//
//    if (show_cps) {
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cps(new pcl::PointCloud<pcl::PointXYZ>);
//        for (int i = 0; i < curve.CVCount(); i++) {
//            ON_3dPoint cp;
//            curve.GetCV(i, cp);
//
//            pcl::PointXYZ p;
//            p.x = float(cp.x);
//            p.y = float(cp.y);
//            p.z = float(cp.z);
//            cps->push_back(p);
//        }
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(cps, 255 * r, 255 * g, 255 * b);
//        viewer.addPointCloud<pcl::PointXYZ>(cps, handler, "cloud_cps");
//    }
//}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg) {

// load rod type point cloud and convert to pcl::PointCloud<pcl::PointXYZRGB>
    pcl::PCLPointCloud2 pcl_pc2;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGBA> temp_cloud;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, temp_cloud);

    PointCloudXYZRGBAtoXYZRGB(temp_cloud, *cloud);

// create object to store filtered pontcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

// build the condition
    int rMax = 60;
    int rMin = 0;
    int gMax = 30;
    int gMin = 0;
    int bMax = 30;
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
    condrem.filter(*cloud_filtered);

    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg(*cloud_filtered, cloud2);
    pub.publish(cloud2);

    // Transform to world
    ros::Time now = ros::Time::now();
    tf::TransformListener listener;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr world_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    tf::StampedTransform transform;
    try {
        world_filtered = cloud_filtered;
//        listener.waitForTransform("/panda_link0", "/camera_link",
//                                  now, ros::Duration(3.0));
//        listener.lookupTransform("/panda_link0", "/camera_link",
//                                 ros::Time(0), transform);
//        pcl_ros::transformPointCloud(*cloud_filtered, *world_filtered, (transform.inverse()) * transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("Unable to perform Look up error: %s", ex.what());
        ros::Duration(1.0).sleep();
    }

    // convert to NURBS data structure
    pcl::on_nurbs::NurbsDataCurve data;
    PointCloud2Vector2d(world_filtered, data.interior);

//    viewer.setSize(1200, 1000);
//    viewer.addPointCloud<pcl::PointXYZRGB>(world_filtered, "world_filtered");
    // #################### CURVE PARAMETERS #########################
    unsigned order(3);
    unsigned n_control_points(200);

    pcl::on_nurbs::FittingCurve::Parameter curve_params;
    curve_params.smoothness = 5;
    // #################### CURVE FITTING #########################
    ON_NurbsCurve curve = pcl::on_nurbs::FittingCurve::initNurbsCurvePCA(order, data.interior, n_control_points);

    pcl::on_nurbs::FittingCurve fit(&data, curve);
    fit.assemble(curve_params);
    fit.solve();
    fit.refine();

    unsigned resolution(10);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr curve_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::on_nurbs::Triangulation::convertCurve2PointCloud(fit.m_nurbs, curve_filtered, resolution);
    nav_msgs::Path path;
    int size = curve_filtered->points.size();
    geometry_msgs::Pose poses[size];
    createPoses(curve_filtered, poses);
    createPath(path, poses, size);
    publishPclPath(path);

    // visualize
//    VisualizeCurve(fit.m_nurbs, 0.0, 0.0, 1.0, false);
//    viewer.spin();
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
    ros::Subscriber sub = nh.subscribe("camera/depth/points", 1, cloud_cb);
    // Spin
    ros::spin();
}



