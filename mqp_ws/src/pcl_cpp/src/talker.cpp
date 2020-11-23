#include <ros/ros.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <string>
#include <pcl/surface/on_nurbs/fitting_curve_pdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>
#include <pcl/visualization/pcl_visualizer.h>

ros::Publisher pub;
pcl::visualization::PCLVisualizer viewer("Curve Fitting 3D");

inline void PointCloudXYZRGBAtoXYZRGB(pcl::PointCloud<pcl::PointXYZRGBA> &in, pcl::PointCloud<pcl::PointXYZRGB> &out) {
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


inline void PointCloud2Vector2d(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data) {
    for (const auto &p : *cloud) {
        if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z)) {
            data.emplace_back(p.x, p.y, p.z);
        }
    }
}

void
VisualizeCurve(ON_NurbsCurve &curve, double r, double g, double b, bool show_cps) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::on_nurbs::Triangulation::convertCurve2PointCloud(curve, cloud, 8);

    for (std::size_t i = 0; i < cloud->size() - 1; i++) {
        pcl::PointXYZRGB &p1 = cloud->at(i);
        pcl::PointXYZRGB &p2 = cloud->at(i + 1);
        std::ostringstream os;
        os << "line_" << r << "_" << g << "_" << b << "_" << i;
        viewer.addLine<pcl::PointXYZRGB>(p1, p2, r, g, b, os.str());
    }

    if (show_cps) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cps(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < curve.CVCount(); i++) {
            ON_3dPoint cp;
            curve.GetCV(i, cp);

            pcl::PointXYZ p;
            p.x = float(cp.x);
            p.y = float(cp.y);
            p.z = float(cp.z);
            cps->push_back(p);
        }
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(cps, 255 * r, 255 * g, 255 * b);
        viewer.addPointCloud<pcl::PointXYZ>(cps, handler, "cloud_cps");
    }
}

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
    int rMax = 255;
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



    // convert to NURBS data structure
    pcl::on_nurbs::NurbsDataCurve data;
    PointCloud2Vector2d(cloud_filtered, data.interior);

    viewer.setSize(800, 600);
    viewer.addPointCloud<pcl::PointXYZRGB>(cloud_filtered, "cloud_filtered");
    // #################### CURVE PARAMETERS #########################
    unsigned order(3);
    unsigned n_control_points(20);

    pcl::on_nurbs::FittingCurve::Parameter curve_params;
    curve_params.smoothness = 0.000001;

    // #################### CURVE FITTING #########################
    ON_NurbsCurve curve = pcl::on_nurbs::FittingCurve::initNurbsCurvePCA(order, data.interior, n_control_points);

    pcl::on_nurbs::FittingCurve fit(&data, curve);
    fit.assemble(curve_params);
    fit.solve();

//  ROS_INFO("data  %s",fit.m_data);
  ROS_INFO("Is valid: %d", fit.m_nurbs.IsValid());

    // visualize
    VisualizeCurve(fit.m_nurbs, 1.0, 0.0, 0.0, false);
    viewer.spin();

    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg(*cloud_filtered, cloud2);
    pub.publish(cloud2);


}

int
main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "frank_camera_processing");
    ros::NodeHandle nh;
    ROS_INFO("PCL Version: %s", PCL_VERSION_PRETTY);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("camera/depth/points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

    // Spin
    ros::spin();
}



