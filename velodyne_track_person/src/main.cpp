#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <shape_msgs/SolidPrimitive.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/centroid.h>
#include <v4r/common/miscellaneous.h>
#include <velodyne_track_person/PersonInstance.h>

#include <iostream>
#include <vector>

typedef pcl::PointXYZ PointInT;

ros::Publisher result_pub;
ros::Publisher pose_pub;
ros::Publisher person_pub;

pcl::octree::OctreePointCloudChangeDetector<PointInT> *octree;

// Octree resolution - side length of octree voxels
float resolution = 0.4;
int noise_filter = 50;

int sor_mean_k = 70;
float sor_stddev_mul_th = 1.0;

float ror_rad = 0.05; // 0.8;
int ror_min_heighbors = 30; // 2;

int min_size_thresh = 1500;

void parseCommandLine()
{
    ros::NodeHandle private_node_handle_("~");

    private_node_handle_.getParam("res", resolution);
    private_node_handle_.getParam("noise", noise_filter);
    private_node_handle_.getParam("mean_k", sor_mean_k);
    private_node_handle_.getParam("std_dev_mul", sor_stddev_mul_th);
    private_node_handle_.getParam("rad", ror_rad);
    private_node_handle_.getParam("min_neighbors", ror_min_heighbors);
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
    sensor_msgs::PointCloud2 result_cloud_msg;

    pcl::PointCloud<PointInT>::Ptr cloud (new pcl::PointCloud<PointInT>);

    pcl::fromROSMsg(*input, *cloud);

    // Filtering out noise
    pcl::StatisticalOutlierRemoval<PointInT> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (sor_mean_k); // 50 - default
    sor.setStddevMulThresh (sor_stddev_mul_th);
    sor.filter (*cloud);

    octree->setInputCloud (cloud);
    octree->addPointsFromInputCloud ();

    std::vector<int> newPointIdxVector;

    octree->getPointIndicesFromNewVoxels (newPointIdxVector, noise_filter);

    if(!newPointIdxVector.size())
    {
        ROS_INFO("Person wasn't detected\n");
        return;
    }

    pcl::PointCloud<PointInT>::Ptr person_cloud (new pcl::PointCloud<PointInT>);

    pcl::copyPointCloud(*cloud, newPointIdxVector, *person_cloud);

    ROS_INFO("Person cloud has %d points", (int)person_cloud->points.size());

    pcl::RadiusOutlierRemoval<PointInT> outrem;
    outrem.setInputCloud(person_cloud);
    outrem.setRadiusSearch(ror_rad);
    outrem.setMinNeighborsInRadius (ror_min_heighbors);
    outrem.filter (*person_cloud);

    // Reject too small cloud
    if(person_cloud->points.size() < min_size_thresh)
    {
        ROS_INFO("Person wasn't detected\n");
        return;
    }

    pcl::toROSMsg(*person_cloud, result_cloud_msg);

    result_cloud_msg.header.frame_id = input->header.frame_id;
    result_pub.publish(result_cloud_msg);

    // Get pose of person and publish results
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*person_cloud, centroid);

    ROS_INFO("Person pose: %.3f, %.3f, %.3f", centroid[0], centroid[1], centroid[2]);

    geometry_msgs::Pose pose;
    pose.position.x = centroid[0];
    pose.position.y = centroid[1];
    pose.position.z = centroid[2];

    pose.orientation.w = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = input->header.frame_id;
    pose_msg.pose = pose;

    pose_pub.publish(pose_msg);


    // Calculate bounding box
    float x_len, y_len, z_len; // lenghts of the bounding box

    Eigen::Vector4f min, max;

    pcl::getMinMax3D(*person_cloud, min, max);

    x_len = max[0] - min[0];
    y_len = max[1] - min[1];
    z_len = max[2] - min[2];

    shape_msgs::SolidPrimitive bbox;
    bbox.type = bbox.BOX;
    bbox.dimensions.resize(3);
    bbox.dimensions[0] = x_len;
    bbox.dimensions[1] = y_len;
    bbox.dimensions[2] = z_len;

    velodyne_track_person::PersonInstance person_instance;
    person_instance.pose = pose;
    person_instance.bbox = bbox;
    person_instance.header.frame_id = input->header.frame_id;
    person_instance.header.stamp = ros::Time::now();

    person_pub.publish(person_instance);

    ROS_INFO("Person pose was published\n");

    // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
    octree->switchBuffers ();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velodyne_track_person");

    ros::NodeHandle nh;

    parseCommandLine();

    std::cout << "resolution: " << resolution << "\n";
    std::cout << "noise_filter: " << noise_filter << "\n";
    std::cout << "mean_k: " << sor_mean_k << "\n";
    std::cout << "std_dev_mult: " << sor_stddev_mul_th << "\n";
    std::cout << "radius: " << ror_rad << "\n";
    std::cout << "min_neighbors: " << ror_min_heighbors << "\n";

    ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

    result_pub = nh.advertise<sensor_msgs::PointCloud2>("person_points", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("person_pose", 1);
    person_pub = nh.advertise<velodyne_track_person::PersonInstance>("person_instance", 1);

    octree = new pcl::octree::OctreePointCloudChangeDetector<PointInT> (resolution);

    ros::spin();
}
