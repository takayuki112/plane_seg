#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

ros::Publisher ground_pub;
ros::Publisher objects_pub;

ros::Publisher ground_mask_pub;
ros::Publisher objects_mask_pub;

char input_topic[] = "/zed/depth/points";

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    // Convert ROS PointCloud2 message to PCL PointCloud type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // THE SEGMENTATION MODEL
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.2);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    //Get coefficients of ground plane ~
    double a = coefficients->values[0];
    double b = coefficients->values[1];
    double c = coefficients->values[2];
    double d = coefficients->values[3];

    // Extract ground points
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract_ground;
    extract_ground.setInputCloud(cloud);
    extract_ground.setIndices(inliers);
    extract_ground.setNegative(false);  // Extract the inliers (ground points)
    extract_ground.filter(*ground_cloud);

    // Extract non-ground points (objects)
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    extract_ground.setNegative(true);  // Extract the outliers (non-ground points)
    extract_ground.filter(*objects_cloud);

    // Publish segmented ground plane
    sensor_msgs::PointCloud2 ground_msg;
    pcl::toROSMsg(*ground_cloud, ground_msg);
    ground_msg.header = cloud_msg->header;
    ground_pub.publish(ground_msg);

    // Publish objects point cloud
    sensor_msgs::PointCloud2 objects_msg;
    pcl::toROSMsg(*objects_cloud, objects_msg);
    objects_msg.header = cloud_msg->header;
    objects_pub.publish(objects_msg);
    
    std::cout << "Ground points: " << ground_cloud->size() << std::endl;
    std::cout << "Objects points: " << objects_cloud->size() << std::endl << std::endl;

}

int main(int argc, char** argv){
    
    ros::init(argc, argv, "ground_plane_segmentation_node");
    ros::NodeHandle nh;

    // Subscribe to the point cloud topic
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(input_topic, 1, cloudCallback);

    // Create publishers for the segmented point clouds
    ground_pub = nh.advertise<sensor_msgs::PointCloud2>("/output_ground_topic", 1, true);
    objects_pub = nh.advertise<sensor_msgs::PointCloud2>("/output_objects_topic", 1, true);

    ground_mask_pub = nh.advertise<sensor_msgs::PointCloud2>("ground_mask", 1);
    objects_mask_pub = nh.advertise<sensor_msgs::PointCloud2>("objects_mask", 1);

    ros::spin();
    return 0;
}
