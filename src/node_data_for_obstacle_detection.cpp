
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/passthrough.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include "pcl_ros/point_cloud.h"

#include <iostream>
#include <fstream>

class DataForObstacleDetectionNode
{
private:
    ros::NodeHandle nh;

    ros::Duration samplingPeriod;
    std::string cloudOutFrameId;

    ros::Time prevCycleTime;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener* tfListener;

    ros::Subscriber cloudSub;

    ros::Publisher cloudPub;


public:
    DataForObstacleDetectionNode(ros::NodeHandle &n)
    {
        nh = n;

        //--- Read params
        std::string topicCloudIn = readParam<std::string>(nh, "topic_cloud_in");
        std::string topicCloudOut = readParam<std::string>(nh, "topic_cloud_out");

        cloudOutFrameId = readParam<std::string>(nh, "cloud_out_frame_id");
        samplingPeriod = ros::Duration(1.0 / readParam<double>(nh, "sampling_rate"));

        //--- Initialize others
        prevCycleTime = ros::Time(1);

        tfListener = new tf2_ros::TransformListener(tfBuffer);

        //--- Initialize Subscribers
        cloudSub = nh.subscribe(topicCloudIn, 
                                5, 
                                &DataForObstacleDetectionNode::receiveCloudCallback, 
                                this, 
                                ros::TransportHints().tcpNoDelay());

        
        //--- Initialize Publishers
        cloudPub = nh.advertise<sensor_msgs::PointCloud2>(topicCloudOut, 5);
    }

    ~DataForObstacleDetectionNode()
    {
        
    }
    

    template <typename T>
    T readParam(ros::NodeHandle &n, std::string name){
        T ans;
        if (n.getParam(name, ans)){
            ROS_INFO_STREAM("Loaded " << name << ": " << ans);
        } else {
            ROS_ERROR_STREAM("Failed to load " << name);
            n.shutdown();
        }
        return ans;
    }


    void receiveCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloudPtr)
    {
        //--- Only process if duration reach setting rate
        ros::Duration period = ros::Time::now() - prevCycleTime;

        if (period < samplingPeriod)
            return;

        prevCycleTime = ros::Time::now();

        ROS_INFO("receiveCloudCallback");
        ROS_INFO("cloud frame: %s", cloudPtr->header.frame_id.c_str());

        cloudHandler(cloudPtr);

    }


    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &cloudMsg)
    {
        ROS_INFO("cloudHandler-1");
        //--- Get cloud msg and convert to pcl pointcloud
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr processedCloud(new pcl::PointCloud<pcl::PointXYZ>);
        // std::string sourceFrameId = cloudMsg->header.frame_id;

        pcl::fromROSMsg(*cloudMsg, *processedCloud);

        ROS_INFO("cloudHandler-2");

        //--- Down sample and Filter the pointcloud
        downSample(processedCloud);
        filterNoise(processedCloud);

        ROS_INFO("cloudHandler-3");

        //--- Do tf transform
        geometry_msgs::TransformStamped transform;
        try {
            transform = tfBuffer.lookupTransform(cloudOutFrameId, cloudMsg->header.frame_id, ros::Time(0));
            ROS_INFO("target frame %s", cloudOutFrameId.c_str());
            ROS_INFO("source frame %s", cloudMsg->header.frame_id.c_str());
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }

        ROS_INFO("cloudHandler-4");

        Eigen::Matrix4d matrix = tf2::transformToEigen(transform).matrix();

        pcl::PointCloud<pcl::PointXYZ>::Ptr processedCloudOut(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*processedCloud, *processedCloudOut, matrix);

        ROS_INFO("cloudHandler-5");

        //--- Publish the processed pointcloud
        sensor_msgs::PointCloud2 cloudOut;
        pcl::toROSMsg(*processedCloudOut, cloudOut);

        cloudOut.header.frame_id = cloudOutFrameId;
        cloudOut.header.stamp = cloudMsg->header.stamp;

        cloudPub.publish(cloudOut);
    }


    void downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr processedCloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud (cloud);
        vg.setLeafSize (0.01f, 0.01f, 0.01f);
        vg.filter (*processedCloud);

        cloud = processedCloud;
    }


    void filterNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr processedCloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud);
        sor.setMeanK (50);
        sor.setStddevMulThresh (1.0);
        sor.filter (*processedCloud);

        cloud = processedCloud;
    }

};


int main(int argc, char** argv) {
    ros::init(argc, argv, "data_for_obstacle_detection_node");
    ros::NodeHandle nh("~");
    DataForObstacleDetectionNode od(nh);
    ros::spin();
    return 0;
}

