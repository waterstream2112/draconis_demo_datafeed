#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/tf.h>
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
    std::string initTransformFrameId;
    double leafSizeDownSample;

    ros::Time prevCycleTime;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener* tfListener;
    Eigen::Transform <float, 3, Eigen::Affine> l515TransformMatrix;
    geometry_msgs::TransformStamped odomTransform;
    sensor_msgs::PointCloud2ConstPtr receivedCloud;
    nav_msgs::OdometryConstPtr currentOdom;

    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

    ros::Subscriber cloudSub;
    ros::Subscriber odomSub;

    ros::Publisher cloudPub;


public:
    DataForObstacleDetectionNode(ros::NodeHandle &n)
    {
        nh = n;

        //--- Read params
        std::string topicCloudIn = readParam<std::string>(nh, "topic_cloud_in");
        std::string topicOdomIn = readParam<std::string>(nh, "topic_odom_in");
        std::string topicCloudOut = readParam<std::string>(nh, "topic_cloud_out");

        cloudOutFrameId = readParam<std::string>(nh, "cloud_out_frame_id");
        samplingPeriod = ros::Duration(1.0 / readParam<double>(nh, "sampling_rate")); 
        leafSizeDownSample = readParam<double>(nh, "leaf_size_down_sample");

        initTransformFrameId = readParam<std::string>(nh, "init_transform_frame_id"); 

        double initFrameRotY = readParam<double>(nh, "init_frame_rot_y"); 
        double initFrameRotZ = readParam<double>(nh, "init_frame_rot_z");
        double initFrameTransX = readParam<double>(nh, "init_frame_trans_x");
        double initFrameTransZ = readParam<double>(nh, "init_frame_trans_z");
        double initFrameRotX = readParam<double>(nh, "init_frame_rot_x");

        //--- Initialize others
        prevCycleTime = ros::Time(1);

        tfListener = new tf2_ros::TransformListener(tfBuffer);

        l515TransformMatrix = Eigen::Transform <float, 3, Eigen::Affine>::Identity() ;
        // l515TransformMatrix.translate( Eigen::Vector3f (0.0f, 0.0f, 0.55f) ) ;
        // l515TransformMatrix.rotate( Eigen::AngleAxisf (M_PI * initFrameRotY / 180 , Eigen::Vector3f::UnitY () ) ) ;
        // l515TransformMatrix.rotate( Eigen::AngleAxisf (M_PI * initFrameRotZ / 180, Eigen::Vector3f::UnitZ () ) ) ;
        // l515TransformMatrix.rotate( Eigen::AngleAxisf (M_PI * initFrameRotX / 180, Eigen::Vector3f::UnitX () ) ) ;


        // Eigen::Transform <float, 3, Eigen::Affine> tf_L515_Velodyne


        //--- Initialize synchronizer
        // message_filters::Subscriber<nav_msgs::Odometry> odomFilter(nh, topicOdomIn, 10);
        // message_filters::Subscriber<sensor_msgs::PointCloud2> cloudFilter(nh, topicCloudIn, 10);

        // MySyncPolicy syncPolicy(MySyncPolicy(5));
        // syncPolicy.setMaxIntervalDuration(ros::Duration(0.1));
        // sync.reset(new Sync(MySyncPolicy(5), odomFilter, cloudFilter));  
        // sync->setMaxIntervalDuration(ros::Duration(0.5));    
        // sync->registerCallback(boost::bind(&DataForObstacleDetectionNode::syncCallback, this, _1, _2));


        //--- Initialize Subscribers
        cloudSub = nh.subscribe(topicCloudIn, 
                                5, 
                                &DataForObstacleDetectionNode::receiveCloudCallback, 
                                this, 
                                ros::TransportHints().tcpNoDelay());

        // odomSub = nh.subscribe("/multijackal_01/odom", 
        //                         5, 
        //                         &DataForObstacleDetectionNode::odomCallback, 
        //                         this, 
        //                         ros::TransportHints().tcpNoDelay());

        
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


    void syncCallback(const nav_msgs::OdometryConstPtr &odomPtr, const sensor_msgs::PointCloud2ConstPtr &cloudPtr)
    {
        //--- Only process if duration reach setting rate
        // ros::Duration period = ros::Time::now() - prevCycleTime;

        // if (period < samplingPeriod)
        // {
        //     return;
        // }
        
        // ROS_INFO("syncCallback, period=%0.3f", period.toSec());
        // prevCycleTime = ros::Time::now();

        ROS_INFO("syncCallback, odom stamp=%0.2f, cloud stamp=%0.2f", odomPtr->header.stamp, cloudPtr->header.stamp);


        //--- process pointcloud
        cloudHandler(cloudPtr);

    }


    void odomCallback(const nav_msgs::OdometryConstPtr &odomPtr)
    {
        
        tf::Quaternion q(
            odomPtr->pose.pose.orientation.x,
            odomPtr->pose.pose.orientation.y,
            odomPtr->pose.pose.orientation.z,
            odomPtr->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);


        ROS_INFO("time: %0.2f, yaw: %0.2f", ros::Time::now().toSec(), yaw);

        currentOdom = odomPtr;

    }


    void receiveCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloudPtr)
    {
        //--- Only process if duration reach setting rate
        ros::Duration period = ros::Time::now() - prevCycleTime;

        if (period < samplingPeriod)
        {
            return;
        }
        
        ROS_INFO("receiveCloudCallback, period=%0.3f", period.toSec());

        prevCycleTime = ros::Time::now();

        ROS_INFO("receiveCloudCallback, cloud frame: %s", cloudPtr->header.frame_id.c_str());

        
        
        cloudHandler(cloudPtr);


        //----------------
        // if (currentOdom == NULL)
        //     return;

        // receivedCloud = cloudPtr;

        // pcl::PointCloud<pcl::PointXYZ>::Ptr processedCloudOut(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>);

        // pcl::fromROSMsg(*receivedCloud, *cloudIn);

        // downSample(cloudIn);
        // filterNoise(cloudIn);

        // odomTransform.transform.translation.x = currentOdom->pose.pose.position.x;
        // odomTransform.transform.translation.y = currentOdom->pose.pose.position.y;
        // odomTransform.transform.translation.z = currentOdom->pose.pose.position.z;
        // odomTransform.transform.rotation = currentOdom->pose.pose.orientation;

        // Eigen::Matrix4d matrix = tf2::transformToEigen(odomTransform).matrix();

        // pcl::transformPointCloud(*cloudIn, *processedCloudOut, matrix);

        // sensor_msgs::PointCloud2 cloudOut;
        // pcl::toROSMsg(*processedCloudOut, cloudOut);

        // cloudOut.header.frame_id = cloudOutFrameId;
        // cloudOut.header.stamp = receivedCloud->header.stamp;

        // cloudPub.publish(cloudOut);

    }


    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &cloudMsg)
    {
        
        //--- Get cloud msg and convert to pcl pointcloud
        ROS_INFO("cloudHandler-1");

        pcl::PointCloud<pcl::PointXYZ>::Ptr processedCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPclIn(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromROSMsg(*cloudMsg, *cloudPclIn);


        //--- Down sample and Filter the pointcloud
        ROS_INFO("cloudHandler-2");

        downSample(cloudPclIn);
        filterNoise(cloudPclIn);


        //--- Do some more transform before go through tf transform
        ROS_INFO("cloudHandler-3");

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloudOut(new pcl::PointCloud<pcl::PointXYZ>);
        // double xMin=0, xMax=4;
        // double yMin=-2.5, yMax=2.5;
        // double zMin=0.0, zMax=0.5;
        
        // for (size_t i = 0; i < cloudPclIn->points.size(); i++)
        // {
        //     Eigen::Vector4f point;
        //     point[0] = cloudPclIn->points[i].x;
        //     point[1] = cloudPclIn->points[i].y;
        //     point[2] = cloudPclIn->points[i].z;
        //     point[3] = 1.0f;

        //     Eigen::Vector4f newPoint;
        //     newPoint = l515TransformMatrix * point;

        //     Only get points within limits
        //     if (((xMin < newPoint[0]) && (newPoint[0] < xMax)) &&
        //         ((yMin < newPoint[1]) && (newPoint[1] < yMax)) &&
        //         ((zMin < newPoint[2]) && (newPoint[2] < zMax)))
        //     {
        //         transformedCloudOut->push_back(pcl::PointXYZ(newPoint[0], newPoint[1], newPoint[2]));
        //     }
        // }

        transformedCloudOut = cloudPclIn;

        ROS_INFO("cloudHandler: Time0=%0.2f, cloudMsg=%0.5f", ros::Time::now().toSec(), cloudMsg->header.stamp.toSec());

        //--- Do tf transform
        geometry_msgs::TransformStamped transform;
        try {
            
            transform = tfBuffer.lookupTransform(cloudOutFrameId, cloudMsg->header.frame_id, ros::Time(0));
            // transform = tfBuffer.lookupTransform(cloudOutFrameId, cloudMsg->header.frame_id, cloudMsg->header.stamp);
            ROS_INFO("target frame %s", cloudOutFrameId.c_str());
            ROS_INFO("source frame %s", cloudMsg->header.frame_id.c_str());
            ROS_INFO("transform stamp %0.5f", transform.header.stamp.toSec());
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }

        ROS_INFO("cloudHandler-4");

        Eigen::Matrix4d matrix = tf2::transformToEigen(transform).matrix();

        pcl::PointCloud<pcl::PointXYZ>::Ptr processedCloudOut(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*transformedCloudOut, *processedCloudOut, matrix);

        
        //--- Publish the processed pointcloud
        ROS_INFO("cloudHandler-5");

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
        vg.setLeafSize (leafSizeDownSample, leafSizeDownSample, leafSizeDownSample);
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

