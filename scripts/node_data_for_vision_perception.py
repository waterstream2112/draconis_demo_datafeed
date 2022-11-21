#!/usr/bin/env python3

import rospy
import message_filters
import tf2_ros

from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import TransformStamped
from draconis_demo_custom_msgs.msg import ImagePointcloudMsg



# from timeit import default_timer as timer 


__NODE_NAME = "node_data_for_vision_perception"


class NodeDataForVisionPerception():
    def __init__(self) -> None:
        
        self.__cameraTopic = rospy.get_param("~camera_input_topic")
        self.__lidarTopic = rospy.get_param("~lidar_input_topic")
        self.__modifiedImageOutputTopic = rospy.get_param("~modified_argus_front_right_output_topic")
        self.__imageCloudOutputTopic = rospy.get_param("~image_cloud_output_topic")
        self.__outputMinPeriod = 1 / rospy.get_param("~output_framerate", 2)
        self.__imageCloudPackageFrameId =  rospy.get_param("~image_cloud_package_frame_id")

        rospy.loginfo(f'{self.__cameraTopic}')
        rospy.loginfo(f'{self.__imageCloudOutputTopic}')

        self.__prevTime = rospy.Time.now()

        self.__imageCloudPackage = ImagePointcloudMsg()

        #--- tf2 
        self.__tfBuffer = tf2_ros.Buffer()
        self.__listener = tf2_ros.TransformListener(self.__tfBuffer)

        # self.__imageFilter = message_filters.Subscriber(self.__modifiedImageOutputTopic, Image)
        self.__imageFilter = message_filters.Subscriber(self.__cameraTopic, Image)
        self.__cloudFilter = message_filters.Subscriber(self.__lidarTopic, PointCloud2)
        self.__sync = message_filters.ApproximateTimeSynchronizer([self.__imageFilter, self.__cloudFilter], \
                                                                  queue_size=5, \
                                                                  slop=0.3)
        self.__sync.registerCallback(self.__image_cloud_receiving_callback)

        self.__imageSub = rospy.Subscriber(self.__cameraTopic, Image, self.__image_callback, queue_size=5, tcp_nodelay=True)
        self.__cloudSub = rospy.Subscriber(self.__lidarTopic, PointCloud2, self.__cloud_callback, queue_size=5, tcp_nodelay=True)

        self.__imageCloudPub = rospy.Publisher(self.__imageCloudOutputTopic, ImagePointcloudMsg, queue_size=5, tcp_nodelay=True)
        self.__modifiedImagePub = rospy.Publisher(self.__modifiedImageOutputTopic, Image, queue_size=5, tcp_nodelay=True)


    def __image_cloud_receiving_callback(self, image: Image, cloud: PointCloud2 ):
        
        rospy.loginfo("__image_cloud_receiving_callback")
        
        #--- do timing 
        elapsedTime = rospy.Time.now() - self.__prevTime

        if (elapsedTime.to_sec() < self.__outputMinPeriod):
            return

        self.__prevTime = rospy.Time.now()

        #--- get TransformStamped
        self.__trans = TransformStamped()
        try:
            self.__trans = self.__tfBuffer.lookup_transform(self.__imageCloudPackageFrameId, cloud.header.frame_id, cloud.header.stamp)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("__image_cloud_receiving_callback, failed to get transform")
            return

        #--- get data and publish
        self.__imageCloudPackage.image = image
        self.__imageCloudPackage.pointcloud = cloud
        self.__imageCloudPackage.transform_stamped = self.__trans

        self.__imageCloudPub.publish(self.__imageCloudPackage)


    def __image_callback(self, image: Image):
        rospy.loginfo(f"__image_callback {(rospy.Time.now() - image.header.stamp).to_sec() : 0.2f}")

        # image.header.stamp = rospy.Time.now()
        # self.__modifiedImagePub.publish(image)


    def __cloud_callback(self, cloud: PointCloud2):
        rospy.loginfo(f"__cloud_callback {(rospy.Time.now() - cloud.header.stamp).to_sec() : 0.2f}")


    def onShutDown(self):
        pass


if __name__ == "__main__":
    rospy.init_node(__NODE_NAME)
    rospy.loginfo("Start " + __NODE_NAME)

    __node = NodeDataForVisionPerception()

    rospy.on_shutdown(__node.onShutDown)
    rospy.spin()   