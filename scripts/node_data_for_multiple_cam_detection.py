#!/usr/bin/env python3

import rospy
import message_filters
import tf2_ros

from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import TransformStamped
from draconis_demo_custom_msgs.msg import ImagePointcloudMsg, ImagesAndPointcloudMsg



# from timeit import default_timer as timer 


__NODE_NAME = "node_data_for_vision_perception"


class NodeDataForMultipleCamDetection():
    def __init__(self) -> None:
        
        rospy.loginfo("<< node::__init__>>")

        self.__cameraFrontTopic = rospy.get_param("~topic_input_camera_front")
        self.__cameraLeftTopic = rospy.get_param("~topic_input_camera_left")
        self.__cameraRightTopic = rospy.get_param("~topic_input_camera_right")
        self.__cameraRearTopic = rospy.get_param("~topic_input_camera_rear")
        self.__velodyneTopic = rospy.get_param("~topic_input_velodyne")

        self.__imagesAndCloudOutputTopic = rospy.get_param("~topic_output_images_and_cloud")

        self.__outputMinPeriod = 1 / rospy.get_param("~output_framerate", 2)
        self.__imagesAndCloudPackageFrameId =  rospy.get_param("~images_and_cloud_package_frame_id")

        rospy.loginfo(f'Cam Front Topic: {self.__cameraFrontTopic}')
        rospy.loginfo(f'Cam Left Topic: {self.__cameraLeftTopic}')
        rospy.loginfo(f'Cam Right Topic: {self.__cameraRightTopic}')
        rospy.loginfo(f'Cam Rear Topic: {self.__cameraRearTopic}')

        rospy.loginfo(f'{self.__imagesAndCloudOutputTopic}')

        self.__prevTime = rospy.Time.now()

        self.__imagesAndCloudPackage = ImagesAndPointcloudMsg()

        #--- tf2 
        # self.__tfBuffer = tf2_ros.Buffer()
        # self.__listener = tf2_ros.TransformListener(self.__tfBuffer)

        #--- Subscribers
        self.__imageFrontFilter = message_filters.Subscriber(self.__cameraFrontTopic, Image)
        self.__imageLeftFilter = message_filters.Subscriber(self.__cameraLeftTopic, Image)
        self.__imageRightFilter = message_filters.Subscriber(self.__cameraRightTopic, Image)
        self.__imageRearFilter = message_filters.Subscriber(self.__cameraRearTopic, Image)
        self.__cloudFilter = message_filters.Subscriber(self.__velodyneTopic, PointCloud2)

        self.__sync = message_filters.ApproximateTimeSynchronizer([self.__imageFrontFilter, \
                                                                   self.__imageLeftFilter, \
                                                                   self.__imageRightFilter, \
                                                                   self.__imageRearFilter, \
                                                                   self.__cloudFilter], \
                                                                   queue_size=5, \
                                                                   slop=0.5)

        self.__sync.registerCallback(self.__images_and_cloud_receiving_callback)

        self.__imageSub = rospy.Subscriber(self.__cameraRearTopic, Image, self.__image_callback, queue_size=5, tcp_nodelay=True)
        self.__cloudSub = rospy.Subscriber(self.__velodyneTopic, PointCloud2, self.__cloud_callback, queue_size=5, tcp_nodelay=True)

        rospy.loginfo("Initialization done!")

    
        #--- Publishers
        self.__imagesAndCloudPub = rospy.Publisher(self.__imagesAndCloudOutputTopic, ImagesAndPointcloudMsg, queue_size=5, tcp_nodelay=True)


    def __images_and_cloud_receiving_callback(self, imageFront: Image, imageLeft: Image, imageRight: Image, imageRear: Image,  cloud: PointCloud2 ):
        
        rospy.loginfo("<< node::__images_and_cloud_receiving_callback >>")
        
        #--- do timing 
        elapsedTime = rospy.Time.now() - self.__prevTime

        if (elapsedTime.to_sec() < self.__outputMinPeriod):
            return

        self.__prevTime = rospy.Time.now()

        #--- get TransformStamped
        self.__trans = TransformStamped()
        # try:
        #     self.__trans = self.__tfBuffer.lookup_transform(self.__imagesAndCloudPackageFrameId, cloud.header.frame_id, rospy.Time(0))
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     rospy.logerr("__images_and_cloud_receiving_callback, failed to get transform")
        #     return

        #--- get data and publish
        self.__imagesAndCloudPackage.image_front = imageFront
        self.__imagesAndCloudPackage.image_left = imageLeft
        self.__imagesAndCloudPackage.image_right = imageRight
        self.__imagesAndCloudPackage.image_rear = imageRear
        self.__imagesAndCloudPackage.pointcloud = cloud
        self.__imagesAndCloudPackage.transform_stamped = self.__trans
        self.__imagesAndCloudPackage.header = cloud.header

        rospy.loginfo("Publish imagesAndCloudPackage")
        self.__imagesAndCloudPub.publish(self.__imagesAndCloudPackage)


    def __image_callback(self, image: Image):

        # rospy.loginfo("<< node::__image_callback >>")

        # rospy.loginfo(f"__image_callback {(rospy.Time.now() - image.header.stamp).to_sec() : 0.2f}")

        pass

        # image.header.stamp = rospy.Time.now()
        # self.__modifiedImagePub.publish(image)


    def __cloud_callback(self, cloud: PointCloud2):

        # rospy.loginfo("<< node::__cloud_callback >>")

        # rospy.loginfo(f"__cloud_callback {(rospy.Time.now() - cloud.header.stamp).to_sec() : 0.2f}")
        # rospy.loginfo(f"__cloud_callback {cloud.header.frame_id} ")
        pass



    def onShutDown(self):
        pass


if __name__ == "__main__":
    rospy.init_node(__NODE_NAME)
    rospy.loginfo("Start " + __NODE_NAME)

    __node = NodeDataForMultipleCamDetection()

    rospy.on_shutdown(__node.onShutDown)
    rospy.spin()   