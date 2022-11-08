#!/usr/bin/env python3

import rospy
import tf2_ros

from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

# from timeit import default_timer as timer 


__NODE_NAME = "node_data_for_obstacle_detection"


class NodeDataForObstacleDetection():
    def __init__(self) -> None:
        
        self.__topicCloudIn = rospy.get_param("~topic_cloud_in")
        self.__topicCloudOut = rospy.get_param("~topic_cloud_out")

        self.__cloudOutFrameId = rospy.get_param("~cloud_out_frame_id")
        self.__outputMinPeriod = 1 / rospy.get_param("~output_framerate", 2)

        self.__prevTime = rospy.Time.now()

        self.__tfBuffer = tf2_ros.Buffer()
        self.__listener = tf2_ros.TransformListener(self.__tfBuffer)
        
        self.__cloudInSub = rospy.Subscriber(self.__topicCloudIn, PointCloud2, self.__cloud_receiving_callback, queue_size=1, tcp_nodelay=True)

        self.__cloudOutPub = rospy.Publisher(self.__topicCloudOut, PointCloud2, queue_size=5, tcp_nodelay=True)
        

    def __cloud_receiving_callback(self, cloud: PointCloud2):
        rospy.loginfo(f"__cloud_receiving_callback: {(rospy.Time.now() - cloud.header.stamp).to_sec() : 0.2f}")

        #--- do timing 
        elapsedTime = rospy.Time.now() - self.__prevTime
        if (elapsedTime.to_sec() < self.__outputMinPeriod):
            return
        self.__prevTime = rospy.Time.now()

        #--- get data and publish
        transformedCloud = self.transformCloudToFrame(self.__cloudOutFrameId, cloud)

        if (transformedCloud != None):
            transformedCloud.header.frame_id = self.__cloudOutFrameId
            transformedCloud.header.stamp = rospy.Time.now()

            self.__cloudOutPub.publish(transformedCloud)


    def transformCloudToFrame(self, targetFrame: str, cloudIn: PointCloud2):
        rospy.loginfo("transformCloudToFrame-1")
        try:
            trans = self.__tfBuffer.lookup_transform(targetFrame, 
                                                     cloudIn.header.frame_id,
                                                     rospy.Time())
        except tf2_ros.LookupException as ex:
            rospy.logwarn(ex)
            return
        except tf2_ros.ExtrapolationException as ex:
            rospy.logwarn(ex)
            return

        return do_transform_cloud(cloudIn, trans)
        

    def onShutDown(self):
        pass


if __name__ == "__main__":
    rospy.init_node(__NODE_NAME)
    rospy.loginfo("Start " + __NODE_NAME)

    __node = NodeDataForObstacleDetection()

    rospy.on_shutdown(__node.onShutDown)
    rospy.spin()   