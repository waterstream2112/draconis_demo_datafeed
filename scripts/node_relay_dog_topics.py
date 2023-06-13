#!/usr/bin/env python3

import rospy
import message_filters
import tf2_ros

from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import TransformStamped
from draconis_demo_custom_msgs.msg import ObjectDetectionMsg
from visualization_msgs.msg import MarkerArray


__NODE_NAME = "node_relay_dog_topics"


class NodeRelayDogTopics():
    def __init__(self) -> None:

        rospy.loginfo("<< node::__init__ >>")

        #--- Read params
        self.__topicInputs = [rospy.get_param("~topic_input_state_heartbeat", ""), \
                              rospy.get_param("~topic_input_ar0234_front_right", ""), \
                              rospy.get_param("~topic_input_ar0234_side_left", ""), \
                              rospy.get_param("~topic_input_ar0234_side_right", ""), \
                              rospy.get_param("~topic_input_ar0234_rear", ""), \
                              rospy.get_param("~topic_input_object_detection", ""), \
                              rospy.get_param("~topic_input_obstacle_edge_cloud", "")
                             ]

        self.__topicOutputs = [rospy.get_param("~topic_output_state_heartbeat", ""), \
                              rospy.get_param("~topic_output_ar0234_front_right", ""), \
                              rospy.get_param("~topic_output_ar0234_side_left", ""), \
                              rospy.get_param("~topic_output_ar0234_side_right", ""), \
                              rospy.get_param("~topic_output_ar0234_rear", ""), \
                              rospy.get_param("~topic_output_object_detection", ""), \
                              rospy.get_param("~topic_output_obstacle_edge_cloud", "")
                              ]

        #--- Subscribers
        self.__subscribers = [rospy.Subscriber(self.__topicInputs[0], UInt8MultiArray, self.__callback_0, queue_size=5, tcp_nodelay=True), \
                              rospy.Subscriber(self.__topicInputs[1], Image, self.__callback_1, queue_size=5, tcp_nodelay=True), \
                              rospy.Subscriber(self.__topicInputs[2], Image, self.__callback_2, queue_size=5, tcp_nodelay=True), \
                              rospy.Subscriber(self.__topicInputs[3], Image, self.__callback_3, queue_size=5, tcp_nodelay=True), \
                              rospy.Subscriber(self.__topicInputs[4], Image, self.__callback_4, queue_size=5, tcp_nodelay=True), \
                              rospy.Subscriber(self.__topicInputs[5], ObjectDetectionMsg, self.__callback_5, queue_size=5, tcp_nodelay=True), \
                              rospy.Subscriber(self.__topicInputs[6], MarkerArray, self.__callback_6, queue_size=5, tcp_nodelay=True)
                             ]

        #--- Publishers
        self.__publishers = [rospy.Publisher(self.__topicOutputs[0], UInt8MultiArray, queue_size=5, tcp_nodelay=True), \
                             rospy.Publisher(self.__topicOutputs[1], Image, queue_size=5, tcp_nodelay=True), \
                             rospy.Publisher(self.__topicOutputs[2], Image, queue_size=5, tcp_nodelay=True), \
                             rospy.Publisher(self.__topicOutputs[3], Image, queue_size=5, tcp_nodelay=True), \
                             rospy.Publisher(self.__topicOutputs[4], Image, queue_size=5, tcp_nodelay=True), \
                             rospy.Publisher(self.__topicOutputs[5], ObjectDetectionMsg, queue_size=5, tcp_nodelay=True), \
                             rospy.Publisher(self.__topicOutputs[6], MarkerArray, queue_size=5, tcp_nodelay=True)
                            ]


    def __callback_0(self, data: UInt8MultiArray):
        # rospy.loginfo("<< node::__callback_0 >>")
        self.__publishers[0].publish(data)


    def __callback_1(self, image: Image):
        # rospy.loginfo("<< node::__callback_1 >>")
        self.__publishers[1].publish(image)


    def __callback_2(self, image: Image):
        self.__publishers[2].publish(image)


    def __callback_3(self, image: Image):
        self.__publishers[3].publish(image)

    
    def __callback_4(self, image: Image):
        self.__publishers[4].publish(image)
        

    def __callback_5(self, data: ObjectDetectionMsg):
        self.__publishers[5].publish(data)

    def __callback_6(self, data: MarkerArray):
        self.__publishers[6].publish(data)
        

    def onShutDown(self):
        pass


if __name__ == "__main__":
    rospy.init_node(__NODE_NAME)
    rospy.loginfo("Start " + __NODE_NAME)

    __node = NodeRelayDogTopics()

    rospy.on_shutdown(__node.onShutDown)
    rospy.spin()   
