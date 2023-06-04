#!/usr/bin/env python3

import rospy
import message_filters
import tf2_ros

from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import TransformStamped


__NODE_NAME = "node_relay_dog_topics"


class NodeRelayDogTopics():
    def __init__(self) -> None:

        rospy.loginfo("<< node::__init__ >>")

        #--- Read params
        self.__topicInputs = [rospy.get_param("~topic_input_ar0234_front_right"), \
                             rospy.get_param("~topic_input_ar0234_side_left"), \
                             rospy.get_param("~topic_input_ar0234_side_right"), \
                             rospy.get_param("~topic_input_state_heartbeat")]

        self.__topicOutputs = [rospy.get_param("~topic_output_ar0234_front_right"), \
                              rospy.get_param("~topic_output_ar0234_side_left"), \
                              rospy.get_param("~topic_output_ar0234_side_right"), \
                              rospy.get_param("~topic_output_state_heartbeat")]

        #--- Subscribers
        self.__subscribers = [rospy.Subscriber(self.__topicInputs[0], Image, self.__callback_0, queue_size=5, tcp_nodelay=True), \
                              rospy.Subscriber(self.__topicInputs[1], Image, self.__callback_1, queue_size=5, tcp_nodelay=True), \
                              rospy.Subscriber(self.__topicInputs[2], Image, self.__callback_2, queue_size=5, tcp_nodelay=True), \
                              rospy.Subscriber(self.__topicInputs[3], Image, self.__callback_3, queue_size=5, tcp_nodelay=True)]

        #--- Publishers
        self.__publishers = [rospy.Publisher(self.__topicOutputs[0], Image, queue_size=5, tcp_nodelay=True), \
                             rospy.Publisher(self.__topicOutputs[1], Image, queue_size=5, tcp_nodelay=True), \
                             rospy.Publisher(self.__topicOutputs[2], Image, queue_size=5, tcp_nodelay=True), \
                             rospy.Publisher(self.__topicOutputs[3], Image, queue_size=5, tcp_nodelay=True)]


    def __callback_0(self, image: Image):
        self.__publishers[0].publish(image)


    def __callback_1(self, image: Image):
        self.__publishers[1].publish(image)


    def __callback_2(self, image: Image):
        self.__publishers[2].publish(image)


    def __callback_3(self, image: Image):
        self.__publishers[3].publish(image)
        

    def onShutDown(self):
        pass


if __name__ == "__main__":
    rospy.init_node(__NODE_NAME)
    rospy.loginfo("Start " + __NODE_NAME)

    __node = NodeRelayDogTopics()

    rospy.on_shutdown(__node.onShutDown)
    rospy.spin()   
