#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np

LIDAR_SCAN_TOPIC = rospy.get_param("/lidar_value_adjuster/lidar_scan_topic")
LIDAR_STATUS_TOPIC = rospy.get_param("/lidar_value_adjuster/lidar_status_topic")
ADJUSTED_SCAN_TOPIC = rospy.get_param("/lidar_value_adjuster/adjusted_scan_topic")

class LidarValueAdjuster():
    def __init__(self):
        self.lidar_state = True
        self.lidar_sub = rospy.Subscriber(LIDAR_SCAN_TOPIC, LaserScan,
                                          callback=self.lidar_callback)
        self.status_sub = rospy.Subscriber(LIDAR_STATUS_TOPIC, Bool,
                                          callback=self.lidar_callback)
        self.scan_pub = rospy.Publisher(ADJUSTED_SCAN_TOPIC, LaserScan,
                                        queue_size=10)

        self.inf_msg = None

    def lidar_callback(self, data):
 
        if not self.inf_msg:
            self.inf_msg = data
            self.inf_msg.ranges[np.isinf] = np.Inf
                
        if self.lidar_state:
            self.scan_pub.publish(data)
        else:
            self.scan_pub.publish(self.inf_msg)

    def status_callback(self, data):
        self.lidar_state = data

if __name__ == '__main__':
    rospy.init_node("lidar_value_adjuster")
    node = LidarValueAdjuster()
    rospy.spin()
