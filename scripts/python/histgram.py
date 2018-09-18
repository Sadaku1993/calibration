#!/usr/bin/env python2

import rospy
from std_msgs.msg import Int32MultiArray

import numpy as np
import matplotlib.pyplot as plt

def callback(msg):
    print("size:%d" % len(msg.data));

    ox = []
    oy = []

    for i in range(len(msg.data)):
        ox.append(i)
        oy.append(msg.data[i])

    plt.bar(ox, oy, color="#1E7F00", width=1.0, align="center")
    plt.pause(0.5)

def main():
    rospy.init_node("histgram")

    rospy.Subscriber('histgram', Int32MultiArray, callback)

    rospy.spin()

if __name__ =="__main__":
    main()
