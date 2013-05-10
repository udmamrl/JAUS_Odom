#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

# Simple ROS Odometry node for combine 
#           GPS-position,IMU-orientation + Vehcile Twist
# Copyright (c) 2013, Cheng-Lung Lee, University of Detroit Mercy.






import roslib; roslib.load_manifest('JAUS_Odom')
import rospy
import PyKDL
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu



class JAUS_Odom(object):
    def __init__(self):
        rospy.init_node('JAUS_Odom')
        # Parameters

        # Set up publishers/subscribers
        # subscribe GPS - odom here
        rospy.Subscriber('gps_odom', Odometry, self.Handle_GpsOdom)
        # subscribe Hushy - odom here
        rospy.Subscriber('husky_odom', Odometry, self.Handle_HuskyOdom)
        # subscribe IMU here
        rospy.Subscriber('KVH_imu/data', Imu, self.HandleImu)
        # publish odometry data here
        self.JAUS_Odom_Publisher = rospy.Publisher("JAUS_Odom", Odometry)    

        # Initialize odometry message
        self.JAUS_Odom = Odometry()
        
        #test
        #self.JAUS_Imu_Publisher = rospy.Publisher("imu/data", Imu)    
        #self.JAUS_Imu_Publisher.publish(Imu())
   
    def Handle_GpsOdom(self,odom):
        # use gps odom update position data 
        self.JAUS_Odom.pose.pose.position=odom.pose.pose.position
        self.JAUS_Odom.header=odom.header
        # publish data when GPS update 
        self.JAUS_Odom_Publisher.publish(self.JAUS_Odom)
        #rospy.loginfo('Got position data[ x:%s y:%s z:%s ]' %(odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z))

    def Handle_HuskyOdom(self,odom):
        # use Husky odom update twist(speed) data 
        self.JAUS_Odom.twist=odom.twist
        #rospy.loginfo('Got twist data[ Vx:%s Va:%s ]' %(odom.twist.twist.linear.x,odom.twist.twist.angular.z))
        #self.JAUS_Imu_Publisher.publish(Imu())

    def HandleImu(self,imu):
        # use imu update orientation data
        self.JAUS_Odom.pose.pose.orientation=imu.orientation
        #rospy.loginfo('Got imu data[ xyzw : (%s,%s,%s,%s) ]' % (imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.w))

if __name__ == "__main__":
    obj = JAUS_Odom()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

