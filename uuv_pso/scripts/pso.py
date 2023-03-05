#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Accel, Vector3
from uuv_sensor_ros_plugins_msgs.msg._ChemicalParticleConcentration import ChemicalParticleConcentration as cpc

class ParticleSwarm():
    def __init__(self):
        print('ParticleSwarm running')

        # Checks for the rosparam uuv_number
        if rospy.has_param('~uuv_number'):
            self.uuv_number = rospy.get_param('~uuv_number')
            print('Total number of robots: ', self.uuv_number)

        # uuv_number -> Total number of uuv robots for PSO
        # Empty array of position Vectors with size uuv_number
        self.pos = [Vector3]*self.uuv_number
        # Empty array of concentration values
        self.cpc_values = np.empty(self.uuv_number, float)

        self._init_subscribers()

        
    def _init_subscribers(self):
        # Subscribe to ROS topics
        #for i in range(1, self.uuv_number + 1):
        # self.sub_cpc = rospy.Subscriber(
        #     '/rexrov'+ str(self.uuv_number) +'/particle_concentration', 
        #     cpc, self.cpc_callback)

        self.sub_cpc1 = rospy.Subscriber(
            '/rexrov/particle_concentration', 
            cpc, self.cpc_callback1)

        self.sub_cpc2 = rospy.Subscriber(
            '/rexrov2/particle_concentration', 
            cpc, self.cpc_callback2)

        self.sub_cpc3 = rospy.Subscriber(
            '/rexrov3/particle_concentration', 
            cpc, self.cpc_callback3)

    #   ==== Subscribers callback functions ====
        
    def cpc_callback1(self, msg):
        self.pos[0] = msg.position
        print(msg.position.x)
        self.cpc_values[0] = msg.concentration
        print('Topic ', self.sub_cpc1.name, ' has the X value: ', self.cpc_values[0])
        
        # End topic subscription 
        self.sub_cpc1.unregister()

    def cpc_callback2(self, msg):
        self.pos[1] = msg.position
        self.cpc_values[1] = msg.concentration
        print('Topic ', self.sub_cpc2.name, ' has the value: %f' % self.cpc_values[1])
        
        # End topic subscription
        self.sub_cpc2.unregister()

    def cpc_callback3(self, msg):
        self.pos[2] = msg.position
        self.cpc_values[2] = msg.concentration
        print('Topic ', self.sub_cpc3.name, ' has the value: %f' % self.cpc_values[2])
        
        # End topic subscription
        self.sub_cpc3.unregister()



if __name__ == "__main__":
    
    # Start the node
    rospy.init_node('pso')
    rospy.loginfo("Starting PSO node.")

    try:
        node = ParticleSwarm()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('Caught exception')
    rospy.loginfo('Shutting down PSO node.')