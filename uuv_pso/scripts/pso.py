#!/usr/bin/env python

import rospy
import numpy as np
import numpy.random as rnd
import copy
from geometry_msgs.msg import Twist, Accel, Vector3
from uuv_sensor_ros_plugins_msgs.msg._ChemicalParticleConcentration import ChemicalParticleConcentration as cpc

class PSO():
    def __init__(self):
        print('PSO running')

        # Checks for the rosparam uuv_number
        # uuv_number -> Total number of uuv robots for PSO
        if rospy.has_param('~uuv_number'):
            self.uuv_number = rospy.get_param('~uuv_number')
            print('Total number of robots: ', self.uuv_number)
        
        # Initializing empty array of position Vectors with size uuv_number
        self.particle_pos = np.zeros(self.uuv_number).tolist()
        self.cpc_values = self.particle_pos[:]

        for i in range(self.uuv_number):
            self.particle_pos[i] = np.zeros(3)
        self.particle_pos = np.array(self.particle_pos)
        # Initializing empty array of concentration values
        self.particle_velocity = copy.deepcopy(self.particle_pos)

        self._init_subscribers()
        print('Going to PSO function')
        self.ParticleSwarm(self.uuv_number)

        
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
        self.particle_pos[0][0] = msg.position.x
        self.particle_pos[0][1] = msg.position.y
        self.particle_pos[0][2] = msg.position.z
        self.cpc_values[0] = msg.concentration
        print('Topic ', self.sub_cpc1.name, ' has the value: %f at x= %f' % (self.cpc_values[0], self.particle_pos[0][0]))
        
        # End topic subscription 
        self.sub_cpc1.unregister()

    def cpc_callback2(self, msg):
        self.particle_pos[1][0] = msg.position.x
        self.particle_pos[1][1] = msg.position.y
        self.particle_pos[1][2] = msg.position.z
        self.cpc_values[1] = msg.concentration
        print('Topic ', self.sub_cpc2.name, ' has the value: %f' % self.cpc_values[1])
        
        # End topic subscription
        self.sub_cpc2.unregister()

    def cpc_callback3(self, msg):
        self.particle_pos[2][0] = msg.position.x
        self.particle_pos[2][1] = msg.position.y
        self.particle_pos[2][2] = msg.position.z
        self.cpc_values[2] = msg.concentration
        print('Topic ', self.sub_cpc3.name, ' has the value: %f' % self.cpc_values[2])
        
        # End topic subscription
        self.sub_cpc3.unregister()

    def ParticleSwarm(self, p):
        print('ParticleSwarm function')
        # PSO constant variables
        c1 = 2.8
        c2 = 1.3
        tol = 0.00000000000001
        d = 3
        c3=c1+c2
        K=2/(abs(2-c3-np.sqrt((c3**2)-(4*c3)))) #creating velocity weighting factor

        # Initialize best swarm value
        f_swarm_best = 0.0
        f_particle_best = [0.0]*p
        swarm_best = np.zeros(d)
        old_swarm_best = [0.0]*d
        particle_best = copy.deepcopy(self.particle_pos)
        # print('TESTE:')
        # print(type(particle_best))
        # print(type(particle_best[0]))
        # print(type(particle_best[0][0]))
        # print(type(self.particle_pos))
        # print(type(self.particle_pos[0]))
        # print(type(self.particle_pos[0][0]))
        # print(type(self.particle_velocity))
        # print(type(self.particle_velocity[0]))
        # print(type(self.particle_velocity[0][0]))
        # print(type(swarm_best))
        # print(type(swarm_best[0]))
        # print(swarm_best)
        for j in range(1000):
            # Iterates over each particle
            for i in range(p):
                # Checks for new particle best
                if self.cpc_values[i] > f_particle_best[i]:
                    particle_best[i] = self.particle_pos[i]
                    f_particle_best[i] = self.cpc_values[i]
                    # Checks for new swarm best
                    if f_particle_best[i] > f_swarm_best:
                        old_swarm_best = swarm_best
                        swarm_best = particle_best[i]
                        f_swarm_best = f_particle_best[i]

            # Iterates over each particle
            for i in range(p):
                rp,rg=rnd.uniform(0,1,2) #creates two random numbers between 0-
                self.particle_velocity[i,:]+=(c1*rp*(particle_best[i,:]-self.particle_pos[i,:]))
                self.particle_velocity[i,:]+=(c2*rg*(swarm_best[:]-self.particle_pos[i,:]))
                self.particle_velocity[i,:]=self.particle_velocity[i,:]*K

            print('Iteration: ', j)
            print('x:\t %f\ny:\t %f\nz:\t %f\n' %(self.particle_velocity[0][0], self.particle_velocity[0][1], self.particle_velocity[0][2]))

        
        # for i in range(p):
        #     if particle_fitness < self.cpc_values[i]:


        # particle_pos=np.zeros(p) #creating empty position array
        # particle_pos=particle_pos.tolist() #converting array to list
        # particle_velocity=particle_pos[:] #empty velocity array
        # particle_pos_val=particle_pos[:] #empty value array

        # # TODO: Function to generate conflict?

        # for i in range(p): #iterates over each particle
        #     rp,rg=rnd.uniform(0,1,2) #creates two random numbers between 0-
        #     particle_velocity[i,:]+=(c1*rp*(particle_best[i,:]-particle_pos[i,:]))
        #     particle_velocity[i,:]+=(c2*rg*(local_best[i,:]-particle_pos[i,:]))
        #     particle_velocity[i,:]=particle_velocity[i,:]*K



        

        # local_best=local_best_get(particle_pos,particle_pos_val,p)

        # swarm_best=particle_pos[np.argmin(particle_pos_val)]#getting the lowest particle value
        # particle_best=copy.deepcopy(particle_pos)#setting all particles current positions to best
        # return d,np.array(particle_pos), np.array(particle_best), \
        #             np.array(swarm_best), np.array(particle_velocity), np.array(local_best), \
        #                 np.array(particle_pos_val)
        


if __name__ == "__main__":
    
    # Start the node
    rospy.init_node('pso')
    rospy.loginfo("Starting PSO node.")

    try:
        node = PSO()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('Caught exception')
    rospy.loginfo('Shutting down PSO node.')