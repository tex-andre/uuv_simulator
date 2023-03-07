# -*- coding: utf-8 -*-
"""
Created on Sun Mar  3 15:59:36 2019

@author: tomsa
"""

import numpy as np 
import copy
import numpy.random as rnd
import time
import matplotlib.pyplot as plt
from matplotlib import cm


def local_best_get(particle_pos,particle_pos_val,p):
    local_best=[0]*p #creating empty local best list
    for j in range(p):  #finding the best particle in each neighbourhood 
                        #and storing it in 'local_best'
        local_vals=np.zeros(3)
        local_vals[0]=particle_pos_val[j-2]
        local_vals[1]=particle_pos_val[j-1]
        local_vals[2]=particle_pos_val[j]
        min_index=int(np.argmin(local_vals))
        local_best[j-1]=particle_pos[min_index+j-2][:]
    return np.array(local_best)




def initiation(f,bounds,p):
    '''
    INPUTS
    f       :function to be searched over
    bounds  :bounds of function in form [[x1,x2],[x3,x4],[x5,x6]...]
    p       :number of particles
    
    OUTPUTS
    particle_pos      :array of random particle positions 
    particle_best     :array of best particle positions (same as current)
    swarm_best        :coordinates of particle with best known position
    particle_velocity :array of random particle velocity arrays
    local_best        :array of the best particle in each neighbourhood 
    local_best_fitness:function value evaluated at each local best
    particle_pos_val  :fitness of each particle 
    
    '''
    d=len(bounds) #finding number of dimensions
    particle_pos=np.zeros(p) #creating empty position array
    particle_pos=particle_pos.tolist() #converting array to list
    particle_velocity=particle_pos[:] #empty velocity array
    particle_pos_val=particle_pos[:] #empty value array
    for j in range(p): #iterating ovre the number of particles
        particle_pos[j]=[rnd.uniform(bounds[i][0],bounds[i][1])\
                    for i in range(d)] #random coordinate within bounds
        particle_pos_val[j]=f(particle_pos[j]) #calculating function value
                                            #at each particle
        particle_velocity[j]=[rnd.uniform(-abs(bounds[i][1]-bounds[i][0])\
                    ,abs(bounds[i][1]-bounds[i][0])) for i in range(d)]
                    #creating random velocity values for each dimension

    local_best=local_best_get(particle_pos,particle_pos_val,p)

    swarm_best=particle_pos[np.argmin(particle_pos_val)]#getting the lowest particle value
    particle_best=copy.deepcopy(particle_pos)#setting all particles current positions to best
    return d,np.array(particle_pos), np.array(particle_best), \
                 np.array(swarm_best), np.array(particle_velocity), np.array(local_best), \
                     np.array(particle_pos_val)
        
def withinbounds(bounds,particle_pos):
    '''
    DESCRIPTION: 
        Checks whether a particle's position is within the bounds of the problem 
        and contraints particles within bounds
        
    INPUTS
    bounds      :bounds of problem in form [[x1,x2],[x3,x4]...]
    particle_pos:coordinates of a particle e.g [p1,p2,p3...]
    
 
    '''
    for i in range(len(bounds)):
        if particle_pos[i]<bounds[i][0]: #if particle is less than lower bound
            particle_pos[i]=bounds[i][0]
        elif particle_pos[i]>bounds[i][1]: #if particle is more than higher bound
            particle_pos[i]=bounds[i][1]
    return


def plot_3D(f,bounds,particle_pos,pos_val,p):
    plt.style.use('_mpl-gallery')

    xlim = bounds[0]
    ylim = bounds[1]
    # make the data
    X = np.arange(xlim[0], xlim[1], 0.2)
    Y = np.arange(ylim[0], ylim[1], 0.2)
    X, Y = np.meshgrid(X, Y)
    Z = f([X,Y])

    xs = [particle_pos[i][0] for i in range(p)]
    ys = [particle_pos[i][1] for i in range(p)]
    zs = pos_val
    # Plot the surface
    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    ax.plot_surface(X, Y, Z, vmin=Z.min() * 2, cmap=cm.Blues)
    #Plot scatter points
    ax.scatter(xs, ys, zs)

    #ax.set(xticklabels=[],
    #       yticklabels=[],
    #       zticklabels=[])

    plt.show()        
                
                
                
                
                
                
    