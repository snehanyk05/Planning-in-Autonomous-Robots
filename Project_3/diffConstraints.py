# -*- coding: utf-8 -*-

import numpy as np

def diffconstraints(ul,ur,x,y,theta,resolution,t):

    r=0.038
    L= 0.23

    k=1
    ul= (ul*2*np.pi)/60
    ur= (ur*2*np.pi)/60
    theta= (r/L)*(ur-ul)*t + theta
    thetadot=(r/L)*(ur-ul)
    
    x= (r/2)*(ul+ur)*np.cos(theta)*t*k +x
    xdot= (r/2)*(ul+ur)*np.cos(theta)
    
    y= (r/2)*(ul+ur)*np.sin(theta)*t*k +y
    ydot= (r/2)*(ul+ur)*np.sin(theta)
    
    
    xy= np.array([round(x, 3),round(y, 3),round(theta, 3)])
    vel=np.array([round(xdot,3), round(ydot,3),round(thetadot,3)])
    return xy,vel