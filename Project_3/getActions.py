# -*- coding: utf-8 -*-

import numpy as np

def getactions(u1,u2):

    actions= np.array([ [u2,u2],[u1,u1],[u2,u1], [u1,0], [u2,0],[0,u2],[0,u1],[u1,u2]])
    
    calc=((2*np.pi)/60)
    actions=actions*calc
    
    return actions



