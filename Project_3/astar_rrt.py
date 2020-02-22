# -*- coding: utf-8 -*-
"""
Created on Sun Apr 21 11:51:54 2019

@author: Sneha
"""
import tkinter as tk
from tkinter import *
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg 
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
from getActions import getactions
from diffConstraints import diffconstraints
import time
from heapq import *
import random
root= tk.Tk()
show_animation = True
init=[]
final=[]
resolution=1
radius=0
clearance=0
start = time.time()
title='Click point in map to select Initial/Final point.'

class Node:

    def __init__(self,node, x, y,theta,vel,cost,ul,ur, pind):
        self.node = node
        self.x = x
        self.y = y
        self.theta = theta
        self.vel = vel
        self.cost = cost
        self.pind = pind
        self.ul = ul
        self.ur = ur
        self.parent = None
    def PrintTree(self,ax):

        if self.parent:
            self.parent.PrintTree(ax)
        ax.scatter(self.x,self.y,s=10,c='b')

    def PrintTreePlot(self,plt,path,pathv):

        if self.parent:
            self.parent.PrintTreePlot(plt,path,pathv)

        plt.plot(self.x,self.y,color='#39ff14', marker='o')

        pathv.append([self.ul,self.ur])
        path.append([self.x,self.y,self.theta,self.ul,self.ur])

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)

def onpick(event):
    print(event.xdata,event.ydata)
    global init,final,title
    if(not(init)):
        print('init')
        init=[(event.xdata),(event.ydata)]
        
    else:
        print('final')
        final=[(event.xdata),(event.ydata)]
        
    title='Node Exploration'
    return True
def animate(listPnts,rectangles):

    global title,root,final,init,resolution,radius,clearance
    fig = plt.Figure(figsize=(5,4), dpi=100)
    ax = fig.add_subplot(111)
    
    
    scatter = FigureCanvasTkAgg(fig, root) 
    scatter.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH)
    ax.fill([-6.55,6.55,6.55,-6.55],[-6.05,-6.05,6.05,6.05], color = (0,0,0))
    count=-1
    for i in (listPnts):
        count+=1
        ax.fill(i[0],i[1], color = i[2])
    for i in (rectangles):
        
            for k in i[0]:
                ax.fill(k[0],k[1], color = i[1])
        

    ax.legend() 
    ax.set_title(title);
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    

    fig.canvas.mpl_connect('button_press_event',onpick)




    tk.Label(root, text="Enter Coordinates").pack()
    tk.Label(root, text="Initial point(comma separated x,y-no spaces)").pack()
    initial=Entry(root)
    if(init):
        init_str=str(init[0])+' '+str(init[1])
        initial.insert(0,init_str)
    initial.pack()
    tk.Label(root, text="Final point(comma separated x,y-no spaces)").pack()
    final1=Entry(root)
    if(final):
        final_str=str(final[0])+' '+str(final[1])
        final1.insert(0,final_str)
    final1.pack()
    

    tk.Button(root, text="Quit", command= lambda:quit(initial,final1)).pack()
   
    root.mainloop()
       
   


xdata=[]
ydata=[]    
def animated(i,nodes,node,test):

    global xdata,ydata
    t, y = i.x,i.y
    xdata.append(t)
    ydata.append(y)
    xmin, xmax = ax.get_xlim()

    if t >= xmax:
        ax.set_xlim(xmin, 2*xmax)
        ax.figure.canvas.draw()
    line.set_data(xdata, ydata)

    
    
    if(((nodes[len(nodes)-1].x) == i.x) and (nodes[len(nodes)-1].y == i.y)):
          node.PrintTree(ax)
    return line,     

def quit(initial,final1):
        global root,init,final,radius,resolution,clearance


        if(initial.get()):
            if(len((initial.get()).split(','))==2):

                x,y=(initial.get()).split(',')
                if(x and y and (float(x)) and (float(y))):
                    init=[(float(x)/resolution),(float(y)/resolution)]
                else:
                    root.quit()
                    root.destroy()
                    test=tk.Tk()
                    test.geometry('400x300')
                    label = Label(test, text= "Please enter valid Initial Point.")

                    label.pack() 

                    test.mainloop()
            else:
                    root.quit()
                    root.destroy()
                    test=tk.Tk()
                    test.geometry('400x300')
                    label = Label(test, text= "Please enter valid comma separated Initial Point.")

                    label.pack() 

                    test.mainloop()

        elif(init):
            
            init=[(init[0]/resolution),(init[1]/resolution)]
            
        else:
            root.quit()
            root.destroy()
            test=tk.Tk()
            test.geometry('400x300')
            label = Label(test, text= "Please enter valid Initial Point.")

            label.pack() 

            test.mainloop()

        if(final1.get()):
            if(len((final1.get()).split(','))==2):
                x1,y1=(final1.get()).split(',')
                if(x1 and y1 and (float(x1)) and (float(y1))):
                    final=[(float(x1)/resolution),(float(y1)/resolution)]
                else:
                    root.quit()
                    root.destroy()
                    test=tk.Tk()
                    test.geometry('400x300')
                    label = Label(test, text= "Please enter valid Final Point.")

                    label.pack() 

                    test.mainloop()
            else:
                    root.quit()
                    root.destroy()
                    test=tk.Tk()
                    test.geometry('400x300')
                    label = Label(test, text= "Please enter valid comma separated Final Point.")

                    label.pack() 

                    test.mainloop()
        elif(final):
            final=[(final[0]/resolution),(final[1]/resolution)]
        else:
            root.quit()
            root.destroy()
            test=tk.Tk()
            test.geometry('400x300')
            label = Label(test, text= "Please enter valid Final Point.")

            label.pack() 

            test.mainloop()
        
        root.quit()
        root.destroy()




def a_star_rrt_planning(sx, sy, gx, gy, ox, oy, reso, rr,rect_corners,actions,plt):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    """
    print("A* Algorithm.....")

    nstart = Node(str(sx)+' '+
                        str(sy),(sx), (sy),0.0, np.array([0,0,0]), 0.0,0,0, -1)
    ngoal = Node(str(gx)+' '+
                        str(gy),(gx), (gy),0.0, np.array([0,0,0]), 0.0,0,0, -1)

    t=10
    motion = get_motion_model(sx,sy,0,reso,t,actions)

    openset, closedset = dict(), dict()

    openset[calc_index(nstart)] = nstart

    cost_queue=[(0,nstart.node)]
    max_iter=2500;
    check_obs_proximity=[nstart]
    flag=0
    count=0
    new_prox_flag=0

    ob_close_ctr=0
    nodesList=[nstart]
    if(verify_node(ngoal, reso, rect_corners) and verify_node(nstart, reso, rect_corners)):
        
        while (openset):
           
            count+=1
            if(new_prox_flag==1):

                    
                check_obs_proximity=[current]
                new_prox_flag=0
 
    
            (cost1,c_id)=heappop(cost_queue)
            
            current = openset[c_id]
#            nodesList.append(current)
            
           
            check= [v for v in (check_obs_proximity) if v.node == current.node]
            
            
            if(len(check)==0):
  
                for i in check_obs_proximity:
                    if(np.abs(current.x-i.x)<(0.2/resolution) and np.abs(current.y-i.y)<(0.2/resolution)):
                       flag=1
                       break;
                    else:
                        flag=0
                        
   
                if(flag==1):
                     check_obs_proximity.append(current)
                else:
                     check_obs_proximity=[current] 
                     
             
            if(len(check_obs_proximity)>max_iter):
                print('Obstacle close -',ob_close_ctr)

                new_prox_flag=1
                ob_close_ctr+=1
   
            if(ob_close_ctr>3):
                print('Switching to RRT algorithm.....')
                RRT(openset,closedset,cost_queue,nstart,ngoal, reso, rect_corners,t,actions, nodesList)
                break
            
            if (np.abs(current.x - ngoal.x)< (0.1/resolution) and np.abs(current.y - ngoal.y)< (0.1/resolution)):
                ngoal.x=current.x
                ngoal.y=current.y
                ngoal.theta=current.theta
                ngoal.pind = current.pind
                ngoal.cost = current.cost
                ngoal.vel = current.vel
                ngoal.ul = current.ul
                ngoal.ur = current.ur
                ngoal.parent = current.parent
                print('Goal found!')
                break
            # Remove the item from the open set
            del openset[c_id]
            # Add it to the closed set
            closedset[c_id] = current
    
            # expand search grid based on motion model
            motion= get_motion_model(current.x,current.y,current.theta,reso,t,actions)

            for i, _ in enumerate(motion):
                
                node = Node(str(motion[i][0][0])+' '+
                            str(motion[i][0][1]),motion[i][0][0],
                            motion[i][0][1],
                            motion[i][0][2],
                            motion[i][1],
                            round(current.cost + motion[i][2],3),motion[i][3],motion[i][4], c_id)
                n_id = calc_index(node)
                if n_id in closedset:
                    continue
    
                if not verify_node(node, reso, rect_corners):
                    continue

                if n_id not in openset:
                    node.parent=current
                    openset[n_id] = node  # Discover a new node
                    heappush(cost_queue, (node.cost+calc_heuristic(ngoal, node.x,node.y), node.node))
                    nodesList.append(node)
                else:
    #                
                    if openset[n_id].cost >= node.cost:
                        node.parent=current
                        count= [i for ((c,v), i) in zip((cost_queue), range(len((cost_queue)))) if v == node.node][0]
                        cost_queue[count]=(node.cost+calc_heuristic(ngoal, node.x,node.y),node.node)
                        openset[n_id] = node

        return nodesList,ngoal
   
    else:
        return 'Initial/Final Point in Obstacle!!',0
    


def calc_heuristic(n1, x,y):
    d = (np.sqrt((x-n1.x)**2 + (y-n1.y)**2))
    return d
def GetNearestListIndex(nodeList, rnd):

        dlist = [((node.x - rnd[0]) ** 2 + (node.y - rnd[1])
                 ** 2,c_id)for c_id,node in nodeList.items()]
        minind,c_id = (min(dlist))

        return c_id
def RRT(openQ,closedQ,costQ,start,end, res, rect_corners,t,actions,nodesList):
        global resolution
        expandDis=0.1/resolution
        goalSampleRate=5
        randArea=[-5.55, 5.55]
        minrand = randArea[0]
        maxrand = randArea[1]
        while True:
            # Random Sampling
            if random.randint(0, 100) > goalSampleRate:
                rnd = [random.uniform(minrand, maxrand), random.uniform(
                    minrand, maxrand)]
            else:
                rnd = [end.x, end.y]

            # Find nearest node
            nind = GetNearestListIndex(openQ, rnd)
          

            # expand tree
            nearestNode = openQ[nind]
            
            
            motion= get_motion_model(nearestNode.x,nearestNode.y,nearestNode.theta,res,t,actions)
           
            
            for i, _ in enumerate(motion):

                
                newNode = Node(str(motion[i][0][0])+' '+
                            str(motion[i][0][1]),motion[i][0][0],
                            motion[i][0][1],
                            motion[i][0][2],
                            motion[i][1],
                            round(nearestNode.cost + motion[i][2],3),motion[i][3],motion[i][4], nind)

                newNode.parent=nearestNode

                if newNode.node in closedQ:
                        continue
        
                if not verify_node(newNode, res, rect_corners):
                    continue
                if newNode.node not in openQ:
                        openQ[newNode.node]=(newNode)
                        heappush(costQ, (newNode.cost+calc_heuristic(end, newNode.x,newNode.y), newNode.node))
                        nodesList.append(newNode)
                else:
        #                
                        if openQ[newNode.node].cost >= newNode.cost:
                            
                            count= [i for ((c,v), i) in zip((costQ), range(len((costQ)))) if v == newNode.node][0]
                            costQ[count]=(newNode.cost+calc_heuristic(end, newNode.x,newNode.y),newNode.node)
                            openQ[newNode.node] = newNode
            
            # check goal
            if (np.abs(newNode.x - end.x)< (expandDis) and np.abs(newNode.y - end.y)< (expandDis)):
                end.x=newNode.x
                end.y=newNode.y
                end.theta=newNode.theta
                end.pind = newNode.pind
                end.cost = newNode.cost
                end.vel = newNode.vel
                end.ul = newNode.ul
                end.ur = newNode.ur
                end.parent = newNode.parent
                print('Goal found!')
                break

def verify_node(node, res, rect_corners):
    global radius,clearance
    x=node.x
    y=node.y

    d=(radius)+(clearance)
    

    c1= ((x-(-1.65/res))*(x-(-1.65/res))+ (y-(4.6/res))*(y-(4.6/res)) - ((0.81/res)+d)*((0.81/res)+d))
    c2= ((x-(-1.17/res))*(x-(-1.17/res))+ (y-(2.31/res))*(y-(2.31/res)) - ((0.81/res)+d)*((0.81/res)+d))
    c3= ((x-(-1.17/res))*(x-(-1.17/res))+ (y-(-2.31/res))*(y-(-2.31/res)) - ((0.81/res)+d)*((0.81/res)+d))
    c4= ((x-(-1.65/res))*(x-(-1.65/res))+ (y-(-4.6/res))*(y-(-4.6/res))- ((0.81/res)+d)*((0.81/res)+d))
    #Capsule
    u=-3.2516     #x-position of the center
    v=3.2505    #y-position of the center
      
    a=(3.1968-1.599)/2   #radius on the x-axis
    b=1.599/2    #radius on the y-axis
    r = [u-a, u+a,u+a, u-a]
    s = [v-b, v-b, v+b,v+b]
    
    u1=u-a
    u2=u+a
    e1= ((x-(u1/res))*(x-(u1/res))+ (y-(v/res))*(y-(v/res)) - ((b/res)+d)*((b/res)+d))
    e2= ((x-(u2/res))*(x-(u2/res))+ (y-(v/res))*(y-(v/res)) - ((b/res)+d)*((b/res)+d))
    exist=True
    if (x>=(-5.55/res)+d and x<=(5.55/res)-d and y>=(-5.05/res)+d and y<=(5.05/res)-d):
        for c in rect_corners:
            
               if( x>=c[0][0]-d and x<=c[0][1]+d and y>=c[1][0]-d and y<=c[1][2]+d):
                   exist = False 
        if(exist is True):
            if( x>=((r[0]/res)-d) and x<=((r[1]/res)+d) and y>=((s[0]/res)-d) and y<=((s[2]/res)+d)):
                       exist = False
            elif (e1<=0):
                exist=False
            elif (e2<=0):
                exist=False 
            elif (c1<=0):
                exist=False
            elif (c2<=0):
                exist=False
            elif (c3<=0):
                exist=False
            elif (c4<=0):
                exist=False  
            else:
                exist=True
           
    else:
        exist=False
        
    return exist


def calc_obstacle_map(ox, oy, reso, vr):

    minx = round(min(ox))
    miny = round(min(oy))
    maxx = round(max(ox))
    maxy = round(max(oy))


    xwidth = int(round(maxx - minx))
    ywidth = int(round(maxy - miny))

    return minx, miny, maxx, maxy, xwidth, ywidth


def calc_index(node):
    return str(node.x)+' '+str(node.y)


def get_motion_model(x,y,theta,resolution,t,actions):
    # dx, dy, cost
    motion = []
    ul=actions[0,0]
    ur=actions[0,1]
    c2cdiff=0.1833
    [xy,v]=diffconstraints(ul,ur,x,y,theta,resolution,t)
    motion.append([xy,v,c2cdiff,actions[0,0],actions[0,1]])
     
#   (Ul,Ur)= (50,50)
    ul=actions[1,0]
    ur=actions[1,1]
    c2cdiff=0.0916
    [xy,v]=diffconstraints(ul,ur,x,y,theta,resolution,t)
    motion.append([xy,v,c2cdiff,actions[1,0],actions[1,1]])
    
#   (Ul,Ur)= (100,50)
    ul=actions[2,0]
    ur=actions[2,1]
    c2cdiff=0.1374
    [xy,v]=diffconstraints(ul,ur,x,y,theta,resolution,t)
    motion.append([xy,v,c2cdiff,actions[2,0],actions[2,1]])
 
#   (Ul,Ur)= (50,0)
    ul=actions[3,0]
    ur=actions[3,1]
    c2cdiff=0.0458
    [xy,v]=diffconstraints(ul,ur,x,y,theta,resolution,t)
    motion.append([xy,v,c2cdiff,actions[3,0],actions[3,1]])
    
#   (Ul,Ur)= (100,0)
    ul=actions[4,0]
    ur=actions[4,1]
    c2cdiff=0.0916
    [xy,v]=diffconstraints(ul,ur,x,y,theta,resolution,t)
    motion.append([xy,v,c2cdiff,actions[4,0],actions[4,1]])

#   (Ul,Ur)= (0,100)
    ul=actions[5,0]
    ur=actions[5,1]
    c2cdiff=0.0916

    [xy,v]=diffconstraints(ul,ur,x,y,theta,resolution,t)
    motion.append([xy,v,c2cdiff,actions[5,0],actions[5,1]])

#   (Ul,Ur)= (0,50)
    ul=actions[6,0]
    ur=actions[6,1]
    c2cdiff=0.0458

    [xy,v]=diffconstraints(ul,ur,x,y,theta,resolution,t)
    motion.append([xy,v,c2cdiff,actions[6,0],actions[6,1]])
  
#   (Ul,Ur)= (50,100)
    ul=actions[7,0]
    ur=actions[7,1]
    c2cdiff=0.1374

    [xy,v]=diffconstraints(ul,ur,x,y,theta,resolution,t)
    motion.append([xy,v,c2cdiff,actions[7,0],actions[7,1]])
  

    return motion
def getxs_ys(xs,ys):
    global resolution,radius,clearance,init,final
    t = np.linspace(0, 2*np.pi, 100)
    res=resolution
    resolution=1
    
    #Circles
    #Circle 1
    r1 = (0.81/2)/resolution
    n1=-1.65/resolution    #x-position of the center
    m1=4.6/resolution   #radius on the y-axis
    p1=n1+r1*np.cos(t)
    q1=m1+r1*np.sin(t)
    for i in p1:
        xs.append(i)
    for i in q1:
        ys.append(i)
    #Circle 2
    r2 =( 0.81/2)/resolution
    n2=-1.17 /resolution    #x-position of the center
    m2=2.31/resolution   #radius on the y-axis
    p2=n2+r2*np.cos(t)
    q2=m2+r2*np.sin(t)
    for i in p2:
        xs.append(i)
    for i in q2:
        ys.append(i)
    #Circle 3
    r3 = (0.81/2)/resolution
    n3=-1.17/resolution    #x-position of the center
    m3=-2.31/resolution   #radius on the y-axis
    p3=n3+r3*np.cos(t)
    q3=m3+r3*np.sin(t)
    for i in p3:
        xs.append(i)
    for i in q3:
        ys.append(i)
    #Circle 4
    r4 = (0.81/2)/resolution
    n4=-1.65 /resolution    #x-position of the center
    m4=-4.6 /resolution  #radius on the y-axis
    p4=n4+r4*np.cos(t)
    q4=m4+r4*np.sin(t)
    for i in p4:
        xs.append(i)
    for i in q4:
        ys.append(i)
    #Capsule
    u=-3.2516/resolution     #x-position of the center
    v=3.2505/resolution    #y-position of the center
      
    a=(((3.1968/resolution)-(1.599/resolution))/2)   #radius on the x-axis
    b=(1.599/2)/resolution    #radius on the y-axis
    r = [u-a, u+a,u+a, u-a]
    s = [v-b, v-b, v+b,v+b]
    for i in r:
        xs.append(i)
    for i in s:
        ys.append(i)
    u1=u-a
    u2=u+a
    r1=u1+b*np.cos(t)
    s1=v+b*np.sin(t)
    r2=u2+b*np.cos(t)
    s2=v+b*np.sin(t)
    for i in r1:
        xs.append(i)
    for i in s1:
        ys.append(i)
    for i in r2:
        xs.append(i)
    for i in s2:
        ys.append(i)
#    #Rectangles
    rectangles =[[3.2,4.135,0.86,1.83],[4.495,4.595,0.43,0.91],[3.72,1.54,3.66,0.76],
    [5.26,0.02,0.58,1.17],[5.095,-0.995,0.91,0.86],[5.26,-2.6825,0.58,1.17],
    [4.635,-4.32,1.83,0.76],[2.825,-4.41,1.17,0.58],[0.56,-3.94,2.74,1.52],
    [-0.715,-0.985,0.91,1.83],[0.660,-2.02,1.83,0.76],[3.06,-1.795,1.52,1.17]]
    for i in range(len(rectangles)):
        for j in range(4):
            rectangles[i][j]=rectangles[i][j]/resolution

#    fig, ax = plt.subplots()
##    
##
#    ax.fill(r,s,'r',edgecolor='b')
#    ax.fill(r1,s1,'r')
#    ax.fill(r2,s2,'r')
#    ax.fill(p1,q1,'r',edgecolor='b')
#    ax.fill(p2,q2,'r',edgecolor='b')
#    ax.fill(p3,q3,'r',edgecolor='b')
#    ax.fill(p4,q4,'r',edgecolor='b')
    # ax.fill(uelpx, uelpy,'b')
    rectangle_corner=[]
    for i in (rectangles):

              x = [i[0]-(i[2]/2), i[0]+(i[2]/2),i[0]+(i[2]/2), i[0]-(i[2]/2)]
              y = [i[1]-(i[3]/2), i[1]-(i[3]/2), i[1]+(i[3]/2),i[1]+(i[3]/2)]
              for j in x:
                  xs.append(j)
              for j in y:
                  ys.append(j)
              rectangle_corner.append([x,y])
#              ax.fill(x, y,'r',edgecolor='b')
    ucir1x=[]
    ucir1y=[]
    for i in range(len(p1)):
        ucir1x.append(p1[i]+radius*np.cos(t))
        ucir1y.append(q1[i]+radius*np.sin(t))
    ucir2x=[]
    ucir2y=[]
    for i in range(len(p2)):
        ucir2x.append(p2[i]+radius*np.cos(t))
        ucir2y.append(q2[i]+radius*np.sin(t))
    ucir3x=[]
    ucir3y=[]
    for i in range(len(p3)):
        ucir3x.append(p3[i]+radius*np.cos(t))
        ucir3y.append(q3[i]+radius*np.sin(t))
    ucir4x=[]
    ucir4y=[]
    for i in range(len(p4)):
        ucir4x.append(p4[i]+radius*np.cos(t))
        ucir4y.append(q4[i]+radius*np.sin(t))
    ucap1x=[]
    ucap1y=[]
    for i in range(len(r1)):
        ucap1x.append(r1[i]+radius*np.cos(t))
        ucap1y.append(s1[i]+radius*np.sin(t))
    ucap2x=[]
    ucap2y=[]
    for i in range(len(r2)):
        ucap2x.append(r2[i]+radius*np.cos(t))
        ucap2y.append(s2[i]+radius*np.sin(t))
    uboxx=[]
    uboxy=[]
    for i in range(4):
        uboxx.append(r[i]+radius*np.cos(t))
        uboxy.append(s[i]+radius*np.sin(t) )
    urecBoxes=[]
    
    for i in rectangle_corner:
        
        uboxrx=[] 
        uboxry=[]
        
        for j in range(4):
            
            uboxrx.append(i[0][j]+radius*np.cos(t))
            uboxry.append(i[1][j]+radius*np.sin(t) )
        urecBoxes.append([uboxrx,uboxry])

    animate([[ucir1x, ucir1y,'b'],[p1, q1,'r'],[ucir2x, ucir2y,'b'], [p2, q2,'r'],[ucir3x, ucir3y,'b'],[p3, q3,'r'], [ucir4x, ucir4y,'b'],[p4, q4,'r'],[ucap1x, ucap1y,'b'],[r1, s1,'r'],[ucap2x, ucap2y,'b'], [r2, s2,'r'],[uboxx, uboxy,'b'],[r, s,'r']],[[urecBoxes,'b'],[rectangle_corner,'r']])
    if(init and final):
        
        resolution=res
        radius=radius/resolution
        clearance=clearance/resolution
        init=[init[0]/resolution,init[1]/resolution]
        final=[final[0]/resolution,final[1]/resolution]
        
        
        r1 = (0.81/2)/resolution
        n1=-1.65/resolution    #x-position of the center
        m1=4.6/resolution   #radius on the y-axis
        p1=n1+r1*np.cos(t)
        q1=m1+r1*np.sin(t)
        for i in p1:
            xs.append(i)
        for i in q1:
            ys.append(i)
        #Circle 2
        r2 =( 0.81/2)/resolution
        n2=-1.17 /resolution    #x-position of the center
        m2=2.31/resolution   #radius on the y-axis
        p2=n2+r2*np.cos(t)
        q2=m2+r2*np.sin(t)
        for i in p2:
            xs.append(i)
        for i in q2:
            ys.append(i)
        #Circle 3
        r3 = (0.81/2)/resolution
        n3=-1.17/resolution    #x-position of the center
        m3=-2.31/resolution   #radius on the y-axis
        p3=n3+r3*np.cos(t)
        q3=m3+r3*np.sin(t)
        for i in p3:
            xs.append(i)
        for i in q3:
            ys.append(i)
        #Circle 4
        r4 = (0.81/2)/resolution
        n4=-1.65 /resolution    #x-position of the center
        m4=-4.6 /resolution  #radius on the y-axis
        p4=n4+r4*np.cos(t)
        q4=m4+r4*np.sin(t)
        for i in p4:
            xs.append(i)
        for i in q4:
            ys.append(i)
        #Capsule
        u=-3.2516/resolution     #x-position of the center
        v=3.2505/resolution    #y-position of the center
          
        a=(((3.1968/resolution)-(1.599/resolution))/2)   #radius on the x-axis
        b=(1.599/2)/resolution    #radius on the y-axis
        r = [u-a, u+a,u+a, u-a]
        s = [v-b, v-b, v+b,v+b]
        for i in r:
            xs.append(i)
        for i in s:
            ys.append(i)
        u1=u-a
        u2=u+a
        r1=u1+b*np.cos(t)
        s1=v+b*np.sin(t)
        r2=u2+b*np.cos(t)
        s2=v+b*np.sin(t)
        for i in r1:
            xs.append(i)
        for i in s1:
            ys.append(i)
        for i in r2:
            xs.append(i)
        for i in s2:
            ys.append(i)
    #    #Rectangles
        rectangles =[[3.2,4.135,0.86,1.83],[4.495,4.595,0.43,0.91],[3.72,1.54,3.66,0.76],
        [5.26,0.02,0.58,1.17],[5.095,-0.995,0.91,0.86],[5.26,-2.6825,0.58,1.17],
        [4.635,-4.32,1.83,0.76],[2.825,-4.41,1.17,0.58],[0.56,-3.94,2.74,1.52],
        [-0.715,-0.985,0.91,1.83],[0.660,-2.02,1.83,0.76],[3.06,-1.795,1.52,1.17]]
        for i in range(len(rectangles)):
            for j in range(4):
                rectangles[i][j]=rectangles[i][j]/resolution
    
    #    fig, ax = plt.subplots()
    #    
    ##
    #    ax.fill(r,s,'r',edgecolor='b')
    #    ax.fill(r1,s1,'r')
    #    ax.fill(r2,s2,'r')
    #    ax.fill(p1,q1,'r',edgecolor='b')
    #    ax.fill(p2,q2,'r',edgecolor='b')
    #    ax.fill(p3,q3,'r',edgecolor='b')
    #    ax.fill(p4,q4,'r',edgecolor='b')
        # ax.fill(uelpx, uelpy,'b')
        rectangle_corner=[]
        for i in (rectangles):
              
                  x = [i[0]-(i[2]/2), i[0]+(i[2]/2),i[0]+(i[2]/2), i[0]-(i[2]/2)]
                  y = [i[1]-(i[3]/2), i[1]-(i[3]/2), i[1]+(i[3]/2),i[1]+(i[3]/2)]
                  for j in x:
                      xs.append(j)
                  for j in y:
                      ys.append(j)
                  rectangle_corner.append([x,y])
    #              ax.fill(x, y,'r',edgecolor='b')
        ucir1x=[]
        ucir1y=[]
        for i in range(len(p1)):
            ucir1x.append(p1[i]+radius*np.cos(t))
            ucir1y.append(q1[i]+radius*np.sin(t))
        ucir2x=[]
        ucir2y=[]
        for i in range(len(p2)):
            ucir2x.append(p2[i]+radius*np.cos(t))
            ucir2y.append(q2[i]+radius*np.sin(t))
        ucir3x=[]
        ucir3y=[]
        for i in range(len(p3)):
            ucir3x.append(p3[i]+radius*np.cos(t))
            ucir3y.append(q3[i]+radius*np.sin(t))
        ucir4x=[]
        ucir4y=[]
        for i in range(len(p4)):
            ucir4x.append(p4[i]+radius*np.cos(t))
            ucir4y.append(q4[i]+radius*np.sin(t))
        ucap1x=[]
        ucap1y=[]
        for i in range(len(r1)):
            ucap1x.append(r1[i]+radius*np.cos(t))
            ucap1y.append(s1[i]+radius*np.sin(t))
        ucap2x=[]
        ucap2y=[]
        for i in range(len(r2)):
            ucap2x.append(r2[i]+radius*np.cos(t))
            ucap2y.append(s2[i]+radius*np.sin(t))
        uboxx=[]
        uboxy=[]
        for i in range(4):
            uboxx.append(r[i]+radius*np.cos(t))
            uboxy.append(s[i]+radius*np.sin(t) )
        urecBoxes=[]
      
        for i in rectangle_corner:
          
            uboxrx=[] 
            uboxry=[]
           
            for j in range(4):
                
                uboxrx.append(i[0][j]+radius*np.cos(t))
                uboxry.append(i[1][j]+radius*np.sin(t) )
            urecBoxes.append([uboxrx,uboxry])
        return rectangle_corner,[[ucir1x, ucir1y,'b'],[p1, q1,'r'],[ucir2x, ucir2y,'b'], [p2, q2,'r'],[ucir3x, ucir3y,'b'],[p3, q3,'r'], [ucir4x, ucir4y,'b'],[p4, q4,'r'],[ucap1x, ucap1y,'b'],[r1, s1,'r'],[ucap2x, ucap2y,'b'], [r2, s2,'r'],[uboxx, uboxy,'b'],[r, s,'r']],[[urecBoxes,'b'],[rectangle_corner,'r']]
    else:
        return "Please enter both Initial and Final Points",[],[]

def handle_close(evt):
    print('Closed Animation!')
    test.quit()




    
actions= getactions(50,100)
resolution=1

radius = 0.35/2 
clearance=0.1
ox, oy = [], []
rect_corners,listPnts,rectangles=getxs_ys(ox,oy)
#     start and goal position
#print(init,final,radius,clearance)


    

if(init and final):
       sx=init[0]
       sy=init[1]
       gx=final[0]
       gy=final[1]
            
       if show_animation:  # pragma: no cover
                 fig,ax1=plt.subplots()
                 for i in (listPnts):
                        
                        ax1.fill(i[0],i[1], color = i[2])
                 for i in (rectangles):
                
                        for k in i[0]:
                            ax1.fill(k[0],k[1], color = i[1])
                 plt.grid(True)
                
        
       fpath=[]
       fpathv=[] 
      
       nodes,gnode = a_star_rrt_planning(sx, sy, gx, gy, ox, oy, resolution, radius,rect_corners,actions,plt)
       if(gnode==0):
           test=tk.Tk()
           test.geometry('400x300')
           label = Label(test, text= nodes)
   
           label.pack() 
   
           test.mainloop()
       else:
           test=tk.Tk()
           fig = plt.Figure(figsize=(5,4), dpi=100)
           ax = fig.add_subplot(111)
           
           line, = ax.plot([], [], 'y.',lw=0.3, alpha=0.2)
           ax.grid()
   
           scatter = FigureCanvasTkAgg(fig, test) 
           scatter.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH)
         
           for i in (listPnts):
                
                ax.fill(i[0],i[1], color = i[2])
           for i in (rectangles):
        
                for k in i[0]:
                    ax.fill(k[0],k[1], color = i[1])
                       
   
           ax.legend()
           ax.grid(color=(0,0,0), linestyle='-', linewidth=1) 
   
           ax.set_title(title+' (Close window to stop animation)');
           ax.set_xlabel('X axis')
           ax.set_ylabel('Y axis')
           fig.canvas.mpl_connect('close_event', handle_close)
 
           ani = animation.FuncAnimation(fig, animated, nodes, fargs=(nodes, gnode,test), interval=10,repeat=False, blit=False)
   




   
       
           test.mainloop()
           gnode.PrintTreePlot(plt,fpath,fpathv)

           plt.xticks(np.arange(-5.55/resolution, 6/resolution, 1/resolution))
           plt.yticks(np.arange(-5.05/resolution, 5.5/resolution, 1/resolution))
           plt.show()
           text_file = open("VREP/FinalPathV.txt", "w")
           for i in fpathv:
                text_file.write(str(i[0])+','+str(i[1])+'  ')
            
           text_file.close()
           text_file = open("VREP/FinalPath.txt", "w")
           for i in fpath:
                text_file.write(str(i[0])+','+str(i[1])+','+str(i[2])+','+str(i[3])+','+str(i[4])+'  ')
            
           text_file.close()
else:
           test=tk.Tk()
           test.geometry('500x200')
           label = Label(test, text= "Please check validity if Initial/Goal Coordinates.")
   
           label.pack() 
   
           test.mainloop()
        #    sys.exit("Program Ended")



end = time.time()
print('Execution time - ', end - start)
