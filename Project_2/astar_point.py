# -*- coding: utf-8 -*-
"""
Created on Mon Apr  1 20:58:36 2019

@author: Sneha
"""

# -*- coding: utf-8 -*-
"""
Created on Tue Mar 26 13:26:27 2019

@author: Sneha
"""
import tkinter as tk
from tkinter import *
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np 
import matplotlib.pyplot as plt
import math
from collections import deque, namedtuple
import sys
from collections import defaultdict
from heapq import *
import matplotlib.animation as animation

from shapely.geometry import Point, Polygon
import time
title='Click point in map to select initial point.'

root= tk.Tk()
class Node:
    def __init__(self, node, cost,x,y):
        self.node = node
        self.parent = None
        self.x=x
        self.y=y
        self.cost = cost
        
    

    
   
# Print the tree
    def PrintTree(self,ax):
        if self.parent:
            self.parent.PrintTree(ax)
        # print( 'Node',self.node,' ,Cost',self.cost)
        ax.scatter(self.x,self.y,s=10,c='b')
init=[]
final=[]
resolution=1
radius=0
clearance=0

# we'll use infinity as a default distance to nodes.
inf = float('inf')
Edge = namedtuple('Edge', 'start, end, cost')

def onpick(event):
    print(event.xdata,event.ydata)
    global init,final,title
    if(not(init)):
        print('init')
        init=[int(event.xdata),int(event.ydata)]
        title='Click point in map to select initial point.' 
    else:
        print('final')
        final=[int(event.xdata),int(event.ydata)]
        title='Choose algorithm.' 
    # init=[0,0]
    # final=[40,43]
    return True
def updateMinMax(x,y,arr):
        if(x>arr[2]):
#            print('x max')
            arr[2]=x+1
        if(x<arr[0]):
#            print('x min')
            arr[0]=x-1
        if(y>arr[3]):
#            print('y max')
            arr[3]=y+1
        if(y<arr[1]):
#            print('y min')
            arr[1]=y-1
            
def pathAvailability(x,y,arr,pol):
    """
    Box
    """
    global radius,clearance,resolution
    d=radius+clearance
    if(((y-((112.5/resolution)+d))<=0) and ((x-((100/resolution)+d))<=0) and ((-y+((67.5/resolution)-d))<=0) and ((-x+((50/resolution)-d))<=0)):
        
         updateMinMax(x,y,arr)
         return 0
    p2 = Point(x,y)

    for i in pol:
        coords = i
        poly = Polygon(i)
        inside2 = p2.within(poly)
        if(inside2==True):
            break
 
       
    if(inside2==True):
            # print(x,y,'in poly')
            updateMinMax(x,y,arr)
            return 0
    if((((math.pow((x-(140/resolution)),2)/math.pow(((15/resolution)+d),2))+(math.pow((y-(120/resolution)),2)/math.pow(((6/resolution)+d),2))-1)<=0)):
        updateMinMax(x,y,arr)
        # print(x,y,'in ellipse')
        return 0
    if((((math.pow((x-(190/resolution)),2))+(math.pow((y-(130/resolution)),2))-(math.pow(((15/resolution)+d),2)))<=0)):
        updateMinMax(x,y,arr)
        # print(x,y,'in circle')
        return 0
    
    else:
        
        return 1
   
    
           
def make_edge(start, end, cost=1):
  return Edge(start, end, cost)
def test(algo_type):
  print(algo_type)
def sorting(vals):
    print(vals)
    return vals
def getKey(item):
    return item[0]
def astar(graph,f,t,paths_to_goal,tempx,tempy,weightx,weighty,costw,final,pol):
    path=[]
    paths_to_goal=[]
    
    
    count=-1
    path=0
    queue=[]
    queue.append((tempx,tempy))
    g = defaultdict(list)
    
    q,seen,mins= [(0,Node(f,0,tempx,tempy))],set(), {f: 0}
    nodes=[]

    count=0
    while q:
        
        (cost,v1) = q.pop(0)
        nodes.append(v1)
        temp_trav(weightx,weighty,costw,final,graph,g,q,v1,seen,mins,pol)
        
    
        
    ans= [v for  v in (nodes) if v.node == t]
    
    if(len(ans)>0):
        return nodes,ans[0]
    else:
        return 'Initial/Final Point in Obstacle!!',0

                    
def animate(listPnts):

    global title,root,final,init
    fig = plt.Figure(figsize=(5,4), dpi=100)
    ax = fig.add_subplot(111)
    
    
    scatter = FigureCanvasTkAgg(fig, root) 
    scatter.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH)
    ax.fill([250,0,0,250],[150,150,0,0], color = (0,0,0))
    
    for i in (listPnts):
        ax.fill(i[0],i[1], color = i[2])
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
    return listPnts

    
def animated(i,nodes,node,test):
    
    if(i):
        if(((nodes[0].x) == i.x) and (nodes[0].y == i.y)):
          ax.scatter(int(i.x), int(i.y),s=5,c='r')
        else:
            ax.scatter(int(i.x), int(i.y),s=5,c='y')
            
        if(((nodes[len(nodes)-1].x) == i.x) and (nodes[len(nodes)-1].y == i.y)):
          node.PrintTree(ax)
          
        
def quit(initial,final1):
    global root,init,final,radius,resolution,clearance
    resolution=1
    if(initial.get()):
        x,y=(initial.get()).split(',')
        init=[int(int(x)/resolution),int(int(y)/resolution)]
    elif(init):
        
        init=[int(init[0]/resolution),int(init[1]/resolution)]
        print('Init in quit',init)
    
    if(final1.get()):
        x1,y1=(final1.get()).split(',')
        final=[int(int(x1)/resolution),int(int(y1)/resolution)]
    elif(final):
        final=[int(final[0]/resolution),int(final[1]/resolution)]
    
    
    radius=0
    clearance=0
    root.quit()
    root.destroy()
    
    
def temp_trav(weightx,weighty,cost,final,graph,g,q,parent,seen,mins,pol):
    
    flag=0
    tempx=parent.x
    tempy=parent.y
    global radius,clearance,resolution

    minx = min(final[0],init[0])-1
    miny = min(final[1],init[1])-1
    maxx= max(final[0],init[0])+1
    maxy= max(final[1],init[1])+1

    for i in range(8):
                
                x=tempx+weightx[i]
                y=tempy+weighty[i]
                
                costw=cost[i]
                a=str(tempx)+' '+str(tempy)
                b=str(x)+' '+str(y)
                tup=(a,b,costw)
                tupin=(b,a,costw)
                
                if((tup not in graph) and (tupin not in graph) and (x>=0 and x<=((250/resolution)+radius) and y>=0 and y<=((150/resolution)+radius))):

                    arr=[minx,miny,maxx,maxy]
                    if((pathAvailability(x,y,arr,pol)==1) and (x>=(minx) and y>=(miny)) and ( x<=(maxx) and y<=(maxy) )):  
                       
                        graph.append(tup)
                        test.append((x,y))
                        if(b not in seen):
                            seen.add(b)
                            dis=np.sqrt(np.square((parent.x)-(x)) + np.square((parent.y)-(y)))

                            next = (costw+parent.cost+dis)
                            v2=(Node(b,(next),x,y))
                            v2.parent=parent
                            mins[b] = next
                            q.append((next,v2))
                        else:
                            
                            ans= [v for i, v in (q) if v.node == b]
                            
                            prev = mins.get(b, None)
                            dis=np.sqrt(np.square((parent.x)-(x)) + np.square((parent.y)-(y)))

                            next = (costw+parent.cost+dis)
                            if prev is None or next < prev:
                                    ans[0].parent=parent
                                    mins[b] = next
                                    
                                    ans[0].cost=next
                    else:
                        minx=arr[0]
                        miny=arr[1]
                        maxx=arr[2]
                        maxy=arr[3]
                    
 
    
t = np.linspace(0, 2*np.pi, 100)

r = 15
n=190     #x-position of the center
m=130   #radius on the y-axis


u=140     #x-position of the center
v=120    #y-position of the center
a=15    #radius on the x-axis
b=6    #radius on the y-axis
p=n+r*np.cos(t)
q=m+r*np.sin(t)
r=u+a*np.cos(t)
s=v+b*np.sin(t)
x = [50, 100, 100, 50]
y = [112.5, 112.5, 67.5, 67.5]
px=[125,163,170,193,173,150]
py=[56,52,90,52,15,15]

fig, ax = plt.subplots()
ax.grid(color=(0,0,0), linestyle='-', linewidth=1)
test=[]



xs=[]
ys=[]

uboxx=[]
uboxy=[]
for i in range(4):
    uboxx.append(x[i]+radius*np.cos(t))
    uboxy.append(y[i]+radius*np.sin(t) )


upolx=[]
upoly=[]
for i in range(6):
    upolx.append(px[i]+radius*np.cos(t))
    upoly.append(py[i]+radius*np.sin(t) )
    
ucirx=[]
uciry=[]
for i in range(len(r)):
    ucirx.append(p[i]+radius*np.cos(t))
    uciry.append(q[i]+radius*np.sin(t))
    
uelpx=[]
uelpy=[]
for i in range(len(r)):
    uelpx.append(r[i]+radius*np.cos(t))
    uelpy.append(s[i]+radius*np.sin(t))

listPnts=animate([[uboxx, uboxy,'b'],[x, y,'r'],[upolx, upoly,'b'], [px, py,'r'],[ucirx, uciry,'b'],[p,q,'r'],[uelpx, uelpy,'b'],[r,s,'r']])

r = 15/resolution
n=190/resolution   #x-position of the center
m=130/resolution  #radius on the y-axis


u=140/resolution     #x-position of the center
v=120/resolution    #y-position of the center
a=15/resolution   #radius on the x-axis
b=6/resolution   #radius on the y-axis
#plt.gca().set_aspect('equal')
p=n+r*np.cos(t)
q=m+r*np.sin(t)
r=u+a*np.cos(t)
s=v+b*np.sin(t)
x = [50/resolution, 100/resolution, 100/resolution, 50/resolution]
y = [112.5/resolution, 112.5/resolution, 67.5/resolution, 67.5/resolution]
px=[125/resolution,163/resolution,170/resolution,193/resolution,173/resolution,150/resolution]
py=[56/resolution,52/resolution,90/resolution,52/resolution,15/resolution,15/resolution]


uboxx=[]
uboxy=[]
for i in range(4):
    uboxx.append(x[i]+radius*np.cos(t))
    uboxy.append(y[i]+radius*np.sin(t) )


upolx=[]
upoly=[]
in_x=[]
in_y=[]
for i in range(6):
    temp_x=px[i]+radius*np.cos(t)
    temp_y=py[i]+radius*np.sin(t) 
    for j in temp_x:
        in_x.append(j)
    for k in temp_y:
        in_y.append(j)
    upolx.append(temp_x)
    upoly.append(temp_y)



ucirx=[]
uciry=[]
for i in range(len(r)):
    ucirx.append(p[i]+radius*np.cos(t))
    uciry.append(q[i]+radius*np.sin(t))
    
uelpx=[]
uelpy=[]
for i in range(len(r)):
    uelpx.append(r[i]+radius*np.cos(t))
    uelpy.append(s[i]+radius*np.sin(t))

ax.fill(uboxx, uboxy,'b')
ax.fill(x, y,'r')
testing=ax.fill(upolx, upoly,'b')
ax.fill(px, py,'r')
ax.fill(ucirx, uciry,'b')
ax.fill(p,q,'r')
ax.fill(uelpx, uelpy,'b')
ax.fill(r,s,'r')
print(testing[0].get_xy())

xs=[]
ys=[]
k=0
pol=[]
for i in testing:
    # print('Is',i)
    arr=i.get_xy()
    print('arr',arr)
    polygon_vertices=[]
    for j in arr:
        # ax.clear()
        print(j[0])
        polygon_vertices.append((j[0],j[1]))
            # xs.append(j[0])
            # ys.append(j[1])
    pol.append(polygon_vertices)





obstacles=[[uboxx, uboxy],[upolx, upoly],[ucirx, uciry],[uelpx, uelpy]]
weightx=[0,1,1,1,0,-1,-1,-1]
weighty=[1,1,0,-1,-1,-1,0,1]
cost=[1,np.sqrt(2),1,np.sqrt(2),1,np.sqrt(2),1,np.sqrt(2)]

graph=[]
tempx=init[0]
tempy=init[1]


pathx=[]
pathy=[]
paths_to_goal=[]

plt.tick_params(axis='both', which='major', labelsize=9)
nodes,node=astar(graph,str(init[0])+' '+str(init[1]),
                        str(final[0])+' '+str(final[1]),paths_to_goal,tempx,tempy,weightx,weighty,cost,final,pol)
if(node==0):
    test=tk.Tk()
    test.geometry('400x300')
    label = Label(test, text= nodes)
    label.pack() 
    test.mainloop()
else:
    listPnts=[[uboxx, uboxy,'b'],[x, y,'r'],[upolx, upoly,'b'], [px, py,'r'],[ucirx, uciry,'b'],[p,q,'r'],[uelpx, uelpy,'b'],[r,s,'r']]
    test=tk.Tk()
    fig = plt.Figure(figsize=(5,4), dpi=100)
    ax = fig.add_subplot(111)
    scatter = FigureCanvasTkAgg(fig, test) 
    scatter.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH)
           
    for i in (listPnts):
            ax.fill(i[0],i[1], color = i[2])
                
    ax.legend()
    ax.grid(color=(0,0,0), linestyle='-', linewidth=1) 
        # ax.set_facecolor((0, 0, 0))
    ax.set_title(title);
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
     
    ani = animation.FuncAnimation(fig, animated, nodes, fargs=(nodes, node,test),repeat=False)
    

  
    test.mainloop()




