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
title='Click point in map to select Initial/Final point.'
arr=[]
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
        
    else:
        print('final')
        final=[int(event.xdata),int(event.ydata)]
        
    title='Node Exploration'
    return True

def updateMinMax(arr,minx,miny,maxx,maxy,d):
        if(maxx>arr[2]):
#            print('x max')
            arr[2]=maxx+1+d
        if(minx<arr[0]):
#            print('x min')
            arr[0]=minx-1-d
        if(maxy>arr[3]):
#            print('y max')
            arr[3]=maxy+1+d
        if(miny<arr[1]):
#            print('y min')
            arr[1]=miny-1-d
            
def pathAvailability(x,y,arr,pol,maxPx,minPx,maxPy,minPy):
    """
    Box
    """
    global radius,clearance,resolution
    d=radius+clearance

    if(((y-((112.5/resolution)+d))<=0) and ((x-((100/resolution)+d))<=0) and ((-y+((67.5/resolution)-d))<=0) and ((-x+((50/resolution)-d))<=0)):

         maxBx=100
         minBx=50
         maxBy=112.5
         minBy=67.5
         
         updateMinMax(arr,minBx,minBy,maxBx,maxBy,d)
         return 0
    #     xpolygon=[120,158, 165,188,168,145];
# %   ypolygon=[55,51,89,51,14,14];
    # % Line 2 with coordinates (125,56) and (150,15)
    p2 = Point(x,y)
   
    for i in pol:
        coords = i
        poly = Polygon(i)

        inside2 = p2.within(poly)
        if(inside2==True):
            break
 
       
    if(inside2==True):
            updateMinMax(arr,minPx,minPy,maxPx,maxPy,d)
            return 0
    if((((math.pow((x-(140/resolution)),2)/math.pow(((15/resolution)+d),2))+(math.pow((y-(120/resolution)),2)/math.pow(((6/resolution)+d),2))-1)<=0)):
        maxEx=140+15
        minEx=140-15
        maxEy=120+6
        minEy=120-6
       
        updateMinMax(arr,minEx,minEy,maxEx,maxEy,d)
        
        return 0
    if((((math.pow((x-(190/resolution)),2))+(math.pow((y-(130/resolution)),2))-(math.pow(((15/resolution)+d),2)))<=0)):
        maxCx=190+15
        minCx=190-15
        maxCy=130+15
        minCy=130-15
        
        updateMinMax(arr,minCx,minCy,maxCx,maxCy,d)
        
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
    
    q,seen,mins,queue= [(0,Node(f,0,tempx,tempy))],set(), {f: 0},[(0,f)]
    nodes=[]

    count=0
    nodes.append(Node(f,0,tempx,tempy))
    node=''
    while (q and node!=t):
        (cost1,node)=heappop(queue)
       
        index= [i for ((c,y), i) in zip(q, range(len(q))) if node==y.node]
        
        (cost,v1) = q.pop(index[0])
        

        temp_trav(weightx,weighty,costw,final,graph,g,q,queue,nodes,v1,seen,mins,pol)

    
        
    ans= [v for  v in (nodes) if v.node == t]
    print(ans)
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
    tk.Label(root, text="Resolution").pack()
    resolution=Entry(root)
    resolution.pack()
    tk.Label(root, text="Radius of Robot").pack()
    radius=Entry(root)
    radius.pack()
    tk.Label(root, text="Clearance of Robot").pack()
    clearance=Entry(root)
    clearance.pack()

    tk.Button(root, text="Quit", command= lambda:quit(initial,final1,resolution,radius,clearance)).pack()
   
    root.mainloop()
       
    return listPnts


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

def quit(initial,final1,res,rad,clr):
    global root,init,final,radius,resolution,clearance,arr

    if((res.get()) and float(res.get())):
        resolution=float(res.get())
        if(initial.get()):
            if(len((initial.get()).split(','))==2):

                x,y=(initial.get()).split(',')
                if(x and y and (int(x)) and (int(y))):
                    init=[int(int(x)/resolution),int(int(y)/resolution)]
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
            
            init=[int(init[0]/resolution),int(init[1]/resolution)]
            
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
                if(x1 and y1 and (int(x1)) and (int(y1))):
                    final=[int(int(x1)/resolution),int(int(y1)/resolution)]
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
            final=[int(final[0]/resolution),int(final[1]/resolution)]
        else:
            root.quit()
            root.destroy()
            test=tk.Tk()
            test.geometry('400x300')
            label = Label(test, text= "Please enter valid Final Point.")

            label.pack() 

            test.mainloop()
        if((rad.get()) and (int(int(rad.get())/resolution))>=0):
            radius=int(int(rad.get())/resolution)
        else:
            root.quit()
            root.destroy()
            test=tk.Tk()
            test.geometry('400x300')
            label = Label(test, text= "Please enter valid Radius.")

            label.pack() 

            test.mainloop()
        if((clr.get()) and (int(int(clr.get())/resolution))>=0):
            clearance=int(int(clr.get())/resolution)
            root.quit()
            root.destroy()
            minx = min(final[0],init[0])-1
            miny = min(final[1],init[1])-1
            maxx= max(final[0],init[0])+1
            maxy= max(final[1],init[1])+1
            arr=[minx,miny,maxx,maxy]
        else:
            root.quit()
            root.destroy()
            test=tk.Tk()
            test.geometry('400x300')
            label = Label(test, text= "Please enter valid Clearance.")

            label.pack() 

            test.mainloop()
    else:
        root.quit()
        root.destroy()
        test=tk.Tk()
        test.geometry('400x300')
        label = Label(test, text= "Please enter valid Resolution.")

        label.pack() 

        test.mainloop()
   
    
def temp_trav(weightx,weighty,cost,final,graph,g,q,queue,nodes,parent,seen,mins,pol):
    global arr,maxPx,minPx,maxPy,minPy
    flag=0
    tempx=parent.x
    tempy=parent.y
    global radius,clearance,resolution
    d=radius+clearance
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
                

                if((tup not in graph) and (tupin not in graph) and (x>=0 and x<=((250/resolution)+radius) and y>=0 and y<=((150/resolution)+radius)) and (((x+d)<=(250/resolution)) and ((y+d)<=(150/resolution)) and ((x-d)>=(0/resolution)) and ((y-d)>=(0/resolution)))):


                    

                    if(((pathAvailability(x,y,arr,pol,maxPx,minPx,maxPy,minPy))==1) and (x>=(arr[0]) and y>=(arr[1])) and ( x<=(arr[2]) and y<=(arr[3]) )):

                        graph.append(tup)

                        test.append((x,y))


                        if(b not in seen):
                            seen.add(b)
                            dis=np.sqrt(np.square((final[0])-(x)) + np.square((final[1])-(y)))

                            next = (costw+parent.cost+dis)
                            v2=(Node(b,(next),x,y))
                            v2.parent=parent
                            mins[b] = next
                            nodes.append(v2)
                            q.append((next,v2))
                            heappush(queue, (next, b))
                        else:
                            
                            ans= [v for v in (nodes) if v.node == b]
                            # ans1= [i for i, v in (queue) if v == b]
                            prev = mins.get(b, None)

                            dis=np.sqrt(np.square((final[0])-(x)) + np.square((final[1])-(y)))

                            next = (costw+parent.cost+dis)
                            if prev is None or next < prev:
                                    ans[0].parent=parent
                                    mins[b] = next
                                    

                                    ans[0].cost=next
                                    
                                    # ans1[0]=next

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


xs=[]
ys=[]
k=0
pol=[]
for i in testing:

    array=i.get_xy()

    polygon_vertices=[]
    for j in array:


        polygon_vertices.append((j[0],j[1]))

    pol.append(polygon_vertices)



maxPx=0
minPx=250
maxPy=0
minPy=150
for i in pol:
        coords = i
        poly = Polygon(i)
        for j in i:
            
            if(minPx>j[0]):
                
                minPx=j[0]
            if(maxPx<j[0]):
                maxPx=j[0]
            if(minPy>j[1]):
                minPy=j[1]
            if(maxPy<j[1]):
                maxPy=j[1]
print(minPx,minPy,maxPx,maxPy)
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
print("Processing.....")
if(init and final):
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
        
        line, = ax.plot([], [], 'y.',lw=0.3, alpha=0.2)
        ax.grid()

        scatter = FigureCanvasTkAgg(fig, test) 
        scatter.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH)
    
        for i in (listPnts):
                ax.fill(i[0],i[1], color = i[2])
                    

        ax.legend()
        ax.grid(color=(0,0,0), linestyle='-', linewidth=1) 

        ax.set_title(title);
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')

        ani = animation.FuncAnimation(fig, animated, nodes, fargs=(nodes, node,test), interval=10,repeat=False, blit=False)



    
        test.mainloop()
else:
        test=tk.Tk()
        test.geometry('400x300')
        label = Label(test, text= "Please check validity if Initial/Goal Coordinates, Resolution, Radius or Clearance.")

        label.pack() 

        test.mainloop()