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
    # print("In pA",x,y)
#    if(x==120 and y==105):
#            plt.scatter(int(x), int(y), s=10,c='g')
    if(((y-((112.5/resolution)+d))<=0) and ((x-((100/resolution)+d))<=0) and ((-y+((67.5/resolution)-d))<=0) and ((-x+((50/resolution)-d))<=0)):
        #  print(x,y,'in box')
#        print(obstacles[0][0])
        
#        plt.scatter(int(x), int(y), s=10,c='g')
#        print(minx,miny,maxx,maxy)
         updateMinMax(x,y,arr)
         return 0
    #     xpolygon=[120,158, 165,188,168,145];
# %   ypolygon=[55,51,89,51,14,14];
    # % Line 2 with coordinates (125,56) and (150,15)
    p2 = Point(x,y)

    for i in pol:
        coords = i
        poly = Polygon(i)
    # path = mpltPath.Path(testing)
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
        
#        print(x,y,'pass')
#        plt.scatter(int(x), int(y), s=10,c='b')
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
def dijkstra(graph,f,t,paths_to_goal,tempx,tempy,weightx,weighty,costw,final,pol):
    path=[]
    paths_to_goal=[]
    #while(q!=(str(final[0])+' '+str(final[1]))):
    
    count=-1
    path=0
    queue=[]
    queue.append((tempx,tempy))
    g = defaultdict(list)
    
    q,seen,mins= [(0,Node(f,0,tempx,tempy))],set(), {f: 0}
    nodes=[]
#        temp_trav(tempx,tempy,weightx,weighty,cost,final,graph,g,queue,obstacles,robr)

    
#    print(seen)
    count=0
    while q:
        
        (cost,v1) = q.pop(0)
        nodes.append(v1)
#        print(len(q))
#        print(cost,v1)
#        v1=(min(mins,key=mins.get))
#        cost = mins.get(v1, None)
#        print(cost)
#        if v1 not in seen:
#            seen.add(v1)
#            path = (v1, path)
#        if v1.node == t: 
#               if((cost,path) not in paths_to_goal):
#                        paths_to_goal.append((cost, path))
#        else:
        temp_trav(weightx,weighty,costw,final,graph,g,q,v1,seen,mins,pol)
        # a=sorted((q), key=getKey)
#        temp_trav(weightx,weighty,cost,final,graph,g,q,parent,seen,mins,obstacles,robr):
#        (cost,v1,path) = heappop(q)
#        if v1 not in seen:
#            seen.add(v1)
#            path = (v1, path)
##            print(path)
#            if v1 == t: 
#                if((cost,path) not in paths_to_goal):
#                    paths_to_goal.append((cost, path))
#                else:
#                    continue
##            else: return 0
##            print(min(mins))
#            a=sorted((g.get(v1, ())), key=getKey)
##            print('here')
#            for c, v2 in a:
#                if v2 in seen: continue
#                prev = mins.get(v2, None)
#                next = cost + c
#                if prev is None or next < prev:
#                    mins[v2] = next
##                    edges.append((v1,v2,next))
#                    heappush(q, (next, v2, path))
    
        
    ans= [v for  v in (nodes) if v.node == t]
    # print(mins)
    # ans[0].PrintTree()
    if(len(ans)>0):
        return nodes,ans[0]
    else:
        return 'Initial/Final Point in Obstacle!!',0
# def astar(edges, f, t):
#     g = defaultdict(list)
#     test=[]
# #    print(g)
#     for l,r,c in edges:
#         g[l].append((c,r))
# #    print(g)
#     q, seen, mins = [(0,f,())], set(), {f: 0}
# #    print(q)
#     count=0
#     while q:
        
#         (cost,v1,path) = heappop(q)
#         if v1 not in seen:
#             seen.add(v1)
#             path = (v1, path)
# #            print(path)
#             if v1 == t: return (cost, path)
            
#             a=sorted((g.get(v1, ())), key=getKey)
            
#             for c, v2 in a:
#                 if v2 in seen: continue
#                 prev = mins.get(v2, None)
#                 x1,y1=v1.split()
#                 x2,y2=v2.split()
#                 dis=np.sqrt(np.square(int(x2)-int(x1)) + np.square(int(y2)-int(y1)))
# #                print(dis)
#                 next = cost + c + dis
#                 if prev is None or next < prev:
#                     mins[v2] = next
#                     heappush(q, (next, v2, path))
                    
def animate(listPnts):
#    X = np.random.rand(100, 1000)
#    xs = np.mean(X, axis=1)
#    ys = np.std(X, axis=1)
 

#    canvas = tk.Canvas(root, width = 300, height = 300)
#    canvas.pack()
    global title,root,final,init
    fig = plt.Figure(figsize=(5,4), dpi=100)
    ax = fig.add_subplot(111)
    
    
    scatter = FigureCanvasTkAgg(fig, root) 
    scatter.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH)
    ax.fill([250,0,0,250],[150,150,0,0], color = (0,0,0))
    
    for i in (listPnts):
        ax.fill(i[0],i[1], color = i[2])
#        
#    line, = ax.fill(listPnts[1][0], listPnts[1][1], 'o', picker=5)  # 5 points tolerance
    ax.legend() 
    ax.set_title(title);
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    
#    scatter._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
#    cursor = Cursor(ax)
#    plt.connect('motion_notify_event', cursor.mouse_move)
    
    fig.canvas.mpl_connect('button_press_event',onpick)

    # btn_blur=tk.Button(root, text="Dijkstra", width=50, command= lambda:test('dijkstra'))
    # btn_blur.pack(anchor=tk.W, expand=False)
    # btn_blur=tk.Button(root, text="A*", width=50, command= lambda:test('astar'))
    # btn_blur.pack(anchor=tk.E, expand=False)

#    while True:
    # master = Tk()

    # e = Entry(master)
    # e.pack()

    # e.focus_set()



    # b = Button(master, text="get", width=10, command=callback)
    # b.pack()

    # mainloop()

    # e = Entry(master, width=50)
    # e.pack()

    # text = e.get()


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
    #print("here")    
    return listPnts

# def callback():
#     print (e.get()) 
# def startAnimation(listPnts,nodes):
    
def animated(i,nodes,node,test):
    print
#    xList = []
#    yList = []
#    for eachLine in dataList:
#        if len(eachLine) > 1:
#            x, y = eachLine.split(',')
#            xList.append(int(x))
#            yList.append(int(y))

#    ax.clear()
#    
#    ax.legend() 
#    ax.set_title(title);
#    ax.set_xlabel('X axis')
#    ax.set_ylabel('Y axis')
#    ax.fill(int(i[0]), int(i[1]),'b')
    if(i):
        if(((nodes[0].x) == i.x) and (nodes[0].y == i.y)):
          ax.scatter(int(i.x), int(i.y),s=5,c='r')
        else:
            ax.scatter(int(i.x), int(i.y),s=5,c='y')
            
        if(((nodes[len(nodes)-1].x) == i.x) and (nodes[len(nodes)-1].y == i.y)):
          node.PrintTree(ax)
          
        
        #   time.sleep(10000)
        #   test.quit()
        #   test.destroy()
        # return i.x

#    point.set_data(i[0], i[1])
def quit(initial,final1,res,rad,clr):
    global root,init,final,radius,resolution,clearance
    # print('here',initial.get(),final1.get(),e3.get())
    resolution=float(res.get())
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
    # init=[141/resolution,42/resolution]
    # final=[190/resolution,50/resolution]
    
    radius=int(int(rad.get())/resolution)
    clearance=int(int(clr.get())/resolution)
    root.quit()
    root.destroy()
    
    
def temp_trav(weightx,weighty,cost,final,graph,g,q,parent,seen,mins,pol):
    
    flag=0
    tempx=parent.x
    tempy=parent.y
    global radius,clearance,resolution
#    graph=[]
#    q=[]
#    q.append((tempx,tempy))
#    checkIfObstacle(tempx, tempy, final)
    minx = min(final[0],init[0])-1
    miny = min(final[1],init[1])-1
    maxx= max(final[0],init[0])+1
    maxy= max(final[1],init[1])+1
#    while(len(q)>0):
#        tempx,tempy=q.pop(0)
    for i in range(8):
                
                x=tempx+weightx[i]
                y=tempy+weighty[i]
#                if(x==120 and y==117):
#                    print("here",tempx,tempy)
                
                costw=cost[i]
                a=str(tempx)+' '+str(tempy)
                b=str(x)+' '+str(y)
                tup=(a,b,costw)
                tupin=(b,a,costw)
                
#                print('Min x',min(final[0],init[0])-1)
#                print('Min y',min(final[1],init[1])-1)
#                print('Max x',max(final[0],init[0])+1)
#                print('Max y',max(final[1],init[1])+1)
#                print(tup)
                
#                print(graph)
                if((tup not in graph) and (tupin not in graph) and (x>=0 and x<=((250/resolution)+radius) and y>=0 and y<=((150/resolution)+radius))):
#                    print(tup)
#                    print('outside',tup)
                    arr=[minx,miny,maxx,maxy]
#                    print(arr,x,y)
#                    arr=[]
                    if((pathAvailability(x,y,arr,pol)==1) and (x>=(minx) and y>=(miny)) and ( x<=(maxx) and y<=(maxy) )):  
#                        print('inside',tup)
                        graph.append(tup)
#                        g[a].append((costw,b))
                        test.append((x,y))
        #                print(x,y,i)
        #                print(x,y,i)
        #                tempx=x
        #                tempy=y
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
                    
#                if x==final[0] and y==final[1]:
#                   return 1;
##                    break;
#                else:
#                    
#                    tempx=x
#                    tempy=y
#                    ans = traverse(tempx,tempy,weightx,weighty,cost,graph,final)
#                    
                    
                
     
#    return graph 
    
t = np.linspace(0, 2*np.pi, 100)

r = 15
n=190     #x-position of the center
m=130   #radius on the y-axis


u=140     #x-position of the center
v=120    #y-position of the center
a=15    #radius on the x-axis
b=6    #radius on the y-axis
#plt.gca().set_aspect('equal')
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

#plt.grid(color='lightgray',linestyle='--')
# print(pol)



obstacles=[[uboxx, uboxy],[upolx, upoly],[ucirx, uciry],[uelpx, uelpy]]
weightx=[0,1,1,1,0,-1,-1,-1]
weighty=[1,1,0,-1,-1,-1,0,1]
cost=[1,np.sqrt(2),1,np.sqrt(2),1,np.sqrt(2),1,np.sqrt(2)]

graph=[]
tempx=init[0]
tempy=init[1]

print(tempx,tempy)

# inside2 = point_inside_polygon(tempx,tempy,polygon_vertices)
# print(inside2)
#for i in range(8):
#    x=init[0]+weightx[i]
#    y=init[1]+weighty[i]
#    plt.plot([x,init[0]], [y,init[1]], linewidth=3)
#for i in range(8):
#    x=final[0]+weightx[i]
#    y=final[1]+weighty[i]
#    plt.plot([x,final[0]], [y,final[1]], linewidth=3)



#print(init,final)
#ans=traverse(tempx,tempy,weightx,weighty,cost,graph,final,minx,miny,maxx,maxy,test)
  




      
#        plt.plot([x,init[0]], [y,init[1]], linewidth=3)
    
#plt.plot([x,init[0]], [y,init[1]], linewidth=3)
#print(graph)
#print(graph)    
#graph1 = Graph(graph)
pathx=[]
pathy=[]
paths_to_goal=[]
#plt.xlabel("Number", fontsize=10)
#
#    # Set y axis label.
#plt.ylabel("Square of Number", fontsize=10)

    # Set size of tick labels.
plt.tick_params(axis='both', which='major', labelsize=9)
#q=dijkstra(graph, "A", "E")
#q=astar(graph,str(init[0])+' '+str(init[1]),
#                str(final[0])+' '+str(final[1]))
# print(str(init[0])+' '+str(init[1]),
                        # str(final[0])+' '+str(final[1]),paths_to_goal,tempx,tempy,weightx,weighty,cost,final,in_x,in_y)
nodes,node=dijkstra(graph,str(init[0])+' '+str(init[1]),
                        str(final[0])+' '+str(final[1]),paths_to_goal,tempx,tempy,weightx,weighty,cost,final,pol)
if(node==0):
    test=tk.Tk()
    test.geometry('400x300')
    label = Label(test, text= nodes)
    #this creates a new label to the GUI
    label.pack() 
    # test.quit()
    test.mainloop()
else:
    listPnts=[[uboxx, uboxy,'b'],[x, y,'r'],[upolx, upoly,'b'], [px, py,'r'],[ucirx, uciry,'b'],[p,q,'r'],[uelpx, uelpy,'b'],[r,s,'r']]
    test=tk.Tk()
    fig = plt.Figure(figsize=(5,4), dpi=100)
    ax = fig.add_subplot(111)
    scatter = FigureCanvasTkAgg(fig, test) 
    scatter.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH)
        #ax.fill([250,0,0,250],[150,150,0,0], color = (0,0,0))
        #   
    for i in (listPnts):
            ax.fill(i[0],i[1], color = i[2])
                
        #    line, = ax.fill(listPnts[1][0], listPnts[1][1], 'o', picker=5)  # 5 points tolerance
    ax.legend()
    ax.grid(color=(0,0,0), linestyle='-', linewidth=1) 
        # ax.set_facecolor((0, 0, 0))
    ax.set_title(title);
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    # ax.xticks(np.arange(0.0, 250/resolution, 20.0/resolution))
    # ax.yticks(np.arange(0,150/resolution, 20.0/resolution))
        #for i in range(len(ans)):  
        #point, = ax.plot(init[0], init[1], 'bo', ms=4) 
        # y = np.unique(ans, axis=0)
        # z = [] 
        # for i in y:
        # z.append(tuple(i)) 
    ani = animation.FuncAnimation(fig, animated, nodes, fargs=(nodes, node,test),repeat=False)
    print('here in main')


    # plt.show()   
    test.mainloop()
#    x,y,c=q.split()
#    
#    tempx=int(x)
#    tempy=int(y)
#    tempc=int(c)
#    path.append((tempx,tempy,tempc))

#for i in (q[1]):
#    x,y=q[i].split()
#    plt.scatter(int(x), int(y), s=10,c='g')
#    pathx.append(x)
#    pathy.append(y)
#for i in range(8):
#    x=final[0]+weightx[i]
#    y=final[1]+weighty[i]
#    plt.plot([x,final[0]], [y,final[1]], linewidth=3)
    
# startAnimation(listPnts,nodes)

#DIJKSTRA
#plt.show()



