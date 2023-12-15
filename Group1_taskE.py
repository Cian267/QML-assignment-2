#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 12 17:41:06 2023

@author: Moa
"""

from gurobipy import *
import numpy as np
import math
import copy
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

with open("data_small_3dep.txt", "r") as f:      # Read the data file
    data = f.readlines()                       

VRP = []                                    # Create array for data related to nodes
i=0                                         # to keep track of lines in data file
for line in data:
    i=i+1
    words = line.split()
    words=[int(i) for i in words]           # Covert data from string to integer
    VRP.append(words)                       # Store node data
VRP = np.array(VRP)                         # original matrix

nodes=np.vstack((VRP, VRP[:3]))             # matrix data nodes: customers + start/end depots

N_row=nodes[:,0]                            # vector of nodes id: customers + start/end depots
n=len(N_row)                                # Number of nodes

K=3                                         # Number of vehicles
vehicle_capacity = 130
b=[vehicle_capacity]*K

V=range(K)                                  # Set of vehicles
N=range (len (N_row))                       # Set of nodes   
C=range(3,len(N)-3)                         # Set of customers

xc=nodes[:,1]                               # X-position of nodes
yc=nodes[:,2]                               # Y-position of nodes
a=nodes[:,3]                                # demand of nodes
r=nodes[:,4]                                # ready time of windows
d=nodes[:,5]                                # due time of windows
s=nodes[:,6]                                # service times at nodes

                          # vehicle capacities
M=3000                                      # big M

# Create array for euclidian distances between nodes - c(i,j)
c=np.zeros((n,n))    
for i in N:
    for j in N:
        c[i][j]=math.sqrt((xc[j] - xc[i])**2 + (yc[j] - yc[i])**2) # Store distance between nodes
            
#========================================OPTIMIZATION MODEL========================================

## Create optimization model
m = Model('VRPmodel')

## Create Decision Variables

#arc travel - if vehicle v is travelling arc(i,j) it is 1, 0 otherwise
x = {}
for i in N:
    for j in N:
        for v in V:  
            x[i,j,v] = m.addVar(vtype=GRB.BINARY, lb = 0, name="X_%s,%s,%s" %(i,j,v))
            
# start time of service at customer i by vehicle v 
t = {}
for i in N:
    for v in V:
        t[i,v] = m.addVar(vtype=GRB.CONTINUOUS, lb = 0, name="T_%s,%s" %(i,v))

## Objective - total distance traveled, i.e., total cost
obj = (quicksum(c[i,j]*x[i,j,v] for i in N for j in N for v in V))
m.setObjective(obj, GRB.MINIMIZE)

## Constraints    

# All customers visited exactly once, except for depots
for i in C:
    m.addConstr(quicksum(x[i,j,v] for j in N if i != j if j!=0 if j!=1 if j!=2 for v in V )==1, 'conA[' + str(i) + ']-')

# The vehicles leave the depots once each
for v in V:
    m.addConstr(quicksum(x[0,j,v] + x[1,j,v] + x[2,j,v] for j in C) ==1, 'conB[' + ']-')

'''    
# All depots are used once - misunderstood the task
m.addConstr(quicksum(x[0,j,v] for j in C for v in V) <=1, 'conB[' + ']-')
m.addConstr(quicksum(x[1,j,v] for j in C for v in V) <=1, 'conC[' + ']-')
m.addConstr(quicksum(x[2,j,v] for j in C for v in V) <=1, 'conD[' + ']-')
'''

# ("end"-depots are not used as outgoing arc)
m.addConstr(quicksum(x[n-1,j,v] for j in C for v in V) ==0, 'conE[' + ']-')
m.addConstr(quicksum(x[n-2,j,v] for j in C for v in V) ==0, 'conF[' + ']-')
m.addConstr(quicksum(x[n-3,j,v] for j in C for v in V) ==0, 'conG[' + ']-')

# All vehicles return to the same depot as it started from
for v in V:
    m.addConstr(quicksum(x[0,j,v] for j in C) - quicksum(x[i,n-3,v] for i in C) ==0, 'conH[' +  str(v) + ']-')  
    m.addConstr(quicksum(x[1,j,v] for j in C) - quicksum(x[i,n-2,v] for i in C) ==0, 'conI[' +  str(v) + ']-')
    m.addConstr(quicksum(x[2,j,v] for j in C) - quicksum(x[i,n-1,v] for i in C) ==0, 'conJ[' +  str(v) + ']-')

# Capacity constraint
for v in V:
    m.addConstr(quicksum(a[i]*x[i,j,v] for i in C for j in N if i != j if j!=0 if j!=1 if j!=2) <= b[v] , 'conK[' +  str(v) + ']-')

# Incoming and outcoming arc
for h in C:
    for v in V:
         m.addConstr(quicksum(x[i,h,v] for i in N if h !=i ) == quicksum(x[h,j,v] for j in N if h!= j ), 'conL[' + str(h) + ',' + str(v) + ']-')    

# Time window - part 1
for i in N:
    for v in V:
        m.addConstr(t[i,v] >= r[i], 'conN[' + str(i) + ',' + str(v) + ']-')   
            
# Time window- part 2
for i in N:
    for v in V:
          m.addConstr(t[i,v] <= d[i], 'conO[' + str(i) + ',' + str(v) + ']-') 

# Subtour elimination
for i in N:
    for j in N:
        for v in V:
            if j!=i: 
               if j!=0:
                    if j!=1:
                        if j!=2:
                            if i!=n-1:
                                if i!=n-2:
                                    if i!=n-3:
                                        m.addConstr(t[i,v] + c[i,j] + s[i] - max(d[i] + c[i, j] + s[i] - r[j], 0)*(1-x[i,j,v]) <= t[j,v], 'conP[' + str(i) + ',' + str(j) + ',' + str(v) + ']-') 

m.update()
m.Params.timeLimit = 1800 #time limit so optimization will stop after 1000 seconds 
m.optimize()
print("\nTotal distance traveled is: ", m.objVal)

# Plot the routes that are decided to be travelled 
arc_solution = m.getAttr('x', x)
#
fig= plt.figure(figsize=(6,6))
plt.xlabel('x-coordinate')
plt.ylabel('y-coordinate')
plt.scatter(xc[1:n-3],yc[1:n-3])
for i in range(1,n-3):
    plt.annotate(str(i),(xc[i],yc[i]))
plt.plot(xc[0],yc[0],c='y',marker='s')
plt.plot(xc[1],yc[1],c='y',marker='s')
plt.plot(xc[2],yc[2],c='y',marker='s')
       
colors = list(mcolors.TABLEAU_COLORS) 
    
for i in range(n):
    for j in range(n):
        for v in V:  
            if arc_solution[i, j, v] > 0.99:
                plt.plot([xc[i], xc[j]], [yc[i], yc[j]], colors[v])
  
# Display the plot
plt.show()  
 
# 
for v in V:
    print('\nTable of tours of vehicle ', v, ':\n')  
    s=' '
    tt = 0
    for j in range(0,len(N)):
        s = s + '%5s' % j
    
    print(s)
    for i in N:
        s = str(tt)
        tt = tt + 1
        for j in N:
            if int(x[i,j,v].x) > 0.5:
                s = s + '    1'
            else:
                s = s + '    .'
        print (s)       

for v in V:
    step = 0
    ii = 0
    jj = 0
    order = [0]
    time = [0]
  
    
  # Error on below
    '''
    while step !=24:
        if int(x[ii,jj,v].x) > 0.5:
            step = jj
            order.append(step)
            time.append(round(float(t[jj,v].x), 1))
            ii = jj
            jj = 0
        else:
            jj = jj + 1
    
    timetable = np.vstack((order, time))
    
    total_demand = 0
    for i in order:
        total_demand = total_demand + a[i]
    
    load = [total_demand]
    newload = total_demand
    
    for i in order:
        load.append(newload - a[i])
        newload = newload - a[i]
    
    load.pop()
    
    timetable = np.vstack((timetable,load))
    
    print('\nTable of arrivel times of vehicle ', v, ':')  
    s = ''
    tt = 0
    names = ['Node', 'Time', 'Load']
    print(s)
    for i in range(len(names)):
        s = names[i]
        for j in range(len(order)):
            s = s + '%8s' % str(timetable[i,j])
        print (s)
'''
