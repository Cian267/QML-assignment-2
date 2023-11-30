#Group project QML group 1

from gurobipy import *
import numpy as np
import math
import copy
import pandas as pd
import matplotlib.pyplot as plt

# Model



with open("data_small.txt", "r") as f:  # Read the data file
    data = f.readlines()                       

VRP = []                                    # Create array for data related to nodes
i=0                                         # to keep track of lines in data file
for line in data:
    i=i+1
    words = line.split()
    words=[int(i) for i in words]           # Covert data from string to integer
    VRP.append(words)                       # Store node data
VRP = np.array(VRP)


N=VRP[:,0]                                  # Nodes/customers
n=len(N)                                    # Number of nodes
NbrOfVehicles=3                             # Nymber of vehicles
V=range(NbrOfVehicles)                      # Set of vehicles

xc=VRP[:,1]                                 # X-position of nodes
yc=VRP[:,2]                                 # Y-position of nodes
demand=VRP[:,3]                             # demand of customers
rdytime=VRP[:,4]                            # ready time of windows
duetime=VRP[:,5]                            # due time of windows
service=VRP[:,6]                            # service times at customers

capacity=130

A=[(i,j) for i in N for j in N if i!=j]     # set of arcs


# Create array for euclidian distances between nodes 
c=np.zeros((n,n))    
for i in N:
    for j in N:
        c[i][j]=math.sqrt((xc[j] - xc[i])**2 + (yc[j] - yc[i])**2) # Store distance between nodes

