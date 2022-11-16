#!/usr/bin/env python3

#This example loads a pickle and shows the map contained within

import pickle
from re import S
import numpy as np
import matplotlib.pyplot as plt

#opens compressed map and assigns start and end points to nodes 
f = open('project3_part1.pickle','rb')
res = pickle.load(f)
grid = res['map']
plt.matshow(grid)
start = res['start']
plt.text(start[0], start[1], 'S', color='r',fontweight='bold')
goal = res['goal']
plt.text(goal[0], goal[1], 'G', color='g',fontweight='bold')
print(grid)


#node class: each node posses a position,parent,f score,g score, and h score as assigned by the heurisitc
class Node:
    def __init__ (self,par = None,pos = None):
      
       self.parent = par
       self.pos = pos
       self.f = 0
       self.g = 0
       self.h = 0

    def __eq__(self,other):
        return self.pos == other.pos

#checks if a node is a wall
def notWall(pos):
   
    if grid[pos[1],pos[0]] == 0:
        return True
    else:
        return False

 #follows node parents from end to source and prints path on map
def getPath(curr):
    path_x = []
    path_y = []
    current = curr

    while current is not None:
        path_x.append(current.pos[0])
        path_y.append(current.pos[1])
        current = current.parent

    plt.plot(path_x,path_y)

#checks if a node is the goal node
def isGoal(pos):
    if pos[0] == goal[0] and pos[1] == goal[1]:
        return True
    else:
        return False

#checks if the node being searched is within the bounds of the map
def inMap(pos):

    if pos[0] > len(grid) or pos[0] < 0 or pos[1] > len(grid[1]) or pos[1] < 0: 
        return False
    else:
        return True

 #the actual A* function implementation 
def aStar():
    #defines an open list and closed list to sort nodes
    ol = []
    cl = []
    #defines starting and ending nodes with null initialized F,G,H scores
    start_node = Node(None,tuple(start))
    start_node.g = 0
    start_node.h = 0
    start_node.f = 0
    target_node = Node(None,tuple(goal))
    target_node.g = 0
    target_node.h = 0
    target_node.f = 0

    ol.append(start_node)

    direction = [ [-1,0],
                  [1,0],
                  [0,1],
                  [0,-1]]
    #runs through nodes until the open list is empty (all nodes checked)
    while len(ol) > 0:
        curr = ol[0]
        curr_index = 0
        #compares F-score of current node vs next node in list and chooses one with lowest value
        for index, node in enumerate(ol):
            if node.f <= curr.f:
                curr = node
                curr_index = index
        
        
        ol.pop(curr_index)
        cl.append(curr)
        #if the node being looked at is the goal, follow the parent path back to the beginning
        if isGoal(curr.pos):
            return getPath(curr)

        #creates children variable for node, looking in each direction
        children = []
       
        for move in direction:
            #creates a position in relation to the current node, looking up,down,left, and right
            pos = (curr.pos[0] + move[0], curr.pos[1] + move[1])
    
            #Checking if node is a valid position-------------
            if inMap(pos) == False:
                continue

            if notWall(pos) == False :
                continue
            #--------------------------------------------------
            
            #add the valid node to list of children
            newNode = Node(curr,pos)
            children.append(newNode)
        
        for child in children:
            if child in cl: 
                continue
            #estimate f score using manhattan distance heuristic for the h score and standard ++ for g score
            child.g = curr.g + 1
            dx = abs(child.pos[1] - target_node.pos[1]) 
            dy = abs(child.pos[0] - target_node.pos[0])
            child.h = 1.5 *  (dx + dy)
            child.f = child.g + child.h
            
            #if node is not already in list or has worse distance score, dont put into open list
            for node in ol:
                if child == node or child.g > node.g:
                    continue
         
            
            ol.append(child)

#runs program and displays map
aStar()
plt.show()

