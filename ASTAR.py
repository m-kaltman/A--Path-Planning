#!/usr/bin/env python3

#This example loads a pickle and shows the map contained within

import pickle
from re import S
import numpy as np
import matplotlib.pyplot as plt


f = open('project3_part1.pickle','rb')
res = pickle.load(f)
grid = res['map']
plt.matshow(grid)
start = res['start']
plt.text(start[0], start[1], 'S', color='r',fontweight='bold')
goal = res['goal']
plt.text(goal[0], goal[1], 'G', color='g',fontweight='bold')
print(grid)



class Node:
    def __init__ (self,par = None,pos = None):
      
       self.parent = par
       self.pos = pos
       self.f = 0
       self.g = 0
       self.h = 0

    def __eq__(self,other):
        return self.pos == other.pos

def notWall(pos):
   
    if grid[pos[1],pos[0]] == 0:
        return True
    else:
        return False

def getPath(curr):
    path_x = []
    path_y = []
    current = curr

    while current is not None:
        path_x.append(current.pos[0])
        path_y.append(current.pos[1])
        current = current.parent

    plt.plot(path_x,path_y)


def isGoal(pos):
    if pos[0] == goal[0] and pos[1] == goal[1]:
        return True
    else:
        return False


def inMap(pos):

    if pos[0] > len(grid) or pos[0] < 0 or pos[1] > len(grid[1]) or pos[1] < 0: 
        return False
    else:
        return True

def aStar():
    ol = []
    cl = []
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

    while len(ol) > 0:
        curr = ol[0]
        curr_index = 0

        for index, node in enumerate(ol):
            if node.f <= curr.f:
                curr = node
                curr_index = index
        
        
        ol.pop(curr_index)
        cl.append(curr)

        if isGoal(curr.pos):
            return getPath(curr)


        children = []
       
        for move in direction:

            pos = (curr.pos[0] + move[0], curr.pos[1] + move[1])

            if inMap(pos) == False:
                continue

            if notWall(pos) == False :
                continue

            newNode = Node(curr,pos)
            children.append(newNode)
        
        for child in children:
            if child in cl: 
                continue

            child.g = curr.g + 1
            dx = abs(child.pos[1] - target_node.pos[1]) 
            dy = abs(child.pos[0] - target_node.pos[0])
            child.h = 1.5 *  (dx + dy)
            child.f = child.g + child.h

            for node in ol:
                if child == node or child.g > node.g:
                    continue
         
            
            ol.append(child)

           
aStar()
plt.show()

