
import json
from os import stat
from tracemalloc import start
import pandas as pd
import numpy as np 
import seaborn as sb
from queue import PriorityQueue 
import math

#To Set The File's in your own folder
#Load all the data set using json.load
with open(r'C:\Users\eric2\Desktop\CZ3005Lab\Cost.json') as dataCost:
    dataCostLoad = json.load(dataCost)
    

with open(r'C:\Users\eric2\Desktop\CZ3005Lab\Coord.json') as dataCoord:
    dataCoordLoad = json.load(dataCoord)


with open(r'C:\Users\eric2\Desktop\CZ3005Lab\Dist.json') as dataDist:
    dataDistLoad = json.load(dataDist)

with open(r'C:\Users\eric2\Desktop\CZ3005Lab\G.json') as dataG:
    dataGLoad = json.load(dataG)

#Task 2 and 3's Settings
startNode = "1"
endNode = "50"
budget = 287932


#Check datatypes of each file
print('datatype for dataDist' )
print( type(dataDistLoad))
print('datatype for dataG')
print( type(dataGLoad))
print('datatype for dataCost' )
print( type(dataCostLoad))
print('datatype for dataCoord' )
print(type(dataCoordLoad))

#Uninform Cost Search Without actual cost
def ucs(startNode, endNode, adjGraph, distances): 
    #Backtracking Method to create develop a path 
    def backTrack(parent_dict, node):
        path = [node]
        while parent_dict[node] is not None:
            node = parent_dict[node]
            path.append(node)
        #return the nodes path 
        return path 

    print("Uniform Cost Search")
    #Sets up a priority queue to put visited nodes and its distance from the startNode
    # Initialise the dicitionaries, sets and queues
    q = PriorityQueue()
    parent = {startNode: None}
    totalDist = {startNode : 0}
    visited = set()
    q.put((0,startNode))


    #If the queue is not empty we get it and check whether if its visited or not
    while not q.empty():
        distance,element = q.get()
        if element in visited:
            continue
        
        #if its not visited we add to the visited set
        visited.add(element)
        
        #Check if its the endNode and return path and distance
        if element == endNode:
            path = backTrack(parent,element)
            return path,distance
    
        #Check for adjacent nodes and check if they are the closer path to be choosen. 
        for adjacentNode in adjGraph[element]:
            nDistance = distance + distances[element + ',' + adjacentNode] 
            if adjacentNode not in visited and (adjacentNode not in totalDist or totalDist[adjacentNode] > nDistance):
                q.put((nDistance, adjacentNode))
                totalDist[adjacentNode] = nDistance
                parent[adjacentNode] = element
    return None,None


def ucsBudget(startNode, endNode, adjGraph, distances, costs, budget):

    #Need a backtracking algorithm to find the root parent/path
    def backTrack(parent_dict, node_cost):
        #Retrieve tuple but only node is stored.
        node,_ = node_cost
        path = [node]
        #Method to trace back to each node's parent
        while parent_dict[node_cost] is not None:
            node_cost = parent_dict[node_cost]
            node,_ = node_cost
            path.append(node)
        #Return the nodes path
        return path 
    
    q = PriorityQueue()
    parent = {(startNode,0) : None} #Parent dictionary of the nodes
    totalDistance = {(startNode, 0) : 0} #calculate the total distance of the node
    visited = set() #Create a visited set to see if node has been visited
    #Create a minimum cost and distance dictionary of each nodes
    minCost = {} 
    minDist = {} 
    q.put((0, (startNode, 0)))
    while not q.empty():
        (distance, (element,cost)) = q.get()
        #Check if its in minCost or minDist

        #Skip this element because it is already checked or its value its actually more than whatever that has been stored in minCost/minDist
        if element in minDist and element in minCost and minDist[element] <= distance and minCost[element] <= cost:
            continue 

        #If element is not in or we found out Minimum distance is actually more than the distance now. 
        if element not in minDist or minDist[element] > distance:
            minDist[element] = distance
        
        # If element in not in min cost and is the mincost is more than the current cost
        if element not in minCost or minCost[element] > cost:
            minCost[element] = cost
        
        #Add into visited to track what node has been explored
        visited.add((element,cost))

        #Check if final node has reached. Once reached call backTrack() to create the path.
        if element == endNode:
            path = backTrack(parent, (element,cost))
            return path, distance, cost

        #You will check the adjacent nodes in the Graph, This will be the for loop that checks for every other neighbour of the node
        for adjNode in adjGraph[element]:
            nDistance = distance + distances[element + "," + adjNode]
            nCost = cost + costs[element + "," + adjNode]
            currAdjNode = (adjNode,nCost)
            if (currAdjNode not in visited and currAdjNode not in totalDistance and nCost <= budget):
                q.put((nDistance, currAdjNode))
                #Update the totalDistance array
                totalDistance[currAdjNode] = nDistance
                #Update the parent array
                parent[currAdjNode] = (element,cost)
                            
    return None,None,None


#Task 1
print ("=========================== Task 1 ================================\n")
path,distance = ucs(startNode,endNode,dataGLoad,dataDistLoad)

ans = str(path.pop())
while len(path) > 0:
    ans += "->" + (str(path.pop()))
print("Shortest Path:" , ans)
print("Shortest Distance" , distance) 

#Task 2
print ("=========================== Task 2 ================================\n")
path2,distance2,cost = ucsBudget(startNode,endNode,dataGLoad,dataDistLoad,dataCostLoad,budget)
ans2 = str(path2.pop())
while len(path2) > 0:
    ans2 += "->" + (str(path2.pop()))
print("Shortest Path: " , ans2)
print("Shortest Distance: " , distance2) 
print("Total Energy Cost: " , cost)

#Task 3

print ("=========================== Task 3 ================================\n")
        





