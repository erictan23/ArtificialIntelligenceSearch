
import json
from os import stat
from tracemalloc import start
from queue import PriorityQueue 
import math
import os

#All files must be in the same folder for this to work
cost_file_path = os.path.join(os.path.dirname(__file__), "Cost.json")
#To Set The File's in your own folder
#Load all the data set using json.load
dataCostLoad = json.load(open(cost_file_path))
#with open(r'C:\Users\eric2\Desktop\CZ3005Lab\Cost.json') as dataCost:
#    dataCostLoad = json.load(dataCost)

coord_file_path = os.path.join(os.path.dirname(__file__), "Coord.json")
dataCoordLoad = json.load(open(coord_file_path))

dist_file_path = os.path.join(os.path.dirname(__file__), "Dist.json")
dataDistLoad = json.load(open(dist_file_path))

g_file_path = os.path.join(os.path.dirname(__file__), "G.json")
dataGLoad = json.load(open(g_file_path))


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
        path = []
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
    visited = set() #Use a set so that there won't be duplicates
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
        node,cost = node_cost
        path = []
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
    visited = set() #Use a set so that there won't be duplicates to see if nodes are visited
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


def aStar(startNode, endNode, adjGraph, distances, costs, budget,coord):

    #Need a backtracking algorithm to find the root parent/path
    def backTrack(parent_dict, node_cost):
        #Retrieve tuple but only node is stored.
        node,cost = node_cost
        path = []
        #Method to trace back to each node's parent
        while parent_dict[node_cost] is not None:
            node_cost = parent_dict[node_cost]
            node,_ = node_cost
            path.append(node)
        #Return the nodes path
        return path 

    #To find the shortest distance between one point to another by being allowed to move anywhere
    #We can use Euclidean Distance Heuristics
    def euclideanDist(nodeA,nodeB):
        x1,y1 = coord[nodeA]
        x2,y2 = coord[nodeB]
        #
        h = math.sqrt((x1-x2)**2 + (y1-y2)**2)
        return h
    
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
        if element in minDist and element in minCost and minDist[element] <= totalDistance[element,cost] and minCost[element] <= cost:
            continue 

        #If element is not in or we found out Minimum distance is actually more than the distance now. 
        if element not in minDist or minDist[element] > totalDistance[element,cost]:
            minDist[element] = totalDistance[element,cost]
        
        # If element in not in min cost and is the mincost is more than the current cost
        if element not in minCost or minCost[element] > cost:
            minCost[element] = cost
        
        #Add into visited to track what node has been explored
        visited.add((element,cost))

        #Check if final node has reached. Once reached call backTrack() to create the path.
        if element == endNode:
            path = backTrack(parent, (element,cost))
            return path, totalDistance[element,cost], cost

        #You will check the adjacent nodes in the Graph, This will be the for loop that checks for every other neighbour of the node
        for adjNode in adjGraph[element]:
            nDistance = totalDistance[element,cost] + distances[element + "," + adjNode]
            nCost = cost + costs[element + "," + adjNode]
            currAdjNode = (adjNode,nCost)
            if (currAdjNode not in visited and currAdjNode not in totalDistance and nCost <= budget):
                #Add with euclidean distance instead of the distance dictionary
                q.put((nDistance +min(distances[element+ "," + adjNode], euclideanDist(adjNode,endNode)) , currAdjNode))
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
        
path3,distance3,cost3 = aStar(startNode,endNode,dataGLoad,dataDistLoad,dataCostLoad,budget,dataCoordLoad)
ans3 = str(path3.pop())
while len(path3) > 0:
    ans3 += "->" + (str(path3.pop()))
print("Shortest Path: " , ans3)
print("Shortest Distance: " , distance3) 
print("Total Energy Cost: " , cost3)




