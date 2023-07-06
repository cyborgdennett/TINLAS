"""

D* grid planning
author: Nirnay Roy & Rens van Vendeloo

See Wikipedia article (https://en.wikipedia.org/wiki/D*)

This unit test demonstrates the usage of the pathplanning, set show_animation to true if you want to see the animated pathplanning

"""
from dataclasses import dataclass
import math

from sys import maxsize
import threading

import matplotlib.pyplot as plt

import numpy as np
from scipy.optimize import linear_sum_assignment

# import the builtin time module
import time

@dataclass
class OriginLocation:
    TOPLEFT: str = "TOPLEFT"
    TOPRIGHT: str = "TOPRIGHT"
    BOTTOMLEFT: str = "BOTTOMLEFT"
    BOTTOMRIGHT: str = "BOTTOMRIGHT"
    CENTER: str = "CENTER"


show_animation = True # DO NOT CHANGE, USE plot = True | False instead!!!!!!!!!!!!!
plot = True
 
DEFAULT_SIZE = 5

class State:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.state = "."
        self.t = "new"  # tag for state
        self.h = 0
        self.k = 0

    def cost(self, state):
        if self.state == "#" or state.state == "#":
            return maxsize

        return math.sqrt(math.pow((self.x - state.x), 2) +
                         math.pow((self.y - state.y), 2))

    def set_state(self, state):
        """
        .: new
        #: obstacle
        e: oparent of current state
        *: closed state
        s: current state
        """
        if state not in ["s", ".", "#", "e", "*"]:
            return
        self.state = state


class Map:

    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.map = self.init_map()

    def init_map(self):
        map_list = []
        for i in range(self.row):
            tmp = []
            for j in range(self.col):
                tmp.append(State(i, j))
            map_list.append(tmp)
        return map_list

    def get_neighbors(self, state):
        state_list = []
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if i == 0 and j == 0:
                    continue
                if state.x + i < 0 or state.x + i >= self.row:
                    continue
                if state.y + j < 0 or state.y + j >= self.col:
                    continue
                state_list.append(self.map[state.x + i][state.y + j])
        return state_list

    def set_obstacle(self, point_list):
        for x, y in point_list:
            if x < 0 or x >= self.row or y < 0 or y >= self.col:
                continue

            self.map[x][y].set_state("#")


class Dstar:
    def __init__(self, maps):
        self.map = maps
        self.open_list = set()

    def process_state(self):
        x = self.min_state()

        if x is None:
            return -1

        k_old = self.get_kmin()
        self.remove(x)

        if k_old < x.h:
            for y in self.map.get_neighbors(x):
                if y.h <= k_old and x.h > y.h + x.cost(y):
                    x.parent = y
                    x.h = y.h + x.cost(y)
        elif k_old == x.h:
            for y in self.map.get_neighbors(x):
                if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y) \
                        or y.parent != x and y.h > x.h + x.cost(y):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))
        else:
            for y in self.map.get_neighbors(x):
                if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))
                else:
                    if y.parent != x and y.h > x.h + x.cost(y):
                        self.insert(y, x.h)
                    else:
                        if y.parent != x and x.h > y.h + x.cost(y) \
                                and y.t == "close" and y.h > k_old:
                            self.insert(y, y.h)
        return self.get_kmin()

    def min_state(self):
        if not self.open_list:
            return None
        min_state = min(self.open_list, key=lambda x: x.k)
        return min_state

    def get_kmin(self):
        if not self.open_list:
            return -1
        k_min = min([x.k for x in self.open_list])
        return k_min

    def insert(self, state, h_new):
        if state.t == "new":
            state.k = h_new
        elif state.t == "open":
            state.k = min(state.k, h_new)
        elif state.t == "close":
            state.k = min(state.h, h_new)
        state.h = h_new
        state.t = "open"
        self.open_list.add(state)

    def remove(self, state):
        if state.t == "open":
            state.t = "close"
        self.open_list.remove(state)

    def modify_cost(self, x):
        if x.t == "close":
            self.insert(x, x.parent.h + x.cost(x.parent))

    def run(self, start, end):

        rx = []
        ry = []

        self.insert(end, 0.0)

        while True:
            self.process_state()
            if start.t == "close":
                break

        start.set_state("s")
        s = start
        s = s.parent
        s.set_state("e")
        tmp = start

        while tmp != end:
            tmp.set_state("*")
            rx.append(tmp.x)
            ry.append(tmp.y)
            if show_animation:
                plt.plot(rx, ry, "-r")
                plt.pause(0.01)
            if tmp.parent.state == "#":
                self.modify(tmp)
                continue
            tmp = tmp.parent
        tmp.set_state("e")

        return rx, ry

    def modify(self, state):
        self.modify_cost(state)
        while True:
            k_min = self.process_state()
            if k_min >= state.h:
                break

def rectObstacle(topLeft,bottomRight,ox = [],oy = []):
    horBounds = bottomRight[0] - topLeft[0]
    if(horBounds < 0):
        horBounds = abs(horBounds)
    minValHor = min(bottomRight[0],topLeft[0])
    vertBounds = topLeft[1] - bottomRight[1]
    if(vertBounds < 0):
        vertBounds = abs(vertBounds)
    minValVert = min(topLeft[1], bottomRight[1])
    # Top hor line
    for i in range(minValHor,minValHor+horBounds+1):
        ox.append(i)
        oy.append(topLeft[1])
    # Bottom hor line
    for i in range(minValHor,minValHor+horBounds):
        ox.append(i)
        oy.append(bottomRight[1])
    # Left vert line
    for i in range(minValVert,minValVert + vertBounds):
        ox.append(topLeft[0])
        oy.append(i)
    for i in range(minValVert,minValVert + vertBounds):
        ox.append(bottomRight[0])
        oy.append(i)
    return ox,oy

def squareObstacle(ox,oy,topLeftCorner,size = DEFAULT_SIZE):
    if(ox == [] or oy == []):
        return rectObstacle(topLeftCorner,[topLeftCorner[0]+10,topLeftCorner[1]-10])
    return rectObstacle(topLeftCorner,[topLeftCorner[0]+10,topLeftCorner[1]-10],ox,oy)

def centerSquareObstacle(ox: list,oy: list, centerPoint, squareSize=DEFAULT_SIZE):
    # unpack centerpoint
    centerX = centerPoint[0]
    centerY = centerPoint[1]
    # establish top left coordinate
    topLeftX = centerX - squareSize
    topLeftY = centerY + squareSize
    # establish bottom right coordinate
    bottomRightX = centerX + squareSize
    bottomRightY = centerY - squareSize
    # pack coordinates
    topLeft = [topLeftX,topLeftY]
    bottomRight = [bottomRightX,bottomRightY]

    ox.append(centerX)
    oy.append(centerY)
    # run function to generate obstacle points
    return rectObstacle(topLeft=topLeft,bottomRight=bottomRight,ox=ox,oy=oy)

def RunDStarAlgorithm(mapSize: list,startPos: list,targetPos: list, obstacleList,scaleFactor,squareSize = 5):
    # scaling of input variables, so Dstar algorithm works
    mapSize[0] = mapSize[0] * scaleFactor
    mapSize[1] = mapSize[1] * scaleFactor
    startPos[0] = startPos[0] * scaleFactor
    startPos[1] = startPos[1] * scaleFactor
    targetPos[0] = targetPos[0] * scaleFactor
    targetPos[1] = targetPos[1] * scaleFactor
    # pre-init for obstacle generation
    ox,oy = [], []
    # generating obstacles from obstacle list
    for i in obstacleList:
        ox,oy = centerSquareObstacle(ox,oy,[i[0] * scaleFactor,i[1] * scaleFactor],squareSize)
    # Create map
    m = Map(mapSize[0], mapSize[1])
    # print([(i, j) for i, j in zip(ox, oy)]) # Print obstacle list
    # Generate obstacles on map
    m.set_obstacle([(i, j) for i, j in zip(ox, oy)])
    # Animation code
    if show_animation:
        plt.plot(ox,oy, ".k") #from m.set_obstacle(2,2)
        plt.plot(startPos[0], startPos[1], "og")
        plt.plot(targetPos[0], targetPos[1], "xb")
        plt.axis("equal")
    # pre-init for Dstar
    start = m.map[startPos[0]][startPos[1]]
    end = m.map[targetPos[0]][targetPos[1]]
    # start dstar with parameters
    dstar = Dstar(m)
    rx, ry = dstar.run(start, end)
    # Animation code
    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.show()
    return rx,ry

def ProcessDStarOutput(rx,ry,scalingFactor,mapSize,originLocation):
    # print("RX: ",rx,"\nRY: ",ry)
    rxConvert, ryConvert = [], []
    if originLocation == OriginLocation.BOTTOMLEFT:
        for i in rx:
            rxConvert.append(math.floor(i/scalingFactor))
        for i in ry:
            ryConvert.append(math.floor(i/scalingFactor))
    elif originLocation == OriginLocation.TOPLEFT:
        for i in rx:
            rxConvert.append(math.floor(i/scalingFactor))
        for i in ry:
            ryConvert.append(mapSize[1] - math.floor(i/scalingFactor))
    else: 
        print("ERROR IN PROCESSDSTAROUTPUT: No origin location provided")
    # print("RX Converted: ", rxConvert, "\nRY Converted: ", ryConvert)
    path = []
    for i in range(len(rxConvert)):
        currRx = rxConvert[i]
        currRy = ryConvert[i]
        lastRx = rxConvert[i-1]
        lastRy = ryConvert[i-1]
        if((lastRx == currRx and lastRy == currRy)): 
            continue
        path.append([currRx,currRy])
    # print("Path: ", path)
# Grab Currrent Time After Running the Code
    # end = time.time()
#Subtract Start Time from The End Time
    # total_time = end - start
    # print("\n"+ str(total_time))
    return path



def GetPathPlanning(sizeOfMap,startingPosition,targetPosition,listWithObstaclePositions,scalingFactor, obstacleSize):
    rx,ry = RunDStarAlgorithm(sizeOfMap,startingPosition,targetPosition,listWithObstaclePositions,scalingFactor,obstacleSize)
    return ProcessDStarOutput(rx,ry,scalingFactor)


def AddTargetPositionObstaclesAsSquare(centerpoint):
    ox, oy = centerSquareObstacle([],[],centerpoint,5)
    listOut = []
    for i in range(len(ox)):
        oXoY = [ox[i],oy[i]]
        listOut.append(oXoY)
    return listOut

def assign_targets(droneStartLocations, droneTargetLocations):
    # Convert the dictionaries to arrays for easier computation
    start_locs = np.array(list(droneStartLocations.values()))
    target_locs = np.array(list(droneTargetLocations.values()))

    # Compute the distance matrix between start locations and target locations
    distance_matrix = np.linalg.norm(start_locs[:, np.newaxis] - target_locs, axis=2)

    # Use the Hungarian algorithm to find the optimal assignment
    row_indices, col_indices = linear_sum_assignment(distance_matrix)

    # Create a new dictionary to store the assigned targets
    assigned_targets = {}

    # Iterate over the assignments and update the dictionary
    for row, col in zip(row_indices, col_indices):
        drone_id = list(droneStartLocations.keys())[row]
        target_id = list(droneTargetLocations.keys())[col]
        assigned_targets[drone_id] = droneTargetLocations[target_id]

    return assigned_targets

def GetSwarmPathPlanning(sizeOfMap,droneID: int,startingPositions: dict,targetPositions:dict,obstaclePositions:list,scalingFactor, obstacleSize, originLocation = "TopLeft"):

    droneTarget = targetPositions[droneID]
    dronePosition = startingPositions[droneID]

    listWithObstaclePositions: list = []

    for id in startingPositions.keys():
        if id == droneID:
            continue
        else:
            listWithObstaclePositions.append(startingPositions[id])

    for id in targetPositions.keys():
        if id == droneID:
            continue
        else:
            oXoY = AddTargetPositionObstaclesAsSquare(targetPositions[id])
            for x in oXoY:
                listWithObstaclePositions.append(x)
                
            # listWithObstaclePositions.append(targetPositions[id])

    listWithObstaclePositions.extend(obstaclePositions)
    rx,ry = RunDStarAlgorithm(sizeOfMap,dronePosition,droneTarget,listWithObstaclePositions,scalingFactor,obstacleSize)
    return ProcessDStarOutput(rx,ry,scalingFactor,sizeOfMap, OriginLocation.BOTTOMLEFT)

# output: dict = {
# ID:   {
#       dronePos : [x,y],
#       targetPost : [x,y]
#       },
# ID:   {
#       dronePos : [x,y],
#       targetPost : [x,y]
#       },
# ID:   {
#       dronePos : [x,y],
#       targetPost : [x,y]
#       }
# }



def GetFiducialFormation(droneIDs, fiducialTarget, amountOfDrones, margin):
    targetPositions = {}
    angle = 2 * math.pi / amountOfDrones

    for i, droneID in enumerate(droneIDs):
        theta = i * angle
        x = fiducialTarget[0] + margin * math.cos(theta)
        y = fiducialTarget[1] + margin * math.sin(theta)

        x = int(math.floor(x))
        y = int(math.floor(y))
        targetPositions[droneID] = [x, y]

    return targetPositions

def CorrectOriginPositionDict(input: dict, mapSize:list,originLocation = OriginLocation.TOPLEFT):
    if(originLocation == OriginLocation.TOPLEFT):
        maxValY = mapSize[1]
        for keys in input.keys():
            newVal = [input[keys][0], maxValY - input[keys][1]]
            input.update({keys:newVal})
    elif(originLocation == OriginLocation.TOPRIGHT):
        pass
    elif(originLocation == OriginLocation.BOTTOMLEFT):
        pass
    elif(originLocation == OriginLocation.BOTTOMRIGHT):
        pass
    elif(originLocation == OriginLocation.CENTER):
        pass
    else:
        print("ERROR: UNDEFINED ORIGIN LOCATION")
    
    return input

def ChopPath(inputPath):
    outputPath = []
    return outputPath

def simplify_path(coordinates, epsilon = 0.5):
    # Check if the path can be simplified
    if len(coordinates) < 3:
        return coordinates

    # Find the point with the maximum distance
    dmax = 0
    index = 0
    end = len(coordinates) - 1
    for i in range(1, end):
        d = perpendicular_distance(coordinates[i], coordinates[0], coordinates[end])
        if d > dmax:
            index = i
            dmax = d

    # Recursively simplify the subpaths
    if dmax > epsilon:
        simplified1 = simplify_path(coordinates[:index + 1], epsilon)
        simplified2 = simplify_path(coordinates[index:], epsilon)

        # Combine the simplified subpaths
        return simplified1[:-1] + simplified2
    else:
        return [coordinates[0], coordinates[end]]

def perpendicular_distance(point, start, end):
    # Calculate the perpendicular distance between the point and the line defined by start and end
    numerator = abs((end[1] - start[1]) * point[0] - (end[0] - start[0]) * point[1] + end[0] * start[1] - end[1] * start[0])
    denominator = ((end[1] - start[1]) ** 2 + (end[0] - start[0]) ** 2) ** 0.5
    return numerator / denominator

def ChopPath(inputDict:dict):
    for key in inputDict.keys():
        path = inputDict.get(key).get("path")
        inputDict.update({key : {"path" : simplify_path(path),
                                 "pathIndex": 0}}) 
    return inputDict

def plotPath(chopPath:dict):

    pathToPlot = []

    for key in chopPath.keys():
        pathToPlot = chopPath.get(key).get("path")
        x_path = [coord[0] for coord in pathToPlot]
        y_path = [coord[1] for coord in pathToPlot]
        plt.plot(x_path, y_path, label=str(key))
    plt.xlabel('X-coordinate')
    plt.ylabel('Y-coordinate')
    plt.title('Paths')
    plt.grid(True)
    plt.legend()
    plt.show()

def StartDronePathPlanning(dronePositions: dict, fiducialTarget: list, mapSize = [100,50], margin = 1):
    # preprocess all incoming coordinates to correct origin difference
    dronePositions = CorrectOriginPositionDict(dronePositions,mapSize,OriginLocation.TOPLEFT)
    fiducialTarget = [fiducialTarget[0],mapSize[1]-fiducialTarget[1]]

    # use fiducialTarget to calculate target locations for drones around the fiducial
    droneTargetPositions: dict = None
    droneTargetPositions = GetFiducialFormation(dronePositions,fiducialTarget=fiducialTarget,amountOfDrones=len(dronePositions),margin=10)
    droneTargetPositions = assign_targets(dronePositions,droneTargetPositions)
    plannedPaths: dict = {}

    obstacleList: list = []

    for droneID in dronePositions.keys():
        pathList = GetSwarmPathPlanning(mapSize,droneID,dronePositions,droneTargetPositions,obstacleList,1,0)
        pathIndex = 0
        plannedPaths[droneID] = {
            "path":pathList,
            "pathIndex":pathIndex
        }
        obstacleList.extend(plannedPaths[droneID]["path"])

    # Process plannedPaths back to original origin location
    # CONVERSION
    for keys in plannedPaths.keys():
        for i in range(len(plannedPaths[keys]["path"])):
            plannedPaths[keys]["path"][i][1] = mapSize[1] - plannedPaths[keys]["path"][i][1]

    chopPath = ChopPath(plannedPaths)

    if plot == True:
        plotPath(chopPath)
    return chopPath




if __name__ == '__main__':
# Grab Currrent Time Before Running the Code
    start = time.time()

    obstacleList = []

    droneDict = {
        2:[33,45],
        9:[61,45],
        5:[20,20],
        7:[44,16]
    }

    fiducialTarget = [50,25]

    sizeOfMap = [100,50]

    paths = StartDronePathPlanning(droneDict,fiducialTarget,sizeOfMap)


    # startPos = [10,10]
    # goalPos = [70,30]
    # scalingFactor = 1

    # path = GetPathPlanning(sizeOfMap,startPos,goalPos,obstacleList,1)



    for keys, values in paths.items():
        print(keys,":",values)
    end = time.time()
    runtime = end - start
    print("- - - - End of main - - - \nRuntime: ",runtime,"\nPathplanning output^\n")
    