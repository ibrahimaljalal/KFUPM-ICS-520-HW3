from random import randint
from pathfinding.core.grid import Grid
from pathfinding.core.diagonal_movement import DiagonalMovement
from copy import deepcopy
'''
This class simulates a real navigation environments.
It creates artificial obstecles and provide sensing information.
'''
class EnvironmentSimulator:
    def dimensions(self, map):          #computes the dimensions of the map
        for row in map:
            length=len(map)
            width=len(row)
            break
        return length, width

    def generateObstcles(self, map,start_x,start_y,goal_x,goal_y):   # places random obstcles in the map
        map_with_obstcles = deepcopy(map)
        self.length, self.width=self.dimensions(map_with_obstcles)
        
        obstclePoints = set()
        while len(obstclePoints) < self.obstcleCount:
            x = randint(0,self.width-1)
            y = randint(0,self.length-1)
            # make sure obstcale is not on start or goal states
            if not((x == start_x and y == start_y) or (x == goal_x and y == goal_y)):
                obstclePoints.add((x,y))

        for obs in obstclePoints:
            map_with_obstcles[obs[1]][obs[0]]=0

        self.grid = Grid(matrix=map_with_obstcles)
    
    def printRealMap(self):            # print map with Obstacles
        print(self.grid.grid_str(show_weight=True))

    def __init__(self, map, start_x,start_y,goal_x,goal_y,obstcleCount=0,diagonalMovementEnabled=False):
        self.obstcleCount=obstcleCount
        self.generateObstcles(map,start_x,start_y,goal_x,goal_y)
        if diagonalMovementEnabled==False:
            self.diagonal_movement=DiagonalMovement.never
        else:
            self.diagonal_movement=DiagonalMovement.always
    
    def sense(self, x, y):              #returns sensing information for location (x,y)
        w = e = n = s = nw = ne = sw = se = True 
        
        neighbors = self.grid.neighbors(self.grid.node(x,y),self.diagonal_movement)
        if x>0:
            if self.grid.node(x-1,y) in neighbors:
                w=False
        if x<self.width-1:
           if self.grid.node(x+1,y) in neighbors:
                e=False
        if y<self.length-1:
            if self.grid.node(x,y+1) in neighbors:
                s=False
        if y>0:
            if self.grid.node(x,y-1) in neighbors:
                n=False
        # if diagnoial movement is enabled, consider corners
        if self.diagonal_movement == DiagonalMovement.always:
            if x>0 and y >0:
                if self.grid.node(x-1,y-1) in neighbors:
                    nw=False
            if x<self.width-1 and y >0:
                if self.grid.node(x+1,y-1) in neighbors:
                    ne=False
            if x>0 and y <self.length-1:
                if self.grid.node(x-1,y+1) in neighbors:
                    sw=False
            if x<self.width-1 and y <self.length-1:
                    if self.grid.node(x+1,y+1) in neighbors:
                        se=False
        
        return ProximitySensorData(w,e,n,s,nw,ne,sw,se)


class ProximitySensorData:
    '''
    Sensor data class. For each value,
    True: Cell is obstructed
    Flase: Cell is free
    '''
    def __init__(self,w,e,n,s,nw,ne,sw,se):
        self.w = w      # west
        self.e = e      # East
        self.n = n      # North
        self.s = s      # South
        self.nw = nw    # North West
        self.ne = ne    # North East
        self.sw = sw    # South West
        self.se = se    # North East