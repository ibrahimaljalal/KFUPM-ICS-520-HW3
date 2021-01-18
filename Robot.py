from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from EnvironmentSimulator import EnvironmentSimulator
from EnvironmentSimulator import ProximitySensorData

class Robot(EnvironmentSimulator):

    @classmethod
    def robotSenseThenUpdateVM(self,x,y,eSimulator,updatedVM):

        """

        This function is a static function (can not be used as an instance).
        Its main goal is to update the virtual map based on the sensing the robot will make.
        x and y are the coordinates as explained in HW 3 PDF file.
        eSimulator is an object which is taken from the EnvironmentSimulator constructor.
        updatedVM is the virtual map  which is continuously being updated based on the the robot sensing

        Why there is try and catch?
        Because the robot might be in the boundares specifically the right or down boundares
        an will generate an error because it will read out of the matrix bound

        What is the purpose of the if statements being more or equal to zero?
        Because it is actually possible in a matrix to have a negative index
        and that wll not giv us what we are intending to get. that will happen in the upper and left bounders.

        """


        checkFromLidar=eSimulator.sense(x,y)
        if checkFromLidar.n==True:
            try:
                if y-1>=0:
                    updatedVM[y-1][x]=0
            except Exception:
                pass
        
        if checkFromLidar.ne==True:
            try:
                if (y-1>=0 and x+1>=0):
                    updatedVM[y-1][x+1]=0
            except Exception:
                pass

        if checkFromLidar.e==True:
            try:
                if  x+1>=0:
                    updatedVM[y][x+1]=0
            except Exception:
                pass

        if checkFromLidar.se==True:
            try:
                if (y+1>=0 and x+1>=0):
                    updatedVM[y+1][x+1]=0
            except Exception:
                pass

        if checkFromLidar.s==True:
            try:
                if  y+1>=0:
                    updatedVM[y+1][x]=0
            except Exception:
                pass

        if checkFromLidar.sw==True:
            try:
                if  (y+1>=0 and x-1>=0):
                    updatedVM[y+1][x-1]=0
            except Exception:
                pass

        if checkFromLidar.w==True:
            try:
                if  x-1>=0:
                    updatedVM[y][x-1]=0
            except Exception:
                pass

        if checkFromLidar.nw==True:
            try:
                if  (y-1>=0 and x-1>=0):
                    updatedVM[y-1][x-1]=0
            except Exception:
                pass
        
        return updatedVM





    @classmethod
    def robotMove(self,robotPathLastX,robotPathLastY,goal_x,goal_y,updatedVM):
        """
        
        This function will take the last coordinates of it actual path.
        in other words it will take its currant position as an input
        (robotPathLastX and robotPathLast)
        then it will only move one step based on the A* search algorithm
        and from knowing the updated virtual map. if it cant move any more than
        it will return None.
        
        """
        grid = Grid(matrix=updatedVM)

        start = grid.node(robotPathLastX,robotPathLastY)
        end = grid.node(goal_x,goal_y)


        finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        path, _ = finder.find_path(start, end, grid)

        try:
            return path[1]
        except Exception:
            return None

        

        
        
