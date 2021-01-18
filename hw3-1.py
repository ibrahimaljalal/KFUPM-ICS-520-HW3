from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from EnvironmentSimulator import EnvironmentSimulator
from EnvironmentSimulator import ProximitySensorData
import copy
import numpy as np
import pandas as pd
from Robot import Robot as robot #Important !!!


""" Note: Most of the programing is in the Robot class which I have made tp organize the code """

"""

In this program the robot will fowllow the Deliberative/Reactive paradigm.
It will continuously sense the obsticals and rigester them in the updated virtual map.
it will then move one step (act) based on the planing from the A* search algorithm
then it will agin sense the environment plan then act and so on util it reaches its destination.
in every grid the robot will be able to sense all of the naghboring grids.
here we will assume that the robot will be able to sense by a Lidar sensor.

"""


# READ the map file and store it in matrix
# Generate (Virtual Map) (Note: we could easily generate our 
# own map without using the virtual.map file by using numpy (vm=np.ones((rows,coloms)))
#We could also multiply it with a random value (integer) to make it more intersisting
#but we will stick to the file


"""
Note the path in the pd.read_csv should be correct !!
vm = virtual map, updatedVM = updated virtual map.
the virtual map contains no obsticals however every time the
robot sense an obstical it will put it in the updated virtual map

"""
#\s means white space such as space taps etc. we put no header because usually pandas 
# library will add a header which will have no meaning in our case
"""YOU NEED TO CHANGE THIS PATH !!!!!"""
vm=pd.read_csv("C:\\Users\\ICTC\\Desktop\\Codes\\ICS 520\\HW3\\virtual.map",delimiter="\s",header=None)#Note for some reason the virtual.map file path somtimes does not work and you might need the absolute path !!!
vm=vm.to_numpy()

updatedVM=copy.copy(vm) #So that  will not have a refrance variable


# Place the simulator settings
#Note: based on these settings the robot may not reach its distention
startXY=(2,2)
endXY=(35,3)
actualPath=[]
actualPath.append(startXY)
pathExistes=True
#Note: the number of obsticlas should be less than the matrices grids or EnvironmentSimulator will generate an error
obstaclesNum=40 

# Generate (Real Map)
eSimulator=EnvironmentSimulator(vm,startXY[0],startXY[1],endXY[0],endXY[1],obstaclesNum,True)


# Start navigation 

"""
This is the most important part of the whole progarm which mainly use the Robot class

"""

while True:
    updatedVM=robot.robotSenseThenUpdateVM(actualPath[-1][0],actualPath[-1][1],eSimulator,updatedVM)
    nextPosition= robot.robotMove(actualPath[-1][0],actualPath[-1][1],endXY[0],endXY[1],updatedVM)
    if nextPosition !=None:
        actualPath.append(nextPosition)
        if actualPath[-1]==endXY:
            break


    else:
        pathExistes=False
        break


# Print the Real Map
# Print the path form start to goal
print("\n")
print("This is the virtual Map:")
print(vm)
print("\n")
print("This is the updated virtual Map:")
print(updatedVM)
print("\n")


if pathExistes==True:
    print("The robot was successfully able to reach its destination and this is the path:")
    print(actualPath)
else:
    print("The robot couldn't  reach its destination. This is the path it was able to go through:")
    print(actualPath)



grid = Grid(matrix=updatedVM)
start = grid.node(startXY[0],startXY[1])
end = grid.node(endXY[0],endXY[1])

print("\n")
print("This is the real map:")
eSimulator.printRealMap()
print("\n")
print("This is a map of the obsticals the robot detected and it's path:")
print(grid.grid_str(path=actualPath, start=start, end=end))


