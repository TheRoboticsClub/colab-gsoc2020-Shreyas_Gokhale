---
title: Beginning of GSoC 2020!
date: 2020-05-11 14:00:00 +05:30
tags: [path-planning, global-navigation, community-bonding]
permalink: /:title
description: Exploring Jderobot academy in the first week of comunnity bonding.
---

# Global Navigation

Created: May 08, 2020 6:50 PM
Tags: Solving Exercise
URL: https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/global_navigation/

## Pre Processing

- Started by adding everything on pycharm
- Created a pip venv for easy access
- in case pyqt5 of problem, use this -> 
[https://stackoverflow.com/questions/18042919/how-to-install-pyqt5-on-a-new-virtualenv-and-work-on-an-idle](https://stackoverflow.com/questions/18042919/how-to-install-pyqt5-on-a-new-virtualenv-and-work-on-an-idle)

# APIs

## Base APIs

- `sensor.getRobotX()` - to obtain the position of the robot
- `sensor.getRobotY()` - to obtain the position of the robot
- `sensor.getRobotTheta()` - to obtain the orientation of the robot with respect to the map
- `vel.setV()` - to set the linear speed
- `vel.setW()` - to set the angular velocity

## Grid APIs

- `grid.getMap()` - returns the image of the map that is being displayed. The image returned will be a 3-channel image with values 0 or 255, where 0 represents the obstacles and 255 the road. Although the image has 3 channels, for this practice it will be useful to use only one.
- `grid.getDestiny()` - returns the selected destination through the GUI as a tuple (x, y).
- `grid.getPose()` - returns the position with respect to the map, not with respect to the world, also as a tuple (x, y).
- `grid.showGrid()` - creates a window in which represents the values ​​of the field that have been assigned to the grid. The smaller values ​​will have a color closer to black, and will become clearer as larger values ​​are involved. For the representation, a copy of the grid is made and its values ​​are normalized so that they are between 0 and 1, and it is represented later with cv2.imshow().
- `grid.getVal(x, y)` - returns the value in that grid position.
- `grid.setVal(x, y, val)` - sets the value val to the indicated position.
- `grid.setPathVal(x, y, val)` - sets the value val to the indicated position.
- `grid.getPathVal(x, y)` - returns the value of the indicated position.
- `grid.setPathFinded()` - establishes that the path has been found to start painting.
- `gridToWorld(gridX, gridY)` - receives the x and y components of the coordinates of the map and returns a tuple with the equivalent coordinates in the world: (worldX, worldY)
- `worldToGrid(worldX, worldY)` - receives the x and y components of the world coordinates and returns a tuple with the equivalent coordinates in the map: (gridX, gridY)

**Responses**

```python
self.grid.getMap() = [[29 13 17 ... 17 12 32]
											[14 0 0 ... 0 0 17]
											[14 0 0 ... 0 0 17]
											...
											[14 0 0 ... 0 0 17]
											[14 0 0 ... 0 0 16]
											[28 12 17 ... 16 12 31]]
self.grid.getDestiny() = (274, 27)
self.grid.getPose() = (200, 200)
```

## Solving A*

Reference:

[Easy A* (star) Pathfinding](https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2)