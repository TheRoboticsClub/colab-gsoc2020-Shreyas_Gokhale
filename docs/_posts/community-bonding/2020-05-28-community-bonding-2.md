---
title: Community Bonding 2
date: 2020-05-28 12:00:00 +02:00
tags: [setup, global-navigation, docker, community-bonding]
permalink: /:title
description: Exploring Jderobot academy in community bonding period 2.
---
> Issues Fixed: [#1](https://github.com/TheRoboticsClub/colab-gsoc2020-Shreyas_Gokhale/issues/1) with [PR](https://github.com/TheRoboticsClub/colab-gsoc2020-Shreyas_Gokhale/pull/2)

 
## Motion Planning

After successfully running A* path planner, I wanted to implement the actual mechanism of following the path AKA motion planner or path follower. I thought this shouldn't take much effort as we have the path and just have to follow it.

Boy I was wrong! It turned out that in absence of prepackaged solutions, following path is much much harder. Normally in ROS, you can use [Move base](http://wiki.ros.org/move_base) or just the [DWA Planner](http://wiki.ros.org/dwa_local_planner) to do fine path planning. This involves actually going to each and every node and controlling the PID for acceleration and turning. But in absence of these packages, you have to do everything by yourself.

The simplest method you can implement is following the pose. You have starting position (x1,y1,z1) and you want to go to (x2,y2,z2). My plan was to apply a hypotenuse vector acceleration proportional to the distance between two points. Then once it reaches the point, turn it according to the pose. However, this resulted in quick overshoots, turning my car into the [tesla roadster in space](https://en.wikipedia.org/wiki/Elon_Musk%27s_Tesla_Roadster). No matter how I tweaked, this was not working.

After my failed attempt at doing it myself, I turned onto some existing python libraries, which I could mould according to my needs. I found [this](https://github.com/AtsushiSakai/PythonRobotics/blob/3607d72b60cd500806e0f026ac8beb82850a01f9/PathTracking/move_to_pose/move_to_pose.py#L127) amazing code set, which I tried to implement. But I still couldn't achieve what I wanted. Then I realised my 2 main mistakes:

- The A* path planner samples all of the points. Hence the path has much higher resolution than it can handle.
- I didn't do the [local planning exercise](https://jderobot.github.io/RoboticsAcademy/exercises/Drones/position_control). Hence, I didn't have any sophisticated PID controller. And as we have to control the car only based on angular and linear acceleration, this is extreamly essential.

So sadly I had to leave the exercise in a bit unsolved case. I surely want to revisit it once I get time!

## ROS Noetic release

As with all the turtle fans, I was excited for the new ROS Noetic Ninjemys release. Ubuntu 20 and Python3 by default are the 2 main highlights of this release. And sadly, this will be the last ROS 1 release ever! But hey, all things happen for a reason, and here it is for ROS2!

![ROS-Noetic](https://raw.githubusercontent.com/ros-infrastructure/artwork/master/distributions/noetic.png)

We decided that it would be the best to take the release out for a spin using Docker or VM. Sadly, OSRF has not published the finalised Docker images on the dockerhub. Hence, for now, I created my own dockerfile based on the official dockerfile. Check it out! 

My next target will be to run our exercise dependencies on noetic, mainly the Jderobot academy base, which currently only supports until melodic

Until the next time! Bis Bald!

> Just in! OSRF pushed the images on the DockerHub. So those will be used in the next releases.