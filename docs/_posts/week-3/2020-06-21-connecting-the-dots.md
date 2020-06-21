---
title:  Connecting the dots!
date: 2020-06-21 12:30:00 +02:00
tags: [comm, connector, porting, week-3]
permalink: /:title
description: Removing Comm Module
---
> Issues Fixed:
>
> [#5](https://github.com/TheRoboticsClub/colab-gsoc2020-Shreyas_Gokhale/issues/5) with PR [#6](https://github.com/TheRoboticsClub/colab-gsoc2020-Shreyas_Gokhale/pull/6).
> 
> Added new branch for Jderobot/base -> [noetic-devel](https://github.com/shreyasgokhale/base/tree/noetic-devel) with connector dependency added

# Week 3 Blog

In last blog I briefly talked about Cmake and porting from the old Jderobot/base code. For this week I wanted to clarify more on how that turned out.

## Jderobot base and comm library

Jderobot Base package is the backbone for all the real and virtual robot development at the academy. It is essentially a collection of various drivers and libraries. It contains source code in C++ for real robots and in python for academy exercises. The code is arranged using CMakeLists and has lots of flags, options and tests depending on which operating system you are running the code. I found out an [excellent paper](https://gsyc.urjc.es/jmplaza/papers/robocity2013-jderobot.pdf) on the base package written by the founders and it is worth a read for anyone who is working on the academy. 

Currently, the base project is going through a major change. We have completely switched from [ICE](https://zeroc.com/) based communication to native ROS middleware based Pub and Sub mechanism. Some of the libraries and packages are now outdated and some of them are now even available in native ros packages. Lastly, python2 is now obsolete and it makes very essential to use python3 in order to support the latest ROS1,ROS2 releases. 

 

So the new release for academy is going to be a major one and as my project already had some ported dependencies I wanted to also contribute to it. The major one was the backbone communication library: `comm` . It was used to get the objects for moules from ICE or ROS depending on the architecture, for example Laser Sensors, Motors or Camera. 

For our use case, the existing version of Comm library acts as a glue between jderobot academy exercise and ros framework. So for example, If I want to perform SLAM in the python application in jderobot exercises, I have to get it somehow from gazebo. The **OLD** flow of information is as follows

<span style="color:magenta">Gazebo → ROS topic → **C++ Comm** </span>  <span style="font-size:2em"> *→*   </span> <span style="color:green"> **Python Comm** → Exercise Application  → My Solution </span>


The left side of the big arrow (or text with magenta colour) is C++ domain and the text on the right side (or the text with green colour) is python domain. Our module of interest here is the bold part.

But now in the new version, we will be using native ros middleware to handle the communication. As we only need the rosnode objects in order to publish / subscribe to these nodes, we don't need this extra **Comm** middleware. Just get the object of `rospy.Publisher` / `rospy.Subscriber` and we are done! So the whole comm stuff becomes redundant. But we cannot simply delete this folder and forget about it like it never existed. We need to somehow connect the exercise to the publisher..

## Connector Package

A new `connector` package is my solution of the problem. It simply returns the Publisher / Subscriber object of the requested module. We still use all the class definitions that existed for sensors and modules, but we don't publish using ICE. Now we can remove all the packages like `easyiceconfig` and refactor `parallelice` directory. Sweet!

This change is however, as they call it, a breaking one! Comm module is referenced by so many other modules and we have to go through each of the CMake dependencies to make sure that it doesn't make other stuff stop working. This work will be continue in the upcoming weeks and will be tested.

You can check out the connector package [here](https://github.com/shreyasgokhale/base/tree/noetic-devel). 

In the next week, the exercise will be launched, ready to be tested by the students!

***Näkemiin!***