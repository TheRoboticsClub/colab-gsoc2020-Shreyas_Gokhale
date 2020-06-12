---
title:  Let's start (c)making!
date: 2020-06-12 14:30:00 +02:00
tags: [cmake, docker, porting, week-2]
permalink: /:title
description: Porting Amazon Exercise to ROS Noetic
---
> Issues Fixed:
>
> [#3](https://github.com/TheRoboticsClub/colab-gsoc2020-Shreyas_Gokhale/issues/3) with PR [#4](https://github.com/TheRoboticsClub/colab-gsoc2020-Shreyas_Gokhale/pull/4/commits).

# Week 2 Blog

I explained in my last [blog](https://theroboticsclub.github.io/colab-gsoc2020-Shreyas_Gokhale/and-we-have-a-liftoff) why it is essential to have packages regularly updated. But what if you are on the other side of the problem. You want to upgrade your package but your dependecies are not yet upgraded. This is the same issue that I was facing with jderobot/base . Our exercise is dependant on some libraries from base, but this package is in process of getting completely upgraded. Until then, I'll have to `make` it myself! 

## CMake Build system

CMake is an open-source, cross-platform family of tools designed to build, test and package software.  It is used widely in software development, and not just for C/ C++.

## What to port?

Currently we have libraries in jderobot/base repository which are used to run the exercises. These libraries represent some tools, communication tools etc. However, most of them are out of date. For example, the comm library, which is still used as a backbone for communication in Python2 based jderobot exercises. However, we have switched to ros based middleware for communication which can be run on python 3 ( and therefore in latest ROS 1 and ROS 2 releases)

The goal here is to find all the related libraries in Jderobot base which my exercise depends on and then just create a simple cmake project which `make install`s them. Now one of the ways is to find what is getting imported in python.

Fortunately, my mentor Pankhuri has already done some heavy lifting. She has configured some of the dependencies for her CarViz package. I used this repo as a reference and started adding modules are required. 

I wanted to have a complete isolation from the base repository. Also, I wanted to know exactly what is required. Lastly, whatever I am doing must be reproducible. 

By end of week 2, I was able to launch the exercise in ROS Noetic and also from a Dockerfile. On the user side, you just have to run the usual `docker-compose up --build` But on the background, there are a lot of things happening. 

Normally, I would have loved to explain all the fine details about CMake, Packaging and catkin. But, as the process of porting is still ongoing, the blog is going to be a bit shorter.  For now, I leave you with this gif and feel free to create issues / PRs on my repo!

![](assets/blog-images/week-2/AmazonExerciseDocker.gif)


***Doei!***