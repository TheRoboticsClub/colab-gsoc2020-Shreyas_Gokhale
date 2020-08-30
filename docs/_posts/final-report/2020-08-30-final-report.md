---
title: The Finish Line
date: 2020-08-30 23:00:00 +02:00
tags: [gsoc, final-report]
permalink: /:title
description: Google Summer of Code 2020 Final Report, JdeMultiBot 
---

 ![Multi Robot Exercise](assets/blog-images/final-report/multi_robot_amazon_warehouse.png)

# GSoC Final Submission

- [Table of Contents](#gsoc-final-submission)
  * [Project Overview](#project-overview)
  * [Goals](#goals)
  * [Contributing](#contributing)
    + [Period 1: ROS1 Kinetic → ROS1 Noetic](#period-1--ros1-kinetic---ros1-noetic)
    + [Period 2: New ROS2 Amazon Robot framework](#period-2--new-ros2-amazon-robot-framework)
    + [Period 3: Navigation 2 and Multi Robot](#period-3--navigation-2-and-multi-robot)
  * [Final deliverables](#final-deliverables)
      - [Videos](#videos)
  * [Timeline](#timeline)
  * [Future Work](#future-work)
  * [Experience](#experience)



## Project Overview

[JdeRobot Academy](https://jderobot.github.io/RoboticsAcademy/) hosts a number of exercises to teach students about robotics and AI. One such exercise was (single robot) [amazon warehouse](https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/amazon_warehouse/) exercise using ROS Kinetic. Students, however, are abstracted from the complexities of the framework and they only need to code the algorithms.

My project [JdeMultiBot](https://summerofcode.withgoogle.com/projects/#5877403320057856) was designed to extend the single robot Amazon warehouse exercise to implement a scalable, cooperative, multi-agent task and path planning system. Additionally, JdeMultiBot was intended to leverage the latest release of ROS2 (Foxy), using its features and giving students a chance to get acquainted with the new age of robotics. You are welcome to read the [initial proposal](assets/pdf/gsoc_proposal.pdf).

## Goals

The end goal of the project was to have a system of collaborating multi robots, in an indoor warehouse setting, in ROS2. But a direct jump from a single robot exercise in an EOL ROS version to using cutting edge navigation 2 features could have been huge. Hence, we decided to split our goals into 3 subgoals

1. Porting Amazon Robot exercise from ROS Kinetic (16.04) to ROS Noetic (20.04)
2. Making a new ROS2 Foxy amazon robot exercise with single robot
3. Extending single robot exercise to a multirobot environment.

However, like there are side quests RPGs, these are some of the side goals that I had to finish in order to achieve our end goal.

1. Creating a custom robot and warehouse environment in Gazebo
2. Making our robot move around using Navigation 2 stack
3. Adding additional functionality to make our robot load and unload pallets
4. Extending the environment to be used for multiple robots
5. Making reproducible and portable code by using docker
6. Designing exercises which are intuitive yet challenging for students. 

## Contributing

After getting the acceptance email, my first priority was to understand how Jderobot Academy is structured. To get a student's perspective, I solved the Global Navigation exercise during the community bonding period. ROS1 released their newest version *"[Noetic](http://wiki.ros.org/noetic)"* around this time and my next week went into investigating how to proceed with the porting. We also set up our meeting structure, [a blog site](https://theroboticsclub.github.io/colab-gsoc2020-Shreyas_Gokhale/) for weekly progress and [a dashboard](https://www.notion.so/GSoC-JdeRobot-ff2c6cf65b3f4a488ca0af77e1daeff1) for notes and scratchpad.

### Period 1: ROS1 Kinetic → ROS1 Noetic

The exercise depended a lot of legacy code such as ICE and Comm, which are now completely replaced by ROS middleware. Moreover there is a major change from python2 to python3, which brakes many dependencies. In order to get around this, I wrote a python *"[Connector](https://theroboticsclub.github.io/colab-gsoc2020-Shreyas_Gokhale/connecting-the-dots#connector-package)"*  package which acts as a glue between the old code and new code. In addition, some dependencies were [ported and published](https://theroboticsclub.github.io/colab-gsoc2020-Shreyas_Gokhale/and-we-have-a-liftoff#releasing-ros-packages) on ROS distro deps.  

After fixing some dependances and also creating a docker image, the exercise was successfully [released](https://theroboticsclub.github.io/colab-gsoc2020-Shreyas_Gokhale/launch-off). 

### Period 2: New ROS2 Amazon Robot framework

ROS2 has a completely [different architecture](https://design.ros2.org/articles/changes.html) than ROS1. Also, as this was going to be the first ROS2 exercise for Jderobot, we wanted to make sure that it provides an example of the exercises further to come. We decided to redesign the exercise from ground up. 

The Python and C++ code conversion was relatively easy compared to creating a new robot model and making it work in gazebo. Gazebo plugins are [completely changed](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-gazebo_ros_api_plugin) in ROS2 and we couldn't deploy over model in xacro at that time. Not having much low level experience with Gazebo, *porting →redesigning → fixing* the robot model was probably the most challenging part for me in my journey. However, by the end, we had a completely ready Amazon Robot with [a working prismatic joint](https://theroboticsclub.github.io/colab-gsoc2020-Shreyas_Gokhale/two-x#gazebo-model-fixes-and-writing-action-plugin) which can lift the warehouse pallets. 

### Period 3: Navigation 2 and Multi Robot

[Navigation 2](https://navigation.ros.org/concepts/index.html) is relatively new project in active development. It is a complete redesign for ROS2 from the original `move_base` package and therefore I had to spend some time in understanding new concepts such as behaviour trees. The project has a high degree of modularity and it took me a while to perfect the bells and whistles (parameters) to suit our robot.

The major development, however, was [creating a custom controller](https://theroboticsclub.github.io/colab-gsoc2020-Shreyas_Gokhale/behaviour-trees#amazon-robot-controller) for our Amazon Robot, along with custom behaviour trees, [plugins](https://theroboticsclub.github.io/colab-gsoc2020-Shreyas_Gokhale/behaviour-trees#creating-followtargets-and-corresponding-nodes), rviz and support for gazebo simulation. It was based on a [PR](https://github.com/ros-planning/navigation2/pull/1928), which is not even merged yet into the official repository. As far as I know, our project is a first of this kind to leverage Navigation 2 in such a way, apart from the officially supported turtlebot3.

In the end, we managed to get not only the [single robot warehouse simulation](https://www.youtube.com/watch?v=xmHCwA0NgjM&feature=emb_title) working, but also [a multirobot environment](https://youtu.be/wmAeoyiUyh4) with coordination between the robots. Final leg of the journey included designing exercises for [single](https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/single_robot_amazon_warehouse/) and [multirobot](https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/multi_robot_amazon_warehouse/) environment and [publishing](https://twitter.com/JdeRobot/status/1299764121089400837?s=20) our work to the wider community. In order for anyone to easily test and start using our work, [the docker images](https://hub.docker.com/repository/docker/jderobot/robotics-academy-ros2) were published on the DockerHub.

## Final deliverables

The code is spread across different repositories. 

- [Working / Blog / Scratchpad Repository](https://github.com/TheRoboticsClub/colab-gsoc2020-Shreyas_Gokhale)
- [Single Robot ROS2 code](https://github.com/JdeRobot/CustomRobots/tree/foxy-devel/amazon_robot)
- [Multi Robot ROS2 code](https://github.com/JdeRobot/CustomRobots/tree/multirobot-testing/amazon_robot)
- Exercises written for [single robot](https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/single_robot_amazon_warehouse/) and [multi robot](https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/multi_robot_amazon_warehouse/)

A brief description of how the code is arranged can be found in [this blog](https://theroboticsclub.github.io/colab-gsoc2020-Shreyas_Gokhale/jde-multibot#single-amazon-robot-package).

The problems that I faced or the developments that I did while working are also applicable for a lot of other developers. I tried to maintain a thorough writeup of my GSoC journey on our [collaboration blog](https://theroboticsclub.github.io/colab-gsoc2020-Shreyas_Gokhale/). Each blog has been written keeping an average user in mind, so it should prove an easy entrypoint. Some of the blogs I recommend are:

- [How to release ROS debian packages using bloom](https://theroboticsclub.github.io/colab-gsoc2020-Shreyas_Gokhale/and-we-have-a-liftoff#why-to-release)
- Creating a new model in gazebo for [ROS1](https://theroboticsclub.github.io/colab-gsoc2020-Shreyas_Gokhale/back-to-the-start) and [ROS2](https://theroboticsclub.github.io/colab-gsoc2020-Shreyas_Gokhale/two-x#gazebo-model-fixes-and-writing-action-plugin)
- [Introduction to ROS2 from ROS1 perspective](https://theroboticsclub.github.io/colab-gsoc2020-Shreyas_Gokhale/level-2#dds)
- [Understanding Navigation 2 and behaviour trees](https://theroboticsclub.github.io/colab-gsoc2020-Shreyas_Gokhale/jde-multibot#single-amazon-robot-package)

#### Videos 

**Single Robot**
{% include youtubePlayer.html id="xmHCwA0NgjM" %}

**Multi Robot**
{% include youtubePlayer.html id="wmAeoyiUyh4" %}


## Timeline

You can find a detailed weekly timelines for each phases here:

[Phase 1](https://theroboticsclub.github.io/colab-gsoc2020-Shreyas_Gokhale/phase-one-report) 

[Phase 2](https://theroboticsclub.github.io/colab-gsoc2020-Shreyas_Gokhale/phase-two-report)

[Phase 3](https://theroboticsclub.github.io/colab-gsoc2020-Shreyas_Gokhale/phase-three-report)

Following is the shorter timeline based on tasks, marking the most important commits and pull requests.

### Task Timeline

| Task                                                                         | Phase                     | Issues / PRs                                                                                                                                                                                                                                                                                                                                          | Videos                                                                                                           |
|------------------------------------------------------------------------------|---------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------|
| Porting Amazon Robot exercise from ROS Kinetic (16.04) to ROS Noetic (20.04) | Phase 1                   | JdeRobot Assets [#54](https://github.com/JdeRobot/assets/pull/54) [#58](https://github.com/JdeRobot/assets/pull/58#event-3488709128), [ROSDistro](https://github.com/ros/rosdistro/pull/25236), Jderobot Base [1401](https://github.com/JdeRobot/base/pull/1401), Colab [8](https://github.com/TheRoboticsClub/colab-gsoc2020-Shreyas_Gokhale/pull/8) | [Video](https://www.youtube.com/watch?v=HtUEAweSmAg)                                                             |
| Creating a custom robot and warehouse environment in Gazebo                  | Phase 1, Phase 2, Phase 3 | CustomRobots [1](https://github.com/JdeRobot/CustomRobots/pull/1), [#7](https://github.com/JdeRobot/CustomRobots/issues/7) Colab [#10](https://github.com/TheRoboticsClub/colab-gsoc2020-Shreyas_Gokhale/issues/10) [#16](https://github.com/JdeRobot/CustomRobots/pull/16)                                                                           | [Video](https://youtu.be/F1WELXcg364)                                                                            |
| Making a new ROS2 Foxy amazon robot exercise with single robot               | Phase 2, Phase 3          | CustomRobots [16](https://github.com/JdeRobot/CustomRobots/pull/16) [#7](https://github.com/JdeRobot/CustomRobots/issues/7)                                                                                                                                                                                                                           | [Video 1](https://www.youtube.com/watch?v=QtHRuF0IS8E&feature=emb_title) [Video 2](https://youtu.be/xmHCwA0NgjM) |
| Making our robot move around using Navigation 2 stack with custom base       | Phase 3                   | CustomRobots [#19](https://github.com/JdeRobot/CustomRobots/pull/19) , Navigation 2 [#1944](https://github.com/ros-planning/navigation2/issues/1944) , Colab [#12](https://github.com/TheRoboticsClub/colab-gsoc2020-Shreyas_Gokhale/issues/12)                                                                                                       | [Video](https://youtu.be/n9NwqWQGTmc)                                                                            |
| Extending the environment to be used for multiple robots                     | Phase 3                   | CustomRobots [#23](https://github.com/JdeRobot/CustomRobots/pull/23) [#9](https://github.com/TheRoboticsClub/colab-gsoc2020-Shreyas_Gokhale/issues/9) Colab [#11](https://github.com/TheRoboticsClub/colab-gsoc2020-Shreyas_Gokhale/issues/11)                                                                                                        | [Video](https://youtu.be/wmAeoyiUyh4)                                                                            |
| Making reproducible and portable code using docker                           | Phase 1, Phase 2, Phase 3 | [Colab](https://github.com/TheRoboticsClub/colab-gsoc2020-Shreyas_Gokhale/pull/8)                                                                                                                                                                                                                                                                     |                                                                                                                  |
| Designing exercises which are intuitive yet challenging for students.        | Phase 3                   | [Robotics Academy](https://github.com/JdeRobot/RoboticsAcademy/pull/586)                                                                                                                                                                                                                                                                              |                                                                                                                  |


We deviated from the original plan mentioned in the proposal, however we managed to get all the original goals completed (notable exception of charging behaviour of robots). In spite of facing the challenges as predicted in the proposal (availability of packages in ROS2 and complex simulation), the most challenging part was figuring out how software works in absence of documentation or examples.

## Future Work

Currently we can successfully run both the exercises for [Single](https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/single_robot_amazon_warehouse/) and [Multirobot](https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/multi_robot_amazon_warehouse/). And all the goals we defined were met. However, the work is not over yet.

The existing system uses static maps and localisation. However, as our system is dynamic, the use of SLAM could improve the navigation many fold. Sadly, due to driver issues, we were [unable to run SLAM toolbox](https://github.com/SteveMacenski/slam_toolbox/issues/278) with our system. In near future, this should be possible and could be even extended with different SLAM algorithms. A tool can be added which can compare results across different algorithms for Path planning, SLAM, task planning. This could be very helpful for benchmarking and in an university research setting.

One of the intention behind this project was also to create a sort-of template for future exercises in ROS2. Hence it was made sure that all the code is well documented, along with the problems that I faced. We also chose cutting edge software rather than old releases (For example, Foxy vs Eloquent) so that this exercise remains relevant for years to come. But with new releases of ROS2/ Navigation2 and Gazebo the dependencies will change and it would be needed to modify the exercise again.

I would like to continue contributing to Jderobot regarding different projects. Including new SLAM and navigation exercises and combining them with ML exercises would be a nice project in my opinion. With the help of Web Loader and Docker, moving the whole academy to a cloud based learning platform would also be really cool to work on!

Because I was working on Navigation 2 for JdeMultibot, I joined their open source community as well. The project that I worked on is something of their interest as well and I will be working on it soon.

Personally, I will focus on finishing my master thesis at Fraunhofer FOKUS regarding "Multi Robot Collaboration and Mapping" (ROS1). I won't be able to use my work at GSoC directly, but my learnings will help me immensely. 

## Experience

It was my first time participating in GSoC and it not only met my expectations but went much beyond that. Before GSoC, I was only using the software as tools (for example, using Gazebo to launch pre made robots). I didn't used to interact on forums. During GSoC, I posted a lot of questions, contacted many people, raised issues and contributed to other open source projects. Now I have a better understanding of how software is developed in open source and how I can do my part in building more. I want to thank everyone who helped me in my journey.

GSoC introduced me to an amazing community at JdeRobot academy which I feel proud be a part of. I want to thank Prof. [JoseMaría Cañas](https://gsyc.urjc.es/jmplaza/) and JdeRobot Admins for the opportunity and guiding my way through the journey (and also bearing with while I ask my stupid questions)

This project could have been impossible to complete without my mentors, [Pankhuri](https://www.linkedin.com/in/pankhuri-vanjani-767283101/) and [Nacho](https://github.com/naxvm), helping me. Every problem I faced, they always knew a way around it. Pankhuri helped me immensely with the project organisation and Nacho even spent a night trying to fix the gazebo issue. I cannot thank them enough!



***Good bye!***