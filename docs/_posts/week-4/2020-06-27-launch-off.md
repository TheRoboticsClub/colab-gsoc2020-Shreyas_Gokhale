---
title:  Launch Off!
date: 2020-06-27 10:00:00 +02:00
tags: [amazon-warehouse, exercise, porting, week-4]
permalink: /:title
description: Launching the exercise
---
> Issues Fixed:
>
> PR [#58](https://github.com/JdeRobot/assets/pull/58#event-3488709128) on Jderobot/assets for gazebo model update.
>
> PR [#1401](https://github.com/JdeRobot/base/pull/1401) on Jderobot/base for connector package.
>
> PR [#8](https://github.com/TheRoboticsClub/colab-gsoc2020-Shreyas_Gokhale/pull/8) on colab repo for launchable docker exercise.

# Week 4

After last week's removal of comm, I was pretty confident about being able to run the exercise. But it seemed like the exercise wasn't ready for that yet! I couldn't seem to get cmd vel (joystick) working. 

After digging up a bit, I traced the problem back to `model.sdf` file. This is gazebo model which describes how our robot looks and behaves like. One major part of gazebo are [plugins](http://gazebosim.org/tutorials?tut=ros_gzplugins) and they are binaries which code the functionality, for example lasers and motors. I was getting following error:

`libgazebo_ros_diff_drive.so not found`

After trying for hours, posting on ROS forum and getting frustrated, I finally found the culprit: `gazebo-plugins`  . This package was necessary to run the default plugins and luckily it just got ported to noetic. After fixing some issues with the sdf, the whole exercise is now up and running!

## Amazon Warehouse exercise

The objective of this practice is to implement the autonomous robot navigation and pick-and-place logic in warehouse.

For trying out, you have 2 options:

### Docker install

This is the preferred installation method.

1. Follow [these instructions](https://docs.docker.com/get-docker/) to install docker and docker-compose.
2. Clone my repo 

    ```bash
    git clone https://github.com/TheRoboticsClub/colab-gsoc2020-Shreyas_Gokhale.git
    ```

3. Run from the root directory

    ```bash
    xhost +"local:docker@"
    docker-compose up --force-recreate
    ```

4. Sit back, sip a coffee or whatever while the dockerfile is being built. The exercise will be launched automatically once building is finished.

### Manual Installation

1. Follow the instructions on [ROS Wiki](http://wiki.ros.org/noetic/Installation/Ubuntu) to install the latest ROS and on [Gazebo Wiki](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install) to install Gazebo 11.
2. Install all the dependencies 

    ```bash
    sudo apt update
    sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential  
    sudo rosdep init  
    rosdep update
    sudo apt install ros-noetic-mavros ros-noetic-gazebo-ros python3-pip ros-noetic-navigation ros-noetic-gazebo-plugins
    ```

3. Install Jderobot Assets

    ```bash
    sudo apt install ros-noetic-jderobot-assets
    ```

    The model.sdf file needs slight modification (until aptitude packages are updated). So grab the updated file and copy it over to the original file

    ```bash
    wget https://raw.githubusercontent.com/JdeRobot/assets/noetic-devel/jderobot_assets/models/amazon_warehouse_robot/model.sdf -o "model.sdf"
    cp -u model.sdf /opt/jderobot/share/jderobot/gazebo/models/amazon_warehouse_robot/model.sdf
    ```

4. Clone my repo 

    ```bash
    git clone https://github.com/TheRoboticsClub/colab-gsoc2020-Shreyas_Gokhale.git
    ```

5. Run the exercise 

    ```bash
    cd exercises/amazon_warehouse/launch 
    roslaunch amazonrobot_1_warehouse.launch map_file:=maps/map.yaml 
    python3 amazonWarehouse.py amazonMap.conf amazonConf.yml

    ```

Try to code your solution in `mysolution.py`

Let me know if you face any issues or bugs.

***Viszlát!***