---
title: Finding the way
date: 2020-08-01 15:30:00 +02:00
tags: [amazon-warehouse-robot, exercise, gazebo, ros2, nav2, week-9]
permalink: /:title
description: Amazon warehouse exercise in ROS2 

---
> Issues Pending:
> 
> [#7](https://github.com/JdeRobot/CustomRobots/issues/7) WIP for ROS2 Amazon Robot model
>

# Week 9 blog

## Navigation 2

This week, we are now finally able to launch the exercise, complete with navigation2! It is the successor of the navigation stack in ROS1 for ROS2 and it is build completely from ground up. Navigation2 also uses Behavior trees instead of the FSMs in move_base and they are much more intuitive and modern way of handling navigation tasks.

Navigation2 already has an up to date and extensive documentation and a really nicely written paper. But if you are a video person, I'd advise watching the minutes of ROS working group meeting to get it started quickly.

{% include youtubePlayer.html id="-MzsEykCXoU" %}

The objective of the exercise is to understand ROS2 actions, behaviour trees and navigation stack. In the end you will be able drive the robot around and interact with it using our Jderobot GUI. 

### ROS Actions

Consider this scenario: You want to buy something, say a toothbrush, online. You find one which you like and now you want to order it. But you find out that the system wants you to go on the warehouse's site and tell the admins that you need this particular item, then tell the packing station to pack it, then order delivery services to pick this package and then drop it at your doorstep, all by yourself. You will think that this is crazy as all you want is the package delivered and you really don't care about the steps in between. Sure, you would *like* to have updates regarding the shipment, but the whole purpose of ordering online is not to deal with the stuff in-between. Well, this is exactly what ROS action does.

![ROS2 actions](https://index.ros.org/doc/ros2/_images/Action-SingleActionClient.gif)

Credits: [Actions Tutorial ROS2](https://index.ros.org/doc/ros2/Tutorials/Understanding-ROS2-Actions/)

ROS actions provide an interface which abstracts long, complex task from the node. You can program an *"Action Server"* to accept requests of a task and return the result of the request once it is completed to the *"Action Client"*. Meanwhile, you can also get updates regarding your request. Action client can also cancel or update your goal for the task.

In Navigation 2, action servers are used to communicate with the highest level BT navigator through a `NavigateToPose` action message. They are also used for the BT navigator to communicate with the subsequent smaller action servers to compute plans, control efforts, and recoveries. Each will have their own unique `.action` type in `nav2_msgs` package for interacting with the servers.

For understanding it better, I'd definitely recommend the reader to have a look on the [ROS2 action design](http://design.ros2.org/articles/actions.html) and [ROS2 action tutorial](https://index.ros.org/doc/ros2/Tutorials/Understanding-ROS2-Actions/). Our focus for this exercise will be to leverage ROS2 actions for Navigation2. For us to continue with the exercise, we need to understand one more concept: Behaviour Trees

### Behaviour Trees

Navigation2 documentation explains behaviour trees the best, so I am just going to link it here. But in essence, it splits the whole operation of a robot into smaller behaviours in a specific structure governed by rules. For example, if we want to have a robot to play football, we can split it in different primitives such as run to the ball, dribble, shoot and these states are further divided into sub primitives. How we can change from one behaviour to another is also dependant on rules and what conditions are satisfied.

Navigation 2 uses [BehaviorTree](https://www.behaviortree.dev/) CPP V3 as the behaviour tree library and we can use the trees created for navigation2 as subtrees of other trees. We create node plugins which can be constructed into a tree, inside the `BT Navigator`. The node plugins are loaded into the BT and when the XML file of the tree is parsed, the registered names are associated.

### Planning and Controlling

Two components of navigation are planning a path and executing the motion. Traditionally, these were handled using respective planning and control servers in ROS1. They make information such as costmaps and parameters available to all the other ROS nodes. In ROS2, they are represented as plugins.

We will keep exploring the navigation2 stack in upcoming blogs so we can program our own plugins and nodes for our own logic. For multirobot exercise, this will be controlling the co-operation between the robots. But for now, let's see how we can run the exercise.

## Launching the exercise

1. Install ROS2 foxy by following [this](https://index.ros.org/doc/ros2/Installation/Foxy/)  guide.
2. Pull the latest [Navigation2](https://navigation.ros.org/build_instructions/index.html) and install it.

    ```bash
    git clone https://github.com/ros-planning/navigation2.git

    cd navigation2 

    source /opt/ros/foxy/setup.sh

    colcon build

    . ./install/setup.sh

    export TURTLEBOT3_MODEL=waffle
    ```

3.  Pull CustomRobots repo and build amazon_robot

    ```bash
    git pull https://github.com/shreyasgokhale/CustomRobots.git
    cd CustomRobots/amazon_robot

    colcon build

    . ./install/setup.sh
    ```

4. Pull latest of my colab repo

    ```bash
    git pull https://github.com/TheRoboticsClub/colab-gsoc2020-Shreyas_Gokhale.git
    ```

5. **Export the paths of models to let gazebo know from where to load them.** In my case, this was the path

    ```bash
    echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/CustomRobots/amazon_robot/amazon_robot_gazebo/models' >> ~/.bashrc

    ```

6. Launch the exercise

    ```bash
    cd ~/colab-gsoc2020-Shreyas_Gokhale/exercises/ros2/amazon_warehouse/launch

    ros2 launch ros2 launch amazon_warehouse_world.py

    ```

7. In another terminal, launch the solution

    ```bash
    cd ~/colab-gsoc2020-Shreyas_Gokhale/exercises/ros2/amazon_warehouse/

    python3 amazonWarehouse.py amazonMap.conf amazonConf.yml

    ```

8. Success!

{% include youtubePlayer.html id="QtHRuF0IS8E" %}

Feel free to explore the exercise. You can right do almost everything except loading the pallet. The prismatic joint is still not working. I have raised the issue on both gazebo and ros forums and also contacted the developers of gazebo. Hopefully, this will get solved soon. Our next stop will be to introduce multi robot dynamics into the picture!

***Na razie!***