---
title:  Back to the start
date: 2020-07-10 22:00:00 +02:00
tags: [amazon-warehouse-robot, exercise, gazebo, week-5, week-6]
permalink: /:title
description: New amazon robot model and finalizing the exercise.

---
> Issues Fixed:
>
> PR [#1](https://github.com/JdeRobot/CustomRobots/pull/1) on Jderobot/CustomRobots for new gazebo model.

# Week 5 + 6 blog

## Going back to the drawing board

The world is constantly changing. 10 years ago, Facebook was just a new craze and Instagram didn't exist. But as new technologies and innovations got introduced, they changed the way we do things, and so the way ROS worked.

Here is an amazing youtube video shot by Redhat explaining the origin story of ROS. I would recommend every ROS developer to watch it at least once, just to get an idea about the complexity of the project and people involved.

{% include youtubePlayer.html id="ErHAhRiUaY0" %}

Coming back to our story, when I started working with the existing amazon robot gazebo model for the exercise, I realized that it is not working properly. The lift, lasers didn't work and the model was in `.sdf` format. This format describes how to display the robots in Gazebo, however, it is not so friendly with other pieces of the software. The "new" way of doing things is to create different `.xacro` format files. This is achieved in by creating three different files

1. `<robot>.gazebo.xacro` which describes our robot for gazebo parameters (colours, physics, plugins for sensors etc)
2. `<robot>.transmission.xacro` for the transmissions of joints.
3. `<robot>.urdf.gazebo` for describing links and joints.

Some of the tutorials that I followed:

[Make a model](http://gazebosim.org/tutorials?tut=build_model)

[URDF in Gazebo](http://gazebosim.org/tutorials/?tut=ros_urdf)

In the beginning I was trying to update the existing robot model to match the new format. Here is the image describing old robot and the robot I was trying to modify.

![assets/blog-images/week-5-6/Untitled.png](assets/blog-images/week-5-6/Untitled.png)

![assets/blog-images/week-5-6/Untitled%201.png](assets/blog-images/week-5-6/Untitled%201.png)

![assets/blog-images/week-5-6/Untitled%202.png](assets/blog-images/week-5-6/Untitled%202.png)

Our model was static, which means it will be ignored by the physics engine. As a result the model will stay in one place and allow us to properly align all the components.

However, while working on this, I realized that the existing model needed to be updated in major way. Creating new links, setting physics properties and adding supported stuff on top of it was essential to make the exercise workable. Instead of reinventing the wheel, I thought, why not find some existing robot bases and build my model on top of it? The best way to create new stuff is to follow the best practices and so I decided to use turtlebot3_waffle_pi as the base.

[ROBOTIS-GIT/turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3/blob/master/turtlebot3_description/urdf/turtlebot3_waffle.gazebo.xacro)

![assets/blog-images/week-5-6/Untitled%203.png](assets/blog-images/week-5-6/Untitled%203.png)

Now we can use xacro publisher to publish robot states. Sweet.

After some tweaks, our initial model is ready

{% include youtubePlayer.html id="4DCr3LRLVLE" %}

In order to control, we need to install ros-control [ROS control](http://gazebosim.org/tutorials?tut=ros_control) packages


```bash
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
sudo apt-get install ros-noetic-gazebo-ros-control
```

However, the robot is too small to push around big pallets of storage. To fix that, I had to make the robot 2ce bigger and had to fix wheels and other joints accordingly. I increased the friction coefficient of platform and wheels, so they grip more. 

And finally, we can make prismatic joint work by publishing to a ros joint.

```bash
rostopic pub -1 /amazon_robot/joint1_position_controller/command std_msgs/Float64 "data: 1"
```

The new exercise is now ready to use! Have fun!

{% include youtubePlayer.html id="HtUEAweSmAg" %}

## References

- [Robots - Dyno 1 documentation](https://dyno-docs.readthedocs.io/en/latest/simulation/robots.html#forklift)
- [samiamlabs/dyno](https://github.com/samiamlabs/dyno)
- [Friction](https://en.wikipedia.org/wiki/Friction#Coefficient_of_friction)
- Plugin for maps from gazebo state [marinaKollmitz/gazebo_ros_2Dmap_plugin](https://github.com/marinaKollmitz/gazebo_ros_2Dmap_plugin)
- [URDF Wiki](http://wiki.ros.org/urdf/XML/joint)


***Viszlát!***