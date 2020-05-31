# colab-gsoc2020-Shreyas_Gokhale

## JdeMultiBot

The GSoC 2020 writeup for this repo is hosted [here](https://theroboticsclub.github.io/colab-gsoc2020-Shreyas_Gokhale/).


## Exercises
You can test the exercises by either running ros and python code directly in the `exercise` folder 
or using docker compose as follows:

If you have an Intel driver, run the following **from the base directory**

```shell script
xhost +"local:docker@" 
docker-compose up --build 
```

For NVIDIA, you will have to jump through some hoops. Follow guides on [[1]]  and [[2]].

> The first command grants x11 access to docker group. After using, just de-elevate the user access. 

[1]: http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration
[2]: https://github.com/NVIDIA/nvidia-docker