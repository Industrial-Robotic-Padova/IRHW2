# README #

This repository will be used for the delivery of the homeworks of the Intelligent Robotics 2021-2022 course at UniPD.

In order to run our code:

the docker base image of tiago should be run:

<code>
rocker --home --user --nvidia --x11 palroboticssl/tiago_tutorials:melodic
</code>

Then our repository should be cloned inside "src" directory:

<code>
sudo git clone https://gkiavash1@bitbucket.org/iaslab-unipd/intelligentrobotics_group_04.git

cd intelligentrobotics_group_04
</code>

and build:

<code>
source /tiago_public_ws/devel/setup.bash

catkin build
</code>

Then, each of the following commands in seperate terminals:

<code>
roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=ias_lab_room_full
</code>

<code>
roslaunch tiago_iaslab_simulation navigation.launch
</code>

<code>
rosrun tiago_iaslab_simulation send_goal_client.py
</code>

<code>
rosrun tiago_iaslab_simulation get_pose_service.py
</code>

in send_goal_client node, you can send the new positions in "x,y,Rz" format. x and y refer to cartesian position and Rz(radian) is the orientation from refrence frame:

<code>
Enter your value: 8,0,0.2
<code>
