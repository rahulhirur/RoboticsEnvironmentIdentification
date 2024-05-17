# Robotics course - Homework 2 Solution

Here is a ROS package that implements the requirements of the second homework.

## Installation

Unzip the package and copy the root folder to the source folder of your workspace, `~/dev_ws/src`:

```shell
$ unzip thymiroomba-solution.zip
$ mv thymiroomba ~/dev_ws/src
```

Build your workspace, so that the new package becomes available

```shell
$ cd ~/dev_ws
$ colcon build
```

## Usage

Open CoppeliaSim:

```shell
$ ~/apps/CoppeliaSim_Edu_V4_4_0_Ubuntu22_04/coppeliaSim.sh
```

Load the first world, `wall_v2.ttt`, from the Coppelia GUI, enable real-time mode and start the simulation.

Open a second terminal and start the Mighty Thymio ROS bridge:

```shell
$ source ~/dev_ws/install/setup.bash
$ ros2 launch thymioid main.launch device:="tcp:host=localhost;port=33333" simulation:=True name:=thymio0
```

Open a third terminal, source your workspace and launch `align.launch.py`, which implements the second and third tasks: 

```shell
$ source ~/dev_ws/install/setup.bash
$ ros2 launch thymiroomba align.launch.py
```

Stop this node with Ctrl-C and stop the simulation. Load the second world, `HW2 scene.ttt` from the Coppelia GUI, enable real-time mode and start the simulation.

Restart the Mighty Thymio ROS bridge.

In the third terminal, launch `explore.launch.py`, which implements first bonus task (and the second, you just need to add a second Thymio and launch a second node!): 

```shell
$ ros2 launch thymiroomba explore.launch.py
```
