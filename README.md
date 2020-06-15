# helping_hand
A framework to facilitate kinesthetic teaching of robot motions.

# Installation instructions

This installation guide assumes you have a working `catkin` workspace set up and all the ROS dependencies already installed

## Clone the repository
Navigate to the `src/` directory of your catkin workspace and clone this repo:
`git clone https://github.com/tgaspar/helping_hand.git`

## Initialize the workspace
Merge all the dependencies

`$ wstool init`
`$ wstool merge helping_hand/dependencies.rosinstall`
`$ wstool up`

## Get the dependencies

The cloned dependencies should show up in your src directory. You can now proceed to query and install all libraries and packages:

`$ rosdep install --from-paths . --ignore-src --rosdistro kinetic`

Finally, build the packages:

`$ cd ..`
`$ catkin build`


# Usage

This package contains various tools to facilitate kinesthetic teaching of robots. However, the main tool is the provided GUI. To launch the GUI we suggest you use the provided launch file:

`roslaunch helping_hand_gui gui.launch`
