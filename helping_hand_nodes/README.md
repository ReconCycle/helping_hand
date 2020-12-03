# Helping Hand Nodes

This package provides ROS nodes to help with the communication to the MongoDB database. See the [mongodb_store](http://wiki.ros.org/mongodb_store) package.

- [Helping Hand Nodes](#helping-hand-nodes)
- [The nodes](#the-nodes)
  - [mongo_tf_republisher](#mongo_tf_republisher)
  - [num2bool_republisher](#num2bool_republisher)
- [The launchfiles](#the-launchfiles)
  - [db_republisher.launch](#db_republisherlaunch)
  - [example_num2bool_rep.launch](#example_num2bool_replaunch)

# The nodes

## mongo_tf_republisher
This node reads the frames that were saved in the database and publishes them on the TF.

## num2bool_republisher
This node converts a topic that publishes a number to one that publishes a boolean message. 

# The launchfiles

We provided a couple of launchfiles to help you get started.

## db_republisher.launch

This launchfile starts the `mongodb_store` and the `mongo_tf_republisher`.

Example:
```
$ roslaunch helping_hand_nodes db_republisher.launch db_path:=/path/to/db
```
Where `/path/to/db` is the path where you have a MongoDB database.

## example_num2bool_rep.launch

This launchfile shows how to use the `num2bool_republisher` node.