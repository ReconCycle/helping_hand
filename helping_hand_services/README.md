# Helping Hand Services

This package provides ROS services that help you get started with kinesthetic teaching.

/dmp_capture
/helping_hand_services/get_loggers
/helping_hand_services/set_logger_level
/helping_hand_services/tf2_frames

# Services 

## `/joint_capture`
The service `/joint_capture` provides a simple interface to store messages of type `sensor_msgs/JointSate` to the MongoDB database.


## `/tf_capture`
The service `/joint_capture` provides a simple interface to store frames found on `TF` as `geometry_msgs/Transform`.

# The launchfiles

We provided a launchfile to help you get started.

## helping_hand_services.launch

Starts all t

This launchfile starts the `mongodb_store`, `mongo_tf_republisher` and the services provided with this ROS package.

Example:
```
$ roslaunch helping_hand_services helping_hand_services.launch db_path:=/path/to/db
```
Where `/path/to/db` is the path where you have a MongoDB database.
