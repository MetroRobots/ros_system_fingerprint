# system_fingerprint
**A simple tool for getting information about a system to share**

Let's say you're helping someone debug their system.
You could ask them a bunch of questions, and get them to send you the output of various commands,
but that sounds tedious.

Instead, have them install this package (`sudo apt-get install ros-$ROS_DISTRO-system-fingerprint`)
and then run a single command:

    ros2 run system_fingerprint imprint

This will generate the file `fingerprint.yaml` which they can send to you for debugging.
What sort of information is in this file?

 * System Information (`system`) - Information about your OS and other tidbits about the machine the command is run on.
 * Environmental Variables (`environmental_variables`) - All the environmental variables with the prefixes `ROS_`, `AMENT_`, `COLCON_` or `RCUTILS_`
    such as `ROS_DISTRO`, `ROS_VERSION`, etc.
 * Parameters (`parameters`) - A full dump of the ROS parameters.
 * Nodes (`nodes`) - The full rosgraph, i.e. every node and their publications, subscriptions and services.
 * Topics (`topics`) - The type of each available topic
 * Services (`services`) - The type of each available service
 * Actions (`actions`) - The type of each available action
 * ROS Workspace (`workspace`) - What build tool you're using, the workspace location,
    which repos you have checked out and what version they are on.

## Example
 See [example_fingerprint.yaml](example_fingerprint.yaml)
