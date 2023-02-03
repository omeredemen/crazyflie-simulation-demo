open new teminal and type these

export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=<absolute path to>/crazyflie-simulation-demo/gazebo-ignition/controllers/build/
export IGN_GAZEBO_RESOURCE_PATH=<absolute path to>/crazyflie-simulation-demo/gazebo-ignition/worlds

to run model 
ign gazebo -r -v 4 crazyflie_world.sdf 

open another terminal 
ros2 run ros_ign_bridge parameter_bridge /foo@std_msgs/msg/Float32]ignition.msgs.Float

open another terminal
ros2 topic pub /foo std_msgs/msg/Float32 "data: 2500"
