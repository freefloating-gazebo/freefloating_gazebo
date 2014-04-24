freefloating_gazebo
===================

A Gazebo plugin to simulate underwater vehicles and visualize with UWsim.


Builds two Gazebo plugins:

- freefloating_gazebo_fluid (world plugin)
simulates buoyancy and viscous force from water

- freefloating_gazebo_control (model plugin)
opens topics for wrench and joint states, in order to control the considered robots

Also builds an external PID controler: pid_control
These PID's allow position or velocity control of the vehicle body and joints. 
Subscribes to setpoint and states topics, and publishes on the wrench and torque topics that are subscribed to by the freefloating_gazebo_control plugin.
