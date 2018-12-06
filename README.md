# A Gazebo plugin to simulate and control underwater vehicles

## Gazebo plugins
The package builds two Gazebo plugins:

- freefloating_gazebo_fluid (world plugin)
simulates buoyancy and viscous force from water

- freefloating_gazebo_control (model plugin)
opens topics for thrusters and joints, in order to control the considered robots

## Built-in PID control

An external PID controler is available: pid_control.

These PID's allow position, velocity or effort control of the vehicle body and joints.

Subscribes to setpoint and states, and publishes on the thruster and torque topics that are subscribed to by the freefloating_gazebo_control plugin.
Private parameters `body_control` and `joint_control` allow setting the default mode (body: position / velocity / effort (body wrench), joint: position / velocity / effort).

This PID advertises the corresponding services to switch some of the axis (joint names or x, y, z, roll, pitch, yaw) to another type of control. 

## Examples

The examples can be downloaded from the freefloating_gazebo_demo package.

## References

Please use the following reference when citing this work:

[O. Kermorgant, "A dynamic simulator for underwater vehicle-manipulators", International Conference on Simulatation, Modeling and Programming for Autonomous Robots SIMPAR 2014, Oct 2014](https://hal.archives-ouvertes.fr/hal-01065812)
