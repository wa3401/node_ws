# Node Workspace
### Written by Will Anderson and Ryan Brown

This repository contains all of the files that were developed for use in the F1Tenth Simulator and the beginnings of ros nodes interacting with the physical F1Tenth cars

## ROS Packages in /src
#### apriltags2 & apriltags2_ros
  - Provided by the F1Tenth Framework for help in lab 8
  - Process calculations and image processing for April Tag detection
#### autoturtle
  - A package that I worked with when I was brand new to using ROS
  - Helped me to understand the message, topic structure of the ROS framework
#### beginner_tutorials
  - Contains all necesary code for F1Tenth Lab 1
  - Another resource for learning ROS
#### car_reactive
  - Developed by Ryan Brown
  - For use on the F1Tenth vehicles using solely lidar
  - This will most likely be the package that we use for actual racing if we attend the F1Tenth Competition
  - Combination of follow the gap and pure pursuit algorithms
#### f1tenth_simulator
  - Package provided by F1Tenth
  - Uses a custom developed version of Rviz to simulate the F1Tenth Vehicle
#### gap_follow
  - F1Tenth Lab 4 - Follow the Gap
  - This is a completed version of follow the gap
#### lab2
  - F1Tenth Lab 2 - Automatic Emergency Braking
  - This is a completed implementation of automatic emergency braking
#### lab7
  - F1Tenth Lab 7 - Motion Planning (RRT)
  - Full implementation of rapidly exploring random trees
#### lidar_lab
  - Test package to make sure that the lidar scanner is working
#### pot_field
  - Add on to F1Tenth pure pursuit lab
  - Different implenmentation using potential fields
  - determined to not work as well as pure pursuit
#### pure_pursuit
  - F1Tenth Lab 5 - Pure Pursuit
  - A full implementation of the pure pursuit algorithim for use in simulation and on the cars
#### vehicle_tracker_predictio_skeleton
  - A package given by F1Tenth to help with the implementation of motion tracking using april tags
  - Used in F1Tenth Lab 8 - April Tag Detection
#### wall_follow
  - F1Tenth Lab 3 - Wall Following
  - A full implementation of the wall following algorithm for use in the simulator and on the cars
#### waypoint_logger
  - A package given by F1Tenth to log waypoints for use in the Pure Pursuit algorithm
