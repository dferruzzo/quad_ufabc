#!/bin/bash

# Launch rqt_plot with attitude topics
#rqt_plot /quad/imu/angular_velocity /gazebo/model_states/twist[2]/angular
rqt_plot /quad/kf/attitude /gazebo/model_states/pose[2]/orientation
