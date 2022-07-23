#!/usr/bin/env bash

echo "Starting node mr_scene_configurator......"
roslaunch mr_tasks mr_scene_configurator.launch &
sleep 5
echo "Started node mr_scene_configurator!"

echo "================================================================================"

echo "Starting node mr_massage_test......"
roslaunch mr_tasks mr_massage_test.launch &
sleep 0.1
echo "Started node mr_massage_test!"

wait
exit 0
