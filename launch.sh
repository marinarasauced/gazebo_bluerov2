#!/usr/bin/env bash

trap "kill -- -$$" EXIT SIGINT SIGTERM

# add ArduPilotPlugin to Gazebo plugin path
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/build/_deps/ardupilot_gazebo-build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$PWD/build/_deps/ardupilot_gazebo-src/models:$PWD/build/_deps/ardupilot_gazebo-src/worlds:$GZ_SIM_RESOURCE_PATH

# add project to Gazebo resource path
export GZ_SIM_RESOURCE_PATH=$PWD/models:$PWD/worlds:$GZ_SIM_RESOURCE_PATH

gz sim -v4 -s -r bluerov2_camera.world &
gz sim -v4 -g

# kill all child processes
kill -- -$$
pkill -9 gz