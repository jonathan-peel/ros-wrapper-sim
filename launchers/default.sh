#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# Run below line inside container before running the wrapper to give your container a display
dt-exec Xvfb :1 -screen 0 1024x768x24 -ac +extension GLX +render -noreset 
export DISPLAY=:1

# launching app
roscore &
sleep 4
dt-exec rosrun ros_wrapper_sim_pkg ros_wrapper_sim_node.py


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
