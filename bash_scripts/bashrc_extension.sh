#!/usr/bin/env bash


# Reset ROS package awarenesses
export PYTHONPATH=
export AMENT_PREFIX_PATH=
export CMAKE_PREFIX_PATH=
export COLCON_PREFIX_PATH=


# 1. ROS 2 distro environment
source /opt/ros/humble/setup.bash


# 2. Workspace overlay (if already built)
[ -f "$HOME/base_ws/install/setup.bash" ] && source "$HOME/base_ws/install/setup.bash"
[ -f "$HOME/sensors_ws/install/setup.bash" ] && source "$HOME/sensors_ws/install/setup.bash"
[ -f "$HOME/task_ws/install/setup.bash" ] && source "$HOME/task_ws/install/setup.bash"


# 3. ROS Configuration
export RCUTILS_CONSOLE_OUTPUT_FORMAT="{severity}: {message}"
export RCUTILS_COLORIZED_OUTPUT=1
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=101
echo "ROS Domain ID - $ROS_DOMAIN_ID (local only = $ROS_LOCALHOST_ONLY)"


# 4. Connect to the can usb adapter
source $HOME/bash_scripts/can_setup.sh
# test can connection using: candump can0


# 5. Custom commands
alias t='tmux'
alias gs='git status'
alias nano='nano -BEPOSUWx -T 4'


# 6. Setup .tmux.conf
TMUX_CONF="$HOME/bash_scripts/tmux.conf"
[ ! -f "$HOME/.tmux.conf" ] && cp $TMUX_CONF "$HOME/.tmux.conf"
