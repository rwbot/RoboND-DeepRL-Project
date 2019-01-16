#!/usr/bin/env bash
# Call from .student_bashrc
# 		source /home/workspace/RoboND-DeepRL-Project/udasetup.sh
# Manually source this file using:
#		sudo -s source /home/workspace/RoboND-DeepRL-Project/udasetup.sh
alias srcb="source /home/workspace/RoboND-DeepRL-Project/udasetup.sh"

alias apm="sudo apt-get install libignition-math2-dev -y"

alias rr="cdcbz; ./gazebo-arm.sh"
alias rro="cdcbz; ./gazebo-arm.sh | tee /home/workspace/RoboND-DeepRL-Project/tests/output.txt"
alias rrd="cdcbz; ./gazebo-arm.sh | tee /home/workspace/RoboND-DeepRL-Project/tests/$(date +%a%H:%M:%S).txt"
alias rrd="srcb; cdcbz; ./gazebo-arm.sh | tee /home/workspace/RoboND-DeepRL-Project/tests/$(date +%a%H:%M:%S).txt"


alias cdc="cd /home/workspace/RoboND-DeepRL-Project" #custom cd command that goes to catkin_ws
alias cdcb="cd /home/workspace/RoboND-DeepRL-Project/build"
alias cdcbz="cd /home/workspace/RoboND-DeepRL-Project/build/x86_64/bin"
alias cdcmk="cdcb; make" #goes to catkin_ws, runs catkin_make, waits until it's made, then sources devel/setup.bash


alias gcm="cdc; git checkout master"
alias gcw="cdc; git checkout workspaces"
alias gpm="cdc; git pull rw master"
alias gpw="cdc; git pull rw workspaces"

alias kz="killall gazebo & killall gzserver & killall gzclient" #kills gazebo if frozen
alias gitcon="git config --global credential.helper 'cache --timeout=999999'; git config --global user.name 'rwbot'; git config --global user.email 'rwbotx@gmail.com'"
