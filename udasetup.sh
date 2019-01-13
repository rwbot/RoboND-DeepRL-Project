#!/usr/bin/env bash
# Manually source this file using:
#     sudo source /home/workspace/RoboND-DeepRL-Project/udasetup.sh

#alias apm="sudo apt-get install libignition-math2-dev"

alias gitcon="git config --global credential.helper 'cache --timeout=999999'; git config --global user.name 'rwbot'; git config --global user.email 'rwbotx@gmail.com'"

alias cdc="cd /home/workspace/RoboND-DeepRL-Project" #custom cd command that goes to catkin_ws
alias cdcb="cd /home/workspace/RoboND-DeepRL-Project/build"
alias cdcbz="cd /home/workspace/RoboND-DeepRL-Project/build/x86_64/bin"
alias cdcmk="cdcb; make" #goes to catkin_ws, runs catkin_make, waits until it's made, then sources devel/setup.bash
alias rr="cdcbz; ./gazebo-arm.sh"

alias gcm="cdc; git checkout master"
alias gcw="cdc; git checkout workspaces"
alias gpm="cdc; git pull rw master"
alias gpw="cdc; git pull rw workspaces"

alias kz="killall gazebo & killall gzserver & killall gzclient" #kills gazebo if frozen
