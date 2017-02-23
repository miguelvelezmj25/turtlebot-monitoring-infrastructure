#!/bin/bash
# Run the scripts that start nodes that alter data from topics.

file=$1
configuration=$2

source $HOME/.bashrc
rosrun measurement ${file} "${configuration}"
