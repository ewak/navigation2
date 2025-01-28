#!/usr/bin/env bash

# Helper to be used as launch prefix to run node
# under gdb in a tmux split window
# with the correct environment enabled
#prefix=[f'tmux split-window {nav2_bringup_dir}/gdb_tmux_splitwindow.sh'],

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

echo $@

#export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#export CYCLONEDDS_URI=
#export GZ_VERSION=harmonic
#User specific env setup like
#/opt/aos/skel/.devenv/zenoh_humble.sh

export ASAN_OPTIONS=new_delete_type_mismatch=0

source /opt/ros/rolling/setup.bash
source ${SCRIPT_DIR}/../../../setup.bash

#Use -q to disable banner on startup and avoid pager
#Use ~/.gdbinit to set breakpoints and/or other options
#cat ~/.gdbinit
#set debuginfod enabled off
#set breakpoint pending on
#break __asan::ReportGenericError

gdb -q -ex run --args $@
#valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes --vgdb --log-file=valgrind_output.txt $@
