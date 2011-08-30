#!/bin/bash
# Kvaser CAN driver                     
# leaf.sh - start/stop leaf and create/delete device files  
# this script can be used if hotplugging doesn't work
# Copyright (C) 2005 Kvaser AB - support@kvaser.com - www.kvaser.com  

LOG=`which logger`

#     
# test kernel version
#     
kernel_ver=`uname -r |awk -F . '{print $2}'` 

case $kernel_ver in
   "6") kv_module_install=modprobe
        ;;
   *)   kv_module_install=insmod
        ;;
esac

#
# Install
#

# Add or remove leaf module
case "$1" in
    start)
      /sbin/rmmod leaf
      /sbin/$kv_module_install leaf || exit 1
      $LOG -t $0 "Module leaf added"
      nrchan=`cat /proc/leaf | grep 'total channels' | awk '{print $3}'`
      major=`cat /proc/devices | grep 'leaf' | awk '{print $1}'`
      rm -f /dev/leaf*
      for (( minor=0 ; minor<$nrchan; minor++ )) ; do
         $LOG -t $0 "Created /dev/leaf$minor"
         mknod /dev/leaf$minor c $major $minor
      done
      ;;
    stop)
      /sbin/rmmod leaf || exit 1
      rm -f /dev/leaf*
      $LOG -t $0 "Module leaf removed"
      ;;
    *)
      printf "Usage: %s {start|stop}\n" $0
esac

exit 0 
