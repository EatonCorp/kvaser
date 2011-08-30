#!/bin/sh
# Kvaser CAN driver                     
# leaf.sh - start/stop leaf and create/delete device files  
# this script can be used if hotplugging doesn't work
# Copyright (C) 2005 Kvaser AB - support@kvaser.com - www.kvaser.com  

LOG=`which logger`

# Add or remove leaf module
case "$1" in
   start)
      /sbin/modprobe -r leaf
      /sbin/modprobe leaf || exit 1
      $LOG -t $0 "Module leaf added"
      nrchan=`cat /proc/leaf | grep 'total channels' | cut -d \  -f 3`
      major=`cat /proc/devices | grep 'leaf' | cut -d \  -f 1`
      rm -f /dev/leaf*
      for minor in $(seq 0 `expr $nrchan - 1`) ; do
         $LOG -t $0 "Created /dev/leaf$minor"
         mknod /dev/leaf$minor c $major $minor
      done
      ;;
   stop)
      /sbin/modprobe -r leaf || exit 1
      rm -f /dev/leaf*
      $LOG -t $0 "Module leaf removed"
      ;;
   restart)
      rm -f /dev/leaf*
      /sbin/modprobe leaf || exit 1
      $LOG -t $0 "Module leaf added"
      nrchan=`cat /proc/leaf | grep 'total channels' | cut -d \  -f 3`
      major=`cat /proc/devices | grep 'leaf' | cut -d \  -f 1`
      rm -f /dev/leaf*
      for minor in $(seq 0 `expr $nrchan - 1`) ; do
         $LOG -t $0 "Created /dev/leaf$minor"
         mknod /dev/leaf$minor c $major $minor
      done
      ;;
   *)
      printf "Usage: %s {start|stop|restart}\n" $0
esac

exit 0 
