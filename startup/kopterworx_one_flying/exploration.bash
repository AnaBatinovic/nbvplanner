#!/bin/bash

# Define folder paths
start_path=/home/ana/nbvp_ws/src/nbvplanner/startup/kopterworx_one_flying

for max_range in {0..10..1}
  do
    rosbag record -o original-maze /red/octomap_volume /red/comp_times __name:=my_bag &
    ./start.sh 
    rosnode kill /my_bag
    killall gzserver
    done
done
