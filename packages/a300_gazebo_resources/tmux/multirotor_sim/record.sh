#!/bin/bash

path="/home/\$(optenv USER mrs)/ssd/bag_files/latest/"

# By default, we record everything.
# Except for this list of EXCLUDED topics:
exclude=(

# IN GENERAL, DON'T RECORD CAMERAS
#
# If you want to record cameras, create a copy of this script
# and place it at your tmux session.
#
# Please, seek an advice of a senior researcher of MRS about
# what can be recorded. Recording too much data can lead to
# ROS communication hiccups, which can lead to eland, failsafe
# or just a CRASH.

# Every topic containing "compressed"
# '(.*)compressed(.*)'
# Every topic containing "image_raw"
# '(.*)image_raw(.*)'
# Every topic containing "theora"
# '(.*)theora(.*)'
# Every topic containing "h264"
# '(.*)h264(.*)'

)

# file's header
filename=`mktemp`
echo "<launch>" > "$filename"
echo "<arg name=\"UAV_NAME\" default=\"\$(env UAV_NAME)\" />" >> "$filename"
echo "<group ns=\"\$(arg UAV_NAME)\">" >> "$filename"

echo -n "<node pkg=\"rosbag\" type=\"record\" name=\"rosbag_record\" output=\"screen\" args=\"-o $path -e /uav52/control_manager/controller_module/Telemetry|/uav52/estimation_manager/uav_state|/uav52/mavros/esc(.*)|/uav52/mavros/debug_value/debug_vector|/uav52/mavros/setpoint(.*)|/uav52/mavros/odometry/in|/uav52/mavros/imu/data_raw|/uav52/mavros/imu/data|/uav52/mavros/battery|/uav52/control_manager/diagnostics|(.*)tf(.*)|/uav52/mavros/local_position/velocity_body" >> "$filename"
# echo -n "<node pkg=\"rosbag\" type=\"record\" name=\"rosbag_record\" output=\"screen\" args=\"-o $path -a" >> "$filename"
# echo -n "<node pkg=\"rosbag\" type=\"record\" name=\"rosbag_record\" output=\"screen\" args=\"-o $path -e /uav52/control_manager/controller_module/(.*)|/uav52/estimation_manager/uav_state|/uav52/mavros/odometry/in|/uav52/mavros/imu/data_raw" >> "$filename"
 # /uav52/control_manager/diagnostics
# if there is anything to exclude
if [ "${#exclude[*]}" -gt 0 ]; then

  echo -n " -x " >> "$filename"

  # list all the string and separate the with |
  for ((i=0; i < ${#exclude[*]}; i++));
  do
    echo -n "${exclude[$i]}" >> "$filename"
    if [ "$i" -lt "$( expr ${#exclude[*]} - 1)" ]; then
      echo -n "|" >> "$filename"
    fi
  done

fi

echo "\">" >> "$filename"

echo "<remap from=\"~status_msg_out\" to=\"mrs_uav_status/display_string\" />" >> "$filename"
echo "<remap from=\"~data_rate_out\" to=\"~data_rate_MB_per_s\" />" >> "$filename"

# file's footer
echo "</node>" >> "$filename"
echo "</group>" >> "$filename"
echo "</launch>" >> "$filename"

cat $filename
roslaunch $filename
