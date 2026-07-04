#!/bin/bash

# Set your paths and filename
SOURCE_DIR="/home/sysadmin/av_ws/edited_launch"
DEST_DIR="/home/sysadmin/project/autoware/src/launcher/autoware_launch/autoware_launch/launch"
FILENAME="autoware.launch.xml"

# Ensure the destination directory exists
mkdir -p "$DEST_DIR"

# Copy the XML file
cp "$SOURCE_DIR/$FILENAME" "$DEST_DIR"


# Confirm the copy
echo "selected Lab to Sub-station Route"

gnome-terminal -- bash -i -c "
  source ~/.bashrc; 
  source ~/.autoware_start.sh; 
  setup_aw_ws; 
  sleep 5;  
  sa;  
  exec bash" 

gnome-terminal -- bash -i -c "
  source ~/.bashrc; 
  source ~/.autoware_start.sh; 
  setup_aw_ws; 
  sleep 5;  
  echo 'c2' | autoware;  
  exec bash" 
  
gnome-terminal -- bash -i -c "
  source ~/.bashrc; 
  source ~/.autoware_start.sh; 
  setup_aw_ws; 
  sleep 5;  
  source ~/av_ws/install/setup.bash;  
  sleep 15;
  ros2 run campus_pkg shuttle
  exec bash" 
