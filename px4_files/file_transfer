#!/bin/bash

# Define the mapping of source files to target destinations
declare -A file_mappings=(
    ["/home/ubuntu/px4_files/airframes/4001_gz_x500"]="/home/ubuntu/px4_ros2_humble_shared_volume/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500"
    ["/home/ubuntu/px4_files/airframes/4016_gz_x500_tag"]="/home/ubuntu/px4_ros2_humble_shared_volume/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4016_gz_x500_tag"
    ["/home/ubuntu/px4_files/airframes/4017_gz_x500_tag_wind"]="/home/ubuntu/px4_ros2_humble_shared_volume/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4017_gz_x500_tag_wind"
    ["/home/ubuntu/px4_files/airframes/CMakeLists.txt"]="/home/ubuntu/px4_ros2_humble_shared_volume/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt"
    ["/home/ubuntu/px4_files/gz_bridge/CMakeLists.txt"]="/home/ubuntu/px4_ros2_humble_shared_volume/PX4-Autopilot/src/modules/simulation/gz_bridge/CMakeLists.txt"
    ["/home/ubuntu/px4_files/x500_tag/model.config"]="/home/ubuntu/px4_ros2_humble_shared_volume/PX4-Autopilot/Tools/simulation/gz/models/x500_tag/model.config"
    ["/home/ubuntu/px4_files/x500_tag/model.sdf"]="/home/ubuntu/px4_ros2_humble_shared_volume/PX4-Autopilot/Tools/simulation/gz/models/x500_tag/model.sdf"
    ["/home/ubuntu/px4_files/worlds/follow_drone.sdf"]="/home/ubuntu/px4_ros2_humble_shared_volume/PX4-Autopilot/Tools/simulation/gz/worlds/follow_drone.sdf"
    ["/home/ubuntu/px4_files/worlds/follow_drone_wind.sdf"]="/home/ubuntu/px4_ros2_humble_shared_volume/PX4-Autopilot/Tools/simulation/gz/worlds/follow_drone_wind.sdf"
)

# Loop through the mappings and copy each file or folder
for src in "${!file_mappings[@]}"; do
    dest="${file_mappings[$src]}"
    
    # Create destination directory if it doesn't exist
    dest_dir=$(dirname "$dest")
    mkdir -p "$dest_dir"
    
    # Copy the file or directory
    if [ -d "$src" ]; then
        echo "Copying directory: $src to $dest"
        cp -r "$src" "$dest"
    elif [ -f "$src" ]; then
        echo "Copying file: $src to $dest"
        cp "$src" "$dest"
    else
        echo "Warning: Source $src does not exist. Skipping."
    fi
done

echo "All files and directories have been copied successfully."