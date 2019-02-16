root_dir=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/nio/parkinglot3
align_gps_addr=/home/chamo/Documents/work/gaia/lidar_localization/devel/lib/align_gps/align_gps

recording_raw_bag=${root_dir}/recording_bag.bag #[input necessary]
lidar_slam_traj=${root_dir}/lidar_slam_traj_true.txt #[input necessary]
anchor=${root_dir}/anchor.txt #[input necessary]

map_gps_file=${root_dir}/map_gps_file.txt #[intermedia files]

transformed_slam_traj=${root_dir}/transformed_slam_traj.txt #[output files]
gps_cartesian=${root_dir}/gps_cartesian.txt #[output files]

rosparam set align_gps/recording_bag ${recording_raw_bag}
rosparam set align_gps/map_gps_file ${map_gps_file}
rosparam set align_gps/lidar_slam_traj ${lidar_slam_traj}
rosparam set align_gps/transformed_slam_traj ${transformed_slam_traj}
rosparam set align_gps/gps_cartesian ${gps_cartesian}
rosparam set align_gps/anchor ${anchor}
rosparam set align_gps/just_extract_gps true
rosparam set align_gps/use_existed_anchor false
rosparam set align_gps/gps_lidar_delay 0.21
rosparam set align_gps/use_nonlinear true
rosparam set align_gps/gps_weight 0.01
rosparam set align_gps/direction_weight 100
#${align_gps_addr}
rosparam set align_gps/just_extract_gps false
${align_gps_addr}

