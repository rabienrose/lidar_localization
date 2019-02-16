root_dir=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/nio/parkinglot2
desc_matcher_addr=/home/chamo/Documents/work/gaia/lidar_localization/devel/lib/desc_matcher/desc_matcher

recording_bag=${root_dir}/recording_bag.bag #[input necessary]
lidar_slam_traj=${root_dir}/lidar_slam_traj_odom.txt #[input necessary]
config_yaml=${root_dir}/config.yaml #algo config [input necessary]
desc_db_folder=${root_dir}/ #[input necessary]
gps_cartesian=${root_dir}/gps_cartesian.txt #[output files]
lidar_height=1.0 #[input necessary]
lidar_pitch=0.0 #[input necessary]

rosparam load ${config_yaml}
rosparam set desc_matcher/desc_db_folder ${desc_db_folder}
rosparam set desc_matcher/recording_bag ${recording_bag}
rosparam set desc_matcher/lidar_slam_traj ${lidar_slam_traj}
rosparam set desc_matcher/lidar_height ${lidar_height}
rosparam set desc_matcher/lidar_pitch ${lidar_pitch}
rosparam set desc_matcher/loc_gps_truth ${gps_cartesian}
rosparam set desc_matcher/operation_type localization
rosparam set desc_matcher/is_online false
${desc_matcher_addr}
