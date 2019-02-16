root_dir=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/nio/parkinglot3
desc_matcher_addr=/home/chamo/Documents/work/gaia/lidar_localization/devel/lib/desc_matcher/desc_matcher
merge_cloud_addr=/home/chamo/Documents/work/gaia/lidar_localization/devel/lib/align_gps/merge_cloud
align_gps_addr=/home/chamo/Documents/work/gaia/lidar_localization/devel/lib/align_gps/align_gps

recording_bag=${root_dir}/recording_bag.bag #[input necessary]
lidar_slam_traj=${root_dir}/lidar_slam_traj_true.txt #[input necessary]
config_yaml=${root_dir}/config.yaml #algo config [input necessary]
gound_remove_pcd=${root_dir}/ground_removed.pcd #[output files]
map_gps_file=${root_dir}/map_gps_file.txt #[intermedia files]
transformed_pcd=${root_dir}/transformed_pcd.pcd #[intermedia files]
transformed_slam_traj=${root_dir}/transformed_slam_traj.txt #[output files]
gps_cartesian=${root_dir}/gps_cartesian.txt #[output files]
desc_db_folder=${root_dir}/ #[output files]
anchor=${root_dir}/anchor.txt #[output files]
lidar_height=1.0 #[input necessary]
lidar_pitch=0.0 #[input necessary]

rosparam set align_gps/recording_bag ${recording_bag}
rosparam set align_gps/map_gps_file ${map_gps_file}
rosparam set align_gps/lidar_slam_traj ${lidar_slam_traj}
rosparam set align_gps/transformed_slam_traj ${transformed_slam_traj}
rosparam set align_gps/gps_cartesian ${gps_cartesian}
rosparam set align_gps/anchor ${anchor}
rosparam set align_gps/just_extract_gps true
rosparam set align_gps/use_existed_anchor false
rosparam set align_gps/use_nonlinear false
rosparam set align_gps/gps_weight 0.01
rosparam set align_gps/direction_weight 100
#${align_gps_addr}
rosparam set align_gps/just_extract_gps false
#${align_gps_addr}

#${merge_cloud_addr} ${recording_bag} ${transformed_slam_traj} ${transformed_pcd}

rosparam load ${config_yaml}
rosparam set desc_matcher/recording_bag ${recording_bag}
rosparam set desc_matcher/lidar_slam_traj ${transformed_slam_traj}
rosparam set desc_matcher/gound_remove_pcd ${gound_remove_pcd}
rosparam set desc_matcher/lidar_height ${lidar_height}
rosparam set desc_matcher/lidar_pitch ${lidar_pitch}
rosparam set desc_matcher/desc_db_folder ${desc_db_folder}
rosparam set desc_matcher/operation_type remove_ground
${desc_matcher_addr}

rosparam set desc_matcher/gound_remove_pcd ${gound_remove_pcd}
rosparam set desc_matcher/operation_type "extract_desc"
${desc_matcher_addr}

