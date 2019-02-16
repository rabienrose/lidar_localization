root_dir=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/nio/parkinglot3
merge_cloud_addr=/home/chamo/Documents/work/gaia/lidar_localization/devel/lib/align_gps/merge_cloud

recording_raw_bag=${root_dir}/recording_bag.bag #[input necessary]
lidar_slam_traj=${root_dir}/transformed_slam_traj.txt #[input necessary]
pc_transformed_out=${root_dir}/transformed_pcd.pcd #[output necessary]

${merge_cloud_addr} ${recording_raw_bag} ${lidar_slam_traj} ${pc_transformed_out}

