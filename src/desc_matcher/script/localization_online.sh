root_dir=/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/nio/parkinglot2
desc_matcher_addr=/home/chamo/Documents/work/gaia/lidar_localization/devel/lib/desc_matcher/desc_matcher

config_yaml=${root_dir}/config.yaml #algo config [input necessary]
desc_db_folder=${root_dir}/ #[input necessary]
lidar_height=1.0 #[input necessary]
lidar_pitch=0.0 #[input necessary]

rosparam load ${config_yaml}
rosparam set desc_matcher/desc_db_folder ${desc_db_folder}
rosparam set desc_matcher/lidar_height ${lidar_height}
rosparam set desc_matcher/lidar_pitch ${lidar_pitch}
rosparam set desc_matcher/operation_type localization
rosparam set desc_matcher/is_online true
rosparam set desc_matcher/trans_baselink_lidar "[-0.0009095137448635066, 0.006226740743189497, 0.6946312115313276, 0.7193385019940092]" # x, y, z, w
${desc_matcher_addr}
