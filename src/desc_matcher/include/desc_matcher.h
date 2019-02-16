#include <memory>
#include <segmatch/utilities.hpp>
#include <ros/ros.h>
#include "segmatch/segmatch.hpp"
#include <nabo/nabo.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

class desc_matcher{
public:
    desc_matcher();
    void publishTargetRepresentation();
    void publishSourceRepresentation();
    void normalizeEigenFeatures(Eigen::MatrixXf* f);
    bool processLocalMap(segmatch::SegMatch::LocalMapT& local_map, Eigen::Matrix4f& pose_out);
    bool readAScan(pcl::PointCloud<segmatch::PclPoint>::Ptr& new_points, laser_slam::Pose& pose, Eigen::Vector3f& out_gps, double& time);
    bool readAScan_ros(pcl::PointCloud<segmatch::PclPoint>::Ptr& new_points, laser_slam::Pose& pose, Eigen::Vector3f& out_gps, double& time);
    void publishPose(Eigen::Matrix4f g_pose);
    void main_thread();
    void gen_map(bool gen_pcd);
    void removeGround(pcl::PointCloud<segmatch::PclPoint>::Ptr& pc_in);
    void removeGround_by_z(pcl::PointCloud<segmatch::PclPoint>::Ptr& pc_in);
    void loadDB();
    void export_map();
    void publishMatches();
    void publishGPS(Eigen::Vector3f gps);
    void odom_pose_callback(const geometry_msgs::PoseStampedPtr& msg);
    void scan_callback(const sensor_msgs::PointCloud2Ptr& msg);
    void publishGlobalPose(Eigen::Matrix4f pose, double time);
private:
    std::shared_ptr<segmatch::SegMatch> segmatch_;
    segmatch::SegMatchParams params_;
    ros::Publisher target_representation_pub_;
    ros::Publisher query_representation_pub_;
    ros::Publisher source_representation_pub_;
    ros::Publisher match_result_centroids_pub_;
    ros::Publisher matches_pub_;
    ros::Publisher gps_pub_;
    ros::Publisher result_pub_;
    ros::Subscriber odom_pose_sub;
    ros::Subscriber scan_sub;
    
    Nabo::NNSearchF* nns_ = NULL;
    Eigen::MatrixXf inverted_max_eigen_float_;
    Eigen::MatrixXf target_matrix_;
    std::vector<unsigned int> indices_list;
    segmatch::LocalMap<segmatch::PclPoint, segmatch::MapPoint>* local_maps_;
    std::map<double, Eigen::Matrix4d> pose_list;
    std::map<double, pcl::PointCloud<segmatch::PclPoint>::Ptr> scan_list;
    std::string lidar_slam_traj;
    std::string recording_bag;
    std::string gound_remove_pcd;
    std::string loc_gps_truth;
    std::string desc_db_folder;
    float lidar_height;
    float lidar_pitch;
    rosbag::Bag in_bag;
    int processed_count;
    Eigen::Matrix4d T_cw_clt1;
    Eigen::Matrix4f cur_pose;
    segmatch::PointICloud match_posi;
    segmatch::PointICloud gps_posi;
    Eigen::Matrix4d T_w_lt1;
    Eigen::Matrix4f last_right_T_w_cw;
    Eigen::Quaterniond T_base_lidar;
    bool is_online;
    std::map<double, Eigen::Vector3f> gps_list;
    std::ofstream lidar_traj_file;
    std::vector<laser_slam::Trajectory> his_traj;
};