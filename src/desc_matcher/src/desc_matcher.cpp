#include "desc_matcher.h"
#include "segmatch/rviz_utilities.hpp"
#include "chamo_util.h"
#include <laser_slam/common.hpp>
#include "segmatch/normal_estimators/normal_estimator.hpp"
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/random_sample.h>
#include "segmatch/database.hpp"
#include <visualization_msgs/Marker.h>



std::vector<std::string> split(const std::string& str, const std::string& delim)
{
    std::vector<std::string> tokens;
    size_t prev = 0, pos = 0;
    do
    {
        pos = str.find(delim, prev);
        if (pos == std::string::npos) pos = str.length();
        std::string token = str.substr(prev, pos-prev);
        if (!token.empty()) tokens.push_back(token);
        prev = pos + delim.length();
    }
    while (pos < str.length() && prev < str.length());
    return tokens;
}

bool get_nn_pose(std::map<double, Eigen::Matrix4d>& poses, double cur_time ,Eigen::Matrix4d& out_pose){
    std::map<double, Eigen::Matrix4d>::iterator it1 =poses.lower_bound (cur_time); 
    if(it1== poses.end()){
        return false;
    }
    std::map<double, Eigen::Matrix4d>::iterator it2=it1--;
    if(it2== poses.end()){
        return false;
    }
    if(cur_time-it1->first<0 || it2->first-cur_time<0){
        std::cout<<"timeline wrong!!"<<std::endl;
        return false;
    }
    if(cur_time-it1->first>0.01 && it2->first-cur_time>0.01){
        std::cout<<"too far!!"<<std::endl;
        return false;
    }
    if(cur_time-it1->first >it2->first-cur_time){
        out_pose=it2->second;
    }else{
        out_pose=it1->second;
    }
    return true;
}

bool get_nn_scan(std::map<double, pcl::PointCloud<segmatch::PclPoint>::Ptr>& scans, double cur_time ,pcl::PointCloud<segmatch::PclPoint>::Ptr& out_scan){
    std::map<double, pcl::PointCloud<segmatch::PclPoint>::Ptr>::iterator it1 =scans.lower_bound (cur_time); 
    if(it1== scans.end()){
        std::cout<<"reach end"<<std::endl;
        return false;
    }
    std::map<double, pcl::PointCloud<segmatch::PclPoint>::Ptr>::iterator it2=it1--;
    if(it2== scans.end()){
        std::cout<<"reach begin"<<std::endl;
        return false;
    }
    if(cur_time-it1->first<0 || it2->first-cur_time<0){
        std::cout<<"timeline wrong!!"<<std::endl;
        return false;
    }
    if(cur_time-it1->first>0.01 && it2->first-cur_time>0.01){
        std::cout<<"too far!!"<<std::endl;
        return false;
    }
    if(cur_time-it1->first >it2->first-cur_time){
        out_scan=it2->second;
        scans.erase(scans.begin(), it2);
    }else{
        out_scan=it1->second;
        scans.erase(scans.begin(), it1);
    }
    
    return true;
}

bool get_nn_posi(std::map<double, Eigen::Vector3f>& posis, double cur_time ,Eigen::Vector3f& out_posi){
    if (posis.size()==0){
        return false;
    }
    std::map<double, Eigen::Vector3f>::iterator it1 =posis.lower_bound (cur_time); 
    if(it1== posis.end()){
        return false;
    }
    std::map<double, Eigen::Vector3f>::iterator it2=it1--;
    if(it2== posis.end()){
        return false;
    }
    if(cur_time-it1->first<0 || it2->first-cur_time<0){
        std::cout<<"timeline wrong!!"<<std::endl;
        return false;
    }
    if(cur_time-it1->first>0.01 && it2->first-cur_time>0.01){
        std::cout<<"too far!!"<<std::endl;
        return false;
    }
    if(cur_time-it1->first >it2->first-cur_time){
        out_posi=it2->second;
    }else{
        out_posi=it1->second;
    }
    return true;
}

void tranforam_pc(pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr new_pt_cloud,Eigen::Matrix4d pose){
    for (int j=0; j<pt_cloud->points.size(); j++){
        Eigen::Vector4d p(pt_cloud->points[j].x, pt_cloud->points[j].y, pt_cloud->points[j].z, 1);
        Eigen::Vector4d p1=pose*p;
        pcl::PointXYZ p_pcl(p1.x(), p1.y(), p1.z());
        new_pt_cloud->points.push_back(p_pcl);
    }
}

void desc_matcher::odom_pose_callback(const geometry_msgs::PoseStampedPtr& msg){
    double lidar_time=msg->header.stamp.toSec();
    Eigen::Vector3d posi(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z); 
    Eigen::Matrix4d pose_m=Eigen::Matrix4d::Identity();
    pose_m.block(0,3,3,1)=posi;
    Eigen::Quaterniond qua;
    qua.x()=msg->pose.orientation.x;
    qua.y()=msg->pose.orientation.y;
    qua.z()=msg->pose.orientation.z;
    qua.w()=msg->pose.orientation.w;
    qua=qua*T_base_lidar;
    pose_m.block(0,0,3,3)=qua.toRotationMatrix();
    pose_list[lidar_time]=pose_m;
//     std::stringstream ss;
//     ss<<std::setprecision (20)<<lidar_time<<","<<msg->pose.position.x<<","<<msg->pose.position.y<<","<<msg->pose.position.z<<","<<qua.w()<<","<<qua.x()<<","<<qua.y()<<","<<qua.z()<<std::endl;
//     lidar_traj_file<<ss.str();
    
}

void desc_matcher::scan_callback(const sensor_msgs::PointCloud2Ptr& lidar_msg){
    double lidar_time=lidar_msg->header.stamp.toSec();
    pcl::PointCloud<segmatch::PclPoint>::Ptr pt_cloud(new pcl::PointCloud<segmatch::PclPoint>);
    pcl::fromROSMsg (*lidar_msg, *pt_cloud);
    removeGround_by_z(pt_cloud);
    scan_list[lidar_time]=pt_cloud;
}


desc_matcher::desc_matcher(){
    ros::NodeHandle node_handle;
    node_handle.getParam("/desc_matcher/recording_bag", recording_bag);
    node_handle.getParam("/desc_matcher/lidar_slam_traj", lidar_slam_traj);
    node_handle.getParam("/desc_matcher/gound_remove_pcd", gound_remove_pcd);
    node_handle.getParam("/desc_matcher/loc_gps_truth", loc_gps_truth);
    node_handle.getParam("/desc_matcher/desc_db_folder", desc_db_folder);
    node_handle.getParam("/desc_matcher/lidar_height", lidar_height);
    node_handle.getParam("/desc_matcher/lidar_pitch", lidar_pitch);
    node_handle.getParam("/desc_matcher/is_online", is_online);
    std::vector<double> temp_vec;
    node_handle.getParam("/desc_matcher/trans_baselink_lidar", temp_vec);
    if(temp_vec.size()==4){
        T_base_lidar.x()=temp_vec[0];
        T_base_lidar.y()=temp_vec[1];
        T_base_lidar.z()=temp_vec[2];
        T_base_lidar.w()=temp_vec[3];
    }
    
    std::cout<<"lidar-baselink: "<<T_base_lidar.x()<<","<<T_base_lidar.y()<<","<<T_base_lidar.z()<<","<<T_base_lidar.w()<<std::endl;
    
    odom_pose_sub = node_handle.subscribe("/odometry_pose", 1000, &desc_matcher::odom_pose_callback, this);
    scan_sub = node_handle.subscribe("/pandar/compensator/PointCloud2", 1000, &desc_matcher::scan_callback, this);
    result_pub_ = node_handle.advertise<geometry_msgs::PoseStamped>("/descriptor_pose", 1);
    params_= getSegMatchParams(node_handle, "/desc_matcher");
    target_representation_pub_ = node_handle.advertise<sensor_msgs::PointCloud2>("/chamo_target", 1);
    source_representation_pub_ = node_handle.advertise<sensor_msgs::PointCloud2>("/chamo_source", 1);
    query_representation_pub_ = node_handle.advertise<sensor_msgs::PointCloud2>("/chamo_query", 1);
    match_result_centroids_pub_ = node_handle.advertise<sensor_msgs::PointCloud2>("/match_result_centroids", 1);
    gps_pub_ = node_handle.advertise<sensor_msgs::PointCloud2>("/chamo/gps", 1);
    matches_pub_ = node_handle.advertise<visualization_msgs::Marker>("/segmatch/predicted_segment_matches", 1);
    segmatch_.reset(new segmatch::SegMatch());
    segmatch_->init(params_, 1);
      
    if(lidar_slam_traj!=""){
        std::ifstream infile(lidar_slam_traj);
        std::string line;
        while(ros::ok()){
            std::getline(infile, line);
            if (line==""){
                break;
            }
            std::vector<std::string> splited = split(line, ",");
            double time_stamp;
            time_stamp=atof(splited[0].c_str());
            double x=atof(splited[1].c_str());
            double y=atof(splited[2].c_str());
            double z=atof(splited[3].c_str());
            double qw=atof(splited[4].c_str());
            double qx=atof(splited[5].c_str());
            double qy=atof(splited[6].c_str());
            double qz=atof(splited[7].c_str());

            Eigen::Vector3d trans=Eigen::Vector3d(x,y,z);
            Eigen::Quaterniond qua=Eigen::Quaterniond(qw, qx, qy, qz);
            Eigen::Matrix3d R = qua.toRotationMatrix();
            Eigen::Matrix4d tran_mat= Eigen::Matrix4d::Identity();
            tran_mat.block<3,3>(0,0)=R;
            tran_mat.block<3,1>(0,3)=trans;
            pose_list[time_stamp]=tran_mat;
        }
        infile.close();
    }
    
    
    if(loc_gps_truth!=""){
        std::ifstream infile_gps(loc_gps_truth);
        std::string line;
        while(ros::ok()){
            std::getline(infile_gps, line);
            if (line==""){
                break;
            }
            std::vector<std::string> splited = split(line, ",");
            double time_stamp;
            time_stamp=atof(splited[0].c_str());
            double x=atof(splited[1].c_str());
            double y=atof(splited[2].c_str());
            double z=atof(splited[3].c_str());

            gps_list[time_stamp]=Eigen::Vector3f(x, y, z);
        }
        infile_gps.close();
    }
    
    
    
    if(recording_bag!=""){
        in_bag.open(recording_bag, rosbag::bagmode::Read);  
    }
      
    processed_count=0;
    std::unique_ptr<segmatch::NormalEstimator> normal_estimator = nullptr;
    local_maps_ = new segmatch::LocalMap<segmatch::PclPoint, segmatch::MapPoint>(params_.local_map_params, std::move(normal_estimator)); 
    lidar_traj_file.open("lidar_traj.txt");

}

void desc_matcher::normalizeEigenFeatures(Eigen::MatrixXf* f) {
    for (size_t i = 0u; i < f->rows(); ++i) {
        f->block(i, 0, 1, 7) = f->block(i, 0, 1, 7).cwiseProduct(inverted_max_eigen_float_);
    }
}

bool desc_matcher::readAScan_ros(pcl::PointCloud<segmatch::PclPoint>::Ptr& new_points, laser_slam::Pose& pose, Eigen::Vector3f& out_gps, double& time){
    
    if(pose_list.size()>0){
        double pose_time=0;
        Eigen::Matrix4d pose_m=Eigen::Matrix4d::Identity();
        for (std::map<double, Eigen::Matrix4d>::iterator pose_it= pose_list.begin(); pose_it!= pose_list.end(); pose_it++){
            pose_time = pose_it->first;
            pose_m = pose_it->second;
            pcl::PointCloud<segmatch::PclPoint>::Ptr new_points_one(new pcl::PointCloud<segmatch::PclPoint>);
            bool re = get_nn_scan(scan_list, pose_time, new_points_one);
            if(re){
                pcl::PointCloud<pcl::PointXYZ>::Ptr g_pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                tranforam_pc(new_points_one, g_pt_cloud,pose_m);
                *new_points+=*g_pt_cloud;
            }
        }
        
        pose.key=processed_count;
        pose.time_ns=pose_time*1000*1000*1000;
        kindr::minimal::QuatTransformationTemplate<double> pose_kindr(pose_m);
        pose.T_w=pose_kindr;
        pose_list.clear();
        processed_count++;
        time=pose_time;
        if(new_points->points.size()>0){
            return true;
        }else{
            return false;
        }
        
    }
    
    return false;
}

bool desc_matcher::readAScan(pcl::PointCloud<segmatch::PclPoint>::Ptr& new_points, laser_slam::Pose& pose, Eigen::Vector3f& out_gps, double& time){
    std::vector<std::string> topics;
    topics.push_back("/pandar/compensator/PointCloud2");
    rosbag::View view(in_bag, rosbag::TopicQuery(topics));
    rosbag::View::iterator it= view.begin();
    int temp_count=0;
    for(;it!=view.end();it++){
        if(processed_count>=temp_count){
            temp_count++;
            continue;
        }
        processed_count++;
        rosbag::MessageInstance m =*it;
        sensor_msgs::PointCloud2Ptr lidar_msg = m.instantiate<sensor_msgs::PointCloud2>();
        double lidar_time=lidar_msg->header.stamp.toSec();
        Eigen::Matrix4d Twl;
        bool re = get_nn_pose(pose_list, lidar_time, Twl);
        //Twl(0,3)=Twl(0,3)+100;
        //std::cout<<Twl<<std::endl;
        if(re==true){
            //std::cout<<"package: "<<processed_count<<std::endl;
            pcl::PointCloud<segmatch::PclPoint>::Ptr pt_cloud(new pcl::PointCloud<segmatch::PclPoint>);
            pcl::fromROSMsg (*lidar_msg, *pt_cloud);
            removeGround_by_z(pt_cloud);
            pcl::PointCloud<pcl::PointXYZ>::Ptr g_pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            tranforam_pc(pt_cloud, g_pt_cloud,Twl);
            *new_points=*g_pt_cloud;
            //std::cout<<"pt_cloud: "<<pt_cloud->points.size()<<std::endl;
            pose.key=processed_count;
            pose.time_ns=lidar_time*1000*1000*1000;
            kindr::minimal::QuatTransformationTemplate<double> pose_kindr(Twl);
            pose.T_w=pose_kindr;
            get_nn_posi(gps_list, lidar_time ,out_gps);
            time=lidar_time;
        }
        return true;
    }
    return false;
}

void desc_matcher::loadDB(){
    segmatch::database::importSessionDataFromDatabase(&segmatch_->segmented_target_cloud_, desc_db_folder);
    segmatch_->setAsTargetCloud();
}

void desc_matcher::removeGround_by_z(pcl::PointCloud<segmatch::PclPoint>::Ptr& pc_in){
    pcl::PointCloud<segmatch::PclPoint>::Ptr cloud_f(new pcl::PointCloud<segmatch::PclPoint>);
    float max_range=100;
    for(int i=0; i<pc_in->points.size(); i++){
        if(pc_in->points[i].z>-lidar_height && pc_in->points[i].x<max_range && pc_in->points[i].x>-max_range && pc_in->points[i].y<max_range && pc_in->points[i].y>-max_range){
            cloud_f->points.push_back(pc_in->points[i]);
        }
    }
    pc_in=cloud_f;
}

void desc_matcher::removeGround(pcl::PointCloud<segmatch::PclPoint>::Ptr& pc_in){
    pcl::PointCloud<segmatch::PclPoint>::Ptr cloud_filtered=pc_in;
    pcl::PointCloud<segmatch::PclPoint>::Ptr cloud_f(new pcl::PointCloud<segmatch::PclPoint>);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    pcl::SACSegmentation<segmatch::PclPoint> seg;

    seg.setOptimizeCoefficients(true);

    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);

    float distanceThreshold=0.3;
    seg.setDistanceThreshold(distanceThreshold);//0.1
    pcl::ExtractIndices<segmatch::PclPoint> extract;
    int nr_points = (int)cloud_filtered->size();

    float ratio=0.3;

    while (cloud_filtered->size() > ratio * nr_points)//0.3
    {
        //std::cout<<"cloud_filtered: "<<cloud_filtered->size()<<std::endl;
        seg.setInputCloud(cloud_filtered);
        clock_t t = clock();
        seg.segment(*inliers, *coefficients);
        t = clock() - t;
        //std::cout<<"ransac time: "<<(float)t/CLOCKS_PER_SEC<<std::endl;
        if (inliers->indices.size() == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(true);
        
        extract.filter(*cloud_f);
        
        
        cloud_filtered.swap(cloud_f);
    }
    pc_in=cloud_filtered;
}

void desc_matcher::publishPose(Eigen::Matrix4f g_pose){
    segmatch::PointI p;
    p.x=g_pose(0, 3);
    p.y=g_pose(1, 3);
    p.z=g_pose(2, 3);
    p.intensity=1.0;
    match_posi.points.push_back(p);
    sensor_msgs::PointCloud2 match_result_as_message;
    convert_to_point_cloud_2_msg(match_posi, "map", &match_result_as_message);
    match_result_centroids_pub_.publish(match_result_as_message);  
}

void desc_matcher::publishGPS(Eigen::Vector3f gps){
    segmatch::PointI p;
    p.x=gps(0, 0);
    p.y=gps(1, 0);
    p.z=gps(2, 0);
    p.intensity=1.0;
    gps_posi.points.push_back(p);
    sensor_msgs::PointCloud2 match_result_as_message;
    convert_to_point_cloud_2_msg(gps_posi, "map", &match_result_as_message);
    gps_pub_.publish(match_result_as_message);  
}

void desc_matcher::gen_map(bool just_remove_ground){
    if (just_remove_ground){
        pcl::PointCloud<segmatch::PclPoint>::Ptr new_pt_cloud(new pcl::PointCloud<segmatch::PclPoint>);
        while(ros::ok()){
            pcl::PointCloud<segmatch::PclPoint>::Ptr new_points(new pcl::PointCloud<segmatch::PclPoint>);
            laser_slam::Pose r_pose;
            Eigen::Vector3f out_gps;
            double time;
            bool data_re = readAScan(new_points, r_pose, out_gps, time);
            if(!data_re){
                break;
            }
            pcl::RandomSample<pcl::PointXYZ> downSizeFilter;
            downSizeFilter.setSample(new_points->points.size()*0.01);
            downSizeFilter.setInputCloud(new_points);
            pcl::PointCloud<pcl::PointXYZ>::Ptr outputDS;
            outputDS.reset(new pcl::PointCloud<pcl::PointXYZ>());
            downSizeFilter.filter(*outputDS);
            *new_pt_cloud += *outputDS;
            //std::cout<<"new_pt_cloud: "<<new_pt_cloud->points.size()<<std::endl;
        }
        if (new_pt_cloud->points.size() != 0){
            pcl::io::savePCDFile(gound_remove_pcd, *new_pt_cloud, true);
        }
    }else{
        segmatch::MapCloud target_cloud;
        segmatch::loadCloud(gound_remove_pcd, &target_cloud);
        segmatch_->processAndSetAsTargetCloud(target_cloud);
        segmatch::database::exportSessionDataToDatabase(segmatch_->segmented_target_cloud_, desc_db_folder);
    } 
}


void desc_matcher::main_thread(){
    loadDB();
    int count=0;
    Eigen::Matrix4d T_o_lt1=Eigen::Matrix4d::Identity();
    T_w_lt1=Eigen::Matrix4d::Identity();
    cur_pose=Eigen::Matrix4f::Zero();
    his_traj.resize(1);
    bool first_frame=true;
    int fail_update_count=50;
    int succ_count=0;
    while(ros::ok()){
        clock_t t = clock();
        ros::spinOnce();
        pcl::PointCloud<segmatch::PclPoint>::Ptr new_cloud(new pcl::PointCloud<segmatch::PclPoint>);
        laser_slam::Pose r_pose;
        Eigen::Vector3f out_gps;
        double time_t;
        bool data_re;
        if(is_online){
            data_re = readAScan_ros(new_cloud, r_pose, out_gps, time_t);
        }else{
            data_re = readAScan(new_cloud, r_pose, out_gps, time_t);
        }
        
        Eigen::Matrix4d T_cw_clt = r_pose.T_w.getTransformationMatrix();
        
        //Eigen::Matrix4d T_clt_cw=T_cw_clt.inverse();
        //Eigen::Matrix4d T_clt_clt1=T_clt_cw*T_cw_clt1;
        
        if(!data_re){
            usleep(1000);
            continue;
        }
        
        count++;
        if(count%100==0){
            publishTargetRepresentation();
        }
        
        if(count%1==0){
//             pcl::RandomSample<pcl::PointXYZ> downSizeFilter;
//             downSizeFilter.setSample(new_cloud->points.size()*0.1);
//             downSizeFilter.setInputCloud(new_cloud);
//             pcl::PointCloud<pcl::PointXYZ>::Ptr outputDS;
//             outputDS.reset(new pcl::PointCloud<pcl::PointXYZ>());
//             downSizeFilter.filter(*outputDS);
            
            std::vector<pcl::PointCloud<segmatch::PclPoint>> new_points;
            new_points.push_back(*new_cloud);
            
            clock_t t1 = clock();
            local_maps_->updatePoseAndAddPoints(new_points, r_pose);
            t1 = clock() - t1;
            //std::cout<<"prepose time: "<<(float)t1/CLOCKS_PER_SEC<<std::endl;
            Eigen::Matrix4f T_w_cw=Eigen::Matrix4f::Identity();
            
            bool re = processLocalMap(*local_maps_, T_w_cw);
            
            if(re){
                bool inliner=false;
                if(first_frame){
                    last_right_T_w_cw=T_w_cw;
                    first_frame=false;
                    inliner=false;
                }else{
                    Eigen::Vector3f last_match_posi=last_right_T_w_cw.block(0,3,3,1);
                    Eigen::Vector3f cur_match_posi=T_w_cw.block(0,3,3,1);
                    Eigen::Vector3f posi_change= last_match_posi-cur_match_posi;
                    
                    last_right_T_w_cw=T_w_cw;
                    if (posi_change.norm()<0.5){
                        if(succ_count>10){
                            inliner=true;
                            if(cur_pose(3,3)==0){
                                cur_pose=T_w_cw;
                            }
                        }else{
                            succ_count++;
                            std::cout<<"wait for stable: "<<succ_count<<std::endl;
                            inliner=false;
                        }
                    }else{
                        if(succ_count>0){
                            succ_count--;
                        }
                        std::cout<<"too large jump, ignore this frame!!: "<<posi_change.norm()<<std::endl;
                        inliner=false;
                    }
                }
                
                if(inliner){
                    float a=0.1;
                    float a_z=0.0001;
                    Eigen::Matrix3f temp= T_w_cw.block(0,0,3,3);
                    Eigen::Quaternionf g_rot(temp);
                    temp= cur_pose.block(0,0,3,3);
                    Eigen::Quaternionf cur_rot(temp);
                    Eigen::Vector3f g_posi=T_w_cw.block(0,3,3,1);
                    Eigen::Vector3f cur_posi=cur_pose.block(0,3,3,1);
                    cur_rot = cur_rot.slerp(a, g_rot);
                    cur_posi.x() = cur_posi.x()+(g_posi.x()-cur_posi.x())*a;
                    cur_posi.y() = cur_posi.y()+(g_posi.y()-cur_posi.y())*a;
                    cur_posi.z() = cur_posi.z()+(g_posi.z()-cur_posi.z())*a_z;
                    cur_pose.block(0,0,3,3) = cur_rot.toRotationMatrix();
                    cur_pose.block(0,3,3,1)=cur_posi;

                    publishSourceRepresentation();
                    publishMatches();
                    fail_update_count=fail_update_count-10;
                }else{
                    fail_update_count++;
                }
            }else{
                fail_update_count++;
            }
        }
        if(fail_update_count<30){
            Eigen::Matrix4f re_pose = cur_pose*r_pose.T_w.getTransformationMatrix().cast<float>();
            publishPose(re_pose);
            publishGlobalPose(re_pose, time_t);
        }
        
        t = clock() - t;
        std::cout<<"total time: "<<(float)t/CLOCKS_PER_SEC<<std::endl;  
    }
}

void desc_matcher::publishGlobalPose(Eigen::Matrix4f pose, double time){
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time(time);
    msg.header.frame_id="map";
    Eigen::Matrix3f rot_m=pose.block(0,0,3,3);
    Eigen::Quaternionf qua(rot_m);
    msg.pose.orientation.x=qua.x();
    msg.pose.orientation.y=qua.y();
    msg.pose.orientation.z=qua.z();
    msg.pose.orientation.w=qua.w();
    msg.pose.position.x=pose(0,3);
    msg.pose.position.y=pose(1,3);
    msg.pose.position.z=pose(2,3);
    result_pub_.publish(msg);
}

void desc_matcher::publishMatches(){
    const segmatch::PairwiseMatches& matches_ =segmatch_->getFilteredMatches();
    segmatch::PointPairs point_pairs;
    for (size_t i = 0u; i < matches_.size(); ++i) {
        segmatch::PclPoint target_segment_centroid = matches_[i].getCentroids().second;
        segmatch::PclPoint source_segment_centroid = matches_[i].getCentroids().first;
        Eigen::Vector4f posi_t;
        posi_t[0] = source_segment_centroid.x;
        posi_t[1] = source_segment_centroid.y;
        posi_t[2] = source_segment_centroid.z;
        posi_t[3] = 1.0;
        posi_t=cur_pose*posi_t;
        source_segment_centroid.x=posi_t.x();
        source_segment_centroid.y=posi_t.y();
        source_segment_centroid.z=posi_t.z();

        source_segment_centroid.z += 25;
        point_pairs.push_back(segmatch::PointPair(source_segment_centroid, target_segment_centroid));
    }
    publishLineSet(point_pairs, "map", 0.4, Color(0.0, 1.0, 0.0), matches_pub_);
}

bool desc_matcher::processLocalMap(segmatch::SegMatch::LocalMapT& local_map, Eigen::Matrix4f& pose_out) {
    clock_t t = clock();
    segmatch_->processAndSetAsSourceCloud(local_map);
    t = clock() - t;
    //std::cout<<"process scan time: "<<(float)t/CLOCKS_PER_SEC<<std::endl;
    t = clock();
    segmatch::PairwiseMatches predicted_matches = segmatch_->findMatches();
    t = clock() - t;
    //std::cout<<"raw match time: "<<(float)t/CLOCKS_PER_SEC<<std::endl;
    //std::cout<<"predicted_matches: "<<predicted_matches.size()<<std::endl;
    t = clock();
    segmatch::PairwiseMatches recognized_matches = segmatch_->recognize(predicted_matches, pose_out);
    t = clock() - t;
    //std::cout<<"refined match time: "<<(float)t/CLOCKS_PER_SEC<<std::endl;
    std::cout<<"recognized_matches: "<<recognized_matches.size()<<std::endl;
    if(recognized_matches.size()>0){
        return true;
    }else{
        return false;
    }
    //std::cout<<pose_out<<std::endl;
}


void desc_matcher::publishTargetRepresentation() {
    segmatch::PointICloud target_representation;
    segmatch_->getTargetRepresentation(&target_representation, false);
    segmatch::translateCloud(segmatch::Translation(0.0, 0.0, 0), &target_representation);
    
    sensor_msgs::PointCloud2 target_representation_as_message;
    convert_to_point_cloud_2_msg(target_representation, "map", &target_representation_as_message);
    
    target_representation_pub_.publish(target_representation_as_message);
}

void desc_matcher::publishSourceRepresentation() {
    segmatch::PointICloud source_representation;
    segmatch_->getSourceRepresentation(&source_representation, 0 ,0);
    for(int i=0; i<source_representation.points.size(); i++){
        pcl::PointXYZI& source_segment_centroid = source_representation.points[i];
                Eigen::Vector4f posi_t;
        posi_t[0] = source_segment_centroid.x;
        posi_t[1] = source_segment_centroid.y;
        posi_t[2] = source_segment_centroid.z;
        posi_t[3] = 1.0;
        posi_t=cur_pose*posi_t;
        source_segment_centroid.x=posi_t.x();
        source_segment_centroid.y=posi_t.y();
        source_segment_centroid.z=posi_t.z();

        source_segment_centroid.z += 25;
    }
    sensor_msgs::PointCloud2 source_representation_as_message;
    convert_to_point_cloud_2_msg(source_representation, "map", &source_representation_as_message);
    
    source_representation_pub_.publish(source_representation_as_message);
}