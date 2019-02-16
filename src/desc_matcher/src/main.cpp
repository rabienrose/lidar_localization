#include "desc_matcher.h"
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl_ros/io/pcd_io.h>


typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloud;

void process_pcd(std::string target_addr, std::string out_addr){
    PointCloud::Ptr in_cloud (new PointCloud);
    pcl::io::loadPCDFile<PointType> (target_addr, *in_cloud);
  
    PointCloud::Ptr out_cloud (new PointCloud);
    int count=0;
    for (size_t i = 0; i < in_cloud->size (); i++) {
        if (in_cloud->points[i].z>0.5 && in_cloud->points[i].x>-100 && in_cloud->points[i].x<100){
            out_cloud->points.push_back(in_cloud->points[i]);
        }
    }
    out_cloud->width=out_cloud->points.size();
    out_cloud->height=1;
    std::cout<<"out pc count: "<<out_cloud->points.size()<<std::endl;
    
    pcl::RandomSample<PointType> downSizeFilter(true);
    downSizeFilter.setInputCloud (out_cloud);
    downSizeFilter.setSample (1000000);
    PointCloud::Ptr outputDS;
    outputDS.reset(new PointCloud());
    downSizeFilter.filter(*outputDS);
    std::cout<<"ds pc count: "<<outputDS->points.size()<<std::endl;
    
    pcl::io::savePCDFile(out_addr, *outputDS, true);
}

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "desc_matcher");
    ros::NodeHandle node_handle;
    std::string operation_type;
    node_handle.getParam("/desc_matcher/operation_type", operation_type);
    desc_matcher matcher;
    if(operation_type=="remove_ground" || operation_type=="extract_desc"){
        std::cout<<"map generation"<<std::endl;
        if(operation_type=="remove_ground"){
            matcher.gen_map(true);
        }else{
            matcher.gen_map(false);
        }
        
    }else{
        std::cout<<"localization"<<std::endl;
        matcher.main_thread();
    }
    return 0;
}

