#include "chamo_util.h"


void publishLineSet(const segmatch::PointPairs& point_pairs,
                           const std::string& frame, const float line_scale,
                           const Color& color, const ros::Publisher& publisher) {
  visualization_msgs::Marker line_list;
  line_list.header.frame_id = frame;
  line_list.header.stamp = ros::Time();
  line_list.ns = "matcher_trainer";
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.color.r = color.r;
  line_list.color.g = color.g;
  line_list.color.b = color.b;
  line_list.color.a = 1.0;
  line_list.scale.x = line_scale;
  for (size_t i = 0u; i < point_pairs.size(); ++i) {
    geometry_msgs::Point p;
    p.x = point_pairs[i].first.x;
    p.y = point_pairs[i].first.y;
    p.z = point_pairs[i].first.z;
    line_list.points.push_back(p);
    p.x = point_pairs[i].second.x;
    p.y = point_pairs[i].second.y;
    p.z = point_pairs[i].second.z;
    line_list.points.push_back(p);
  }
  publisher.publish(line_list);
}

void convert_to_point_cloud_2_msg(const segmatch::PointICloud& cloud,
                                         const std::string& frame,
                                         sensor_msgs::PointCloud2* converted) {
  CHECK_NOTNULL(converted);
  // Convert to PCLPointCloud2.
  pcl::PCLPointCloud2 pcl_point_cloud_2;
  pcl::toPCLPointCloud2(cloud, pcl_point_cloud_2);
  // Convert to sensor_msgs::PointCloud2.
  pcl_conversions::fromPCL(pcl_point_cloud_2, *converted);
  // Apply frame to msg.
  converted->header.frame_id = frame;
}

segmatch::SegMatchParams getSegMatchParams(const ros::NodeHandle& nh, const std::string& prefix) {
    segmatch::SegMatchParams params;

    std::string ns = prefix + "/SegMatch";

    nh.getParam(ns + "/segmentation_radius_m",
                params.segmentation_radius_m);
    nh.getParam(ns + "/segmentation_height_above_m",
                params.segmentation_height_above_m);
    nh.getParam(ns + "/segmentation_height_below_m",
                params.segmentation_height_below_m);

    nh.getParam(ns + "/filter_boundary_segments",
                params.filter_boundary_segments);
    nh.getParam(ns + "/boundary_radius_m",
                params.boundary_radius_m);
    nh.getParam(ns + "/filter_duplicate_segments",
                params.filter_duplicate_segments);
    nh.getParam(ns + "/centroid_distance_threshold_m",
                params.centroid_distance_threshold_m);
    int min_time_between_segment_for_matches_s;
    nh.getParam(ns + "/min_time_between_segment_for_matches_s",
                min_time_between_segment_for_matches_s);
    params.min_time_between_segment_for_matches_ns =
        laser_slam::Time(min_time_between_segment_for_matches_s) * 1000000000u;
    nh.getParam(ns + "/check_pose_lies_below_segments",
                params.check_pose_lies_below_segments);
    nh.getParam(ns + "/radius_for_normal_estimation_m",
                params.radius_for_normal_estimation_m);
    nh.getParam(ns + "/normal_estimator_type",
                params.normal_estimator_type);

    // Local map parameters.
    nh.getParam(ns + "/LocalMap/voxel_size_m",
                params.local_map_params.voxel_size_m);
    nh.getParam(ns + "/LocalMap/min_points_per_voxel",
                params.local_map_params.min_points_per_voxel);
    nh.getParam(ns + "/LocalMap/radius_m",
                params.local_map_params.radius_m);
    nh.getParam(ns + "/LocalMap/min_vertical_distance_m",
                params.local_map_params.min_vertical_distance_m);
    nh.getParam(ns + "/LocalMap/max_vertical_distance_m",
                params.local_map_params.max_vertical_distance_m);
    nh.getParam(ns + "/LocalMap/neighbors_provider_type",
                params.local_map_params.neighbors_provider_type);

    // Descriptors parameters.
    nh.getParam(ns + "/Descriptors/descriptor_types",
                params.descriptors_params.descriptor_types);
    nh.getParam(ns + "/Descriptors/fast_point_feature_histograms_search_radius",
                params.descriptors_params.fast_point_feature_histograms_search_radius);
    nh.getParam(ns + "/Descriptors/fast_point_feature_histograms_normals_search_radius",
                params.descriptors_params.
                fast_point_feature_histograms_normals_search_radius);
    nh.getParam(ns + "/Descriptors/point_feature_histograms_search_radius",
                params.descriptors_params.point_feature_histograms_search_radius);
    nh.getParam(ns + "/Descriptors/point_feature_histograms_normals_search_radius",
                params.descriptors_params.point_feature_histograms_normals_search_radius);
    nh.getParam(ns + "/Descriptors/cnn_model_path",
                params.descriptors_params.cnn_model_path);
    nh.getParam(ns + "/Descriptors/semantics_nn_path",
                params.descriptors_params.semantics_nn_path);

    // Segmenter parameters.
    nh.getParam(ns + "/Segmenters/segmenter_type",
                params.segmenter_params.segmenter_type);
    nh.getParam(ns + "/Segmenters/min_cluster_size",
                params.segmenter_params.min_cluster_size);
    nh.getParam(ns + "/Segmenters/max_cluster_size",
                params.segmenter_params.max_cluster_size);
    nh.getParam(ns + "/Segmenters/radius_for_growing",
                params.segmenter_params.radius_for_growing);
    nh.getParam(ns + "/Segmenters/sc_smoothness_threshold_deg",
                params.segmenter_params.sc_smoothness_threshold_deg);
    nh.getParam(ns + "/Segmenters/sc_curvature_threshold",
                params.segmenter_params.sc_curvature_threshold);

    // Classifier parameters.
    nh.getParam(ns + "/Classifier/classifier_filename",
                params.classifier_params.classifier_filename);
    nh.getParam(ns + "/Classifier/threshold_to_accept_match",
                params.classifier_params.threshold_to_accept_match);

    nh.getParam(ns + "/Classifier/rf_max_depth",
                params.classifier_params.rf_max_depth);
    nh.getParam(ns + "/Classifier/rf_min_sample_ratio",
                params.classifier_params.rf_min_sample_ratio);
    nh.getParam(ns + "/Classifier/rf_regression_accuracy",
                params.classifier_params.rf_regression_accuracy);
    nh.getParam(ns + "/Classifier/rf_use_surrogates",
                params.classifier_params.rf_use_surrogates);
    nh.getParam(ns + "/Classifier/rf_max_categories",
                params.classifier_params.rf_max_categories);
    nh.getParam(ns + "/Classifier/rf_priors",
                params.classifier_params.rf_priors);
    nh.getParam(ns + "/Classifier/rf_calc_var_importance",
                params.classifier_params.rf_calc_var_importance);
    nh.getParam(ns + "/Classifier/rf_n_active_vars",
                params.classifier_params.rf_n_active_vars);
    nh.getParam(ns + "/Classifier/rf_max_num_of_trees",
                params.classifier_params.rf_max_num_of_trees);
    nh.getParam(ns + "/Classifier/rf_accuracy",
                params.classifier_params.rf_accuracy);

    nh.getParam(ns + "/Classifier/do_not_use_cars",
                params.classifier_params.do_not_use_cars);

    // Convenience copy to find the correct feature distance according to
    // descriptors types.
    nh.getParam(ns + "/Descriptors/descriptor_types",
                params.classifier_params.descriptor_types);

    nh.getParam(ns + "/Classifier/n_nearest_neighbours",
                params.classifier_params.n_nearest_neighbours);
    nh.getParam(ns + "/Classifier/enable_two_stage_retrieval",
                params.classifier_params.enable_two_stage_retrieval);
    nh.getParam(ns + "/Classifier/knn_feature_dim",
                params.classifier_params.knn_feature_dim);
    nh.getParam(ns + "/Classifier/apply_hard_threshold_on_feature_distance",
                params.classifier_params.apply_hard_threshold_on_feature_distance);
    nh.getParam(ns + "/Classifier/feature_distance_threshold",
                params.classifier_params.feature_distance_threshold);

    nh.getParam(ns + "/Classifier/normalize_eigen_for_knn",
                params.classifier_params.normalize_eigen_for_knn);
    nh.getParam(ns + "/Classifier/normalize_eigen_for_hard_threshold",
                params.classifier_params.normalize_eigen_for_hard_threshold);
    nh.getParam(ns + "/Classifier/max_eigen_features_values",
                params.classifier_params.max_eigen_features_values);


    // Geometric Consistency Parameters.
    nh.getParam(ns + "/GeometricConsistency/recognizer_type",
                params.geometric_consistency_params.recognizer_type);
    nh.getParam(ns + "/GeometricConsistency/resolution",
                params.geometric_consistency_params.resolution);
    nh.getParam(ns + "/GeometricConsistency/min_cluster_size",
                params.geometric_consistency_params.min_cluster_size);
    nh.getParam(ns + "/GeometricConsistency/max_consistency_distance_for_caching",
                params.geometric_consistency_params.max_consistency_distance_for_caching);

    return params;
}