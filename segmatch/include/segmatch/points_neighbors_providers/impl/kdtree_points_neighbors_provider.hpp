#ifndef SEGMATCH_IMPL_KDTREE_POINTS_NEIGHBORS_PROVIDER_HPP_
#define SEGMATCH_IMPL_KDTREE_POINTS_NEIGHBORS_PROVIDER_HPP_

#include <glog/logging.h>

#include "segmatch/points_neighbors_providers/kdtree_points_neighbors_provider.hpp"

namespace segmatch {

// Force the compiler to reuse instantiations provided in kdtree_points_neighbors_provider.cpp
extern template class KdTreePointsNeighborsProvider<MapPoint>;

//=================================================================================================
//    KdTreePointsNeighborsProvider public methods implementation
//=================================================================================================

template<typename PointT>
void KdTreePointsNeighborsProvider<PointT>::update(
    const typename pcl::PointCloud<PointT>::ConstPtr point_cloud,
    const std::vector<int64_t>& points_mapping) {
  // Build k-d tree
  point_cloud_ = point_cloud;
  kd_tree_.setInputCloud(point_cloud_);
//  std::vector<int> neighbor_tmp;
//  neighbors_ = std::vector<std::vector<int> >(point_cloud_->size(),neighbor_tmp);
//  std::vector<float> distances_tmp;
//  distances_ = std::vector<std::vector<float> >(point_cloud_->size(),distances_tmp);
}

template<typename PointT>
const PointNeighbors KdTreePointsNeighborsProvider<PointT>::getNeighborsOf(
    const size_t point_index, const float search_radius) {
  CHECK(point_cloud_ != nullptr);

  // Get the neighbors from the kd-tree.
  std::vector<int> neighbors_indices;
  std::vector<float> neighbors_distances;

  kd_tree_.radiusSearch((*point_cloud_)[point_index], search_radius, neighbors_indices,
                        neighbors_distances);

  neighbors_ = neighbors_indices;
  distances_ = neighbors_distances;

  return neighbors_indices;
}

template<typename PointT>
const PointNeighbors KdTreePointsNeighborsProvider<PointT>::getNeighborsOfPoint(const size_t point_index) {
  return neighbors_;
}

template<typename PointT>
const std::vector<float> KdTreePointsNeighborsProvider<PointT>::getDistancesOfPoint(const size_t point_index) {
  return distances_;
}

} // namespace segmatch

#endif // SEGMATCH_IMPL_KDTREE_POINTS_NEIGHBORS_PROVIDER_HPP_
