#ifndef SEGMATCH_IMPL_INCREMENTAL_SEGMENTER_HPP_
#define SEGMATCH_IMPL_INCREMENTAL_SEGMENTER_HPP_

#include "segmatch/segmenters/incremental_segmenter.hpp"

#include <algorithm>

#include <laser_slam/benchmarker.hpp>

#include "segmatch/segmented_cloud.hpp"

namespace segmatch {

// Force the compiler to reuse instantiations provided in incremental_segmenter.cpp
extern template class IncrementalSegmenter<MapPoint, EuclideanDistance>;
extern template class IncrementalSegmenter<MapPoint, SmoothnessConstraints>;
//extern template class IncrementalSegmenter<MapPoint, Rgb>;

//=================================================================================================
//    IncrementalSegmenter public methods implementation
//=================================================================================================

template<typename ClusteredPointT, typename PolicyName>
void IncrementalSegmenter<ClusteredPointT, PolicyName>::segment(
    const PointNormals& normals, const std::vector<bool>& is_point_modified, ClusteredCloud& cloud,
    PointsNeighborsProvider<ClusteredPointT>& points_neighbors_provider,
    SegmentedCloud& segmented_cloud, std::vector<Id>& cluster_ids_to_segment_ids,
    std::vector<std::pair<Id, Id>>& renamed_segments) {
  BENCHMARK_BLOCK("SM.Worker.Segmenter");
  renamed_segments.clear();

  number_of_segments_ = 0;

  std::cout << "cluster_ids_to_segment_ids.size(): " << cluster_ids_to_segment_ids.size() << std::endl;
  // Build partial cluster sets for the old clusters.
  PartialClusters partial_clusters(cluster_ids_to_segment_ids.size());
  for (size_t i = 0u; i < partial_clusters.size(); i++) {
    partial_clusters[i].partial_clusters_set->partial_clusters_indices.insert(i);
    partial_clusters[i].partial_clusters_set->segment_id = cluster_ids_to_segment_ids[i];
  }

  std::cout << "Partielle Cluster alt: " << (int)partial_clusters.size() << std::endl;


  // Find old clusters and new partial clusters.
  growRegions(normals, is_point_modified, cluster_ids_to_segment_ids, cloud,
              points_neighbors_provider, partial_clusters, renamed_segments);


  std::cout << "Partielle Cluster: " << (int)partial_clusters.size() << std::endl;
  number_of_segments_ = static_cast<int>(partial_clusters.size());
  std::cout << "number_of_segments_: " << number_of_segments_ << std::endl;

  if (use_color_information_) {

    std::cout << "Nutzt Farbe!!" << std::endl;
    // Merge RGB Clusters
    findSegmentNeighbours (partial_clusters, cloud, points_neighbors_provider);
    applyRegionMergingAlgorithm (partial_clusters, cloud, renamed_segments);
  }

  // Compute and write cluster indices.
  const size_t num_clusters = assignClusterIndices(partial_clusters);
  writeClusterIndicesToCloud(partial_clusters, cloud);

  std::cout << "Cluster: " << num_clusters << std::endl;

  // Extract the valid segment and add them to the segmented cloud.
  addSegmentsToSegmentedCloud(cloud, partial_clusters, num_clusters, cluster_ids_to_segment_ids,
                              segmented_cloud);
}

//=================================================================================================
//    IncrementalSegmenter private methods implementation
//=================================================================================================

template<typename ClusteredPointT, typename PolicyName>
inline std::pair<Id, Id> IncrementalSegmenter<ClusteredPointT, PolicyName>::mergeSegmentIds(
    const Id id_1, const Id id_2) const {
  if (id_1 == kInvId || id_2 == kInvId) {
    // Invalidated segments stay invalid.
    return  { kInvId, kInvId };
  } else if (id_1 == kNoId) {
    // In case one cluster doesn't belong to a segment keep the only segment ID (if any). No
    // renaming necessary.
    return { kNoId, id_2 };
  } else if (id_2 == kNoId) {
    // In case one cluster doesn't belong to a segment keep the only segment ID (if any). No
    // renaming necessary.
    return { kNoId, id_1 };
  } else {
    // Otherwise take the minimum segment ID (the one that has been around for longer). The segment
    // with maximum ID is renamed to the minimum ID.
    return { std::max(id_1, id_2), std::min(id_1, id_2) };
  }
}

template<typename ClusteredPointT, typename PolicyName>
inline void IncrementalSegmenter<ClusteredPointT, PolicyName>::linkPartialClusters(
    const size_t partial_cluster_1_index, const size_t partial_cluster_2_index,
    PartialClusters& partial_clusters, std::vector<std::pair<Id, Id>>& renamed_segments) const {
  // Get pointers to the partial cluster sets
  PartialClustersSetPtr set_1 = partial_clusters[partial_cluster_1_index].partial_clusters_set;
  PartialClustersSetPtr set_2 = partial_clusters[partial_cluster_2_index].partial_clusters_set;

  // Both partial clusters belong to the same set. Nothing to do.
  if (set_1 == set_2) return;

  // Swap the partial cluster indices if it makes the merge operation faster.
  if (set_1->partial_clusters_indices.size() < set_2->partial_clusters_indices.size()) {
    std::swap(set_1, set_2);
  }

  // Move the linked indices from set_2 to set_1 and determine the segment ID.
  set_1->partial_clusters_indices.insert(set_2->partial_clusters_indices.begin(),
                                         set_2->partial_clusters_indices.end());
  Id old_segment_id;
  std::tie(old_segment_id, set_1->segment_id) = mergeSegmentIds(set_1->segment_id,
                                                                set_2->segment_id);

  // Detect if a segment renaming happened
  if (old_segment_id != kNoId && old_segment_id != kInvId)
    renamed_segments.push_back({ old_segment_id, set_1->segment_id });

  // Update all partial clusters contained in set_2 so that they point to set_1.
  for (const auto partial_cluster_index : set_2->partial_clusters_indices) {
    partial_clusters[partial_cluster_index].partial_clusters_set = set_1;
  }
}

template<typename ClusteredPointT, typename PolicyName>
inline void IncrementalSegmenter<ClusteredPointT, PolicyName>::growRegionFromSeed(
    const PointNormals& normals, const ClusteredCloud& cloud,
    PointsNeighborsProvider<ClusteredPointT>& points_neighbors_provider, const size_t seed_index,
    std::vector<bool>& processed, PartialClusters& partial_clusters,
    std::vector<std::pair<Id, Id>>& renamed_segments) const {
  // Create a new partial cluster.
  partial_clusters.emplace_back();
  PartialCluster& partial_cluster = partial_clusters.back();
  size_t partial_cluster_id = partial_clusters.size() - 1u;
  partial_cluster.partial_clusters_set = std::make_shared<PartialClustersSet>();
  partial_cluster.partial_clusters_set->partial_clusters_indices.insert(partial_cluster_id);

  // Initialize the seeds queue.
  std::vector<size_t>& region_indices = partial_cluster.point_indices;
  std::vector<size_t> seed_queue;
  size_t current_seed_index = 0u;
  seed_queue.push_back(seed_index);
  region_indices.push_back(seed_index);

  // Search for neighbors until there are no more seeds.
  while (current_seed_index < seed_queue.size()) {
    // Search for points around the seed.
    std::vector<int> neighbors_indices = points_neighbors_provider.getNeighborsOf(
        seed_queue[current_seed_index], search_radius_);

    // Decide on which points should we continue the search and if we have to link partial
    // clusters.
    for (const auto neighbor_index : neighbors_indices) {
      if (neighbor_index != -1 && Policy::canGrowToPoint(
          policy_params_, normals, seed_queue[current_seed_index], neighbor_index, cloud[current_seed_index], cloud[neighbor_index])) {
        if (isPointAssignedToCluster(cloud[neighbor_index])) {
          // If the search reaches an existing cluster we link to its partial clusters set.
          if (partial_cluster_id != getClusterId(cloud[neighbor_index])) {
            linkPartialClusters(partial_cluster_id, getClusterId(cloud[neighbor_index]),
                                partial_clusters, renamed_segments);
          }
        } else if (!processed[neighbor_index]) {
          // Determine if the point can be used as seed for the region.
          if (Policy::canPointBeSeed(policy_params_, normals, neighbor_index)) {
            seed_queue.push_back(neighbor_index);
          }
          // Assign the point to the current partial cluster.
          region_indices.push_back(neighbor_index);
          processed[neighbor_index] = true;
        }
      }
    }
    ++current_seed_index;
  }
}

template<typename ClusteredPointT, typename PolicyName>
inline void IncrementalSegmenter<ClusteredPointT, PolicyName>::growRegions(
    const PointNormals& normals, const std::vector<bool>& is_point_modified,
    const std::vector<Id>& cluster_ids_to_segment_ids, ClusteredCloud& cloud,
    PointsNeighborsProvider<ClusteredPointT>& points_neighbors_provider,
    PartialClusters& partial_clusters, std::vector<std::pair<Id, Id>>& renamed_segments) const {
  BENCHMARK_BLOCK("SM.Worker.Segmenter.GrowRegions");

  std::vector<bool> processed(cloud.size(), false);
  std::vector<size_t> new_points_indices;
  new_points_indices.reserve(cloud.size());

  for (size_t i = 0u; i < cloud.size(); ++i) {
    if (isPointAssignedToCluster(cloud[i])) {
      // No need to cluster points that are already assigned.
      partial_clusters[getClusterId(cloud[i])].point_indices.push_back(i);
    } else if (Policy::canPointBeSeed(policy_params_, normals, i)) {
      new_points_indices.emplace_back(i);
    }
  }

  // Prepare the seed indices.
  Policy::prepareSeedIndices(normals, new_points_indices.begin(), new_points_indices.end());

  // Process the new points.
  // TODO: The current implementation ignores any change in the normal/curvature of a point,
  // ignoring cases in which changes in the properties of a point would lead to different
  // clustering decisions. It would be nice to add segmentation policies covering this case.
  for (const auto i : new_points_indices) {
    if (!processed[i]) {
      // Mark the point as processed and grow the cluster starting from it.
      processed[i] = true;
      growRegionFromSeed(normals, cloud, points_neighbors_provider, i, processed, partial_clusters,
                         renamed_segments);
    }
  }
}

template<typename ClusteredPointT, typename PolicyName>
inline size_t IncrementalSegmenter<ClusteredPointT, PolicyName>::assignClusterIndices(
    const PartialClusters& partial_clusters) const {
  BENCHMARK_BLOCK("SM.Worker.Segmenter.AssignClusterIndices");

  // Assign cluster IDs.
  ClusterId next_cluster_id = 1u;
  for (const auto& partial_cluster : partial_clusters) {
    const PartialClustersSetPtr& partial_clusters_set = partial_cluster.partial_clusters_set;
    if (!partial_cluster.point_indices.empty() &&
        partial_clusters_set->cluster_id == kUnassignedClusterId) {
      // Assign a cluster index only if the set didn't get one yet and the partial cluster
      // contains at least one point.
      partial_clusters_set->cluster_id = next_cluster_id;
      ++next_cluster_id;
    }
  }

  return static_cast<size_t>(next_cluster_id);
}

template<typename ClusteredPointT, typename PolicyName>
inline void IncrementalSegmenter<ClusteredPointT, PolicyName>::writeClusterIndicesToCloud(
    const PartialClusters& partial_clusters, ClusteredCloud& cloud) const {
  BENCHMARK_BLOCK("SM.Worker.Segmenter.WriteClusterIndices");

  // Write cluster IDs in the point cloud.
  for (const auto& partial_cluster : partial_clusters) {
    for (const auto point_id : partial_cluster.point_indices) {
      setClusterId(cloud[point_id], partial_cluster.partial_clusters_set->cluster_id);
    }
  }
}

template<typename ClusteredPointT, typename PolicyName>
inline void IncrementalSegmenter<ClusteredPointT, PolicyName>::addSegmentsToSegmentedCloud(
    const ClusteredCloud& cloud, const PartialClusters& partial_clusters,
    const size_t num_clusters, std::vector<Id>& cluster_ids_to_segment_ids,
    SegmentedCloud& segmented_cloud) const {
  BENCHMARK_BLOCK("SM.Worker.Segmenter.AddSegments");
  BENCHMARK_RECORD_VALUE("SM.NumClusters", num_clusters);

  // Initially all clusters don't have a segment ID.
  cluster_ids_to_segment_ids = std::vector<Id>(num_clusters, kUnassignedId);
  if (!cluster_ids_to_segment_ids.empty()) cluster_ids_to_segment_ids[0] = kNoId;

  std::vector<Id> segment_ids_to_keep;

  for (size_t i = 0u; i < partial_clusters.size(); i++) {
    const PartialClustersSetPtr& partial_clusters_set = partial_clusters[i].partial_clusters_set;
    const ClusterId cluster_id = partial_clusters_set->cluster_id;

    // Only process clusters once.
    if (cluster_ids_to_segment_ids[cluster_id] != kUnassignedId) continue;

    const Id old_segment_id = partial_clusters_set->segment_id;
    if (old_segment_id == kInvId) {
      // Skip invalidated segments
      cluster_ids_to_segment_ids[cluster_id] = kInvId;
    } else {
      const size_t points_in_cluster = getClusterSize(partial_clusters, i);
      if (points_in_cluster > max_segment_size_) {
        // Invalidate segments with too many points.
        cluster_ids_to_segment_ids[cluster_id] = kInvId;
      } else if (old_segment_id != kNoId || points_in_cluster >= min_segment_size_) {
        // Create the segment, reusing the previous segment ID if present.
        pcl::PointIndices point_indices;
        point_indices.indices = getClusterIndices(partial_clusters, i);
        cluster_ids_to_segment_ids[cluster_id] = segmented_cloud.addSegment(
            point_indices, cloud, old_segment_id);

        segment_ids_to_keep.push_back(cluster_ids_to_segment_ids[cluster_id]);
        BENCHMARK_RECORD_VALUE("SM.SegmentSize", point_indices.indices.size());
      } else {
        // The cluster doesn't have enough points, don't assign a segment yet.
        cluster_ids_to_segment_ids[cluster_id] = kNoId;
      }
    }
  }

  // Delete the segments that we did not keep.
  segmented_cloud.deleteSegmentsExcept(segment_ids_to_keep);
}

template<typename ClusteredPointT, typename PolicyName>
inline size_t IncrementalSegmenter<ClusteredPointT, PolicyName>::findSegmentNeighbours(PartialClusters& partial_clusters, ClusteredCloud& cloud, PointsNeighborsProvider<ClusteredPointT>& points_neighbors_provider) {
  std::vector<int> neighbours;
  std::vector<float> distances;
  segment_neighbours_ =  std::vector<std::vector<int> >(number_of_segments_, neighbours);
  segment_distances_ =  std::vector<std::vector<float> >(number_of_segments_, distances);

  for (int i_seg = 0; i_seg < number_of_segments_; i_seg++)   // durchlaufen aller Cluster
  {
    std::vector<int> nghbrs;
    std::vector<float> dist;
    findRegionsKNN (i_seg, region_neighbour_number_, partial_clusters, cloud, points_neighbors_provider, nghbrs, dist);
    segment_neighbours_[i_seg].swap (nghbrs);
    segment_distances_[i_seg].swap (dist);
  }
}

template<typename ClusteredPointT, typename PolicyName>
inline size_t IncrementalSegmenter<ClusteredPointT, PolicyName>::findRegionsKNN (int index, int nghbr_number, PartialClusters& partial_clusters, ClusteredCloud& cloud, PointsNeighborsProvider<ClusteredPointT>& points_neighbors_provider, std::vector<int>& nghbrs, std::vector<float>& dist)
{
  std::vector<float> distances;
  float max_dist = std::numeric_limits<float>::max ();
  distances.resize (number_of_segments_, max_dist);

  //int number_of_points = getClusterSize(partial_clusters, index);
  //std::vector<size_t>::iterator it;

  //loop throug every point in this segment and check neighbours
  //for (int i_point = 0; i_point < number_of_points; i_point++)
  //for(it = partial_clusters[index].point_indices.begin(), it < partial_clusters[index].point_indices.end(); it++)
  int number_of_points = partial_clusters[index].point_indices.size();
  for(int i_point = 0; i_point < number_of_points; i_point++)      // durchlauf aller Punkte eines clusters
  {
    int point_index = partial_clusters[index].point_indices[i_point];
    std::vector<int> point_neighbours = points_neighbors_provider.getNeighborsOfPoint(point_index);
    std::vector<float> point_distances = points_neighbors_provider.getDistancesOfPoint(point_index);

    int number_of_neighbours = static_cast<int> (point_neighbours.size ());
    //loop throug every neighbour of the current point, find out to which segment it belongs
    //and if it belongs to neighbouring segment and is close enough then remember segment and its distance
    for (int i_nghbr = 0; i_nghbr < number_of_neighbours; i_nghbr++)   // durchlauf aller nachbarn eines punktes
    {
      // find segment
      int cluster_index = -1;
      //cluster_index = point_labels_[ point_neighbours_[point_index][i_nghbr] ];
//      if (params_.segmenter_type == "IncrementalEuclideanDistance")
//        cluster_index = cloud[ point_neighbours[i_nghbr] ].ed_cluster_id;
//      else if (params_.segmenter_type == "IncrementalSmoothnessConstraints")
//        cluster_index = cloud[ point_neighbours[i_nghbr] ].sc_cluster_id;
      cluster_index = getClusterId(cloud[point_neighbours[i_nghbr]]);

      if ( cluster_index != index ) //index
      {
        // try to push it to the queue
        if (distances[cluster_index] > point_distances[i_nghbr])
          distances[cluster_index] = point_distances[i_nghbr];
      }
    }
  }// next point

  std::priority_queue<std::pair<float, int> > cluster_neighbours;    // sortiert Nachbarn eines Clusters so, dass das weiteste entfernte ganz vorne steht und dann absteigend
  for (int i_seg = 0; i_seg < number_of_segments_; i_seg++)
  {
    if (distances[i_seg] < max_dist)
    {
      cluster_neighbours.push (std::make_pair (distances[i_seg], i_seg) );
      if (int (cluster_neighbours.size ()) > nghbr_number)
        cluster_neighbours.pop ();
    }
  }

  int size = std::min<int> (static_cast<int> (cluster_neighbours.size ()), nghbr_number);
  nghbrs.resize (size, 0);
  dist.resize (size, 0);
  int counter = 0;
  while ( !cluster_neighbours.empty () && counter < nghbr_number )
  {
    dist[counter] = cluster_neighbours.top ().first;
    nghbrs[counter] = cluster_neighbours.top ().second;
    cluster_neighbours.pop ();
    counter++;
  }
}

template<typename ClusteredPointT, typename PolicyName>
inline size_t IncrementalSegmenter<ClusteredPointT, PolicyName>::applyRegionMergingAlgorithm ( PartialClusters& partial_clusters, ClusteredCloud& cloud,std::vector<std::pair<Id, Id>>& renamed_segments)
{
  int number_of_points = static_cast<int> (cloud.size());

  // calculate color of each segment
  std::vector< std::vector<unsigned int> > segment_color;
  std::vector<unsigned int> color;
  color.resize (3, 0);
  segment_color.resize (number_of_segments_, color);

  std::vector<int> num_pts_in_segment;
  //num_pts_in_segment.resize (number_of_segments_, 0);

  for (int i_point = 0; i_point < number_of_points; i_point++)  // durchlaufen jeden Punkt in geclusterter Wolke
  {
    //int point_index = cloud[i_point];
    //int cluster_index = point_labels_[point_index];
    int cluster_index;
    cluster_index = getClusterId(cloud[i_point]);
//    if (params_.segmenter_type == "IncrementalEuclideanDistance")
//      cluster_index = cloud[ i_point ].ed_cluster_id;
//    else if (params_.segmenter_type == "IncrementalSmoothnessConstraints")
//      cluster_index = cloud[ i_point ].sc_cluster_id;

    segment_color[cluster_index][0] += cloud[i_point].r;
    segment_color[cluster_index][1] += cloud[i_point].g;
    segment_color[cluster_index][2] += cloud[i_point].b;
    //num_pts_in_segment[cluster_index] += 1;
  }
  for (int i_seg = 0; i_seg < number_of_segments_; i_seg++)   // durchlauf aller cluster
  {
    int num_pts_in_segment = int(partial_clusters[i_seg].point_indices.size());
    segment_color[i_seg][0] = static_cast<unsigned int> (static_cast<float> (segment_color[i_seg][0]) / static_cast<float> (num_pts_in_segment));//[i_seg]));
    segment_color[i_seg][1] = static_cast<unsigned int> (static_cast<float> (segment_color[i_seg][1]) / static_cast<float> (num_pts_in_segment));//[i_seg]));
    segment_color[i_seg][2] = static_cast<unsigned int> (static_cast<float> (segment_color[i_seg][2]) / static_cast<float> (num_pts_in_segment));//[i_seg]));
  }

  // now it is time to find out if there are segments with a similar color
  // and merge them together
  std::vector<unsigned int> num_pts_in_homogeneous_region;
  std::vector<int> num_seg_in_homogeneous_region;

  std::vector<int> segment_labels;
  segment_labels.resize (number_of_segments_, -1);

  float dist_thresh = search_radius_; //distance_threshold_;
  int homogeneous_region_number = 0;
  int curr_homogeneous_region = 0;
  for (int i_seg = 0; i_seg < number_of_segments_; i_seg++)     // jedes Segment durchgehen
  {
    curr_homogeneous_region = 0;
    if (segment_labels[i_seg] == -1)     // Nummer der aktuellen homogenen Region setzen
    {
      segment_labels[i_seg] = homogeneous_region_number;
      curr_homogeneous_region = homogeneous_region_number;
      num_pts_in_homogeneous_region.push_back (int(partial_clusters[i_seg].point_indices.size()));//(num_pts_in_segment[i_seg]);
      num_seg_in_homogeneous_region.push_back (1);
      homogeneous_region_number++;
    }
    else
      curr_homogeneous_region = segment_labels[i_seg];

    unsigned int i_nghbr = 0;
    while ( i_nghbr < region_neighbour_number_ && i_nghbr < segment_neighbours_[i_seg].size () )   // alle Segmentnachbarn durchgehen
    {
      int index = segment_neighbours_[i_seg][i_nghbr];
      if (segment_distances_[i_seg][i_nghbr] > dist_thresh)
      {
        i_nghbr++;
        continue;
      }
      if ( segment_labels[index] == -1 )
      {
        float difference;
        uint16_t difference_tmp = 0u;
        difference_tmp += (segment_color[i_seg][0] - segment_color[index][0]) * (segment_color[i_seg][0] - segment_color[index][0]);
        difference_tmp += (segment_color[i_seg][1] - segment_color[index][1]) * (segment_color[i_seg][1] - segment_color[index][1]);
        difference_tmp += (segment_color[i_seg][2] - segment_color[index][2]) * (segment_color[i_seg][2] - segment_color[index][2]);

        difference = static_cast<float>(sqrt(difference_tmp));
        //float color_r2r_threshold = 25.0; // -------------------------------------------------------------------------------------------------------------------------------------------------------------
        if (difference < color_r2r_threshold_)
        {
          segment_labels[index] = curr_homogeneous_region;
          num_pts_in_homogeneous_region[curr_homogeneous_region] += num_pts_in_segment[index];
          num_seg_in_homogeneous_region[curr_homogeneous_region] += 1;
          linkPartialClusters(i_seg, index, partial_clusters, renamed_segments);
        }
      }
      i_nghbr++;
    }// next neighbour
  }// next segment
  std::cout << "Grenzwert: " << color_r2r_threshold_ << std::endl;
  segment_color.clear ();
  color.clear ();

//  for (int i = 0; i < segment_labels.size(); i++) {
//   // std::cout << "Label: " << segment_labels[i] << std::endl;

//    for (int j = (i+1); j < segment_labels.size(); j++ ) {
//      partial_clusters[i].partial_clusters_set->partial_clusters_indices.insert(j);
////      if (segment_labels[i] == segment_labels[j] ) {
////        partial_clusters[i].partial_clusters_set->partial_clusters_indices.insert(j);

////      }
//    }
//  }
//std::cout << "Partielle Cluster neu: " << (int)partial_clusters.size() << std::endl;
  segment_neighbours_.clear();
  segment_distances_.clear();


//  std::vector< std::vector<int> > final_segments;
//  std::vector<int> region;
//  final_segments.resize (homogeneous_region_number, region);
//  //final_segments = std::vector<std::vector<int> >(number_of_segments_, neighbours);
//  for (int i_reg = 0; i_reg < homogeneous_region_number; i_reg++)
//  {
//    final_segments[i_reg].resize (num_seg_in_homogeneous_region[i_reg], 0);
//  }

//  std::vector<int> counter;
//  counter.resize (homogeneous_region_number, 0);
//  for (int i_seg = 0; i_seg < number_of_segments_; i_seg++)   // alle Segmente durchlaufen
//  {
//    int index = segment_labels[i_seg];
//    final_segments[ index ][ counter[index] ] = i_seg;
//    counter[index] += 1;
//  }

//  std::vector< std::vector< std::pair<float, int> > > region_neighbours;
//  findRegionNeighbours (region_neighbours, final_segments);

//  int final_segment_number = homogeneous_region_number;
//  for (int i_reg = 0; i_reg < homogeneous_region_number; i_reg++)
//  {
//    if (static_cast<int> (num_pts_in_homogeneous_region[i_reg]) < min_segment_size_)
//    {
//      if ( region_neighbours[i_reg].empty () )
//        continue;
//      int nearest_neighbour = region_neighbours[i_reg][0].second;
//      if ( region_neighbours[i_reg][0].first == std::numeric_limits<float>::max () )
//        continue;
//      int reg_index = segment_labels[nearest_neighbour];
//      int num_seg_in_reg = num_seg_in_homogeneous_region[i_reg];
//      for (int i_seg = 0; i_seg < num_seg_in_reg; i_seg++)
//      {
//        int segment_index = final_segments[i_reg][i_seg];
//        final_segments[reg_index].push_back (segment_index);
//        segment_labels[segment_index] = reg_index;
//      }
//      final_segments[i_reg].clear ();
//      num_pts_in_homogeneous_region[reg_index] += num_pts_in_homogeneous_region[i_reg];
//      num_pts_in_homogeneous_region[i_reg] = 0;
//      num_seg_in_homogeneous_region[reg_index] += num_seg_in_homogeneous_region[i_reg];
//      num_seg_in_homogeneous_region[i_reg] = 0;
//      final_segment_number -= 1;

//      int nghbr_number = static_cast<int> (region_neighbours[reg_index].size ());
//      for (int i_nghbr = 0; i_nghbr < nghbr_number; i_nghbr++)
//      {
//        if ( segment_labels[ region_neighbours[reg_index][i_nghbr].second ] == reg_index )
//        {
//          region_neighbours[reg_index][i_nghbr].first = std::numeric_limits<float>::max ();
//          region_neighbours[reg_index][i_nghbr].second = 0;
//        }
//      }
//      nghbr_number = static_cast<int> (region_neighbours[i_reg].size ());
//      for (int i_nghbr = 0; i_nghbr < nghbr_number; i_nghbr++)
//      {
//        if ( segment_labels[ region_neighbours[i_reg][i_nghbr].second ] != reg_index )
//        {
//          std::pair<float, int> pair;
//          pair.first = region_neighbours[i_reg][i_nghbr].first;
//          pair.second = region_neighbours[i_reg][i_nghbr].second;
//          region_neighbours[reg_index].push_back (pair);
//        }
//      }
//      region_neighbours[i_reg].clear ();
//      std::sort (region_neighbours[reg_index].begin (), region_neighbours[reg_index].end (), comparePair);
//    }
//  }

//  assembleRegions (num_pts_in_homogeneous_region, static_cast<int> (num_pts_in_homogeneous_region.size ()));

//  number_of_segments_ = final_segment_number;
}

//template<typename ClusteredPointT, typename PolicyName>
//inline size_t IncrementalSegmenter<ClusteredPointT, PolicyName>::assembleRegions (std::vector<unsigned int>& num_pts_in_region, int num_regions)
//{
//  clusters_.clear ();
//  pcl::PointIndices segment;
//  clusters_.resize (num_regions, segment);
//  for (int i_seg = 0; i_seg < num_regions; i_seg++)   // Platz in Vector anlegen
//  {
//    clusters_[i_seg].indices.resize (num_pts_in_region[i_seg]);
//  }

//  std::vector<int> counter;
//  counter.resize (num_regions, 0);
//  int point_number = static_cast<int> (indices_->size ());
//  for (int i_point = 0; i_point < point_number; i_point++)  // Alle Punkte durchlaufen
//  {
//    int point_index = (*indices_)[i_point];
//    int index = point_labels_[point_index];
//    index = segment_labels_[index];
//    clusters_[index].indices[ counter[index] ] = point_index;
//    counter[index] += 1;
//  }

//  // now we need to erase empty regions
//  if (clusters_.empty ())
//    return;

//  std::vector<pcl::PointIndices>::iterator itr1, itr2;
//  itr1 = clusters_.begin ();
//  itr2 = clusters_.end () - 1;

//  while (itr1 < itr2)
//  {
//    while (!(itr1->indices.empty ()) && itr1 < itr2)
//      itr1++;
//    while (  itr2->indices.empty ()  && itr1 < itr2)
//      itr2--;

//    if (itr1 != itr2)
//      itr1->indices.swap (itr2->indices);
//  }

//  if (itr2->indices.empty ())
//    clusters_.erase (itr2, clusters_.end ());
//}


//template<typename ClusteredPointT, typename PolicyName>
//inline size_t IncrementalSegmenter<ClusteredPointT, PolicyName>::findRegionNeighbours (std::vector< std::vector< std::pair<float, int> > >& neighbours_out, std::vector< std::vector<int> >& regions_in)
//{
//  int region_number = static_cast<int> (regions_in.size ());
//  neighbours_out.clear ();
//  neighbours_out.resize (region_number);

//  for (int i_reg = 0; i_reg < region_number; i_reg++)
//  {
//    int segment_num = static_cast<int> (regions_in[i_reg].size ());
//    neighbours_out[i_reg].reserve (segment_num * region_neighbour_number_);
//  for (int i_seg = 0; i_seg < segment_num; i_seg++)
//    {
//      int curr_segment = regions_in[i_reg][i_seg];
//      int nghbr_number = static_cast<int> (segment_neighbours_[curr_segment].size ());
//      std::pair<float, int> pair;
//      for (int i_nghbr = 0; i_nghbr < nghbr_number; i_nghbr++)
//      {
//        int segment_index = segment_neighbours_[curr_segment][i_nghbr];
//        if ( segment_distances_[curr_segment][i_nghbr] == std::numeric_limits<float>::max () )
//          continue;
//        if (segment_labels_[segment_index] != i_reg)
//        {
//          pair.first = segment_distances_[curr_segment][i_nghbr];
//          pair.second = segment_index;
//          neighbours_out[i_reg].push_back (pair);
//        }
//      }// next neighbour
//    }// next segment
//    std::sort (neighbours_out[i_reg].begin (), neighbours_out[i_reg].end (), comparePair);
//  }// next homogeneous region
//}


template<typename ClusteredPointT, typename PolicyName>
inline size_t IncrementalSegmenter<ClusteredPointT, PolicyName>::getClusterSize(
    const PartialClusters& partial_clusters, const size_t partial_cluster_index) const {
  size_t points_in_cluster = 0u;
  const PartialClustersSetPtr& partial_clusters_set =
      partial_clusters[partial_cluster_index].partial_clusters_set;
  for (const auto linked_partial_cluster_index : partial_clusters_set->partial_clusters_indices) {
    points_in_cluster += partial_clusters[linked_partial_cluster_index].point_indices.size();
  }
  return points_in_cluster;
}

template<typename ClusteredPointT, typename PolicyName>
inline std::vector<int> IncrementalSegmenter<ClusteredPointT, PolicyName>::getClusterIndices(
    const PartialClusters& partial_clusters, const size_t partial_cluster_index) const {
  const PartialClustersSetPtr& partial_clusters_set =
      partial_clusters[partial_cluster_index].partial_clusters_set;

  std::vector<int> point_indices;
  point_indices.reserve(getClusterSize(partial_clusters, partial_cluster_index));
  for (const auto linked_partial_cluster_index : partial_clusters_set->partial_clusters_indices) {
    point_indices.insert(point_indices.end(),
                         partial_clusters[linked_partial_cluster_index].point_indices.begin(),
                         partial_clusters[linked_partial_cluster_index].point_indices.end());
  }
  return point_indices;
}

template<typename ClusteredPointT, typename PolicyName>
inline bool IncrementalSegmenter<ClusteredPointT, PolicyName>::isPointAssignedToCluster(
    const ClusteredPointT& point) const noexcept {
  return getClusterId(point) != 0u;
}

template<typename ClusteredPointT, typename PolicyName>
inline typename IncrementalSegmenter<ClusteredPointT, PolicyName>::ClusterId
IncrementalSegmenter<ClusteredPointT, PolicyName>::getClusterId(
    const ClusteredPointT& point) const noexcept{
  return Policy::getPointClusterId(point);
}

template<typename ClusteredPointT, typename PolicyName>
inline void IncrementalSegmenter<ClusteredPointT, PolicyName>::setClusterId(
    ClusteredPointT& point, const ClusterId cluster_id) const noexcept {
  Policy::setPointClusterId(point, cluster_id);
}

} // namespace segmatch

#endif // SEGMATCH_IMPL_INCREMENTAL_SEGMENTER_HPP_
