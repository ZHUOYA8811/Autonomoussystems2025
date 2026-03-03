// Copyright (c) 2020 Papa Libasse Sow.
// https://github.com/Nandite/Pcl-Optics
// Distributed under the MIT Software License (X11 license).
// Adapted for ROS2 in 2026

#ifndef OPTICS_HPP
#define OPTICS_HPP

#include <pcl/common/geometry.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <limits>

namespace Optics {

struct ReachabilityDistance {
  ReachabilityDistance(const int pointIndex, const double reachabilityDistance)
      : pointIndex(pointIndex), reachabilityDistance(reachabilityDistance) {}

  int pointIndex;
  double reachabilityDistance;
};

inline bool operator<(const ReachabilityDistance& lhs, const ReachabilityDistance& rhs) {
  return (std::abs(lhs.reachabilityDistance - rhs.reachabilityDistance) < 1e-9)
             ? (lhs.pointIndex < rhs.pointIndex)
             : (lhs.reachabilityDistance < rhs.reachabilityDistance);
}

inline bool operator==(const ReachabilityDistance& lhs, const ReachabilityDistance& rhs) {
  return (std::abs(lhs.reachabilityDistance - rhs.reachabilityDistance) < 1e-9) &&
         (lhs.pointIndex == rhs.pointIndex);
}

template <typename PointT>
void optics(typename pcl::PointCloud<PointT>::Ptr cloud, 
            size_t minPts, 
            double epsilon,
            std::vector<pcl::PointIndicesPtr>& cluster_indices) {
  
  cluster_indices.clear();
  
  if (cloud->points.empty() || cloud->points.size() < minPts) {
    return;
  }

  // Build KdTree for neighbor search
  typename pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(cloud);

  const size_t n = cloud->points.size();
  std::vector<bool> processed(n, false);
  std::vector<double> reachability_dist(n, std::numeric_limits<double>::infinity());
  std::vector<int> ordered_list;
  ordered_list.reserve(n);

  // Lambda to compute core distance
  auto coreDistance = [&](int idx) -> double {
    std::vector<int> neighbors;
    std::vector<float> sq_distances;
    kdtree.radiusSearch(cloud->points[idx], epsilon, neighbors, sq_distances);
    
    if (neighbors.size() < minPts) {
      return std::numeric_limits<double>::infinity();
    }
    
    std::sort(sq_distances.begin(), sq_distances.end());
    return std::sqrt(sq_distances[static_cast<int>(minPts) - 1]);
  };

  // OPTICS algorithm
  for (size_t i = 0; i < n; ++i) {
    if (processed[i]) continue;

    processed[i] = true;
    ordered_list.push_back(i);
    
    double core_dist = coreDistance(i);
    if (std::isinf(core_dist)) continue;

    // Priority queue for ordered expansion
    std::vector<ReachabilityDistance> seeds;
    std::vector<int> neighbors;
    std::vector<float> sq_distances;
    kdtree.radiusSearch(cloud->points[i], epsilon, neighbors, sq_distances);

    for (size_t j = 0; j < neighbors.size(); ++j) {
      int neighbor_idx = neighbors[j];
      if (!processed[neighbor_idx]) {
        double new_reach_dist = std::max(core_dist, static_cast<double>(std::sqrt(sq_distances[j])));
        if (new_reach_dist < reachability_dist[neighbor_idx]) {
          reachability_dist[neighbor_idx] = new_reach_dist;
          seeds.push_back(ReachabilityDistance(neighbor_idx, new_reach_dist));
        }
      }
    }

    std::sort(seeds.begin(), seeds.end());

    while (!seeds.empty()) {
      auto current = seeds.front();
      seeds.erase(seeds.begin());

      if (processed[current.pointIndex]) continue;

      processed[current.pointIndex] = true;
      ordered_list.push_back(current.pointIndex);

      double current_core_dist = coreDistance(current.pointIndex);
      if (std::isinf(current_core_dist)) continue;

      neighbors.clear();
      sq_distances.clear();
      kdtree.radiusSearch(cloud->points[current.pointIndex], epsilon, neighbors, sq_distances);

      for (size_t j = 0; j < neighbors.size(); ++j) {
        int neighbor_idx = neighbors[j];
        if (!processed[neighbor_idx]) {
          double new_reach_dist = std::max(current_core_dist, static_cast<double>(std::sqrt(sq_distances[j])));
          if (new_reach_dist < reachability_dist[neighbor_idx]) {
            reachability_dist[neighbor_idx] = new_reach_dist;
            
            // Remove old entry and add new one
            seeds.erase(
              std::remove_if(seeds.begin(), seeds.end(),
                [neighbor_idx](const ReachabilityDistance& rd) {
                  return rd.pointIndex == neighbor_idx;
                }),
              seeds.end());
            
            seeds.push_back(ReachabilityDistance(neighbor_idx, new_reach_dist));
            std::sort(seeds.begin(), seeds.end());
          }
        }
      }
    }
  }

  // Extract clusters using reachability threshold
  std::vector<int> current_cluster;
  
  for (int idx : ordered_list) {
    if (reachability_dist[idx] <= epsilon) {
      current_cluster.push_back(idx);
    } else {
      if (current_cluster.size() >= minPts) {
        pcl::PointIndicesPtr cluster(new pcl::PointIndices);
        cluster->indices = current_cluster;
        cluster_indices.push_back(cluster);
      }
      current_cluster.clear();
      current_cluster.push_back(idx);
    }
  }

  // Add last cluster if valid
  if (current_cluster.size() >= minPts) {
    pcl::PointIndicesPtr cluster(new pcl::PointIndices);
    cluster->indices = current_cluster;
    cluster_indices.push_back(cluster);
  }
}

} // namespace Optics

#endif // OPTICS_HPP
