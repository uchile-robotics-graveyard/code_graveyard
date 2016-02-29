/*
 * depth_connected_components.h
 *
 *  Created on: 5 Aug 2011
 *      Author: jc310
 */

#ifndef DEPTH_CONNECTED_COMPONENTS_H_
#define DEPTH_CONNECTED_COMPONENTS_H_
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <boost/function.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <map>
#include <limits>
#include <vector>

namespace icl {

template<typename Point_t, typename THRESHOLD>
void extractEuclideanClustersFromImage(const pcl::PointCloud<Point_t> &pc,
                                       std::vector<pcl::PointIndices> &output_indices,
                                       const THRESHOLD &threshold = 0.05,
                                       unsigned int min_cluster_size = 0,
                                       unsigned int max_cluster_size = 20000,
                                       const boost::function<THRESHOLD (const Point_t &, const Point_t&)> &distance = &pcl::euclideanDistance<Point_t, Point_t>);

}

#include "depth_connected_components.hpp"
#endif /* DEPTH_CONNECTED_COMPONENTS_H_ */
