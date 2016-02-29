/*
 * depth_connected_components.cpp
 *
 *  Created on: 5 Aug 2011
 *      Author: jc310
 */
// The type of the labels
typedef unsigned int label_t;
// Internal types for the set-union data type
typedef std::map<label_t, label_t> map_t;
typedef boost::associative_property_map<map_t> assoc_t;

template<typename Point_t, typename THRESHOLD>
void icl::extractEuclideanClustersFromImage(const pcl::PointCloud<Point_t> &pc,
                                            std::vector<pcl::PointIndices> &output_indices,
                                            const THRESHOLD &threshold,
                                            unsigned int min_cluster_size,
                                            unsigned int max_cluster_size,
                                            const boost::function<THRESHOLD (const Point_t&, const Point_t&)> &distance_function) {
	// The pointcloud has not to be dense, it is assumed that is sorted as an image (neighbors are closed in index)
	assert(!pc.is_dense);
	assert(pc.height > 0);
	const unsigned int &width = pc.width;
	const unsigned int &height = pc.height;

	// Clear the output
	output_indices.resize(0);

	// Auxiliary matrix were the labels will be kept
	std::vector<label_t> labels(pc.points.size());
	std::fill(labels.begin(), labels.end(), std::numeric_limits<label_t>::max());

	// The label
	label_t label = 0;

	// Disjoint set structure
	map_t ranks;
	map_t parents;
	assoc_t ranks_assoc(ranks);
	assoc_t parents_assoc(parents);
	boost::disjoint_sets<assoc_t, assoc_t> dj_set(ranks_assoc, parents_assoc);
	dj_set.make_set(label);

	// Iterate over the image
	for(unsigned int j = 0; j < height; j++) {
	        for(unsigned int i = 0; i < width; i++) {
			const int index = i + j * width;
			const int index_n1 = i + (j-1) * width;
			const int index_n2 = i-1 + (j-1) * width;
			const int index_n3 = i-1 + j * width;
			const int index_n4 = i+1 + (j-1) * width;
			//			const int index_n4 = i-1 + (j+1) * width;
			const Point_t &current = pc.points[index];    //(i,j);

			unsigned char neighbors = 0x0;
			if(!(std::isnan(current.x) || std::isnan(current.y) || std::isnan(current.z))) {
				if(j > 0) {
					const Point_t &neighbor = pc.points[index_n1];
					if(distance_function(current, neighbor) < threshold)
						neighbors |= 0x01;
				}
				if(i > 0 && j > 0) {
					const Point_t &neighbor = pc.points[index_n2];
					if(distance_function(current, neighbor) < threshold)
						neighbors |= 0x02;
				}
				if(i > 0) {
					const Point_t &neighbor = pc.points[index_n3];
					if(distance_function(current, neighbor) < threshold)
						neighbors |= 0x04;
				}
				if(i < height - 1 && j > 0) {
					const Point_t &neighbor = pc.points[index_n4];
					if(distance_function(current, neighbor) < threshold)
						neighbors |= 0x08;
				}
				label_t new_label = label;
				if(!neighbors) {
					label++;
					dj_set.make_set(label);
				} else {
					if(neighbors & 0x01) {
						if(labels[index_n1] != 0 && labels[index_n1] < new_label)
							new_label = labels[index_n1];
					}
					if(neighbors & 0x02) {
						if(labels[index_n2] != 0 && labels[index_n2] < new_label)
							new_label = labels[index_n2];
					}
					if(neighbors & 0x04) {
						if(labels[index_n3] != 0 && labels[index_n3] < new_label)
							new_label = labels[index_n3];
					}
					if(neighbors & 0x08) {
						if(labels[index_n4] != 0 && labels[index_n4] < new_label)
							new_label = labels[index_n4];
					}
					// Add to the linked lists
					if((neighbors & 0x01) && labels[index_n1] > new_label)
						dj_set.union_set(new_label, labels[index_n1]);
					if((neighbors & 0x02) && labels[index_n2] > new_label)
						dj_set.union_set(new_label, labels[index_n2]);
					if((neighbors & 0x04) && labels[index_n3] > new_label)
						dj_set.union_set(new_label, labels[index_n3]);
					if((neighbors & 0x08) && labels[index_n4] > new_label) {
						dj_set.union_set(new_label, labels[index_n4]);
					}
				}
				// Assign the calculated label
				labels[index] = new_label;
			} else {
				labels[index] = 0;
			}
		}
	}

	// Get the final clusters
	//note that NaN clusters have an index of 0
	std::map<label_t, std::vector<unsigned int> > final_clusters;
	for(unsigned int j = 0; j < height; j++) {
        	for(unsigned int i = 0; i < width; i++) {
			unsigned int indx = i + j*width;
			label_t local_label = labels[indx];
			if(local_label != std::numeric_limits<label_t>::max() && local_label != 0) {
				label_t real_label = dj_set.find_set(local_label);
				if(final_clusters.find(real_label) == final_clusters.end()) {
					final_clusters[real_label].resize(0);
				}
				final_clusters[real_label].push_back(indx);
			}
		}
	}
	// Filter by min size and max size
	for(std::map<label_t, std::vector<unsigned int> >::iterator it = final_clusters.begin(); it != final_clusters.end(); it++) {
	        const size_t cluster_size = it->second.size();
		if(cluster_size > min_cluster_size && cluster_size < max_cluster_size) {
			pcl::PointIndices cluster_indices;
			cluster_indices.indices.resize(cluster_size);
			std::copy(it->second.begin(), it->second.end(), cluster_indices.indices.begin());    //cant use mem copy becuase final_clusters is unsigned int, whilst pcl::PointIndices::indices is a vector of ints
			std::sort(cluster_indices.indices.begin(), cluster_indices.indices.end());
			output_indices.push_back(cluster_indices);
		}
	}

}
