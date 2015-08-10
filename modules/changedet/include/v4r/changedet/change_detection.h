#ifndef _V4R_CHANGE_DETECTION_H_
#define _V4R_CHANGE_DETECTION_H_

#include <pcl/common/eigen.h>
#include <pcl/search/kdtree.h>

#include <pcl/segmentation/segment_differences.h>

namespace v4r {

template<class PointType>
class ChangeDetector {

public:
	typedef pcl::PointCloud<PointType> Cloud;
	typedef typename Cloud::Ptr CloudPtr;
    typedef pcl::search::KdTree<PointType> Tree;

    ChangeDetector() :
    	added(new Cloud()), removed(new Cloud) {
    }

	void detect(const CloudPtr source, const CloudPtr target, const Eigen::Affine3f sensor_pose,
			float diff_tolerance = PCL_DIFFERENCE_TOLERANCE);

	bool isObjectRemoved(CloudPtr object_cloud) const;

	static float computePlanarity(const CloudPtr input_cloud);

	std::vector<typename pcl::PointCloud<PointType>::Ptr>
	static clusterPointCloud(CloudPtr input_cloud, double tolerance,
			int min_cluster_size, int max_cluster_size);

	/**
	 * diff = A \ B
	 * indices = indexes of preserved points from A
	 */
	static void difference(const CloudPtr A, const CloudPtr B, CloudPtr &diff,
			std::vector<int> &indices, float tolerance = PCL_DIFFERENCE_TOLERANCE) {
		if(A->empty()) {
			return;
		}
		if(B->empty()) {
			pcl::copyPointCloud(*A, *diff);
			for(size_t i = 0; i < A->size(); i++) {
				indices.push_back(i);
			}
			return;
		}

		// We're interested in a single nearest neighbor only
		std::vector<int> nn_indices(1);
		std::vector<float> nn_distances(1);

		typename Tree::Ptr tree(new pcl::search::KdTree<PointType>);
		tree->setInputCloud(B);

		// Iterate through the source data set
		for (int i = 0; i < static_cast<int>(A->points.size()); ++i) {
			if (!pcl::isFinite(A->points[i]))
				continue;
			// Search for the closest point in the target data set (number of neighbors to find = 1)
			if (!tree->nearestKSearch(A->points[i], 1, nn_indices,
					nn_distances)) {
				PCL_WARN("No neighbor found for point %lu (%f %f %f)!\n", i,
						A->points[i].x, A->points[i].y, A->points[i].z);
				continue;
			}

			if (nn_distances[0] > tolerance*tolerance)
				indices.push_back(i);
		}

		// Allocate enough space and copy the basics
		diff->points.resize(indices.size());
		diff->header = A->header;
		diff->width = static_cast<uint32_t>(indices.size());
		diff->height = 1;
		//if (src.is_dense)
		diff->is_dense = true;
		//else
		// It's not necessarily true that is_dense is false if cloud_in.is_dense is false
		// To verify this, we would need to iterate over all points and check for NaNs
		//output.is_dense = false;

		// Copy all the data fields from the input cloud to the output one
		copyPointCloud(*A, indices, *diff);
	}

	static void difference(const CloudPtr A, const CloudPtr B, CloudPtr diff) {
		std::vector<int> indices;
		difference(A, B, diff, indices);
	}

	static void removePointsFrom(const CloudPtr cloud, const CloudPtr toBeRemoved);

	static int overlapingPoints(const CloudPtr train, const CloudPtr query,
			float tolerance = PCL_DIFFERENCE_TOLERANCE);

	const CloudPtr getAdded() const {
		return added;
	}

	const CloudPtr getRemoved() const {
		return removed;
	}

	static CloudPtr getNonplanarClusters(CloudPtr removed_points) {
		std::vector<CloudPtr> clusters = clusterPointCloud(removed_points,
				MAXIMAL_INTRA_CLUSTER_DIST, MINIMAL_CLUSTER_POINTS, MAXIMAL_CLUSTER_POINTS);
		CloudPtr nonplanarClusters(new Cloud());
		for(typename  std::vector<CloudPtr>::iterator c = clusters.begin();
				c < clusters.end(); c++) {
			if(computePlanarity(*c) < PLANARITY_THRESHOLD) {
				*nonplanarClusters += **c;
			}
		}
		return nonplanarClusters;
	}

	static CloudPtr removalSupport(CloudPtr removed_points, CloudPtr &object_cloud) {
		CloudPtr support(new Cloud());
		std::vector<CloudPtr> removed_clusters = clusterPointCloud(removed_points,
				MAXIMAL_INTRA_CLUSTER_DIST, MINIMAL_CLUSTER_POINTS, MAXIMAL_CLUSTER_POINTS);
		for(typename  std::vector<CloudPtr>::iterator c = removed_clusters.begin();
				c < removed_clusters.end(); c++) {

			// only non planar cluster considered:
			if(computePlanarity(*c) < PLANARITY_THRESHOLD) {
				if(overlapingPoints(object_cloud, *c) > (MIN_REMOVAL_OVERLAP * (*c)->size())) {
					*support += **c;
				}
			}
		}
		return support;
	}

protected:

	CloudPtr removalSupport(CloudPtr &object_cloud) const;

private:

	CloudPtr added, removed;

	static const int OCCLUSION_CHECKER_BINS = 180;
	static const int MINIMAL_CLUSTER_POINTS = 50;
	static const int MAXIMAL_CLUSTER_POINTS = 1000000;
	static const float MAXIMAL_INTRA_CLUSTER_DIST = 0.03;
	static const float PLANARITY_THRESHOLD = 0.95;
	static const float MIN_REMOVAL_OVERLAP = 0.8;
	static const float PCL_DIFFERENCE_TOLERANCE = 0.01;
};

}

#include <v4r/changedet/impl/change_detection.hpp>

#endif
