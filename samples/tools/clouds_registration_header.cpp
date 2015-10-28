/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

/* \author Radu Bogdan Rusu
 * adaptation Raphael Favier, Martin Velas
 */

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <v4r/features/opencv_sift_local_estimator.h>
#include <v4r/common/miscellaneous.h>
#include <v4r/registration/FeatureBasedRegistration.h>
#include <v4r/changedet/Visualizer3D.h>

//#define VISUALISATION 1

typedef flann::L1<float> DistT;

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef pcl::Histogram<128> FeatureT;

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation: public pcl::PointRepresentation<PointNormalT> {
	using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
	MyPointRepresentation() {
		// Define the number of dimensions
		nr_dimensions_ = 4;
	}

	// Override the copyToFloatArray method to define our feature vector
	virtual void copyToFloatArray(const PointNormalT &p, float * out) const {
		// < x, y, z, curvature >
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
		out[3] = p.curvature;
	}
};

class Registration {
public:
	Registration() :
		scene_signatures(new pcl::PointCloud<FeatureT>()),
		scene_keypoints(new PointCloud()),
		scene_points(new PointCloud()),
		new_keypoints(new PointCloud()),
		new_signatures(new pcl::PointCloud<FeatureT>()) {

#ifdef VISUALISATION
		// Create a PCLVisualizer object
		p = new pcl::visualization::PCLVisualizer("Pairwise Incremental Registration example");
		p->createViewPort(0.0, 0, 0.5, 1.0, vp_1);
		p->createViewPort(0.5, 0, 1.0, 1.0, vp_2);
#endif
	}

	/** \brief Align a pair of PointCloud datasets and return the result
		 * \param cloud_src the source PointCloud
		 * \param cloud_tgt the target PointCloud
		 * \param output the resultant aligned source PointCloud
		 * \param final_transform the resultant transform between source and target
		 */
	Eigen::Matrix4f align(const PointCloud::Ptr cloud_original) {

		Eigen::Matrix4f init_transform = alignBySIFT(cloud_original);
		PointCloud::Ptr cloud(new PointCloud());
		pcl::transformPointCloud(*cloud_original, *cloud, init_transform);

		Eigen::Matrix4f icp_transform = Eigen::Matrix4f::Identity();

		if(!scene_points->empty()) {
			showCloudsLeft(scene_points, cloud_original);
			showCloudsLeft(scene_points, cloud);

			//remove NAN points from the cloud
			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

			// Downsample for consistency and speed
			// \note enable this for large datasets
			PointCloud::Ptr src(new PointCloud);
			PointCloud::Ptr tgt(new PointCloud);
			pcl::VoxelGrid<PointT> grid;
			grid.setLeafSize(0.03, 0.03, 0.03);
			grid.setInputCloud(scene_points);
			grid.filter(*src);

			grid.setInputCloud(cloud);
			grid.filter(*tgt);

			// Compute surface normals and curvature
			PointCloudWithNormals::Ptr points_with_normals_src(
					new PointCloudWithNormals);
			PointCloudWithNormals::Ptr points_with_normals_tgt(
					new PointCloudWithNormals);

			pcl::NormalEstimation<PointT, PointNormalT> norm_est;
			pcl::search::KdTree<PointT>::Ptr tree(
					new pcl::search::KdTree<PointT>());
			norm_est.setSearchMethod(tree);
			norm_est.setKSearch(30);

			norm_est.setInputCloud(src);
			norm_est.compute(*points_with_normals_src);
			pcl::copyPointCloud(*src, *points_with_normals_src);

			norm_est.setInputCloud(tgt);
			norm_est.compute(*points_with_normals_tgt);
			pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

			//
			// Instantiate our custom point representation (defined above) ...
			MyPointRepresentation point_representation;
			// ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
			float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
			point_representation.setRescaleValues(alpha);

			//
			// Align
			pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
			reg.setTransformationEpsilon(1e-6);
			// Set the maximum distance between two correspondences (src<->tgt) to 10cm
			// Note: adjust this based on the size of your datasets
			reg.setMaxCorrespondenceDistance(0.03);
			// Set the point representation
			reg.setPointRepresentation(
					boost::make_shared<const MyPointRepresentation>(
							point_representation));

			reg.setInputSource(points_with_normals_src);
			reg.setInputTarget(points_with_normals_tgt);

			//
			// Run the same optimization in a loop and visualize the results
			Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev;
			PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
			reg.setMaximumIterations(2);
			for (int i = 0; i < 30; ++i) {
				PCL_INFO("Iteration Nr. %d.\n", i);

				// save cloud for visualization purpose
				points_with_normals_src = reg_result;

				// Estimate
				reg.setInputSource(points_with_normals_src);
				reg.align(*reg_result);

				//accumulate transformation between each Iteration
				Ti = reg.getFinalTransformation() * Ti;

				//if the difference between this transformation and the previous one
				//is smaller than the threshold, refine the process by reducing
				//the maximal correspondence distance
				if (fabs((reg.getLastIncrementalTransformation() - prev).sum())
						< reg.getTransformationEpsilon())
					reg.setMaxCorrespondenceDistance(
							reg.getMaxCorrespondenceDistance()*0.98);

				prev = reg.getLastIncrementalTransformation();

				// visualize current state
				showCloudsRight(points_with_normals_tgt, points_with_normals_src);
			}

			//
			// Get the transformation from target to source
			icp_transform = Ti.inverse();

			//
			// Transform target back in source frame
			pcl::transformPointCloud(*cloud, *cloud, icp_transform);

#ifdef VISUALISATION
			p->removePointCloud("source");
			p->removePointCloud("target");

			PointCloudColorHandlerCustom<PointT> cloud_tgt_h(cloud, 0, 255, 0);
			PointCloudColorHandlerCustom<PointT> cloud_src_h(scene_points, 255, 0, 0);
			p->addPointCloud(cloud, cloud_tgt_h, "target", vp_2);
			p->addPointCloud(scene_points, cloud_src_h, "source", vp_2);

			PCL_INFO("Press q to continue the registration.\n");
			p->spin();

			p->removePointCloud("source");
			p->removePointCloud("target");
#endif
		}
		*scene_points += *cloud;
		*scene_signatures += *new_signatures;
		PointCloud keypoints_transformed;
		pcl::transformPointCloud(*new_keypoints, keypoints_transformed, init_transform*icp_transform);
		*scene_keypoints += keypoints_transformed;
		return init_transform*icp_transform;
	}

private:

	bool calcSiftFeatures (PointCloud::Ptr &cloud, pcl::PointCloud<FeatureT>::Ptr &signatures,
			PointCloud::Ptr &keypoints, pcl::PointIndices &incdices)
	{
	    ;
	    pcl::PointCloud<PointT>::Ptr processed_foo;
	    v4r::OpenCVSIFTLocalEstimation<PointT, FeatureT > estimator;
	    bool ret = estimator.estimate(cloud, processed_foo, keypoints, signatures);
	    estimator.getKeypointIndices(incdices);

	    //----display-keypoints--------------------
	    /*static v4r::Visualizer3D vis;
	    vis.getViewer()->removeAllPointClouds();
	    vis.getViewer()->removeAllShapes();
	    vis.addColorPointCloud(cloud);
		for (size_t keyId = 0; keyId < pSiftKeypoints->size (); keyId++)
		{
			std::stringstream sphere_name;
			sphere_name << "sphere_" << keyId;
			vis.getViewer()->addSphere<PointT> (pSiftKeypoints->at (keyId), 0.1, sphere_name.str ());
		}
		vis.show();*/

		return ret;
	}

	Eigen::Matrix4f alignBySIFT(PointCloud::Ptr new_cloud)
	{
		pcl::PointIndices new_incdices;
		new_signatures->clear();
		new_keypoints->clear();
		calcSiftFeatures(new_cloud, new_signatures, new_keypoints, new_incdices);

		Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

		if(!scene_keypoints->empty()) {
		    pcl::CorrespondencesPtr correspondences ( new pcl::Correspondences );
		    correspondences->resize(new_keypoints->size());

		    boost::shared_ptr< flann::Index<DistT> > flann_index;
		    v4r::convertToFLANN<FeatureT, DistT >(scene_signatures, flann_index);
		    const int K = 1;
		    flann::Matrix<int> indices = flann::Matrix<int> ( new int[K], 1, K );
		    flann::Matrix<float> distances = flann::Matrix<float> ( new float[K], 1, K );
		    for ( size_t keypointId = 0; keypointId < new_keypoints->size (); keypointId++ )
		    {
		        FeatureT searchFeature = new_signatures->at(keypointId);
		        int size_feat = sizeof ( searchFeature.histogram ) / sizeof ( float );
		        v4r::nearestKSearch ( flann_index, searchFeature.histogram, size_feat, K, indices, distances );

		        pcl::Correspondence corr;
		        corr.distance = distances[0][0];
		        corr.index_query = keypointId;
		        corr.index_match = indices[0][0];
		        correspondences->at(keypointId) = corr;
		    }

			pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>::Ptr rej;
			rej.reset (new pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> ());
			pcl::CorrespondencesPtr after_rej_correspondences (new pcl::Correspondences ());

			rej->setMaximumIterations (50000);
			rej->setInlierThreshold (0.02);
			rej->setInputTarget (scene_keypoints);
			rej->setInputSource (new_keypoints);
			rej->setInputCorrespondences (correspondences);
			rej->getCorrespondences (*after_rej_correspondences);

			pcl::registration::TransformationEstimationSVD<PointT, PointT> t_est;
			t_est.estimateRigidTransformation (*new_keypoints, *scene_keypoints,
					*after_rej_correspondences, transformation);
		}
		return transformation;
	}

	/** \brief Display source and target on the first viewport of the visualizer
	 *
	 */
	void showCloudsLeft(const PointCloud::Ptr cloud_target,
			const PointCloud::Ptr cloud_source) {
#ifdef VISUALISATION
		p->removePointCloud("vp1_target");
		p->removePointCloud("vp1_source");

		PointCloudColorHandlerCustom<PointT> tgt_h(cloud_target, 0, 255, 0);
		PointCloudColorHandlerCustom<PointT> src_h(cloud_source, 255, 0, 0);
		p->addPointCloud(cloud_target, tgt_h, "vp1_target", vp_1);
		p->addPointCloud(cloud_source, src_h, "vp1_source", vp_1);

		PCL_INFO("Press q to begin the registration.\n");
		p->spin();
#endif
	}

	/** \brief Display source and target on the second viewport of the visualizer
	 *
	 */
	void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target,
			const PointCloudWithNormals::Ptr cloud_source) {
#ifdef VISUALISATION
		p->removePointCloud("source");
		p->removePointCloud("target");

		PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler(
				cloud_target, "curvature");
		if (!tgt_color_handler.isCapable())
			PCL_WARN("Cannot create curvature color handler!");

		PointCloudColorHandlerGenericField<PointNormalT> src_color_handler(
				cloud_source, "curvature");
		if (!src_color_handler.isCapable())
			PCL_WARN("Cannot create curvature color handler!");

		p->addPointCloud(cloud_target, tgt_color_handler, "target", vp_2);
		p->addPointCloud(cloud_source, src_color_handler, "source", vp_2);

		p->spinOnce();
#endif
	}

	pcl::PointCloud<FeatureT>::Ptr new_signatures;
	PointCloud::Ptr new_keypoints;
	pcl::PointCloud<FeatureT>::Ptr scene_signatures;
	PointCloud::Ptr scene_keypoints;
	PointCloud::Ptr scene_points;
#ifdef VISUALISATION
	pcl::visualization::PCLVisualizer *p;
	int vp_1, vp_2;
#endif
};

/* ---[ */
int main(int argc, char** argv) {

	Registration registration;
	for(int i = 1; i < argc; i++) {
		PointCloud::Ptr cloud(new PointCloud());
		pcl::io::loadPCDFile (argv[i], *cloud);

		PCL_INFO("Aligning %s (%d pts).\n", argv[i], cloud->size());
		Eigen::Matrix4f transform = registration.align(cloud);

		v4r::Mat4f2RotTrans(transform, cloud->sensor_orientation_, cloud->sensor_origin_);
		pcl::io::savePCDFile(argv[i], *cloud, true);
	}
}
