/******************************************************************************
 * Copyright (c) 2015 Thomas Faeulhammer
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 ******************************************************************************/

/**
*
*      @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
*      @date August, 2015
*      @brief multiview object instance recognizer
*      Reference(s): Faeulhammer et al, ICRA 2015
*                    Faeulhammer et al, MVA 2015
*/

#ifndef V4R_MULTIVIEW_OBJECT_RECOGNIZER_H__
#define V4R_MULTIVIEW_OBJECT_RECOGNIZER_H__

#include <vector>
#include <iostream>
#include <string>
#include <sstream>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/flann_search.hpp>

#include <v4r_config.h>
#include <v4r/common/noise_models.h>
#include <v4r/recognition/boost_graph_extension.h>
#include <v4r/recognition/multi_pipeline_recognizer.h>

#include <SiftGPU/SiftGPU.h>

#ifdef HAVE_SIFTGPU
#include <v4r/features/sift_local_estimator.h>
#else
#include <v4r/features/opencv_sift_local_estimator.h>
#endif

namespace v4r
{

template<typename PointT>
class V4R_EXPORTS MultiviewRecognizer : public Recognizer<PointT>
{
protected:
    typedef Model<PointT> ModelT;
    typedef boost::shared_ptr<ModelT> ModelTPtr;
    typedef flann::L1<float> DistT;
    typedef pcl::PointCloud<PointT> Cloud;

    using Recognizer<PointT>::scene_;
    using Recognizer<PointT>::scene_normals_;
    using Recognizer<PointT>::models_;
    using Recognizer<PointT>::model_or_plane_is_verified_;
    using Recognizer<PointT>::transforms_;
    using Recognizer<PointT>::planes_;
    using Recognizer<PointT>::hv_algorithm_;

    using Recognizer<PointT>::poseRefinement;
    using Recognizer<PointT>::hypothesisVerification;
    using Recognizer<PointT>::icp_scene_indices_;
    using Recognizer<PointT>::visResStore;

    boost::shared_ptr<MultiRecognitionPipeline<PointT> > rr_;

    typedef boost::property<boost::edge_weight_t, CamConnect> EdgeWeightProperty;
    typedef boost::adjacency_list < boost::vecS, boost::vecS, boost::undirectedS, size_t, EdgeWeightProperty> Graph;
    typedef boost::graph_traits < Graph >::vertex_descriptor ViewD;
    typedef boost::graph_traits < Graph >::edge_descriptor EdgeD;
    typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
    typedef boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;
    Graph gs_;

    pcl::visualization::PCLVisualizer::Ptr go3d_vis_;
    std::vector<int> go_3d_viewports_;

    typedef typename std::map<std::string, ObjectHypothesis<PointT> > symHyp;

    size_t id_;

    typename std::map<size_t, View<PointT> > views_;

    std::string scene_name_;

    /** \brief stores keypoint correspondences */
    symHyp obj_hypotheses_;

    /** \brief Point-to-point correspondence grouping algorithm */
    typename boost::shared_ptr<v4r::CorrespondenceGrouping<PointT, PointT> > cg_algorithm_;

    Eigen::Matrix4f pose_;

    cv::Ptr<SiftGPU> sift_;

    bool computeAbsolutePose(CamConnect & e, bool is_first_edge = false);
    
    std::map < size_t, typename pcl::PointCloud<PointT>::Ptr > removed_points_history_;
    std::map < size_t, typename pcl::PointCloud<PointT>::Ptr > added_points_history_;
    pcl::PointCloud<PointT> changes_visualization;

    /** \brief removes vertices from graph if max_vertices_in_graph has been reached */
    void pruneGraph();

    void correspondenceGrouping();

    float calcEdgeWeightAndRefineTf (const typename pcl::PointCloud<PointT>::ConstPtr &cloud_src,
                                    const typename pcl::PointCloud<PointT>::ConstPtr &cloud_dst,
                                    Eigen::Matrix4f &refined_transform,
                                    const Eigen::Matrix4f &transform = Eigen::Matrix4f::Identity());

    bool calcSiftFeatures (const typename pcl::PointCloud<PointT>::Ptr &cloud_src,
                           typename pcl::PointCloud<PointT>::Ptr &sift_keypoints,
                           std::vector< int > &sift_keypoint_indices,
                           pcl::PointCloud<FeatureT>::Ptr &sift_signatures,
                           std::vector<float> &sift_keypoint_scales);

    void estimateViewTransformationBySIFT(const pcl::PointCloud<PointT> &src_cloud,
                                          const pcl::PointCloud<PointT> &dst_cloud,
                                          const std::vector<int> &src_sift_keypoint_indices,
                                          const std::vector<int> &dst_sift_keypoint_indices,
                                          const pcl::PointCloud<FeatureT> &src_sift_signatures,
                                          boost::shared_ptr< flann::Index<DistT> > &dst_flann_index,
                                          std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &transformations,
                                          bool use_gc = false );

public:
    class Parameter : public Recognizer<PointT>::Parameter
    {
    public:
        using Recognizer<PointT>::Parameter::icp_iterations_;
        using Recognizer<PointT>::Parameter::icp_type_;
        using Recognizer<PointT>::Parameter::normal_computation_method_;
        using Recognizer<PointT>::Parameter::voxel_size_icp_;
        using Recognizer<PointT>::Parameter::store_vis_results_to_;
        using Recognizer<PointT>::Parameter::merge_close_hypotheses_;
        using Recognizer<PointT>::Parameter::merge_close_hypotheses_dist_;
        using Recognizer<PointT>::Parameter::merge_close_hypotheses_angle_;

        bool scene_to_scene_;  /// @brief if true, tries to register two views based on SIFT background matching
        bool use_robot_pose_;   /// @brief if true, uses given pose between two views as relative camera pose estimate
        bool hyp_to_hyp_;   /// @brief if true adds edges for common object hypotheses (not implemented atm)
        bool use_gc_s2s_;   /// @brief defines method used for SIFT background matching
        double distance_same_keypoint_; /// @brief defines the minimum distance between two keypoints (of same model) to be seperated
        float same_keypoint_dot_product_; /// @brief defines the minimum dot distance between the normals of two keypoints (of same model) to be seperated
        int extension_mode_; /// @brief defines method used to extend information from other views (0 = keypoint correspondences (ICRA2015 paper); 1 = full hypotheses only (MVA2015 paper))
        int max_vertices_in_graph_; /// @brief maximum number of views taken into account (views selected in order of latest recognition calls)
        float chop_z_;  /// @brief points with z-component higher than chop_z_ will be ignored (low chop_z reduces computation time and false positives (noise increase with z)
        bool compute_mst_; /// @brief if true, does point cloud registration by SIFT background matching (given scene_to_scene_ == true), by using given pose (if use_robot_pose_ == true) and by common object hypotheses (if hyp_to_hyp_ == true) from all the possible connection a Mimimum Spanning Tree is computed. If false, it only uses the given pose for each point cloud

        bool use_change_detection_;	// use change detection for old hypothesis/keypoints removal
        int min_points_for_hyp_removal_;	// minimal overlap for removal
        bool use_novelty_filter_;	// use additional filtering - only hypothesis overlapping novel points preserved
        int min_points_for_hyp_preserve_;	// minimal overlap with novel pointy for preserving
        bool use_chdet_for_reconstruction_;	// use change detection also for scene reconstruction

        Parameter (
                bool scene_to_scene = true,
                bool use_robot_pose = true,
                bool hyp_to_hyp = false,
                bool use_gc_s2s = true,
                double distance_same_keypoint = 0.005f*0.005f,
                float same_keypoint_dot_product = 0.8f,
                int extension_mode = 0,
                int max_vertices_in_graph = 3,
                float chop_z = std::numeric_limits<float>::max(),
                bool compute_mst = true,
                bool use_change_detection = false,
                int min_points_for_hyp_removal = 50,
                bool use_novelty_filter = false,
                int min_points_for_hyp_preserve = 50,
                bool use_chdet_for_reconstruction = false) :

            Recognizer<PointT>::Parameter(),
            scene_to_scene_ (scene_to_scene),
            use_robot_pose_ (use_robot_pose),
            hyp_to_hyp_ (hyp_to_hyp),
            use_gc_s2s_ (use_gc_s2s),
            distance_same_keypoint_ (distance_same_keypoint),
            same_keypoint_dot_product_ (same_keypoint_dot_product),
            extension_mode_ (extension_mode),
            max_vertices_in_graph_ (max_vertices_in_graph),
            chop_z_ (chop_z),
            compute_mst_ (compute_mst),
            use_change_detection_(use_change_detection),
            min_points_for_hyp_removal_(min_points_for_hyp_removal),
            use_novelty_filter_(use_novelty_filter),
            min_points_for_hyp_preserve_(min_points_for_hyp_preserve),
            use_chdet_for_reconstruction_(use_chdet_for_reconstruction)
        {}
    } param_;

    MultiviewRecognizer(const Parameter &p = Parameter()) : Recognizer<PointT>(p) {
        param_ = p;
        id_ = 0;
        pose_ = Eigen::Matrix4f::Identity();
    }

    /**
     * @brief sets the underlying single-view recognition
     * @param single-view recognizer
     */
    void setSingleViewRecognizer(const typename boost::shared_ptr<MultiRecognitionPipeline<PointT> > & rec)
    {
        rr_ = rec;
    }

    typename noise_models::NguyenNoiseModel<PointT>::Parameter nm_param_;


    std::string get_scene_name() const
    {
        return scene_name_;
    }

    void set_sift(cv::Ptr<SiftGPU> &sift)
    {
        sift_ = sift;
    }


    /** \brief Sets the algorithm for Correspondence Grouping (Hypotheses generation from keypoint correspondences) */
    void setCGAlgorithm (const typename boost::shared_ptr<v4r::CorrespondenceGrouping<PointT, PointT> > & alg)
    {
      cg_algorithm_ = alg;
    }

    /** \brief sets the camera pose of the input cloud */
    void setCameraPose(const Eigen::Matrix4f &tf)
    {
        pose_ = tf;
    }

    typename boost::shared_ptr<Source<PointT> >
    getDataSource () const
    {
        return rr_->getDataSource();
    }

    void recognize();

protected:
    void findChanges();

	virtual void findChangedPoints(
			pcl::PointCloud<PointT> observation,
			Eigen::Affine3f pose,
			pcl::PointCloud<PointT> &removed_points,
			pcl::PointCloud<PointT> &added_points) {
		// NO change detection by default
		// change detection must be realized by overriding this method in derived class
		// e.g. by ROS wrapper which communicates with other change detection node
		pcl::transformPointCloud(observation, added_points, pose);
	}

	bool isRemovedByChange(const PointT &keypoint, size_t origin_id);

	bool isRemovedByChange(ModelTPtr hypothesis_model, Eigen::Matrix4f transform, size_t origin_id);

	bool isPreservedByNovelty(ModelTPtr hypothesis_model, Eigen::Matrix4f transform);

	template <class Content, class Allocator>
	static void filterVector(typename std::vector<Content, Allocator> &container,
			const std::vector<bool> &mask) {
		assert(container.size() == mask.size());
		typename std::vector<Content, Allocator>::iterator ci = container.begin();
		for(std::vector<bool>::const_iterator mi = mask.begin(); mi < mask.end(); mi++) {
			if(*mi) {
				ci++;
			} else {
				ci = container.erase(ci);
			}
		}
	}

	void filterByChangesForReconstruction(typename Cloud::Ptr cloud,
			pcl::PointCloud<pcl::Normal>::Ptr normals, size_t view_id);

	void setNan(pcl::Normal &normal);
	void setNan(PointT &pt);
};
}

#endif
