#include <v4r/recognition/boost_graph_extension.h>

namespace v4r
{

template<typename PointT>
View<PointT>::View () :
	scene_f_( new pcl::PointCloud<PointT> ),
	scene_normals_( new pcl::PointCloud<pcl::Normal> ),
	// kp_normals_( new pcl::PointCloud<pcl::Normal> ),
	absolute_pose_(Eigen::Matrix4f::Identity()),
	has_been_hopped_(false),
	cumulative_weight_to_new_vrtx_(0)
{
}

}
