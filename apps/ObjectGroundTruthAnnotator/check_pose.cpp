#include "pcl/visualization/pcl_visualizer.h"
#include <boost/filesystem.hpp>
#include "or_evaluator.h"
#include <pcl/filters/voxel_grid.h>
#include <v4r/recognition/model_only_source.h>
#include <v4r/common/miscellaneous.h>
#include <v4r/changedet/Visualizer3D.h>

using namespace std;
using namespace pcl;

int main(int argc, char *argv[]) {
	if(argc != 4) {
		cerr << "Expected arguments <cloud> <model> <pose>" << endl;
		return 1;
	}

	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
	io::loadPCDFile(argv[1], *cloud);
	cloud->sensor_orientation_ = Eigen::Quaternionf::Identity();
	cloud->sensor_origin_ = Eigen::Vector4f::Zero();
//	cerr << "origin:" << endl << cloud->sensor_origin_ << endl <<
//			"orientation:" << endl << cloud->sensor_orientation_.matrix() << endl;

	PointCloud<PointXYZ> model;
	io::loadPCDFile(argv[2], model);

	ifstream t_file(argv[3]);
	Eigen::Matrix4f t;
	for(int r = 0; r < 4; r++) {
		for(int c = 0; c < 4; c++) {
			t_file >> t(r, c);
		}
	}

	v4r::Visualizer3D().addColorPointCloud(cloud).addPointCloud(model, t).show();

	return 0;
}
