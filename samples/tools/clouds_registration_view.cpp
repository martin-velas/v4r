#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include <v4r/changedet/Visualizer3D.h>

using namespace pcl;
using namespace std;

int main(int argc, char** argv) {

	PointCloud<PointXYZRGB>::Ptr scene(new PointCloud<PointXYZRGB>());

	for(int i = 1; i < argc; i++) {
		string filename(argv[i]);
		cerr << "Loading: " << filename << endl;

		PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>());
		pcl::io::loadPCDFile (filename, *cloud);

		std::ifstream poses_file((filename + string(".transform")).c_str());
		if(!poses_file.is_open()) {
			PCL_ERROR("Unable to open poses file\n");
		}
		Eigen::Matrix4f t;
		for(int r = 0; r < 4; r++) {
			for(int c = 0; c < 4; c++) {
				float n;
				poses_file >> n;
				t(r, c) = n;
			}
		}

		transformPointCloud(*cloud, *cloud, t);
		*scene += *cloud;
	}

	pcl::VoxelGrid<PointXYZRGB> grid;
	grid.setLeafSize(0.01, 0.01, 0.01);
	grid.setInputCloud(scene);
	grid.filter(*scene);

	v4r::Visualizer3D().addColorPointCloud(scene).show();

	return 0;
}
