#ifndef _VISUAL_RESULTS_STORAGE_H_
#define _VISUAL_RESULTS_STORAGE_H_

#include <pcl/io/pcd_io.h>
#include <sstream>
#include <iomanip>

namespace v4r {

class VisualResultsStorage {
public:
	typedef unsigned char uchar;

	VisualResultsStorage(const std::string &base_dir = ".", bool enabled = true) :
		base_dir(base_dir), enabled(enabled) {
	}

	void newView() {
		view_counter++;
	}

	void setBaseDir(const std::string& baseDir) {
		base_dir = baseDir;
	}

	std::string getNameOf(const std::string what, const std::string ext = ".pcd") const;

	template <class PointT>
	void savePcd(const std::string what, const pcl::PointCloud<PointT> &cloud) const {
		if(enabled) {
			std::string filename = getNameOf(what, cloud.empty() ? ".empty" : ".pcd");
			PCL_INFO("Saving [%s] to '%s'\n", what.c_str(), filename.c_str());
			if(cloud.empty()) {
				std::ofstream empty_pcd(filename.c_str());
				empty_pcd << "Point cloud of " << what << " was empty." << std::endl;
			} else {
				pcl::io::savePCDFile<PointT>(filename, cloud, true);
			}
		}
	}

	template <class PointT>
	void savePcd(const char* what, const pcl::PointCloud<PointT> &cloud) const {
		if(enabled) {
			savePcd(std::string(what), cloud);
		}
	}

        template <class PointT>
        static void copyCloudColored(const pcl::PointCloud<PointT> &src,
                        pcl::PointCloud<pcl::PointXYZRGB> &target, uchar r, uchar g, uchar b) {
                for(size_t i = 0; i < src.size(); i++) {
                        pcl::PointXYZRGB pt(r, g, b);
                        pt.x = src[i].x;
                        pt.y = src[i].y;
                        pt.z = src[i].z;
                        target.push_back(pt);
                }
        }

        static void copyCloudColorReduced(const pcl::PointCloud<pcl::PointXYZRGB> &src,
                        pcl::PointCloud<pcl::PointXYZRGB> &target) {
          pcl::PointCloud<pcl::PointXYZRGB> src_copy;
          src_copy += src;
          for(size_t i = 0; i < src_copy.size(); i++) {
            src_copy[i].r /= 2;
            src_copy[i].g /= 2;
            src_copy[i].b /= 2;
          }
          target +=  src_copy;
        }

	void setEnabled(bool enable) {
		enabled = enable;
	}

private:
	static int view_counter;
	std::string base_dir;
	bool enabled;
};

}

#endif
