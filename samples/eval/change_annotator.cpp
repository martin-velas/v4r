/*
 * change_annotator.cpp
 *
 *  Created on: 8.12.2015
 *      Author: ivelas
 */

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>

#include <v4r_config.h>
#include <v4r/common/miscellaneous.h>
#include <v4r/io/filesystem.h>

#include <boost/program_options.hpp>
#include <glog/logging.h>

#include <iostream>
#include <sstream>
#include <time.h>
#include <stdlib.h>

using namespace std;
namespace po = boost::program_options;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;

typedef struct ArgumentsT {
	string model_dir = "/media/files/TUW_DATA/TUW_dynamic_dataset_icra15/models/";
	string sequence_dir = "/media/files/TUW_DATA/tuw_dyn_data/set_00016/";
	string gt_dir = "/media/files/TUW_DATA/tuw_dyn_data_gt_changes/set_00016/";
	string output_file = "/media/files/TUW_DATA/tuw_dyn_data_gt_changes/set_00016.changes";
	string start_by_object;
} Arguments;

class View {
public:
	typedef boost::shared_ptr<View> Ptr;

	int id;
	Cloud::Ptr cloud;
	Eigen::Matrix4f pose;

	View(int id_) : id(id_), cloud(new Cloud) {
	}

	static void loadFrom(const string &dir, vector<View> &output) {
		std::vector<std::string> views;
		v4r::io::getFilesInDirectory(dir, views, "", ".*.pcd", false);
		std::sort(views.begin(), views.end());
		for (size_t i = 0; i <views.size(); i++) {
			View v(atoi(views[i].c_str()));
			const std::string fn = dir + "/" + views[i];
			LOG(INFO) << "Adding view " << fn;
			pcl::io::loadPCDFile(fn, *v.cloud);

			v.pose = v4r::RotTrans2Mat4f(v.cloud->sensor_orientation_, v.cloud->sensor_origin_);
			// reset view point otherwise pcl visualization is potentially messed up
			Eigen::Vector4f zero_origin; zero_origin[0] = zero_origin[1] = zero_origin[2] = zero_origin[3] = 0.f;
			v.cloud->sensor_orientation_ = Eigen::Quaternionf::Identity();
			v.cloud->sensor_origin_ = zero_origin;

			output.push_back(v);
		}
	}
};

class Model {
public:
	typedef boost::shared_ptr<Model> Ptr;
	typedef map<string, Model> Db;

	string name;
	Cloud::Ptr cloud;

	static void loadFrom(const string &dir, Model::Db &output) {
		// TODO
	}
};

class ObjectAnnotation {
public:
	Model::Ptr model;
	string filename;
	View::Ptr view;
	Eigen::Matrix4f relative_pose;
	int relative_id;

	static void loadFrom(const string &dir, const vector<View> &views, const Model::Db &models,
			vector<ObjectAnnotation> &output, const string &since_model = "") {
		// TODO
	}

	bool operator >= (ObjectAnnotation &other) {
		if(this->model->name.compare(other.model->name) > 0) {
			return true;
		} else if(this->view->id > other.view->id) {
			return true;
		} else {
			return this->relative_id >= other.relative_id;
		}
	}
};

class ChangeAnnotator {
public:

	ChangeAnnotator(Arguments arg) {
		View::loadFrom(arg.sequence_dir, views);
		Model::loadFrom(arg.model_dir, models);
		ObjectAnnotation::loadFrom(arg.gt_dir, views, models, annotations, arg.start_by_object);
	}

	void run() {
		// TODO
	}

private:
	vector<View> views;
	Model::Db models;
	vector<ObjectAnnotation> annotations;
};

int main(int argc, char *argv[]) {

	Arguments arg;
    po::options_description desc("Annotator of object changes in the data sequence.");
    desc.add_options()
            ("help,h", "produce help message")
            ("model_dir,m", po::value<string>(&arg.model_dir)->default_value(arg.model_dir), "Directory of the known object models")
            ("test_seq_dir,t", po::value<string>(&arg.sequence_dir)->default_value(arg.sequence_dir), "Directory of the point cloud data sequence")
            ("gt_dir,g", po::value<string>(&arg.gt_dir)->default_value(arg.gt_dir), "Directory of the GT for point cloud data sequence")
            ("output_file,o", po::value<string>(&arg.output_file)->default_value(arg.output_file), "Output file for change annotation")
    		("start_by_object,s", po::value<string>(&arg.start_by_object)->default_value(""), "Ignore alphabetically preceding objects");
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return false;
    }
    try
    {
        po::notify(vm);
    }
    catch(std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
        return false;
    }

    ChangeAnnotator annotator(arg);
    annotator.run();
}
