/*
 * change_annotator.cpp
 *
 *  Created on: 8.12.2015
 *      Author: ivelas
 */

#include <boost/make_shared.hpp>
#include <boost/algorithm/string.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

#include <v4r_config.h>
#include <v4r/common/miscellaneous.h>
#include <v4r/io/filesystem.h>

#include <boost/program_options.hpp>
#include <glog/logging.h>

#include <cv.h>

#include <iostream>
#include <sstream>
#include <time.h>
#include <stdlib.h>
#include <algorithm>

using namespace std;
using namespace cv;
namespace po = boost::program_options;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;

typedef struct ArgumentsT {
	string model_dir;
	string sequence_dir;
	string gt_dir;
	string output_file;
	string start_by_object;
	string preview_only_path;

	ArgumentsT() :
		model_dir("/media/files/TUW_DATA/TUW_dynamic_dataset_icra15/models/"),
		sequence_dir("/media/files/TUW_DATA/tuw_dyn_data/set_00016/"),
		gt_dir("/media/files/TUW_DATA/tuw_dyn_data_gt_changes/set_00016/"),
		output_file("/media/files/TUW_DATA/tuw_dyn_data_gt_changes/set_00016.changes"),
		start_by_object(""),
		preview_only_path("") {
	}
} Arguments;

class View {
public:
	typedef boost::shared_ptr<View> Ptr;
	typedef boost::shared_ptr<const View> ConstPtr;
	typedef map<int, View> Db;

	int id;
	Cloud::Ptr cloud;
	Eigen::Matrix4f pose;

	View(int id_) : id(id_), cloud(new Cloud) {
	}

	static void loadFrom(const string &dir, View::Db &output) {
		std::vector<std::string> view_files;
		v4r::io::getFilesInDirectory(dir, view_files, "", ".*.pcd", false);
		std::sort(view_files.begin(), view_files.end());
		for (size_t i = 0; i < view_files.size(); i++) {
			View v(getId(view_files[i]));
			const std::string fn = dir + "/" + view_files[i];
			LOG(INFO) << "Adding view " << fn << " (id " << v.id << ")";
			pcl::io::loadPCDFile(fn, *v.cloud);

			v.pose = v4r::RotTrans2Mat4f(v.cloud->sensor_orientation_, v.cloud->sensor_origin_);
			// reset view point otherwise pcl visualization is potentially messed up
			Eigen::Vector4f zero_origin; zero_origin[0] = zero_origin[1] = zero_origin[2] = zero_origin[3] = 0.f;
			v.cloud->sensor_orientation_ = Eigen::Quaternionf::Identity();
			v.cloud->sensor_origin_ = zero_origin;

			output.insert(make_pair(v.id, v));
		}
	}

	static int getId(const string &filename) {
		size_t find_pos = filename.find("cloud_");
		if(find_pos == string::npos) {
			return atoi(filename.c_str());
		} else {
			return atoi(filename.substr(6).c_str());
		}
	}
};

class Model {
public:
	typedef boost::shared_ptr<Model> Ptr;
	typedef boost::shared_ptr<const Model> ConstPtr;
	typedef map<string, Model> Db;

	string name;
	Cloud::Ptr cloud;

	Model(const string &name_) : name(name_), cloud(new Cloud) {
	}

	static void loadFrom(const string &dir, Model::Db &output) {
		std::vector<std::string> model_files;
		v4r::io::getFilesInDirectory(dir, model_files, "", ".*.pcd", false);
		for(size_t i = 0; i < model_files.size(); i++) {
			string name = model_files[i].substr(0, model_files[i].find('.'));
			Model m(name);
			const std::string fn = dir + "/" + model_files[i];
			LOG(INFO) << "Adding model " << fn << " (id " << m.name << ")";

			pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_with_normals;
			pcl::io::loadPCDFile(fn, cloud_with_normals);
			pcl::copyPointCloud(cloud_with_normals, *m.cloud);

			/*pcl::visualization::PCLVisualizer vis("Model loaded");
			vis.addCoordinateSystem(0.5);
			vis.initCameraParameters();
		    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_vis(m.cloud);
		    std::string id = "cloud";
		    vis.addPointCloud<pcl::PointXYZRGB>(m.cloud, rgb_vis, id);
		    vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id);
		    vis.spin();*/

		    output.insert(make_pair(name, m));
		}
	}
};

class ObjectAnnotation {
public:
	string filename;
	int relative_id;
	const Model *model;
	const View *view;
	Eigen::Matrix4f relative_pose;

	ObjectAnnotation(string filename_, int relative_id_, const Model *model_, const View *view_) :
		filename(filename_), relative_id(relative_id_), model(model_), view(view_) {
	}

	static void loadFrom(const string &dir, const View::Db &views, const Model::Db &models,
			vector<ObjectAnnotation> &output, const string &since_model = "") {
		std::vector<std::string> gt_files;
		v4r::io::getFilesInDirectory(dir, gt_files, "", ".*.txt", false);
		for(size_t i = 0; i < gt_files.size(); i++) {
			int view_id, rel_id;
			string model_name;
			splitFilename(gt_files[i], view_id, model_name, rel_id);
			if(gt_files[i].find("occlusion") != string::npos ||				// ignore occlusion files
					gt_files[i].find("transformation") != string::npos ||	// ignore transformation files
					model_name < since_model) {							// skip models of previous objects (e.g. already done)
				continue;
			}
			LOG(INFO) << "Reading annotation " << dir << "/" << gt_files[i] << " (view: " << view_id << ", name: '" << model_name << "' rel_id: " << rel_id << ")";

			ObjectAnnotation ann(gt_files[i], rel_id, &models.find(model_name)->second, &views.find(view_id)->second);
			ann.readPoseFrom(dir + "/" + gt_files[i]);

			output.push_back(ann);
		}
		sort(output.begin(), output.end());
	}

	static void splitFilename(const string &filename, int &view_id, string &object_name, int &relative_id) {
		std::vector<std::string> tokens;
		boost::split(tokens, filename, boost::is_any_of("_."));

		int to_skip = (filename.find("cloud_") == 0) ? 1 : 0;

		// e.g. cloud_10_jasmine_green_tea_0.txt
		view_id = atoi(tokens[to_skip].c_str());

		std::vector<std::string>::iterator rel_id_token = tokens.begin() + tokens.size() - 2;
		relative_id = atoi(rel_id_token->c_str());

		std::vector<std::string>::iterator name_start = tokens.begin() + to_skip + 1;
		std::vector<std::string> name_tokens(name_start, rel_id_token);
		object_name = boost::join(name_tokens, "_");
	}

	void readPoseFrom(const string &fn) {
		std::ifstream f(fn.c_str());
		if(!f.is_open()) {
		  std::cerr << "Unable to read matrix: " << filename << std::endl;
		  exit(1);
		}

		for(int i = 0; i < 16; i++) {
			f >> relative_pose(i/4, i%4);
		}
		f.close();
	}

	bool operator < (const ObjectAnnotation &other) const {
		if(this->model->name != other.model->name) {
			return this->model->name < other.model->name;
		} else if(this->view->id != other.view->id) {
			return this->view->id < other.view->id;
		} else {
			return this->relative_id < other.relative_id;
		}
	}

	void getVisualization(Cloud &output, bool highlight_r = true, bool highlight_g = true, bool highlight_b = true) const {
		pcl::transformPointCloud(*view->cloud, output, view->pose);
		for(Cloud::iterator pt = output.begin(); pt < output.end(); pt++) {
			pt->r /= 2;
			pt->g /= 2;
			pt->b /= 2;
		}

		Cloud model_transformed;
		pcl::transformPointCloud(*model->cloud, model_transformed, view->pose * relative_pose);
		for(Cloud::iterator pt = model_transformed.begin(); pt < model_transformed.end(); pt++) {
			if(highlight_r)
				pt->r = min((pt->r*1.5), 255.0);
			if(highlight_g)
				pt->g = min((pt->g*1.5), 255.0);
			if(highlight_b)
				pt->b = min((pt->b*1.5), 255.0);
		}
		output += model_transformed;
	}
};

class ChangeAnnotator;

class ChangeAnnotatorGui {
public:
	ChangeAnnotatorGui();

	bool testLink(const ObjectAnnotation &prev_ann, const ObjectAnnotation &current_ann);

	void renderAnnotations();

	void keyboardEventCallback(const pcl::visualization::KeyboardEvent &event);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	int viewport_prev;
	int viewport_current;
	bool link_accepted;

	const ObjectAnnotation *prev_annotation;
	const ObjectAnnotation *currrent_annotation;
};

class ChangeAnnotator {
public:

	ChangeAnnotator(Arguments arg) {
		View::loadFrom(arg.sequence_dir, views);
		Model::loadFrom(arg.model_dir, models);
		ObjectAnnotation::loadFrom(arg.gt_dir, views, models, annotations, arg.start_by_object);
		out_file.open(arg.output_file.c_str());
		if(!out_file.is_open()) {
			perror(arg.output_file.c_str());
			exit(1);
		}
	}

	void run() {
		ObjectAnnotation prev_ann = *annotations.begin();
		out_file << prev_ann.filename;
		vector<ObjectAnnotation>::iterator current_ann = annotations.erase(annotations.begin());
		ChangeAnnotatorGui gui;
		while(!annotations.empty()) {

			/*for(vector<ObjectAnnotation>::iterator it = annotations.begin(); it < annotations.end(); it++) {
				cerr << it->filename << "("  << it->model->name << "," << it->view->id << "," << it->relative_id << ");  ";
			}
			cerr << endl << endl;*/

			if(prev_ann.model->name != current_ann->model->name) {
				LOG(INFO) << "Unable to link " << prev_ann.filename << " & " << current_ann->filename << " (different objects)";
				prev_ann = *annotations.begin();
				out_file << endl << prev_ann.filename;
				current_ann = annotations.erase(annotations.begin());
			}
			else if(gui.testLink(prev_ann, *current_ann)) {
				LOG(INFO) << "Linking " << prev_ann.filename << " & " << current_ann->filename;
				out_file << " " << current_ann->filename;
				prev_ann = *current_ann;
				current_ann = annotations.erase(current_ann);
			} else {
				LOG(INFO) << "No link found for " << prev_ann.filename << " & " << current_ann->filename;
				if(current_ann == annotations.end()) {
					prev_ann = *annotations.begin();
					out_file << endl << prev_ann.filename;
					current_ann = annotations.erase(annotations.begin());
				} else {
					current_ann++;
				}
			}
		}
		out_file << endl;
	}

private:
	View::Db views;
	Model::Db models;
	vector<ObjectAnnotation> annotations;
	ofstream out_file;
};

ChangeAnnotatorGui::ChangeAnnotatorGui() : viewer(new pcl::visualization::PCLVisualizer("Change Annotator")),
		viewport_prev(0), viewport_current(1), link_accepted(true), prev_annotation(NULL), currrent_annotation(NULL) {

	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, viewport_prev);
	viewer->setBackgroundColor(0, 0, 0, viewport_prev);

	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, viewport_current);
	viewer->setBackgroundColor(0, 0, 0, viewport_current);
}

class Stopwatch {
public:

	Stopwatch() {
		start();
	}

	void start() {
		start_time = clock();
	}

	// [sec]
	double elapsed() {
		return double(clock() - start_time) / CLOCKS_PER_SEC;
	}

protected:
	clock_t start_time;
};

void ChangeAnnotatorGui::keyboardEventCallback(const pcl::visualization::KeyboardEvent &event)
{
	static Stopwatch stopwatch;

	std::string key = event.getKeySym();
	if ((stopwatch.elapsed() > 0.1) && event.keyDown() && (key == "a" || key == "d")) {
		stopwatch.start();
		link_accepted = (key == "a");
		cerr << "Link is " << (link_accepted ? "accepted" : "declined") << endl;
		renderAnnotations();
		viewer->spin();
	}
}

bool ChangeAnnotatorGui::testLink(const ObjectAnnotation &prev_ann, const ObjectAnnotation &current_ann) {
	prev_annotation = &prev_ann;
	currrent_annotation = &current_ann;
	renderAnnotations();

	viewer->registerKeyboardCallback(boost::bind(&ChangeAnnotatorGui::keyboardEventCallback, this, _1));

	viewer->spin();
	return link_accepted;
}

void ChangeAnnotatorGui::renderAnnotations() {
	assert(prev_annotation != NULL);
	assert(currrent_annotation != NULL);

	viewer->removeAllPointClouds(viewport_prev);
	viewer->removeAllPointClouds(viewport_current);

	Cloud::Ptr prev_vis(new Cloud);
	prev_annotation->getVisualization(*prev_vis);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_prev(prev_vis);
	viewer->addPointCloud<pcl::PointXYZRGB>(prev_vis, handler_prev, "previous_view", viewport_prev);

	Cloud::Ptr current_vis(new Cloud);
	currrent_annotation->getVisualization(*current_vis, !link_accepted, link_accepted, false);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_current(current_vis);
	viewer->addPointCloud<pcl::PointXYZRGB> (current_vis, handler_current, "current_view", viewport_current);
}

class ChangeAnnotationPreview {
public:
	ChangeAnnotationPreview(Arguments arg) : ann_dir(arg.gt_dir), rng(cv::theRNG()) {
		View::loadFrom(arg.sequence_dir, views);
		Model::loadFrom(arg.model_dir, models);
		in_file.open(arg.preview_only_path.c_str());
		if(!in_file.is_open()) {
			perror(arg.output_file.c_str());
			exit(1);
		}
		for(int i = 0; i < ROWS*COLS; i++) {
			viewports.push_back(i);
			vis_clouds.push_back(Cloud::Ptr(new Cloud));
		}
	}

	void run() {
		fillByViews();

		bool finished = false;
		while(!finished) {
			vector<string> annotation_sequence = getSequence();
			uchar r = rng(256);
			uchar g = rng(256);
			uchar b = rng(256);
			for(vector<string>::iterator ann_file = annotation_sequence.begin(); ann_file < annotation_sequence.end(); ann_file++) {
				if(ann_file->empty()) {
					finished = true;
					break;
				}
				int view_id, rel_id;
				string model_name;
				ObjectAnnotation::splitFilename(*ann_file, view_id, model_name, rel_id);

				ObjectAnnotation ann(*ann_file, rel_id, &models.find(model_name)->second, &views.find(view_id)->second);
				ann.readPoseFrom(ann_dir + "/" + *ann_file);

				Cloud model_transformed;
				pcl::transformPointCloud(*ann.model->cloud, model_transformed, ann.view->pose * ann.relative_pose);
				for(Cloud::iterator pt = model_transformed.begin(); pt < model_transformed.end(); pt++) {
					pt->r = r;
					pt->g = g;
					pt->b = b;
				}
				*vis_clouds[view_id] += model_transformed;
			}
		}
		show();
	}

	void fillByViews() {
		for(int i = 0; i < ROWS*COLS; i++) {
			View::Db::iterator view = views.find(i);
			if(view != views.end()) {
				pcl::transformPointCloud(*view->second.cloud, *vis_clouds[i], view->second.pose);
			}
		}
	}

	vector<string> getSequence() {
		vector<string> annotation_sequence;
		string line;
		getline(in_file, line);
		boost::split(annotation_sequence, line, boost::is_any_of(" "));
		return annotation_sequence;
	}

	void show() {
		pcl::visualization::PCLVisualizer vis("Changes preview");
		vis.initCameraParameters();

		float width = 1.0/COLS;
		float height = 1.0/ROWS;
		for(int r = 0; r < ROWS; r++) {
			for(int c = 0; c < COLS; c++) {
				int i = (r*COLS) + c;
				vis.createViewPort(width*c, height*r, width*(c+1), height*(r+1), viewports[i]);
				vis.setBackgroundColor(0, 0, 0, viewports[i]);
				pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_prev(vis_clouds[i]);
				stringstream ss;
				ss << "view_" << i;
				vis.addPointCloud<pcl::PointXYZRGB>(vis_clouds[i], handler_prev, ss.str(), viewports[i]);
			}
		}

	    vis.spin();
	}

private:
	View::Db views;
	Model::Db models;
	ifstream in_file;
	string ann_dir;
	cv::RNG& rng;

	static const int COLS = 6;
	static const int ROWS = 6;
	vector<int> viewports;
	vector<Cloud::Ptr> vis_clouds;
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
            ("start_by_object,s", po::value<string>(&arg.start_by_object)->default_value(""), "Ignore alphabetically preceding objects")
            ("preview_only_path,p", po::value<string>(&arg.preview_only_path)->default_value(""), "Only preview annotation.");
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

	if (arg.preview_only_path.empty()) {
		ChangeAnnotator annotator(arg);
		annotator.run();
	} else {
		ChangeAnnotationPreview preview(arg);
		preview.run();
	}
}
