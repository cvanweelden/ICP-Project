#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <vector>
#include <limits>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/filters/voxel_grid.h>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include "registration.h"
#include "util.h"

using namespace std;
using namespace pcl;

enum ModelMode {
	PREVIOUS,
	RESAMPLE,
	ADD
};

void saveToPLY( PointCloud<PointXYZRGB>::Ptr cloud, string filepath )
{
	uint8_t tmp;
	for (size_t j=0; j < cloud->points.size(); j++) {
		PointXYZRGB *p = &cloud->points[j];
		// unpack rgb into r/g/b
		uint32_t rgb = *reinterpret_cast<int*>(&p->rgb);
		uint8_t b = (rgb >> 16) & 0x0000ff;
		uint8_t g = (rgb >> 8)  & 0x0000ff;
		uint8_t r = (rgb)       & 0x0000ff;
		// pack r/g/b into rgb
		rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
		p->rgb = *reinterpret_cast<float*>(&rgb);
	}
	io::savePLYFileBinary<PointXYZRGB> (filepath, *cloud);
}

int main (int argc, char** argv)
{	
	string usage = "usage: $register <datadir> <outputdir> [-t <filetype> (default: .ply)] [-s <frameskip> (default: 0)] [-f <limit_frames> (default: unlimited)] [-r <registration_method{NONE,FPFH}> (default: FPFH)] [-m <model_mode{PREVIOUS,RESAMPLE,ADD}> (default: PREVIOUS)] [--no_icp]";
	
	int num_args = argc;
	int offset = 1;
	int max_frames = numeric_limits<int>::max();
	string filetype = ".ply";
	Method registration_method = FPFH;
	ModelMode model_mode = PREVIOUS;
	bool use_icp = true;
	
	for (int i=4; i<argc; i++) {
		if (strcmp(argv[i],"-s") == 0) {
			offset = 1 + boost::lexical_cast<int>(argv[i+1]);
			num_args -= 2;
			i++;
		}
		else if (strcmp(argv[i],"-f") == 0) {
			max_frames = boost::lexical_cast<int>(argv[i+1]);
			num_args -= 2;
			i++;
		}
		else if (strcmp(argv[i],"-t") == 0) {
			filetype = argv[i+1];
			num_args -= 2;
			i++;
		}
		else if (strcmp(argv[i],"-r") == 0) {
			if (strcmp(argv[i+1],"NONE") == 0) {
				registration_method = NONE;
			}
			num_args -= 2;
			i++;
		}
		else if (strcmp(argv[i],"-m") == 0) {
			if (strcmp(argv[i+1],"RESAMPLE") == 0) {
				model_mode = RESAMPLE;
			}
			else if (strcmp(argv[i+1],"ADD") == 0) {
				model_mode = ADD;
			}
			num_args -= 2;
			i++;
		}
		else if (strcmp(argv[i],"--no_icp") == 0) {
			use_icp = false;
			num_args -= 1;
		}
		else {
			cout << "Warning! Unrecognized parameter: " << argv[i] << endl;
			cout << usage << endl;
		}
	}
	
	if (num_args < 3) {
		cout << usage << endl;
		return -1;
	}
	
	//Summary message at start
	cout << "Registration with frameskip " << offset-1
	<< ", using ";
	if (model_mode == PREVIOUS) {
		cout << "previous frame ";
	}
	if (model_mode == RESAMPLE) {
		cout << "resampled concatenated frames ";
	}
	if (model_mode == ADD) {
		cout << "concatenated frames ";
	}
	cout << "as registration model and ";
	if (registration_method == NONE) {
		cout << "no initial guess";
	}
	if (registration_method == FPFH) {
		cout << "FPFH features";
	}
	if (use_icp) {
		cout << " with ICP";
	}
	cout << " for transformation estimation." << endl;
	
	//Read file list
	cout << "Reading file list" << endl;
    string dir = argv[1];
    vector<string> files = vector<string>();
	vector<double> timestamps = vector<double>();
    getFiles(dir, files, filetype, timestamps);
	
	//Create output directory
	if (!(boost::filesystem::exists(argv[2]))) {
		if(!boost::filesystem::create_directory(argv[2])) {
			cout << "Error! Could not create output directory." << endl;
			return -1;
		}
	}
	string outdir = argv[2];
	
	//Initialize the model
	PointCloud<PointXYZRGB>::Ptr model = loadFrame(files[0], filetype);
	saveToPLY(model, outdir + files[0].substr(0,files[0].size()-4));
	
	//Initialize transformation from points in frame to points in model
	Eigen::Matrix4f current_transformation = Eigen::Matrix4f::Identity();
	
	for (size_t i=1; i<files.size() && i<=max_frames; i+=offset) {
		//load frame
		PointCloud<PointXYZRGB>::Ptr frame = loadFrame(files[i], filetype);
		//transform according to current transformation
		transformPointCloud(*frame, *frame, current_transformation);
		
		//Do the registration and update the transformation
		Eigen::Matrix4f transformation = registerFrame(frame, model, registration_method, use_icp);
		current_transformation = transformation * current_transformation;
		
		//Write the registered frame:
		saveToPLY(frame, outdir + files[i].substr(0,files[i].size()-4));
		
		//Get the model for the next registration step
		if (model_mode == PREVIOUS) {
			*model = *frame;
		}
		else if (model_mode == ADD) {
			*model = *model + *frame;
		}
		else if (model_mode == RESAMPLE) {
			*model = *model + *frame;
			VoxelGrid<PointXYZRGB> downsample_filter;
			downsample_filter.setLeafSize (VOXEL_GRID_SIZE, VOXEL_GRID_SIZE, VOXEL_GRID_SIZE);
			downsample_filter.setInputCloud (model);
			downsample_filter.filter (*model);
		}
	}
	
	
    return 0;	
}