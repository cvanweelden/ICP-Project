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

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include "registration.h"
#include "util.h"

using namespace std;
using namespace pcl;

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
	string usage = "usage: $register <datadir> <outputdir> [-t <filetype> (default: .ply)] [-d <frame_distance> (default: 1)] [-f <limit_frames> (default: unlimited)] [-m <registration_method{NONE,FPFH}> (default: FPFH)] [--no_icp]";
	
	int num_args = argc;
	int frame_distance = 1;
	int max_frames = numeric_limits<int>::max();
	string filetype = ".ply";
	Method registration_method = FPFH;
	bool use_icp = true;
	
	for (int i=4; i<argc; i++) {
		if (strcmp(argv[i],"-d") == 0) {
			frame_distance = boost::lexical_cast<int>(argv[i+1]);
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
		else if (strcmp(argv[i],"-m") == 0) {
			if (strcmp(argv[i+1],"NONE") == 0) {
				registration_method = NONE;
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
	cout << "Registration with frameskip " << frame_distance-1
	<< ", using ";
	if (registration_method == NONE) {
		cout << "no initial guess";
	}
	if (registration_method == FPFH) {
		cout << "FPFH features";
	}
	if (use_icp) {
		cout << " and ICP";
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
	PointCloud<PointXYZRGB>::Ptr model = loadFrame(files[0], filetype);;
	saveToPLY(model, outdir + files[0].substr(0,files[0].size()-4));
	
	PointCloud<PointXYZRGB>::Ptr frame;
	
	
	
    return 0;	
}