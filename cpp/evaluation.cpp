#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <vector>
#include <limits>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/lexical_cast.hpp>
#include <boost/math/constants/constants.hpp>

#include <Eigen/Geometry>

#include "registration.h"
#include "util.h"

using namespace std;
using namespace pcl;

string pprint (Eigen::Vector3f t)
{
	stringstream s;
	s << t(0) << " " << t(1) << " " << t(2);
	return s.str();
}

string pprint (Eigen::Quaternionf q)
{
	stringstream s;
	s << q.x() << " " << q.y() << " " << q.z() << " " << q.w();
	return s.str();
}

void getGroundTruth (const char* filepath, vector<double> &timestamp_truth,
					 vector<Eigen::Quaternionf> &orientation_truth, 
					 vector<Eigen::Vector3f> &translation_truth)
{
	ifstream infile(filepath);
	
	string line;
	while (getline(infile, line))
	{
		//std::cout << "Reading line:\n" << line << endl; 
		
		size_t comment_start = line.find_first_of("#");
		if (comment_start != line.npos) {
			line.erase(comment_start);
		}
		
		//std::cout << "After removing comments:\n" << line << endl; 
		
		istringstream iss(line);
		
		double time;
		float tx, ty, tz, qx, qy, qz, qw;
		if ((iss >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) {
		
			Eigen::Vector3f t(tx, ty, tz);
			Eigen::Quaternionf q(qw, qx, qy, qz);
		
			timestamp_truth.push_back(time);
			translation_truth.push_back(t);
			orientation_truth.push_back(q);
		
			//std::cout << "Read orientation: [" << q.x() << " " << q.y() <<  " " << q.z() << " " << q.w() << "]" << endl;
			//std::cout << "at position: " << endl << t << endl;
			//std::cout << "at time: " << endl << time << endl;
		}
	}
}

void getGroundTruthPose(const double timestamp, const vector<double> &timestamp_truth,
						const vector<Eigen::Quaternionf> &orientation_truth, 
						const vector<Eigen::Vector3f> &translation_truth,
						Eigen::Quaternionf &orientation, Eigen::Vector3f &translation)
{
	//find bracketing timestamps in groundtruth
	double time;
	int i = 0;
	while (timestamp_truth[i] <= timestamp) {
		i++;
	}
	//interpolate between those timestamps
	float alpha = (timestamp - timestamp_truth[i-1])/(timestamp_truth[i]-timestamp_truth[i-1]);
	orientation = orientation_truth[i-1].slerp(alpha, orientation_truth[i]);
	translation = translation_truth[i-1] + alpha * (translation_truth[i]-translation_truth[i-1]);
	/*
	orientation = orientation_truth[i];
	translation = translation_truth[i]; */
}

int main (int argc, char** argv)
{	
	string usage = "usage: $evaluation <datadir> <groundtruth> <outputfile> [-t <filetype> (default: .ply)] [-s <max_frameskip_offset> (default: 1)] [-f <limit_frames> (default: unlimited)] [-r <registration_method{NONE,FPFH}> (default: FPFH)] [-fr <FPFH_feature_radius> (default: 0.3)] [-d <ICP_max_correspondence_distance> (default: 0.25)] [--no_icp]";
	
	int num_args = argc;
	int max_frame_distance = 1;
	int max_frames = numeric_limits<int>::max();
	string filetype = ".ply";
	Method registration_method = FPFH;
	bool use_icp = true;
	
	double fpfh_radius = 0.3;
	double icp_max_dist = 0.25;
	
	for (int i=4; i<argc; i++) {
		if (strcmp(argv[i],"-s") == 0) {
			max_frame_distance = boost::lexical_cast<int>(argv[i+1]);
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
		else if (strcmp(argv[i],"--no_icp") == 0) {
			use_icp = false;
			num_args -= 1;
		}
		else if (strcmp(argv[i],"-fr") == 0) {
			fpfh_radius = boost::lexical_cast<double>(argv[i+1]);
			num_args -= 2;
			i++;
		}
		else if (strcmp(argv[i],"-d") == 0) {
			icp_max_dist = boost::lexical_cast<double>(argv[i+1]);
			num_args -= 2;
			i++;
		}
		else {
			cout << "Warning! Unrecognized parameter: " << argv[i] << endl;
			cout << usage << endl;
		}
	}
	
	if (num_args < 4) {
		cout << usage << endl;
		return -1;
	}
	
	//Summary message at start
	cout << "Registration evaluation of frame pairs up to " << max_frame_distance
	<< " frames apart, using ";
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
	
	//Read the ground truth
	cout << "Reading ground truth" << endl;
	vector<double> timestamp_truth;
	vector<Eigen::Quaternionf> orientation_truth;
	vector<Eigen::Vector3f> translation_truth;
	getGroundTruth(argv[2], timestamp_truth, orientation_truth, translation_truth);
	
	//Read file list
	cout << "Reading file list" << endl;
    string dir = argv[1];
    vector<string> files = vector<string>();
	vector<double> timestamps = vector<double>();
    getFiles(dir, files, filetype, timestamps);
	
	ofstream outfile;
	outfile.open (argv[3]);
	
	PointCloud<PointXYZRGB>::Ptr frame1;
	Eigen::Quaternionf orientation1;
	Eigen::Vector3f translation1;
	PointCloud<PointXYZRGB>::Ptr frame2;
	Eigen::Quaternionf orientation2;
	Eigen::Vector3f translation2;
	
	cout << "Starting registration evaluation" << endl;
	for (size_t i=0; i<files.size() && i<max_frames; i++) {
		frame1 = loadFrame(files[i], filetype);
		getGroundTruthPose(timestamps[i], timestamp_truth, orientation_truth, translation_truth,
						   orientation1, translation1);
		
		for (int offset=1; offset<=max_frame_distance && i+offset<files.size(); offset++) {
			frame2 = loadFrame(files[i+offset], filetype);
			getGroundTruthPose(timestamps[i+offset], timestamp_truth, orientation_truth,
							   translation_truth, orientation2, translation2);
			

			Eigen::Matrix4f transformation = registerFrame(frame2, frame1, registration_method, use_icp, VOXEL_GRID_SIZE, fpfh_radius, icp_max_dist);
			cout << "Registered frame " << i+offset << " to frame " << i << endl;
			
			Eigen::Matrix3f rotation;
			for (int r=0; r<3; r++) {
				for (int c=0; c<3; c++) {
					rotation(r,c) = transformation(r,c);
				}
			}
			Eigen::Quaternionf q(rotation);
			Eigen::Vector3f t;
			for (int r=0; r<3; r++) {
				t(r) = transformation(r,3);
			}
		
			string s = pprint(translation1) + " " + pprint(orientation1) + " "
			+ pprint(translation2) + " " + pprint(orientation2) + " "
			+ pprint(t) + " " + pprint(q);
			
			outfile << i << " " << i+offset << " " << s << endl;
		}
	}
	
    return 0;	
}