#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>

#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>

#include "pcl/kdtree/kdtree_flann.h"


//#include <pcl/registration/gicp.h> Broken in 1.4.0: http://www.pcl-users.org/GICP-in-PCL-1-4-td3635906.html

using namespace std;

const float VOXEL_GRID_SIZE = 0.01; 
const double NORMALS_RADIUS = 0.03; 
const double FEATURES_RADIUS = 0.08; 
const double SAC_MAX_CORRESPONDENCE_DIST = 1; 
const double SAC_MIN_SAMPLE_DIST = 0.01;
const int SAC_MAX_ITERATIONS = 50;

bool hasEnding (std::string const &fullString, std::string const &ending)
{
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

int getfiles (string dir, vector<string> &files, string const &extension)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL) {
        cout << "Error(" << errno << ") opening " << dir << endl;
        return errno;
    }

    while ((dirp = readdir(dp)) != NULL) {
        if (hasEnding(dirp->d_name, extension))
            files.push_back(string(dirp->d_name));
    }
    closedir(dp);
    return 0;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr getFeatures( pcl::PointCloud<pcl::PointXYZRGB>::Ptr
												  cloud, pcl::PointCloud<pcl::Normal>::Ptr normals ) {
	
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr features =
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr (new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr search_method_ptr =
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>);
	pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
	fpfh_est.setInputCloud( cloud );
	fpfh_est.setInputNormals( normals );
	fpfh_est.setSearchMethod( search_method_ptr );
	fpfh_est.setRadiusSearch( FEATURES_RADIUS );
	fpfh_est.compute( *features );
	return features;
}

pcl::PointCloud<pcl::Normal>::Ptr getNormals( pcl::PointCloud<pcl::PointXYZRGB>::Ptr incloud ) {
	
	pcl::PointCloud<pcl::Normal>::Ptr normalsPtr = pcl::PointCloud<pcl::Normal>::Ptr (new
																  pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
	norm_est.setInputCloud( incloud );
	norm_est.setRadiusSearch( NORMALS_RADIUS );
	norm_est.compute( *normalsPtr );
	return normalsPtr;
}

int main (int argc, char** argv)
{
	//Read file list
    string dir = argv[1];
    vector<string> files = vector<string>();
    getfiles(dir, files, ".ply");
    std::cout << "Reading " << files[0] << endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr model (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr frame (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB> alligned_frame;
	Eigen::Matrix4f current_transform = Eigen::Matrix4f::Identity();
	
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outlier_filter;
	outlier_filter.setMeanK (50);
	outlier_filter.setStddevMulThresh (1.0);
	
	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZRGB> downsample_filter;
	downsample_filter.setLeafSize (VOXEL_GRID_SIZE, VOXEL_GRID_SIZE, VOXEL_GRID_SIZE);
	
	//Initialize the model
	if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(dir + "/" + files[0], *model) == -1)
	{
	     PCL_ERROR ("Couldn't read first frame \n");
	     return (-1);
	}
	outlier_filter.setInputCloud (model);
	outlier_filter.filter (*model);
	downsample_filter.setInputCloud (model);
	downsample_filter.filter (*model);
	pcl::io::savePLYFileBinary<pcl::PointXYZRGB>(files[0].substr(0,files[0].size()-4) + "_alligned.ply", *model);
	
	//Registration loop
	for (size_t i=1; i<files.size(); i++) {
		std::cout << "Current transformation: " << current_transform << std::endl;
		
		std::cout << "Reading " << files[i] << endl;
		//Load and filter the incoming frames
		if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(dir + "/" + files[i], *frame) == -1)
		{
			PCL_ERROR ("Couldn't read some frame \n");
			return (-1);
		}
		outlier_filter.setInputCloud (frame);
		outlier_filter.filter (*frame);
		downsample_filter.setInputCloud (frame);
		downsample_filter.filter (*frame);
		
		//Transform the new frame to the current estimate
		pcl::transformPointCloud(*frame, *frame, current_transform);
		
		//Compute normals
		pcl::PointCloud<pcl::Normal>::Ptr model_normals = getNormals( model );
		pcl::PointCloud<pcl::Normal>::Ptr frame_normals = getNormals( frame );
		
		//Compute FPFH features
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr model_features= getFeatures( model, model_normals );
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr frame_features= getFeatures( frame, frame_normals );
		
		std::cout << "Alligning " << files[i] << endl;
		//Allign the new frame using FPFH features and RANSAC
		pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB,
		pcl::FPFHSignature33> sac_ia;
        sac_ia.setInputCloud( frame );
        sac_ia.setSourceFeatures( frame_features );
        sac_ia.setInputTarget( model );
        sac_ia.setTargetFeatures( model_features );
		sac_ia.setMaxCorrespondenceDistance( SAC_MAX_CORRESPONDENCE_DIST );
		sac_ia.setMinSampleDistance( SAC_MIN_SAMPLE_DIST );
        //sac_ia.setMaximumIterations( SAC_MAX_ITERATIONS );
        sac_ia.align( alligned_frame );
		std::cout << "Done! MSE: " << sac_ia.getFitnessScore() << endl;
		
		//Get the transformation
		Eigen::Matrix4f final_transformation = sac_ia.getFinalTransformation();
		
		//std::cout << "Refining allignment of " << files[i] << endl;
		//Set the ICP parameters
		//pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
		//pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
		//icp.setMaxCorrespondenceDistance(0.002);
		//icp.setMaximumIterations(50);
		//icp.setRANSACOutlierRejectionThreshold(0.01); //default: 0.05m
		//icp.setRANSACIterations(50);
		//icp.setTransformationEpsilon(0.001);
		//icp.setEuclideanFitnessEpsilon(0.001);
		
		//Refine the allignment using ICP
		//icp.setInputCloud(frame);
		//icp.setInputTarget(model);
		//icp.align(alligned_frame);
		//pcl::transformPointCloud(*frame, *frame, current_transform);
		
		//Some debug printing
		//std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		//icp.getFitnessScore() << std::endl;
		std::cout << "Found transformation: " << final_transformation << std::endl;
		
		current_transform = current_transform * final_transformation;
		
		//Write the alligned frame
		pcl::io::savePLYFileBinary<pcl::PointXYZRGB>(files[i].substr(0,files[i].size()-4) + "_alligned.ply", alligned_frame);
		
		*model = alligned_frame;
	}
	    
    

  return (0);
}