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

#include <Eigen/Geometry>

#include "registration.h"

//#include <pcl/registration/gicp.h> Broken in 1.4.0: http://www.pcl-users.org/GICP-in-PCL-1-4-td3635906.html

using namespace std;
using namespace pcl;
 
PointCloud<FPFHSignature33>::Ptr getFeaturesFPFH( PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr normals )
{	
	PointCloud<FPFHSignature33>::Ptr features = PointCloud<FPFHSignature33>::Ptr (new PointCloud<FPFHSignature33>);
	search::KdTree<pcl::PointXYZRGB>::Ptr search_method_ptr = search::KdTree<PointXYZRGB>::Ptr (new search::KdTree<PointXYZRGB>);
	FPFHEstimation<PointXYZRGB, Normal, FPFHSignature33> fpfh_est;
	fpfh_est.setInputCloud( cloud );
	fpfh_est.setInputNormals( normals );
	fpfh_est.setSearchMethod( search_method_ptr );
	fpfh_est.setRadiusSearch( FEATURES_RADIUS );
	fpfh_est.compute( *features );
	return features;
}

PointCloud<Normal>::Ptr getNormals( PointCloud<PointXYZRGB>::Ptr incloud )
{	
	PointCloud<Normal>::Ptr normalsPtr = PointCloud<Normal>::Ptr (new PointCloud<Normal>);
	NormalEstimation<PointXYZRGB, Normal> norm_est;
	norm_est.setInputCloud( incloud );
	norm_est.setRadiusSearch( NORMALS_RADIUS );
	norm_est.compute( *normalsPtr );
	return normalsPtr;
}

PointCloud<PointXYZRGB>::Ptr loadFrame( string filepath, string filetype, float downsample_size)
{
	PointCloud<PointXYZRGB>::Ptr frame (new PointCloud<PointXYZRGB>);
	
	//Read the pointcloud from file
	if ( filetype == ".ply") {
		if (io::loadPLYFile<PointXYZRGB>(filepath, *frame) == -1) {
			PCL_ERROR ("Couldn't read file\n");
		}
	} else if ( filetype == ".pcd") {
		if (io::loadPCDFile<PointXYZRGB>(filepath, *frame) == -1) {
			PCL_ERROR ("Couldn't read file\n");
		}
	}
	
	// Remove outliers
	StatisticalOutlierRemoval<PointXYZRGB> outlier_filter;
	outlier_filter.setMeanK (50);
	outlier_filter.setStddevMulThresh (1.0);
	outlier_filter.setInputCloud (frame);
	outlier_filter.filter (*frame);
	
	// Downsample using voxelgrid
	VoxelGrid<PointXYZRGB> downsample_filter;
	downsample_filter.setLeafSize (downsample_size, downsample_size, downsample_size);
	downsample_filter.setInputCloud (frame);
	downsample_filter.filter (*frame);
	
	return frame;
}

Eigen::Matrix4f registerFrame( PointCloud<PointXYZRGB>::Ptr frame,
							  PointCloud<PointXYZRGB>::Ptr model,
							  Method initial_method,
							  bool use_ICP)
{
	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
	
	if (initial_method == FPFH) {
		//Compute normals
		PointCloud<Normal>::Ptr model_normals = getNormals( model );
		PointCloud<Normal>::Ptr frame_normals = getNormals( frame );
		
		//Compute FPFH features
		PointCloud<FPFHSignature33>::Ptr model_features= getFeaturesFPFH( model, model_normals );
		PointCloud<FPFHSignature33>::Ptr frame_features= getFeaturesFPFH( frame, frame_normals );
		
		//Initialize allignment method
		SampleConsensusInitialAlignment<PointXYZRGB, PointXYZRGB, FPFHSignature33> sac_ia;
		sac_ia.setInputCloud( frame );
		sac_ia.setSourceFeatures( frame_features );
		sac_ia.setInputTarget( model );
		sac_ia.setTargetFeatures( model_features );
		
		//Set parameters for allignment and RANSAC
		sac_ia.setMaxCorrespondenceDistance( SAC_MAX_CORRESPONDENCE_DIST );
		sac_ia.setMinSampleDistance( SAC_MIN_SAMPLE_DIST );
		sac_ia.setMaximumIterations( SAC_MAX_ITERATIONS );
		
		//Allign frame using FPFH features
		sac_ia.align( *frame );
		//std::cout << "Done! MSE: " << sac_ia.getFitnessScore() << endl;
		
		//Get the transformation
		transformation = sac_ia.getFinalTransformation();
	}
	
	if (use_ICP) {
		IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
		icp.setInputCloud( frame );
		icp.setInputTarget( model );
		
		//Set the ICP parameters
		icp.setMaxCorrespondenceDistance(ICP_MAX_CORRESPONDENCE_DISTANCE);
		icp.setMaximumIterations(ICP_MAX_ITERATIONS);
		icp.setTransformationEpsilon(ICP_TRANSFORMATION_EPSILON);
		icp.setEuclideanFitnessEpsilon(ICP_EUCLIDEAN_FITNESS_EPSILON);
		
		//Refine the allignment using ICP
		icp.align( *frame );
		
		//Some debug printing
		//std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		//icp.getFitnessScore() << std::endl;
		
		transformation = icp.getFinalTransformation() * transformation;
	}
	
	std::cout << "Found transformation: " << endl << transformation << std::endl;
	
	return transformation;
	
}

/*
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
	
	//Write the model
	uint8_t tmp;
	for (size_t j=0; j < model->points.size(); j++) {
		pcl::PointXYZRGB *p = &model->points[j];
		// unpack rgb into r/g/b
		uint32_t rgb = *reinterpret_cast<int*>(&p->rgb);
		uint8_t b = (rgb >> 16) & 0x0000ff;
		uint8_t g = (rgb >> 8)  & 0x0000ff;
		uint8_t r = (rgb)       & 0x0000ff;
		// pack r/g/b into rgb
		rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
		p->rgb = *reinterpret_cast<float*>(&rgb);
	}
	pcl::io::savePLYFileBinary<pcl::PointXYZRGB>(files[0].substr(0,files[0].size()-4) + "_alligned.ply", *model);
	
	//Registration loop
	for (size_t i=1; i<files.size(); i++) {
		
		std::cout << "Current transformation: " << endl << current_transform << std::endl;
		
		//Get quaternion plus translation vector pose notation from current_transform
		Eigen::Matrix3f rotation = current_transform.block(0,0,2,2);
		Eigen::Quaternionf q(rotation);
		Eigen::Vector3f t = current_transform.block(0,3,2,3);
		
		std::cout << "As quaternion: [" << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << "]" << endl;
		std::cout << "with translation vector: " << endl << t << endl;
		
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
		
		std::cout << "Alligning " << files[i] << endl;
		Eigen::Matrix4f transformation = registerFrame(frame, model);
		alligned_frame = *frame; //FIXME: remove alligned_frame completely
		
		current_transform = current_transform * transformation;
		
		//Write the alligned frame
		uint8_t tmp;
		for (size_t j=0; j < alligned_frame.points.size(); j++) {
			pcl::PointXYZRGB *p = &alligned_frame.points[j];
			// unpack rgb into r/g/b
			uint32_t rgb = *reinterpret_cast<int*>(&p->rgb);
			uint8_t b = (rgb >> 16) & 0x0000ff;
			uint8_t g = (rgb >> 8)  & 0x0000ff;
			uint8_t r = (rgb)       & 0x0000ff;
			// pack r/g/b into rgb
			rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
			p->rgb = *reinterpret_cast<float*>(&rgb);
		}
		pcl::io::savePLYFileBinary<pcl::PointXYZRGB>(files[i].substr(0,files[i].size()-4) + "_alligned.ply", alligned_frame);
		
		
		*model = *model + alligned_frame;
		downsample_filter.setInputCloud (model);
		downsample_filter.filter (*model);
	}
	    
    

  return (0);
} */