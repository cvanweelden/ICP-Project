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

PointCloud<Normal>::Ptr getNormals( PointCloud<PointXYZRGB>::Ptr incloud, PointCloud<PointXYZRGB>::Ptr fullcloud )
{	
	PointCloud<Normal>::Ptr normalsPtr = PointCloud<Normal>::Ptr (new PointCloud<Normal>);
	NormalEstimation<PointXYZRGB, Normal> norm_est;
	norm_est.setInputCloud( incloud );
	norm_est.setRadiusSearch( NORMALS_RADIUS );
	norm_est.compute( *normalsPtr );
	return normalsPtr;
}

PointCloud<PointXYZRGB>::Ptr loadFrame( string filepath, string filetype)
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
	
	return frame;
}

PointCloud<PointXYZRGB>::Ptr downsample(PointCloud<PointXYZRGB>::Ptr frame, float downsample_size)
{
	PointCloud<PointXYZRGB>::Ptr sampled_frame (new PointCloud<PointXYZRGB>);
	
	// Downsample using voxelgrid
	VoxelGrid<PointXYZRGB> downsample_filter;
	downsample_filter.setLeafSize (downsample_size, downsample_size, downsample_size);
	downsample_filter.setInputCloud (frame);
	downsample_filter.filter (*sampled_frame);
	
	return sampled_frame;
}

/* Registers the frame and also transforms it (changing the input frame).
 */
Eigen::Matrix4f registerFrame( PointCloud<PointXYZRGB>::Ptr frame,
							  PointCloud<PointXYZRGB>::Ptr model,
							  Method initial_method,
							  bool use_ICP, float downsample_size)
{
	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

	//Downsample the input clouds
	PointCloud<PointXYZRGB>::Ptr sampled_model = downsample(model, downsample_size);
	PointCloud<PointXYZRGB>::Ptr sampled_frame = downsample(frame, downsample_size);
	
	if (initial_method == FPFH) {
		//Compute normals
		PointCloud<Normal>::Ptr model_normals = getNormals( sampled_model, model );
		PointCloud<Normal>::Ptr frame_normals = getNormals( sampled_frame, frame );
		
		//Compute FPFH features
		PointCloud<FPFHSignature33>::Ptr model_features= getFeaturesFPFH( sampled_model, model_normals );
		PointCloud<FPFHSignature33>::Ptr frame_features= getFeaturesFPFH( sampled_frame, frame_normals );
		
		//Initialize allignment method
		SampleConsensusInitialAlignment<PointXYZRGB, PointXYZRGB, FPFHSignature33> sac_ia;
		sac_ia.setInputCloud( sampled_frame );
		sac_ia.setSourceFeatures( frame_features );
		sac_ia.setInputTarget( sampled_model );
		sac_ia.setTargetFeatures( model_features );
		
		//Set parameters for allignment and RANSAC
		sac_ia.setMaxCorrespondenceDistance( SAC_MAX_CORRESPONDENCE_DIST );
		sac_ia.setMinSampleDistance( SAC_MIN_SAMPLE_DIST );
		sac_ia.setMaximumIterations( SAC_MAX_ITERATIONS );
		
		//Allign frame using FPFH features
		sac_ia.align( *sampled_frame );
		//std::cout << "Done! MSE: " << sac_ia.getFitnessScore() << endl;
		
		//Get the transformation
		transformation = sac_ia.getFinalTransformation();
	}
	
	if (use_ICP) {
		IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
		icp.setInputCloud( sampled_frame );
		icp.setInputTarget( sampled_model );
		
		//Set the ICP parameters
		icp.setMaxCorrespondenceDistance(ICP_MAX_CORRESPONDENCE_DISTANCE);
		icp.setMaximumIterations(ICP_MAX_ITERATIONS);
		icp.setTransformationEpsilon(ICP_TRANSFORMATION_EPSILON);
		icp.setEuclideanFitnessEpsilon(ICP_EUCLIDEAN_FITNESS_EPSILON);
		
		//Refine the allignment using ICP
		icp.align( *sampled_frame );
		
		//Some debug printing
		//std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		//icp.getFitnessScore() << std::endl;
		
		transformation = icp.getFinalTransformation() * transformation;
	}
	
	std::cout << "Found transformation: " << endl << transformation << std::endl;
	pcl::transformPointCloud(*frame, *frame, transformation);
	
	return transformation;
	
}