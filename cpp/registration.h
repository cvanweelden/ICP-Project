#include <string>

using namespace pcl;
using namespace std;

//Downsampling parameter
const float VOXEL_GRID_SIZE = 0.05; 

//FPFH feature parameters
const double NORMALS_RADIUS = 0.1; 
const double FEATURES_RADIUS = 0.5;

//FPFH allignment parameters
const double SAC_MAX_CORRESPONDENCE_DIST = 0.1; 
const double SAC_MIN_SAMPLE_DIST = 0.5;
const int SAC_MAX_ITERATIONS = 1000;

//ICP allingment parameters
const int ICP_MAX_ITERATIONS = 25;
const double ICP_MAX_CORRESPONDENCE_DISTANCE = 0.25;
const double ICP_TRANSFORMATION_EPSILON = 1e-6;
const double ICP_EUCLIDEAN_FITNESS_EPSILON = 1e-5;

enum Method {
	NONE,
	FPFH
};

PointCloud<PointXYZRGB>::Ptr loadFrame( string filepath, string filetype, 
									   float downsample_size=VOXEL_GRID_SIZE);

Eigen::Matrix4f registerFrame( PointCloud<PointXYZRGB>::Ptr frame, 
							  PointCloud<PointXYZRGB>::Ptr model,
							  Method initial_method = FPFH,
							  bool use_ICP = true);
