#include <string>

using namespace pcl;
using namespace std;

const float VOXEL_GRID_SIZE = 0.05; 
const double NORMALS_RADIUS = 0.1; 
const double FEATURES_RADIUS = 0.5; 
const double SAC_MAX_CORRESPONDENCE_DIST = 0.1; 
const double SAC_MIN_SAMPLE_DIST = 0.5;
const int SAC_MAX_ITERATIONS = 1000;


PointCloud<PointXYZRGB>::Ptr loadFrame( string filepath, string filetype, float downsample_size=VOXEL_GRID_SIZE);

Eigen::Matrix4f registerFrame( PointCloud<PointXYZRGB>::Ptr frame, PointCloud<PointXYZRGB>::Ptr model );
