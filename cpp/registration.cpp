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
//#include <pcl/registration/gicp.h> Broken in 1.4.0: http://www.pcl-users.org/GICP-in-PCL-1-4-td3635906.html

using namespace std;

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
	
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outlier_filter;
	outlier_filter.setMeanK (50);
	outlier_filter.setStddevMulThresh (1.0);
	
	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZRGB> downsample_filter;
	downsample_filter.setLeafSize (0.015f, 0.015f, 0.015f);
	
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
		if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(dir + "/" + files[i], *frame) == -1)
		{
			PCL_ERROR ("Couldn't read some frame \n");
			return (-1);
		}
		outlier_filter.setInputCloud (frame);
		outlier_filter.filter (*frame);
		downsample_filter.setInputCloud (frame);
		downsample_filter.filter (*frame);
		
		//Set the ICP parameters
		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
		//pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
		//icp.setMaxCorrespondenceDistance(0.0001);
		icp.setMaximumIterations(50);
		icp.setRANSACOutlierRejectionThreshold(0.01); //default: 0.05m
		//icp.setRANSACIterations(50);
		icp.setTransformationEpsilon(0.01);
		icp.setEuclideanFitnessEpsilon(0.01);
		
		//Allign the new frame
		icp.setInputCloud(frame);
		icp.setInputTarget(model);
		std::cout << "Alligning " << files[i] << endl;
		icp.align(alligned_frame);
		
		//Some debug printing
		std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << std::endl;
		std::cout << icp.getFinalTransformation() << std::endl;
		
		//Write the alligned frame
		pcl::io::savePLYFileBinary<pcl::PointXYZRGB>(files[i].substr(0,files[i].size()-4) + "_alligned.ply", alligned_frame);
		
		*model = *frame;
	}
	    
    

  return (0);
}