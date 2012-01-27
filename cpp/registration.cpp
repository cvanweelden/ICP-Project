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
    string dir = argv[1];
    vector<string> files = vector<string>();
    getfiles(dir, files, ".ply");
    std::cout << "Reading " << files[1] << endl;

    sensor_msgs::PointCloud2::Ptr cloud (new sensor_msgs::PointCloud2 ());
    
    pcl::PLYReader reader;
    reader.read(dir + "/" + files[1], *cloud);
    
    std::cout << "Read " << cloud->width * cloud ->height << " points." << endl;
  // sensor_msgs::PointCloud2::Ptr cloud (new sensor_msgs::PointCloud2 ());
  //   sensor_msgs::PointCloud2::Ptr cloud_filtered (new sensor_msgs::PointCloud2 ());
  // 
  //   // Fill in the cloud data
  //   pcl::PCDReader reader;
  //   // Replace the path below with the path where you saved your file
  //   reader.read ("table_scene_lms400.pcd", *cloud); // Remember to download the file first!
  // 
  //   std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
  //        << " data points (" << pcl::getFieldsList (*cloud) << ").";
  // 
  //   // Create the filtering object
  //   pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
  //   sor.setInputCloud (cloud);
  //   sor.setLeafSize (0.01f, 0.01f, 0.01f);
  //   sor.filter (*cloud_filtered);
  // 
  //   std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
  //        << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";
  // 
  //   pcl::PLYWriter writer;
  //   writer.write ("downsampled.ply", *cloud_filtered, 
  //          Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false, true);

  return (0);
}