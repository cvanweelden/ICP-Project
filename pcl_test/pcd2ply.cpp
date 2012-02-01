#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/passthrough.h>

#ifdef WIN32
# define sleep(x) Sleep((x)*1000) 
#endif

int main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	
	std::cout << "Reading: " << argv[1] << "\n";
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("test.pcd", *cloud) == 0) {
		std::cout << "Read: " << cloud->points.size() << " points.\n";
	}
	else {
		return 1;
	}
	
	for (size_t i = 0; i < cloud->points.size(); i++) {
		uint32_t rgb = *reinterpret_cast<int*>(&cloud->points[i].rgb);
		uint8_t r = (rgb >> 8) & 0x0000ff;
		uint8_t g = (rgb >> 16)  & 0x0000ff;
		uint8_t b = (rgb >> 24)     & 0x0000ff;
		
		printf("%x %x %x\n", r, g, b);
		
		//cloud->points[i].rgb = ;
	}
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (-DBL_MAX, DBL_MAX);
	pass.filter (*filtered_cloud);
	
	std::cout << "Writing " << filtered_cloud->points.size() << " points.\n";
	return pcl::io::savePLYFileBinary<pcl::PointXYZRGB>("test.ply", *filtered_cloud);

}
