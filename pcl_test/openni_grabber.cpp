#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>

#include <iostream>
#include <pcl/io/pcd_io.h>

#ifdef WIN32
# define sleep(x) Sleep((x)*1000) 
#endif

class SimpleOpenNIProcessor
{
public:
  void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
  {
	  static int frame = 0;
	  pcl::io::savePCDFileASCII ("frame" + boost::lexical_cast<std::string>(frame) + ".pcd", *cloud);
	  frame+=1;
  }
  
  void run ()
  {
    // create a new grabber for OpenNI devices
    pcl::Grabber* interface = new pcl::OpenNIGrabber();

    // make callback function from member function
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
      boost::bind (&SimpleOpenNIProcessor::cloud_cb_, this, _1);

    // connect callback function for desired signal. In this case its a point cloud with color values
    boost::signals2::connection c = interface->registerCallback (f);

    // start receiving point clouds
    interface->start ();

    // wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
    while (true)
      sleep(1);

    // stop the grabber
    interface->stop ();
  }
};

int main ()
{
SimpleOpenNIProcessor v;
v.run ();
return 0;
}
