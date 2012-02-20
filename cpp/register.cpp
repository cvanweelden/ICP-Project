#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include "registration.h"
#include "util.h"

using namespace std;
using namespace pcl;

void saveToPLY( PointCloud<PointXYZRGB>::Ptr cloud, string filepath )
{
	uint8_t tmp;
	for (size_t j=0; j < cloud->points.size(); j++) {
		PointXYZRGB *p = &cloud->points[j];
		// unpack rgb into r/g/b
		uint32_t rgb = *reinterpret_cast<int*>(&p->rgb);
		uint8_t b = (rgb >> 16) & 0x0000ff;
		uint8_t g = (rgb >> 8)  & 0x0000ff;
		uint8_t r = (rgb)       & 0x0000ff;
		// pack r/g/b into rgb
		rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
		p->rgb = *reinterpret_cast<float*>(&rgb);
	}
	io::savePLYFileBinary<PointXYZRGB> (filepath, *cloud);
}

int main (int argc, char** argv)
{	
	string usage = "usage: $register <datadir> <outputdir> [-t <filetype> (default: .ply)] [-d <frame_skip> (default: 1)] [-m <registration_method{NONE,FPFH}> (default: FPFH)] [--no_icp]";
	
	int num_args = argc;
	int frameskip = 1;
	string filetype = ".ply";
	Method registration_method = FPFH;
	bool use_icp = true;
	
	for (int i=4; i<argc; i++) {
		if (strcmp(argv[i],"-d") == 0) {
			frameskip = boost::lexical_cast<int>(argv[i+1]);
			num_args -= 2;
			i++;
		}
		else if (strcmp(argv[i],"-t") == 0) {
			filetype = argv[i+1];
			num_args -= 2;
			i++;
		}
		else if (strcmp(argv[i],"-m") == 0) {
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
	cout << "Registration of one frame in " << frameskip
	<< ", using ";
	if (registration_method == NONE) {
		cout << "no initial guess";
	}
	if (registration_method == FPFH) {
		cout << "FPFH features for initial guess";
	}
	cout << " and ";
	if (!use_icp) {
		cout << "no ";
	}
	cout << "refinement based on ICP." << endl;
	
	//Read file list
	cout << "Reading file list" << endl;
    string dir = argv[1];
    vector<string> files = vector<string>();
	vector<double> timestamps = vector<double>();
    getFiles(dir, files, filetype, timestamps);
	
	PointCloud<PointXYZRGB>::Ptr model = loadFrame(files[0], filetype);;
	PointCloud<PointXYZRGB>::Ptr frame;
	
	
	
    return 0;	
}