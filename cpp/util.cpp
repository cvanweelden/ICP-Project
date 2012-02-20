#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <vector>
#include <dirent.h>
#include <errno.h>

#include <boost/lexical_cast.hpp>

#include "util.h"

using namespace std;

bool hasEnding (string const &fullString, string const &ending)
{
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

int getFiles (string dir, vector<string> &files, string const &extension, vector<double> &timestamps)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL) {
        cout << "Error(" << errno << ") opening " << dir << endl;
        return errno;
    }
	
    while ((dirp = readdir(dp)) != NULL) {
		string filename = dirp->d_name;
        if (hasEnding(filename, extension)) {
            files.push_back(dir+string(dirp->d_name));
			filename.erase(filename.size()-extension.size());
			timestamps.push_back(boost::lexical_cast<double>(filename));
		}
    }
    closedir(dp);
    return 0;
}