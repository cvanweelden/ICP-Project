#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <vector>
#include <dirent.h>
#include <errno.h>

using namespace std;

bool hasEnding (string const &fullString, string const &ending);

int getFiles (string dir, vector<string> &files, string const &extension);

int getFiles (string dir, vector<string> &files, string const &extension, vector<double> &timestamps);