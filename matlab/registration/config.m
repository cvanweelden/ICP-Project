% Set the relative path to your compiled KinectMatlab.
global KINECT_MATLAB_PATH KDTREE_PATH SIFT_PATH DATA_PATH DROPBOX_PATH;
KINECT_MATLAB_PATH = '../../libs/kinectmatlab/Mex';
KDTREE_PATH = '../../libs/kdtree';
SIFT_PATH = '../../libs/sift';
DROPBOX_PATH = '~/Dropbox/Studie/ICP-Project/';
DATA_PATH = '../../data';
addpath(SIFT_PATH);
addpath(KINECT_MATLAB_PATH);
addpath(KDTREE_PATH);
