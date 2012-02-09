% Set the relative path to your compiled KinectMatlab.
global KINECT_MATLAB_PATH KDTREE_PATH KDTREE_MATLAB_PATH DATA_PATH DROPBOX_PATH;
KINECT_MATLAB_PATH = '../../libs/kinectmatlab/Mex';
KDTREE_PATH = '../../libs/kdtree';
KDTREE_MATLAB_PATH = '../../libs/kdtree_matlab';
DROPBOX_PATH = '~/Dropbox/Studie/ICP-Project/';
DATA_PATH = '../../data';
addpath(KDTREE_MATLAB_PATH);
addpath(KINECT_MATLAB_PATH);
addpath(KDTREE_PATH);
