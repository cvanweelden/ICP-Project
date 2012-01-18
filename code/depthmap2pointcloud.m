function [ pc ] = depthmap2pointcloud(depthmap, depth_factor, p)
%DEPTHMAP2POINTCLOUD Compute pointcloud from depthmap. 
%   DEPTHMAP is the depth image in image format, DEPTH_FACTOR is the factor
%   by which to divide the intensity value to get the depth, P are the
%   intrinsic parameters of the Kinect.

%Default the parameters to the Freiburg set in PNG format
if (nargin < 3)
    p.fx = 525; %focal length x
    p.fy = 525; %focal length y
    p.cx = 319.5; %optical center x
    p.cy = 239.5; %optical center y
    p.ds = 1; %depth scaling
end
if (nargin < 2)
    depth_factor = 5000; %depth factor for the image format
end

depthmap = double(depthmap);
pc = zeros([size(depthmap) 3]);

for i = 1:size(depthmap,1)
    for j = 1:size(depthmap,2)
        Z = (depthmap(i,j) / depth_factor) * p.ds;
        X = (i - p.cx) * Z / p.fx;
        Y = (j - p.cy) * Z / p.fy;
        pc(i,j,:) = [X Y Z];
    end
end

end