function [ xyz rgb ] = depthmap2pointcloud(depthmap, rgb_frame, depth_factor, p)
%DEPTHMAP2POINTCLOUD Compute pointcloud from depthmap. 
%   DEPTHMAP is the depth image in image format, DEPTH_FACTOR is the factor
%   by which to divide the intensity value to get the depth, P are the
%   intrinsic parameters of the Kinect.

%Default the parameters to the Freiburg set in PNG format
if (nargin < 4)
    p.fx = 525; %focal length x
    p.fy = 525; %focal length y
    p.cx = 319.5; %optical center x
    p.cy = 239.5; %optical center y
    p.ds = 1; %depth scaling
end
if (nargin < 3)
    depth_factor = 5000; %depth factor for the image format
end

[i j] = meshgrid(1:size(depthmap,2), 1:size(depthmap,1));
Z = (double(depthmap) ./ depth_factor) .* p.ds;
X = (i - p.cx) .* Z ./ p.fx;
Y = (j - p.cy) .* Z ./ p.fy;

mask = Z==0;

X(mask) = [];
Y(mask) = [];
Z(mask) = [];

R = rgb_frame(:,:,1);
G = rgb_frame(:,:,2);
B = rgb_frame(:,:,3);

R(mask) = [];
G(mask) = [];
B(mask) = [];

xyz = [X; Y; Z];
rgb = [R; G; B]; 

end