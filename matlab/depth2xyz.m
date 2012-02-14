function [ xyz ] = depth2xyz( depth, camvars )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    if nargin < 2
        camvars = struct('fx',525,'fy',525,'cx',319.5,'cy',239.5,'ds',1/5000);
    end
    
    % Straighten the XYZ coordinates using the camera parameters
    [I J] = meshgrid(1:size(depth,2), 1:size(depth,1));
    Z = double(depth) * camvars.ds;
    X = (I - camvars.cx) .* Z / camvars.fx;
    Y = (J - camvars.cy) .* Z / camvars.fy;
    xyz = cat(3, X, Y, Z);

end

