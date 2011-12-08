% Code for project "Visual Classification of Websites"
% BSc Artificial Intelligence at University of Amsterdam
% By Thomas van den Berg, 2010
% thomas.g.vandenberg@gmail.com

function [ pixels ] = im2pixels( im , mask)
%IM2PIXELS Convert image to (HxW)-by-3 array of pixels
    if (nargin > 1)
        mask3 = cat(3,mask,mask,mask);
        pixels = im(mask3);
        pixels = reshape(pixels, sum(sum(mask)), 3);
    else
        pixels = reshape(im, size(im,1)*size(im,2), 3);
    end
end

