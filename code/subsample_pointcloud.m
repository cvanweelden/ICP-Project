function [out] = subsample_pointcloud(in, k)

p = randperm(size(in,2));
out = in(:,p(1:k));

end

