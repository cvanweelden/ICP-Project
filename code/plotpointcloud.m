function [ output_args ] = plotpointcloud( cloud, meshsize, varargin )
%POINTCLOUD Plots a pointcloud, cloud is a 3xN array of x,y,z points.

    p = inputParser;
    p.addParamValue('FilterZeros', true);
    p.addParamValue('SubSample', 20, @(x)isnumeric(x));
    p.addParamValue('Mask', []);
    p.parse(varargin{:}); 
    
    filterzeros = p.Results.FilterZeros;
    subsample = p.Results.SubSample;
    mask = p.Results.Mask;
    
    
    % Reshape the incoming cloud (3xK) to a mesh (MxNx3)
    M = reshape(cloud', meshsize(1), [], 3);
    
    % Reshape/create the mask
    if filterzeros
        mask = any(cloud);
    end
    if numel(mask) > 0
        mask = reshape(mask, meshsize(1), []);
    else
        mask = ones(size(M,1), size(M,2));
    end
    
    % Apply subsampling
    M = M(1:subsample:end, 1:subsample:end, :);
    mask = mask(1:subsample:end, 1:subsample:end);

    
    % Render the point cloud as a mesh
    
%     for i = 1:size(M,1)
%         for j = 1:size(M,2)
%             p = M(i,j,:);
%             if i > 1 && mask(i,j) && mask(i-1,j)
%                 up = M(i-1,j,:);
%                 line([p(1) up(1)], [p(2) up(2)], [p(3) up(3)], 'Color','b','LineWidth',1);
%             end
%             if j > 1 && mask(i,j) && mask(i,j-1)
%                 left = M(i,j-1,:);
%                 line([p(1) left(1)], [p(2) left(2)], [p(3) left(3)], 'Color','b','LineWidth',1);
%             end
%         end
%     end
    
    % Render as a surface
    for i = 2:size(M,1)
        for j = 2:size(M,2)
            if mask(i,j) && mask(i-1,j) && mask(i,j-1) && mask(i-1,j-1)
                br = M(i,j,:);  % bottom right
                tr = M(i-1,j,:);% top right, etc
                bl = M(i,j-1,:);
                tl = M(i-1,j-1,:);
                polys = [br tl bl; br tr tl];
                fill3(polys(:,:,1)', polys(:,:,2)', polys(:,:,3)', [0 0 1]);
                hold on
            end
        end
    end

end

