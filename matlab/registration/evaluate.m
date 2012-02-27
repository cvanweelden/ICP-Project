function [ varargout ] = evaluate( trajectory, ground_truth )
%EVALUATE Summary of this function goes here
%   Detailed explanation goes here
    global DATA_PATH

    if nargin < 2
        ground_truth = fullfile(DATA_PATH, 'rgbd_dataset_freiburg2_xyz','groundtruth.txt');
    end
    
    if ischar(ground_truth)
        ground_truth = cell2mat(textscan(fopen(ground_truth), '%f %f %f %f %f %f %f %f', ...
            'HeaderLines',3));
    end
    
    if ischar(trajectory)
        fprintf('Reading %s\n',trajectory);
        trajectory = cell2mat(textscan(fopen(trajectory), '%f %f %f %f %f %f %f %f'));
    end
    
    function rt = reltrans(from, to)
        displacement = to(1:3) - from(1:3);
        rt = quatrotate(quatinv(from(4:7)), displacement);
    end


    
    % Get closest timestamp for each trajectory point
    tr_rp = repmat(trajectory(:,1)',size(ground_truth,1),1);
    gt_rp = repmat(ground_truth(:,1),1, size(trajectory,1));
    [~,closest] = min(abs(tr_rp-gt_rp));
    
    % Compute relative transforms:
    tr_rel = zeros(length(closest)-1,3);
    gt_rel = zeros(length(closest)-1,3);
    for i=2:length(closest)
        tr_rel(i-1, :) = reltrans(trajectory(i-1,:), trajectory(i,:));
        gt_rel(i-1, :) = reltrans(ground_truth(closest(i-1),:),  ground_truth(closest(i),:));
    end
    
    d = sum((gt_rel-tr_rel).^2,2);
    
    if nargout == 0
        plot(closest(2:end), d, 'r')
    else
        varargout{1} = closest(2:end);
        varargout{2} = d;
        varargout{3} = gt_rel;
        varargout{4} = tr_rel;
    end

end

