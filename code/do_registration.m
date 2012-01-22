function [results] = do_registration(input_dir, output_dir, icp_params)

MAX_CLOUD_SIZE = 1e4;

%Default parameters for the ICP method.
if (nargin < 3)
    icp_params.num_iterations = 50;
    icp_params.mse_threshold = 0.01;
    icp_params.debug = 0;
end

mkdir(output_dir);
mkdir([output_dir filesep 'frames']);

pci = PointCloudIterator(input_dir);

results.avg_mse = 0.0;
results.mse_profile = cell(pci.num_frames-1,1);
results.num_iter = zeros(pci.num_frames-1,1);
results.transformations = cell(pci.num_frames-1,1);

%Initialize the registration with the first depth frame.
pci.next();
writeply(pci.pointcloud, pci.rgb_values, [output_dir filesep 'frames' filesep 'frame1.ply']);
model = subsample_pointcloud(pci.pointcloud, MAX_CLOUD_SIZE);
Q_prev = eye(4);

for i = 2:pci.num_frames
    fprintf('Alligning frame %i\n', i);
    
    pci.next();
    new_frame = subsample_pointcloud(pci.pointcloud, MAX_CLOUD_SIZE);
    new_frame = apply_transform(new_frame, Q_prev);
    
    %Call the ICP method and save the results.
    [new_frame_alligned dQ mse mse_profile num_iter] = icp(new_frame, model, icp_params);
    Q_prev = dQ * Q_prev;
    results.avg_mse = results.avg_mse + mse;
    results.mse_profile{i-1} = mse_profile;
    results.num_iter(i-1) = num_iter;
    results.transformations{i-1} = dQ;
    
    writeply(apply_transform(pci.pointcloud,Q_prev), pci.rgb_values, [output_dir filesep 'frames' filesep 'frame' int2str(i) '.ply']);
    
    model = subsample_pointcloud([model new_frame_alligned], MAX_CLOUD_SIZE);
end

results.avg_mse = results.avg_mse / (pci.num_frames-1);
save([output_dir filesep 'results.mat'], 'results');
clear('icp');


