function [results] = do_registration(input_dir, output_dir, varargin)


% Parse input arguments
p = inputParser;
p.addParamValue('ModelMode',     'global', @(x)strcmpi(x,'global') || strcmpi(x,'previous'));
p.addParamValue('MaxCloudSize',  1e4, @(x)isnumeric(x));
p.addParamValue('MaxIterations', 50, @(x)isnumeric(x));
p.addParamValue('MSEThreshold',  1e-5, @(x)isnumeric(x));
p.parse(varargin{:});

cloudsize = p.Results.MaxCloudSize;
maxiter = p.Results.MaxIterations;
msethresh = p.Results.MSEThreshold;

mkdir(output_dir);
mkdir(fullfile(output_dir,'frames'));

pcs = PointCloudSet(input_dir);

% results.avg_mse = 0.0;
% results.mse_profile = cell(pci.num_frames-1,1);
% results.num_iter = zeros(pci.num_frames-1,1);
% results.transformations = cell(pci.num_frames-1,1);

%Initialize the registration with the first depth frame.
pci.next();
writeply(pci.pointcloud, pci.rgb_values, [output_dir filesep 'frames' filesep 'frame1.ply']);

model = pcs{1};
model.write(fullfile(output_dir,'frames','frame1.ply'));
model.subsample(cloudsize);

A_prev = eye(4);

for i = 2:pci.num_frames
    fprintf('Aligning frame %i\n', i);
    
    new_frame = pcs{i};
    new_frame.subsample(cloudsize)
    new_frame.apply_matrix(A_prev);
    
    %Call the ICP method and save the results.
    [new_frame_alligned dQ mse mse_profile num_iter] = icp(new_frame, model, icp_params);
    A_prev = dA * A_prev;
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


