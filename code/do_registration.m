function [results] = do_registration(input_dir, output_dir, varargin)


% Parse input arguments
p = inputParser;
p.addParamValue('ModelMode',     'global', @(x)strcmpi(x,'global') || strcmpi(x,'previous'));
p.addParamValue('MaxCloudSize',  1e4, @(x)isnumeric(x));
p.addParamValue('GICPArgs', {});
p.parse(varargin{:});

modelmode = p.Results.ModelMode;
cloudsize = p.Results.MaxCloudSize;
gicpargs = p.Results.GICPArgs;

mkdir(output_dir);
mkdir(fullfile(output_dir,'frames'));

pcs = PointCloudSet(input_dir);

results.avg_mse = 0.0;

%Initialize the registration with the first depth frame.

model = pcs{1};
% model.write(fullfile(output_dir,'frames','frame1.ply'));
model.subsample(cloudsize);

A_prev = eye(4);

for i = 2:pcs.num_frames
    fprintf('Aligning frame %i\n', i);
    
    % Compute normals for the model (TODO: here?)
    model.computenormals();
    
    % Load up a new frame
    frame = pcs{i};
    new_frame = frame.copy();
    new_frame.subsample(cloudsize);
    new_frame.apply_matrix(A_prev);
    new_frame.computenormals();
    
    % Call the ICP method and save the results.
    [dA mse_profile] = gicp(new_frame, model, gicpargs{:});
    A_prev = dA * A_prev;
    results.avg_mse = results.avg_mse + mse_profile(end);
    results.mse_profile{i-1} = mse_profile;
    results.num_iter(i-1) = numel(mse_profile);
    results.transformations{i-1} = dA;
    
    % Write the new frame
    frame.apply_matrix(A_prev)
    frame.write(fullfile(output_dir, 'frames', sprintf('frame%d.ply',i)));
    % TODO: Below is a bit awkward, this used to be 'new_frame_aligned',
    % but there is a bit of a discrepancy between pointCloud classes
    % and XYZ matrices in gicp.
    frame.subsample(cloudsize); 
    
    % Update the model
    if strcmpi(modelmode, 'global')
        model = [model frame]; %#ok<AGROW>
    else
        model = frame;
    end
    model.subsample(cloudsize);
end

results.avg_mse = results.avg_mse / (pci.num_frames-1);
save([output_dir filesep 'results.mat'], 'results');
clear('icp');


