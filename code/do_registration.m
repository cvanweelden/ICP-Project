function [results] = do_registration(input_dir, output_dir, varargin)


% Parse input arguments
p = inputParser;
p.addParamValue('ModelMode',     'global', @(x)strcmpi(x,'global') || strcmpi(x,'previous'));
p.addParamValue('MaxCloudSize',  1e4, @(x)isnumeric(x));
p.addParamValue('MaxFrames',  inf, @(x)isnumeric(x));
p.addParamValue('StartPosition', [0 0 0], @(x)isnumeric(x) && numel(x) == 3);
p.addParamValue('StartOrientation', [1 0 0 0], @(x)isnumeric(x) && numel(x) == 4);
p.addParamValue('GICPArgs', {});
p.parse(varargin{:});

modelmode = p.Results.ModelMode;
cloudsize = p.Results.MaxCloudSize;
gicpargs = p.Results.GICPArgs;
maxframes = p.Results.MaxFrames;

mkdir(output_dir);
mkdir(fullfile(output_dir,'frames'));

pcs = PointCloudSet(input_dir);

%Write the first frame with the initial pose.
qt = [p.Results.StartOrientation p.Results.StartPosition];
frame = pcs{1};
frame.apply_qt(qt); 
frame.write(fullfile(output_dir,'frames','frame1.ply'));

%Initialize the registration with the first depth frame.
model = frame.copy();
model.subsample(cloudsize);

results.avg_mse = 0.0;
results.mse_profile{1} = [];
results.num_iter(1) = 0;
results.transformations{1} = qt;
results.pose{1} = qt;
results.timestamp{1} = model.timestamp;

for i = 2:min(pcs.num_frames, maxframes)
    fprintf('Aligning frame %i\n', i);
    
    % Compute normals for the model (TODO: here?)
    model.computenormals();
    
    % Load up a new frame
    frame = pcs{i};
    new_frame = frame.copy();
    new_frame.subsample(cloudsize);
    new_frame.computenormals();
    new_frame.apply_qt(qt);
    
    % Call the ICP method and save the results.
    [dqt mse_profile] = gicp(new_frame, model, gicpargs{:});
    qt = rigid_multiply(dqt, qt);
    results.avg_mse = results.avg_mse + mse_profile(end);
    results.mse_profile{i} = mse_profile;
    results.num_iter(i) = numel(mse_profile);
    results.transformations{i} = dqt;
    results.pose{i} = qt;
    results.timestamp{i} = new_frame.timestamp;
    
    % Write the new frame
    frame.apply_qt(qt);
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

results.avg_mse = results.avg_mse / (pcs.num_frames-1);
save(fullfile(output_dir, 'results.mat'), 'results');

write_trajectory(results, fullfile(output_dir, 'trajectory.txt'));

clear('icp');


