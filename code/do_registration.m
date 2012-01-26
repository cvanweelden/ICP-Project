function [results] = do_registration(input_dir, output_dir, varargin)


% Parse input arguments
p = inputParser;
p.addParamValue('ModelMode',     'global', @(x)strcmpi(x,'global') || strcmpi(x,'previous'));
p.addParamValue('MaxCloudSize',  1e3, @(x)isnumeric(x));
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

%Initialize the registration with the first depth frame.
frame = pcs{1};
model = frame.copy();
model.subsample(cloudsize);

%Write the first frame with the initial pose.
q = p.Results.StartOrientation;
t = p.Results.StartPosition;
frame.apply_qt(q,t);
frame.write(fullfile(output_dir,'frames','frame1.ply'));

results.avg_mse = 0.0;
results.mse_profile{1} = [];
results.num_iter(1) = 0;
results.transformations{1} = [q t];
results.pose{1} = [q t];
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
    
    % Call the ICP method and save the results.
    [dq dt mse_profile] = gicp(new_frame, model, gicpargs{:});
    t = t + quatrotate(q,dt);
    q = quatmultiply(dq, q);
    results.avg_mse = results.avg_mse + mse_profile(end);
    results.mse_profile{i} = mse_profile;
    results.num_iter(i) = numel(mse_profile);
    results.transformations{i} = [dq dt];
    results.pose{i} = [q t];
    results.timestamp{i} = new_frame.timestamp;
    
    % Write the new frame
    frame.apply_qt(q, t);
    frame.write(fullfile(output_dir, 'frames', sprintf('frame%d.ply',i)));
    % TODO: Below is a bit awkward, this used to be 'new_frame_aligned',
    % but there is a bit of a discrepancy between pointCloud classes
    % and XYZ matrices in gicp. 
    
    % Update the model
    if strcmpi(modelmode, 'global')
        model.apply_qt_inv(dq, dt);
        model = [model new_frame]; %#ok<AGROW>
    else
        model = new_frame;
    end
    model.subsample(cloudsize);
end

results.avg_mse = results.avg_mse / (pcs.num_frames-1);
save(fullfile(output_dir, 'results.mat'), 'results');

write_trajectory(results, fullfile(output_dir, 'trajectory.txt'));

clear('icp');


