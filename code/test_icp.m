config;
tic;
results = do_registration('../data/rgbd_dataset_freiburg2_xyz', '../output/test', 'MaxCloudSize', 1e4, 'StartPosition', [0.1170 -1.1503 1.4005], 'StartOrientation', [-0.5709 0.6523 -0.3566 0.3486], 'ModelMode', 'global', 'GICPArgs', {'Method','gicp', 'MaxIterations', 15});
% results = do_registration('../data/rgbd_dataset_freiburg2_xyz', '../output/test', 'ModelMode', 'global', 'GICPArgs', {'Method','icp', 'MaxIterations', 15, 'MaxCorrespondenceDist', 0.5});
time = toc;
