config;
tic;
results = do_registration('../data/rgbd_dataset_freiburg2_xyz', '../output/test', 'MaxFrames', 10, 'StartPosition', [0.1170 -1.1503 1.4005], 'StartOrientation', [-0.5709 0.6523 -0.3566 0.3486], 'ModelMode', 'global', 'GICPArgs', {'Method','gicp'});
time = toc;
