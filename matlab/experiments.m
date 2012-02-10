function results = experiments()

    global DATA_PATH DROPBOX_PATH 
    

    function r = do_one(prefix, method, frameskip, modelmode, overlap, cloudsize)
        fn = sprintf('%s%s_1in%d_%d%s_%.0fperc',prefix,method, frameskip, cloudsize, modelmode, overlap*100)
        r = do_registration(dataset, fullfile(output_dir, fn), ...
                    'MaxCloudSize',cloudsize, ...
                    'MaxFrames', 400, ...
                    'ModelMode', modelmode, ...
                    'GICPArgs', {'Method', method, 'MaxIterations', 20, 'OverlapFraction', overlap},...
                    'FrameSkip', frameskip);
    end


    dataset = fullfile(DATA_PATH, 'rgbd_dataset_freiburg2_xyz');
    output_dir = fullfile(DROPBOX_PATH, 'results');

%     do_one('','icp', 1, 'global', 0.8, 1e3);
%     do_one('icp', 1, 'global', 0.8, 1e5);
    
%     do_one('icp', 2, 'global', 0.7, 1e4);
%     do_one('icp', 4, 'global', 0.7, 1e4);
%     results = do_one('icp', 8, 'global', 0.8, 1e4);
%     results = do_one('icp', 16, 'global', 0.8, 1e4);
%     
%     results = do_one('gicp', 1, 'global', 0.8, 1e3);
%     results = do_one('gicp', 2, 'global', 0.8, 1e3);
%     results = do_one('gicp', 4, 'global', 0.8, 1e3);
%     results = do_one('gicp', 8, 'global', 0.8, 1e3);
%     results = do_one('gicp', 16, 'global', 0.8, 1e3);
% %   
%     do_one('icp', 2, 'global');
%     do_one('icp', 4, 'global');
%     do_one('icp', 8, 'global');
%     do_one('icp', 16, 'global');
% 
% 
%     do_one('icp', 1, 'previous');
%     do_one('icp', 2, 'previous');
%     do_one('icp', 4, 'previous');
%     do_one('icp', 8, 'previous');
%     do_one('icp', 16, 'previous');

    pcs = PointCloudSet(dataset);
    for i=1:pcs.num_frames
        i
        pc = pcs{i};
        pc.subsample(100000);
        pc.write(fullfile(output_dir,'unaligned',sprintf('frame%03d.ply',i)));
    end


end