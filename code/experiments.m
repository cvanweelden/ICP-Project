function experiments()

    global DATA_PATH DROPBOX_PATH

    function results = do_one(method, frameskip, modelmode)
        fn = [method '_' num2str(frameskip) '_' modelmode];
        results = do_registration(dataset, fullfile(output_dir, fn), ...
                    'MaxCloudSize',1e5, ...
                    'MaxFrames', 500, ...
                    'ModelMode', modelmode, ...
                    'GICPArgs', {'Method', method, 'MaxIterations', 100},...
                    'FrameSkip', frameskip);
    end


    dataset = fullfile(DATA_PATH, 'rgbd_dataset_freiburg2_xyz');
    output_dir = fullfile(DROPBOX_PATH, 'results');

    do_one('icp', 1, 'global');
    do_one('icp', 2, 'global');
    do_one('icp', 4, 'global');
    do_one('icp', 8, 'global');
    do_one('icp', 16, 'global');


    do_one('icp', 1, 'previous');
    do_one('icp', 2, 'previous');
    do_one('icp', 4, 'previous');
    do_one('icp', 8, 'previous');
    do_one('icp', 16, 'previous');


end