%#ok<*SAGROW>
clear
config

frameskip  = 3;
max_frames = 100;
cloudsize  = 10000;

ds = PointCloudSet( fullfile(DATA_PATH, 'rgbd_dataset_freiburg1_xyz') );

ii = 1:frameskip:min(max_frames, ds.num_frames);
translation_error = [];
sifticp_transerr = [];
for i=1:numel(ii)-1
    
    frame = ds.cloud(ii(i+1));
    model = ds.cloud(ii(i));
    
    sift_qt = siftstep(frame, model);
    
    frame = frame.subsample(cloudsize).apply_qt(sift_qt);
    icp_qt = gicp(frame, model.subsample(cloudsize));
    
    sifticp_qt = rigid_multiply(icp_qt, sift_qt);
    
    real_qt = ds.relpose(ii(i+1),ii(i));
    sift_transerr(i) = sum((real_qt(5:7) - sift_qt(5:7)).^2); 
    sifticp_transerr(i) = sum((real_qt(5:7) - sifticp_qt(5:7)).^2); 
end

