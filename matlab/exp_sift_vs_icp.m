%#ok<*SAGROW>
clear
config

frameskip  = 20;
max_frames = 400;
cloudsize  = 10000;

ds = PointCloudSet( fullfile(DATA_PATH, 'rgbd_dataset_freiburg1_xyz') );

ii = 1:frameskip:min(max_frames, ds.num_frames);

for i=1:numel(ii)-1
    fprintf('\n ==== Frame %d ==== \n\n',ii(i));
    frame = ds.cloud(ii(i+1));
    model = ds.cloud(ii(i));
    
    sift_qt = siftstep(frame, model);
    
    icponly_qt = gicp(frame.subsample(cloudsize), model.subsample(cloudsize));
    
    frame = frame.apply_qt(sift_qt);
    icp_qt = gicp(frame, model.subsample(cloudsize));
    
    sifticp_qt = rigid_multiply(icp_qt, sift_qt);
    
    real_qt = ds.relpose(ii(i+1),ii(i));
    sift_transerr(i) = sum((real_qt(5:7) - sift_qt(5:7)).^2); 
    icp_transerr(i) = sum((real_qt(5:7) - icponly_qt(5:7)).^2);
    sifticp_transerr(i) = sum((real_qt(5:7) - sifticp_qt(5:7)).^2); 
end


%%
plot(icp_transerr, 'r', 'LineWidth',2)
hold on
plot(sift_transerr, 'b', 'LineWidth',2)
plot(sifticp_transerr, 'g', 'LineWidth', 2);

legend('ICP', 'SIFT', 'SIFT+ICP');

