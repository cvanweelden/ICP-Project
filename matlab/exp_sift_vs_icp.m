%#ok<*SAGROW>
clear
config

frameskip  = 4;
max_frames = 100;
cloudsize  = 10000;

ds = PointCloudSet( fullfile(DATA_PATH, 'rgbd_dataset_freiburg2_xyz') );

icpargs = {'OverlapFraction',0.95}

ii = 1:frameskip:min(max_frames, frameskip*ds.num_frames);

for i=1:numel(ii)-1
    fprintf('\n ==== Frame %d ==== \n\n',ii(i));
    frame = ds.cloud(ii(i+1));
    model = ds.cloud(ii(i));
    
    sift_qt = siftstep(frame, model);
    frame.subsample(cloudsize);
    model.subsample(cloudsize);
    
    % Try a pure ICP step
    [icp_qt, mse_icp] = gicp(frame, model, icpargs{:});
    
    % Try ICP initialized with SIFT's transformation
    frame = frame.apply_qt(sift_qt);
    [sicp_qt, mse_sicp] = gicp(frame, model.subsample(cloudsize), icpargs{:});
    
    sifticp_qt = rigid_multiply(sicp_qt, sift_qt);
    
    icp_qt
    sift_qt
    sicp_qt
    sifticp_qt
    
    real_qt = ds.relpose(ii(i+1),ii(i));
    maxerr(i) = sum(real_qt(5:7).^2);
    sift_transerr(i) = sum((real_qt(5:7) - sift_qt(5:7)).^2); 
    icp_transerr(i) = sum((real_qt(5:7) - icp_qt(5:7)).^2);
    sifticp_transerr(i) = sum((real_qt(5:7) - sifticp_qt(5:7)).^2); 
    icp_steps(i) = numel(mse_icp);
    sifticp_steps(i) = numel(mse_sicp);
    
end

%%
save(sprintf('results_sifticp_skip%d',frameskip),'maxerr','sift_transerr',...
        'icp_transerr','sifticp_transerr','icp_steps','sifticp_steps');


%%
subplot(2,1,1);
hold on
semilogy(maxerr, 'k--');
semilogy(icp_transerr, 'r', 'LineWidth',2)
% semilogy(sift_transerr, 'b', 'LineWidth',2)
semilogy(sifticp_transerr, 'g', 'LineWidth', 2);

legend('MAX','ICP',  'SIFT+ICP');

subplot(2,1,2);
plot(icp_steps, 'r', 'LineWidth',2);
hold on
plot(sifticp_steps, 'g', 'LineWidth',2);

