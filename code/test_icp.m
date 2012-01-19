DATA_DIR = '../rgbd_dataset_freiburg1_xyz';
OUT_DIR = '../xyz_test';

param.num_iterations = 50;
param.mse_threshold = 0.01;
param.debug = 0;
param.sample_stepsize = 10;

depth = dir('../rgbd_dataset_freiburg1_xyz/depth/*.png');
rgb = dir('../rgbd_dataset_freiburg1_xyz/rgb/*.png');

old_frame = depthmap2pointcloud(imread([DATA_DIR filesep 'depth' filesep depth(1).name]));
writeply(old_frame, imread([DATA_DIR filesep 'rgb' filesep rgb(1).name]), [OUT_DIR filesep 'frame1.ply']);

for i = 2:size(depth,1)
    
    new_frame = depthmap2pointcloud(imread([DATA_DIR filesep 'depth' filesep depth(i).name]));
    
    Q = icp(new_frame, old_frame, param);
    P = [im2pixels(new_frame); ones(1, size(new_frame,1)*size(new_frame,2))];
    P = Q * P;
    P = P(1:3,:);
    new_frame_alligned = reshape(P', size(new_frame,1), size(new_frame,2), 3);

    writeply(new_frame_alligned, imread([DATA_DIR filesep 'rgb' filesep rgb(i).name]), [OUT_DIR filesep 'frame' int2str(i) '.ply']);
    old_frame = new_frame_alligned;
end



