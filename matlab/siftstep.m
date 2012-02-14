function qt = siftstep(frame, model)

AFFINE_THRESHOLD = 1e-1;

[rgb1, d1] = model.images();
[rgb2, d2] = frame.images();

i1 = rgb2gray(im2double(rgb1));
i2 = rgb2gray(im2double(rgb2));

args = {'Threshold',0.05, 'EdgeThreshold',5};
[fr1,dscr1] = sift(i1, args{:});
[fr2,dscr2] = sift(i2, args{:});

matches = siftmatch(dscr1, dscr2);
n_matches = size(matches,2);

xyz1 = depth2xyz(d1);
xyz2 = depth2xyz(d2);

matchfr1 = fr1(1:2,matches(1,:));
matchfr2 = fr2(1:2,matches(2,:));

% plotmatches(i1,i2,fr1,fr2,matches);

% Do 2d ransac to filter outliers
most_inliers = 0;

for i=1:1000
    idxs = randsample(1:n_matches,3);
    base_points = matchfr1(1:2, idxs)';
    input_points = matchfr2(1:2, idxs)';
    
    if (size(unique(base_points, 'rows'),1) < size(base_points,1) || ...
        size(unique(input_points, 'rows'),1) < size(input_points,1))
        continue
    end
        
    tform = cp2tform(input_points, base_points, 'affine');
    
    transformed = tformfwd(tform, matchfr2');
    
    sum((matchfr1' - transformed).^2, 2);
    inliers = sum((matchfr1' - transformed).^2, 2) < AFFINE_THRESHOLD;
    if sum(inliers) > most_inliers
        most_inliers = sum(inliers)
        best_inliers = inliers;
    end
    
end

% Use only the inliers
matchfr1 = matchfr1(:,best_inliers);
matchfr2 = matchfr2(:,best_inliers);

% Pick the 3d Points
ij1 = floor(matchfr1);
ij2 = floor(matchfr2);
corrxyz1 = zeros(3,n_matches);
corrxyz2 = zeros(3,n_matches);
for i=1:most_inliers
    corrxyz1(:,i) = permute(xyz1(ij1(2,i), ij1(1,i), :),[3 1 2]);
    corrxyz2(:,i) = permute(xyz2(ij2(2,i), ij2(1,i), :),[3 1 2]);
end

% Find the transform
qt = get_transformation(corrxyz2, corrxyz1);

% best_mse = inf;
% best_qt = [0 0 0 0 0 0 0];
% for i=1:10000
%     selected = randsample(n_matches, 5);
%     qt = get_transformation(corrxyz2(:,selected), corrxyz1(:,selected));
%     
%     mse = mean(sum((rigid_transform(qt, corrxyz2) - corrxyz1).^2));
%     
%     if mse < best_mse
%         best_mse = mse
%         best_qt = qt;
%     end
% end
% 

% pc1 = ds.cloud(f1);
% pc2 = ds.cloud(f2);
% 
% fprintf('Write files...\n');
% pc2.apply_qt(qt);
% pc1.write('usift_model.ply');
% pc2.write('usift_frame.ply');
% 
% fprintf('Start ICP...\n');
% subs1 = pc1.copy().subsample(10000);
% subs2 = pc2.copy().subsample(10000);
% qt2 = gicp(subs2, subs1);
% 
% pc2.apply_qt(qt2);
% 
% pc2.write('usift_refined.ply');

end