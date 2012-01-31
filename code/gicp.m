function [qt mse_profile] = gicp( frame, model, varargin)
% Generalized ICP based on:
% [1] Generalized-ICP
%     Segal, A. and Haehnel, D. and Thrun, 
%     S.?Proc. of Robotics: Science and Systems (RSS) ? ? (2009)


p = inputParser;
p.addParamValue('MaxIterations', 50, @(x)isnumeric(x));
p.addParamValue('MSEThreshold',  1e-6, @(x)isnumeric(x));
p.addParamValue('MaxCorrespondenceDist', 1e-4, @(x)isnumeric(x));
p.addParamValue('OverlapFraction', 0.8);
p.addParamValue('CovarEpsilon',  0.01, @(x)isnumeric(x));
p.addParamValue('Method', 'gicp', @(x)strcmpi(x,'gicp') || strcmpi(x,'wsm') || strcmpi(x,'icp'));
p.addParamValue('Verbose', true);
p.parse(varargin{:});

maxiter = p.Results.MaxIterations;
msethresh = p.Results.MSEThreshold;
d_max = p.Results.MaxCorrespondenceDist;
e = p.Results.CovarEpsilon;
verbose = p.Results.Verbose;
method = p.Results.Method;
overlap = p.Results.OverlapFraction;

A = frame.xyz;
B = model.xyz;

use_covars = strcmpi(method, 'gicp') || strcmpi(method, 'wsm');

if use_covars
    model.computenormals();
    frame.computenormals();
    
    % Precompute covariances ( see [1] Section III.B )
    C = [e 0 0;
         0 1 0;
         0 0 1];
    Ca = zeros(3,3,size(A,2));
    Cb = zeros(3,3,size(A,2));
    for i=1:size(B,2)
        Rmu = [model.normals(:,i) [0 1 0]' [0 0 1]'];
        Rnu = [frame.normals(:,i) [0 1 0]' [0 0 1]'];
        Cb(:,:,i) = Rmu * C * Rmu';
        Ca(:,:,i) = Rnu * C * Rnu';
    end

end


% Octree for model data
tree = kdtree_build(B');

% The cost function we'll try to minimize: (2) in [1]
    function c = cost(Tp)
        R = quat2dcm(Tp(1:4))';
        T = [R Tp(5:7)'; 0 0 0 1];
        At = T * [A(:,A_idx); ones(1,size(A_idx,2))];
        At = At(1:3, :);
        
        D = B(:,B_idx) - At;
        c = 0;
        for j=1:size(B_idx,2)
            covar_error = Cb(:,:,B_idx(j)) + R*Ca(:,:,A_idx(j))*R';
            er = D(:,j)' * (covar_error\D(:,j));
            c = c + er;
        end 
%         c = sum(dot(D,D));
    end

% The transformation, T, as it's called in [1].
% This is [q(1) q(2) q(3) q(4) t(1) t(2) t(3)]

qt = [1 0 0 0 0 0 0];
AT = A;

%Run the ICP loop until difference in MSE is < msethresh

last_mse = inf;
mse_profile = [];


for iter = 1:maxiter
        

    A_idx = 1:size(A,2); % A used indexes (so we can filter)
    B_idx = 1:size(B,2); % B used indexes (NB)
    
    % Find closest points
    for i = 1:size(A,2)
        B_idx(i) = kdtree_k_nearest_neighbors(tree, AT(:,i)', 1);
    end
    
    d = sum((B(:, B_idx)-AT).^2);
    
    % Filter the percentage of closest points
    if overlap < 1
        [~,idx] = sort(d);
        idx = idx(1:double(numel(idx)*overlap));
        A_idx = A_idx(idx);
        B_idx = B_idx(idx);
        fprintf('Filtering for %.0f%% overlap. %d points left.\n',overlap*100,numel(idx));
    %Filter out non-correspondences according to max distance.
    elseif d_max < inf
        mask = d < d_max;
        A_idx = A_idx(mask);
        B_idx = B_idx(mask);
        fprintf('Filtering for d_max %.5f. %d points left.\n',d_max,sum(mask));
    end

    % Compute the new transformation
    % As [1] mentions, there is no closed-form solution anymore
    % So we use fminsearch
    
    if (strcmpi(method, 'gicp'))
        [qt, ~] = fminsearch(@cost, qt , struct('Display', 'final', 'TolFun', msethresh*100, 'TolX',0.1));
    elseif (strcmpi(method, 'icp'))
        qt = get_transformation(A(:,A_idx), B(:,B_idx));
    end
    
    AT = rigid_transform(qt, A);
    mse = mean(sum((B(:, B_idx)-AT(:,A_idx)).^2));
    
    if verbose
        fprintf('Iter %d MSE %g\n',iter, mse);
    end
    
    mse_profile(iter) = mse; %#ok<AGROW>
    
    % Break if done
    if strcmpi(method,'icp') && (abs(mse - last_mse) < msethresh)
        break
    end
    
    last_mse = mse;
    
end



kdtree_delete(tree)

end

