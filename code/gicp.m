function [qt mse_profile] = gicp( frame, model, varargin)
% Generalized ICP based on:
% [1] Generalized-ICP
%     Segal, A. and Haehnel, D. and Thrun, 
%     S.?Proc. of Robotics: Science and Systems (RSS) ? ? (2009)


p = inputParser;
p.addParamValue('MaxIterations', 50, @(x)isnumeric(x));
p.addParamValue('MSEThreshold',  1e-5, @(x)isnumeric(x));
p.addParamValue('MaxCorrespondenceDist', inf, @(x)isnumeric(x));
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

A_frame = frame.copy();
A = A_frame.xyz;
B = model.xyz;
Nu = A_frame.normals;
Mu = model.normals;

% Octree for model data
tree = kdtree_build(B');
NB = zeros(size(A,2),1);

% Precompute covariances ( see [1] Section III.B )
C = [e 0 0;
     0 1 0;
     0 0 1];
Ca = zeros(3,3,size(A,2));
Cb = zeros(3,3,size(A,2));
for i=1:size(B,2)
    Rmu = [Mu(:,i) [0 1 0]' [0 0 1]'];
    Rnu = [Nu(:,i) [0 1 0]' [0 0 1]'];
    Cb(:,:,i) = Rmu * C * Rmu';
    Ca(:,:,i) = Rnu * C * Rnu';
end

% The cost function we'll try to minimize: (2) in [1]
    function c = cost(Tp)
        q = quatnormalize(Tp(1:4));
        R = quat2rot(q);
        T = [R Tp(5:7)'; 0 0 0 1];
        At = T * [A_corr; ones(1,size(A_corr,2))];
        At = At(1:3, :);
        
        D = B_corr - At;
        c = 0;
        for j=1:size(B_corr,2)
            covar_error = Cb_corr(:,:,j) + R*Ca_corr(:,:,j)*R';
            er = D(:,i)' * (covar_error\D(:,i));
            c = c + er;
        end 
%         c = sum(dot(D,D));
    end

% The transformation, T, as it's called in [1].
% This is [q(1) q(2) q(3) q(4) t(1) t(2) t(3)]

qt = [1 0 0 0 0 0 0];

%Run the ICP loop until difference in MSE is < msethresh
mse = inf;
mse_profile = [];
for iter = 1:maxiter

    % Find closest points
    for i = 1:size(A,2)
        NB(i) = kdtree_k_nearest_neighbors(tree, A(:,i)', 1);
    end
    
    % Reorder B and corresponding covariance matrices
    B_corr = B(:,NB);
    Cb_corr = Cb(:, :, NB);
    A_corr = A;
    Ca_corr = Ca;
    
    %Filter out non-correspondences according to max distance.
    mask = sqrt(sum((A_corr - B_corr).^2)) > d_max;
    B_corr(:,mask) = [];
    Cb_corr(:,:,mask) = [];
    A_corr(:,mask) = [];
    Ca_corr(:,:,mask) = [];    
    
    % Compute the new transformation
    % As [1] mentions, there is no closed-form solution anymore
    % So we use fminsearch
    
    if (strcmpi(method, 'gicp'))
        dqt = fminsearch(@cost, [1 0 0 0 0 0 0] , struct('Display', 'iter', 'TolFun', 0.1));
    elseif (strcmpi(method, 'icp'))
        dqt = get_transformation(A_corr, B_corr);
    end
    
    % Apply the new transformation to the point cloud
%     A = rigid_transform(dqt, A);
    A_frame.apply_qt(dqt);
    A = A_frame.xyz;
    Nu = A_frame.normals;
    for i=1:size(B,2)
        Rnu = [Nu(:,i) [0 1 0]' [0 0 1]'];
        Ca(:,:,i) = Rnu * C * Rnu';
    end    
%     dR = quat2rot(quatnormalize(dqt(1:4)));
%     for i = 1:size(Ca,3)
%         Ca(:,:,i) = dR * Ca(:,:,i) * dR';
%     end
    
    % Apply the new transformation to the transform
    qt = rigid_multiply(dqt, qt);
    
    mse_n = mean(sum((B(:,NB)-A).^2));
    if verbose
        fprintf('Iter %d MSE %g\n',iter, mse_n);
    end
     
    if abs(mse - mse_n) < msethresh
        break
    end
    mse = mse_n;
    mse_profile(iter) = mse; %#ok<AGROW>
end



kdtree_delete(tree)

end

