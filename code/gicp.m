function [Mcum mse_profile] = gicp( model, frame, varargin)
% Generalized ICP based on:
% [1] Generalized-ICP
%     Segal, A. and Haehnel, D. and Thrun, 
%     S.?Proc. of Robotics: Science and Systems (RSS) ? ? (2009)
% P and X are point clouds.

p = inputParser;
p.addParamValue('MaxIterations', 50, @(x)isnumeric(x));
p.addParamValue('MSEThreshold',  1e-5, @(x)isnumeric(x));
p.addParamValue('MaxCorrespondenceDist', inf, @(x)isnumeric(x));
p.addParamValue('CovarEpsilon',  0.01, @(x)isnumeric(x));
p.addParamValue('Verbose', true);
p.parse(varargin{:});

maxiter = p.Results.MaxIterations;
msethresh = p.Results.MSEThreshold;
d_max = p.Results.MaxCorrespondenceDist;
e = p.Results.CovarEpsilon;
verbose = p.Results.Verbose;

A = model.xyz; %P
B = frame.xyz; %X
Nu = model.normals;
Mu = frame.normals;

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
        q = Tp(1:4);
        q = q/norm(q);
        R = quat2rot(q);
        t = Tp(1:3);
        c = 0;
        for j=1:size(B,2)
            di = B(:,j) - R*A(:,j) + t';
            covar_error = Cb(:,:,j) + R*Ca(:,:,j)*R;
            er = (di' * (covar_error\di));
            c = c + er;
        end
        disp(c)
    end

% The transformation, T, as it's called in [1].
% This is [q(1) q(2) q(3) q(4) t(1) t(2) t(3)]

Mcum = eye(4); % The cumulative transform

%Run the ICP loop until difference in MSE is < msethresh
mse = inf;
mse_profile = [];
for iter = 1:maxiter
    
    if verbose
        fprintf('Iter %d MSE %g',iter, mse);
    end
    
    % Find point correspondences
    for i = 1:size(A,2)
        NB(i) = kdtree_k_nearest_neighbors(tree, A(:,i)', 1);
    end
    
    % Reorder B and corresponding covariance matrices
    B = B(:, NB);
    Cb = Cb(:, :, NB);
    
    % Compute the new transformation
    % As [1] mentions, there is no closed-form solution anymore
    % So we use fminsearch
    qt = fminsearch(@cost, [1 0 0 0 0 0 0] , struct('Display', 'iter'));
    
    % Create a homogeneous transformation matrix
    M = [quat2rot(qt(1:4)) qt(1:3)'; 0 0 0 1];
    
    % Apply the new transformation to the model
    % Tcum = ... = [quat2rot(q_r) q_t; 0 0 0 1] * Tcum;
    A = M * [A; ones(1, size(A,2))];
    A = A(1:3, :);
    mse_n = mean(sum((A-B).^2,2));
    Mcum = M * Mcum;
    
    if mse - mse_n < msethresh
      break
    end
    mse = mse_n;
    mse_profile(iter) = mse; %#ok<AGROW>
end



kdtree_delete(tree)

end

