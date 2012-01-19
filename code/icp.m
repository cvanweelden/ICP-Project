function [ Q mse ] = icp( P, X, par)
%ICP Iterated Closest Point allignment of data P to model X
%   P and X are HEIGHTxWIDTHx3 point clouds. MSE_THRESHOLD is the desired
%   precision in mean squared errror. DEBUG (optional) 0 (default):no debug
%   info, 1:print MSE on each iteration.

if (nargin < 3)
    par.num_iterations = 1000;
    par.mse_threshold = 0.01;
    par.debug = 0;
    par.sample_stepsize = 10;
end

%Subsample:
P = P(1:par.sample_stepsize:end, 1:par.sample_stepsize:end, :);
X = X(1:par.sample_stepsize:end, 1:par.sample_stepsize:end, :);

%Reshape to a vector matrix.
P = reshape(P, size(P,1)*size(P,2), 3)';
X = reshape(X, size(X,1)*size(X,2), 3)';

%Remove pixels with unknown distance.
P(:,P(1,:)==0 & P(2,:)==0 & P(3,:)==0) = [];
X(:,X(1,:)==0 & X(2,:)==0 & X(3,:)==0) = [];

%Octree for model data X
tree = kdtree_build(X');
NB = zeros(size(P,2),1);

%The final transformation:
Q = eye(4);

%Run the ICP loop untill MSE is small enough
mse = inf;
for j = 1:par.num_iterations

    %NB(i) = index for nearest neighbor of P(i) in X
    for i = 1:size(P,2)
        NB(i) = kdtree_k_nearest_neighbors(tree, P(:,i)', 1);
    end
    X_NB = X(:,NB); % Reorder X
    
    %Compute and apply the transformation
    [q_r, q_t] = get_transformation(P, X_NB);
    Q = [quat2rot(q_r) q_t; 0 0 0 1] * Q;
    P = [quat2rot(q_r) q_t; 0 0 0 1] * [P; ones(1, size(P,2))];
    P = P(1:3, :);
    mse = mean(sum((P-X_NB).^2,2));
    
    if mse < par.mse_threshold
      break
    end
    if (par.debug > 0)
        fprintf('MSE: %g > %g\n', mse, par.mse_threshold);
    end
    
end

kdtree_delete(tree)

end

